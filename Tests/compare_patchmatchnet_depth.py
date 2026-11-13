"""Compare existing camera-Z maps without running either estimator or fusion.

Uses the explicit CVP/PatchmatchNet view mapping, validates poses/intrinsics,
and samples both estimators on identical undistorted rays. Sparse COLMAP
points and cross-view agreement are proxies, not independent ground truth.
"""
import argparse
import csv
import json
from functools import lru_cache
from pathlib import Path

import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
from scipy.spatial.transform import Rotation


def read_depth(path):
    with Path(path).open('rb') as f:
        if Path(path).suffix == '.bin':
            w, h = np.fromfile(f, '<u4', 2)
            data = np.fromfile(f, '<f4')
        else:
            if f.readline().strip() != b'Pf':
                raise ValueError(f'Expected single-channel PFM: {path}')
            w, h = map(int, f.readline().split())
            scale = float(f.readline())
            if abs(scale) != 1:
                raise ValueError('Only unit-scale PFM supported')
            data = np.fromfile(f, '<f4' if scale < 0 else '>f4')
        result = data.reshape(int(h), int(w))
        return np.flipud(result).copy() if Path(path).suffix == '.pfm' else result


def sample(data, x, y):
    """Bilinear sample, requiring four finite positive supporting pixels."""
    h, w = data.shape
    inside = np.isfinite(x) & np.isfinite(y) & (x >= 0) & (y >= 0) & (x < w-1) & (y < h-1)
    sx = np.clip(np.nan_to_num(x, nan=0, posinf=0, neginf=0), 0, w-1.001)
    sy = np.clip(np.nan_to_num(y, nan=0, posinf=0, neginf=0), 0, h-1.001)
    ix, iy = sx.astype(int), sy.astype(int)
    a, b, c, d = data[iy, ix], data[iy, ix+1], data[iy+1, ix], data[iy+1, ix+1]
    ok = inside & np.isfinite(a+b+c+d) & (a > 0) & (b > 0) & (c > 0) & (d > 0)
    dx, dy = sx-ix, sy-iy
    value = (a*(1-dx)+b*dx)*(1-dy)+(c*(1-dx)+d*dx)*dy
    return np.where(ok, value, 0), ok


def stats(v):
    v = np.asarray(v)
    return dict(n=int(v.size), median_pct=float(np.median(v)*100),
                mean_pct=float(v.mean()*100), p90_pct=float(np.quantile(v, .9)*100),
                within_1pct=float((v <= .01).mean()*100), within_2pct=float((v <= .02).mean()*100),
                within_5pct=float((v <= .05).mean()*100)) if v.size else dict(n=0)


def records(path):
    return (s.split() for s in Path(path).read_text().splitlines() if s.strip() and not s.startswith('#'))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--capture', type=Path, required=True)
    p.add_argument('--app-depth', type=Path, required=True)
    p.add_argument('--pm-input', type=Path, required=True, help='Prepared scan folder containing cams/images/pair.txt')
    p.add_argument('--pm-output', type=Path, required=True, help='Output scan folder containing depth_est')
    p.add_argument('--mapping', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--stride', type=int, default=4)
    p.add_argument('--geometry-stride', type=int, default=12)
    p.add_argument('--sources', type=int, default=4)
    p.add_argument('--max-views', type=int, default=0)
    args = p.parse_args()
    if min(args.stride, args.geometry_stride, args.sources) < 1:
        p.error('Strides and source count must be positive')
    if args.output.exists():
        raise FileExistsError('Choose a new output folder')
    cams = {}
    for c in records(args.capture/'cameras.txt'):
        cid, model, w, h = int(c[0]), c[1], int(c[2]), int(c[3])
        if model != 'OPENCV':
            raise ValueError('Only OPENCV cameras supported')
        fx, fy, cx, cy, *dist = map(float, c[4:])
        cams[cid] = (w, h, np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]]), np.array(dist))
    poses = {}
    with (args.capture/'images.txt').open() as f:
        for line in f:
            if not line.strip() or line.startswith('#'):
                continue
            v = line.split()
            qw, qx, qy, qz = map(float, v[1:5])
            rot = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()
            obs = np.fromstring(f.readline(), sep=' ').reshape(-1, 3)
            poses[int(v[0])] = (rot, np.array(list(map(float, v[5:8]))), int(v[8]), v[9], obs)
    points = {int(v[0]): list(map(float, v[1:4])) for v in records(args.capture/'points3D.txt')}
    mapping = list(csv.DictReader(args.mapping.open(newline='')))
    views = {}
    manifest = []
    for row in mapping:
        idx, iid = int(row['cvp_index']), int(row['colmap_image_id'])
        rot, t, cid, name, obs = poses[iid]
        if name != row['source_image'] or cid != int(row['colmap_camera_id']):
            raise ValueError('View mapping mismatch')
        lines = (args.pm_input/'cams'/f'{idx:08d}_cam.txt').read_text().splitlines()
        e = np.loadtxt(lines[1:5]); k = np.loadtxt(lines[7:10])
        pose_error = max(np.abs(e[:3, :3]-rot).max(), np.abs(e[:3, 3]-t).max())
        if pose_error > 1e-7:
            raise ValueError(f'Camera pose/scale mismatch for {name}: {pose_error}')
        with Image.open(args.pm_input/'images'/f'{idx:08d}.png') as im:
            pw, ph = im.size
        ow, oh, raw_k, dist = cams[cid]
        resized_h = round(oh*pw/ow)
        expected = raw_k.copy(); expected[0] *= pw/ow; expected[1] *= resized_h/oh
        expected[1, 2] -= (resized_h-ph)//2
        if np.max(np.abs(k-expected)) > 1e-6:
            raise ValueError('Prepared image crop/intrinsics mismatch')
        app = args.app_depth/f'PlaneSweep_{iid}_{Path(name).stem}.bin'
        pm = args.pm_output/'depth_est'/f'{idx:08d}.pfm'
        for file in (app, pm):
            st = file.stat()
            manifest.append(dict(path=str(file.resolve()), bytes=st.st_size, mtime_ns=st.st_mtime_ns))
        with app.open('rb') as f:
            aw, ah = map(int, np.fromfile(f, '<u4', 2))
        with pm.open('rb') as f:
            f.readline(); dw, dh = map(int, f.readline().split())
        kd = k.copy(); kd[0] *= dw/pw; kd[1] *= dh/ph
        views[idx] = dict(idx=idx, iid=iid, name=name, R=rot, t=t, K=kd, raw_K=raw_k,
                          dist=dist, original=(ow, oh), app_shape=(ah, aw), shape=(dh, dw),
                          app=app, pm=pm, obs=obs, pose_error=pose_error)
    pairs = {}
    with (args.pm_input/'pair.txt').open() as f:
        for _ in range(int(f.readline())):
            idx = int(f.readline()); pairs[idx] = list(map(int, f.readline().split()[1::2]))[:args.sources]

    @lru_cache(maxsize=16)
    def depth(idx, method):
        return read_depth(views[idx]['app' if method == 0 else 'pm'])

    @lru_cache(maxsize=8)
    def mask(idx):
        v = views[idx]
        # Original mask is fixed independently of either depth estimate.
        with Image.open(args.capture/'Mask_shrink'/v['name']) as im:
            return np.asarray(im.convert('L')) > 127

    def app_coords(v, rays):
        x, y = rays[:, 0], rays[:, 1]
        k1, k2, p1, p2 = v['dist']
        r2 = x*x+y*y; radial = 1+k1*r2+k2*r2*r2
        k = v['raw_K']; ow, oh = v['original']; ah, aw = v['app_shape']
        u = k[0, 0]*(x*radial+2*p1*x*y+p2*(r2+2*x*x))+k[0, 2]
        vv = k[1, 1]*(y*radial+p1*(r2+2*y*y)+2*p2*x*y)+k[1, 2]
        return u*aw/ow-.5, vv*ah/oh-.5, u, vv

    def samples(idx, method, rays):
        v = views[idx]
        if method == 0:
            x, y, _, _ = app_coords(v, rays)
        else:
            xy = rays @ v['K'].T; x, y = xy[:, 0], xy[:, 1]
        return sample(depth(idx, method), x, y)

    totals = dict(roi=0, common=0, app_valid=0, pm_valid=0, sparse_roi=0,
                  sparse_common=0, app_sparse_valid=0, pm_sparse_valid=0,
                  grid_roi=0, geo_common_eligible=0, app_geo=0, pm_geo=0,
                  app_geo_common=0, pm_geo_common=0)
    errors = [[], []]; own_errors = [[], []]; disagreements = []; rows = []; panels = []
    ids = sorted(views)
    if args.max_views:
        ids = ids[:args.max_views]
    selected = {ids[0], ids[len(ids)//3], ids[2*len(ids)//3]}
    args.output.mkdir(parents=True)
    for index, idx in enumerate(ids):
        v = views[idx]; h, w = v['shape']
        yy, xx = np.mgrid[0:h:args.stride, 0:w:args.stride]
        rays = np.column_stack([xx.ravel(), yy.ravel(), np.ones(xx.size)]) @ np.linalg.inv(v['K']).T
        ax, ay, u, vv = app_coords(v, rays)
        ow, oh = v['original']
        inside = (u >= 0) & (vv >= 0) & (u < ow) & (vv < oh)
        roi = inside & mask(idx)[np.clip(vv.astype(int), 0, oh-1), np.clip(u.astype(int), 0, ow-1)]
        estimates = [samples(idx, m, rays) for m in range(2)]
        common = roi & estimates[0][1] & estimates[1][1]
        totals['roi'] += int(roi.sum()); totals['common'] += int(common.sum())
        row = dict(index=idx, image=v['name'], roi_samples=int(roi.sum()), common_samples=int(common.sum()))
        for m, name in enumerate(['app', 'pm']):
            n = int((roi & estimates[m][1]).sum()); totals[name+'_valid'] += n
            row[name+'_coverage_pct'] = 100*n/max(int(roi.sum()), 1)
        d = np.abs(estimates[0][0][common]-estimates[1][0][common])/np.maximum(estimates[0][0][common], estimates[1][0][common])
        disagreements.append(d); row['depth_disagreement_median_pct'] = stats(d).get('median_pct')
        # Evaluate observed sparse 3D points along their projected camera rays.
        point_ids = sorted({int(o[2]) for o in v['obs'] if int(o[2]) in points})
        xyz = np.array([points[pid] for pid in point_ids]) @ v['R'].T + v['t']
        sr = xyz/xyz[:, 2:3]
        _, _, su, sv = app_coords(v, sr)
        sroi = (xyz[:, 2] > 0) & (su >= 0) & (sv >= 0) & (su < ow) & (sv < oh)
        sroi &= mask(idx)[np.clip(sv.astype(int), 0, oh-1), np.clip(su.astype(int), 0, ow-1)]
        sparse_pixels = sr @ v['K'].T
        sroi &= (sparse_pixels[:, 0] >= 0) & (sparse_pixels[:, 0] < w-1)
        sroi &= (sparse_pixels[:, 1] >= 0) & (sparse_pixels[:, 1] < h-1)
        sp = [samples(idx, m, sr) for m in range(2)]
        sc = sroi & sp[0][1] & sp[1][1]
        totals['sparse_roi'] += int(sroi.sum()); totals['sparse_common'] += int(sc.sum())
        for m, name in enumerate(['app', 'pm']):
            er = np.abs(sp[m][0]-xyz[:, 2])/np.maximum(xyz[:, 2], 1e-9)
            errors[m].append(er[sc]); own_errors[m].append(er[sroi & sp[m][1]])
            totals[name+'_sparse_valid'] += int((sroi & sp[m][1]).sum())
            row[name+'_paired_sparse_median_pct'] = stats(er[sc]).get('median_pct')
        # Identical reference grid/source cameras and thresholds for both methods.
        gy, gx = np.mgrid[0:h:args.geometry_stride, 0:w:args.geometry_stride]
        gr = np.column_stack([gx.ravel(), gy.ravel(), np.ones(gx.size)]) @ np.linalg.inv(v['K']).T
        _, _, gu, gv = app_coords(v, gr)
        groi = (gu >= 0) & (gv >= 0) & (gu < ow) & (gv < oh)
        groi &= mask(idx)[np.clip(gv.astype(int), 0, oh-1), np.clip(gu.astype(int), 0, ow-1)]
        gr, gx, gy = gr[groi], gx.ravel()[groi], gy.ravel()[groi]
        totals['grid_roi'] += len(gr)
        obs_counts = [np.zeros(len(gr), int) for _ in range(2)]
        good_counts = [np.zeros(len(gr), int) for _ in range(2)]
        paired_obs = np.zeros(len(gr), int); paired_good = [np.zeros(len(gr), int) for _ in range(2)]
        refs = [samples(idx, m, gr) for m in range(2)]
        for sid in pairs[idx]:
            src = views[sid]; measured = []; agrees = []
            for m in range(2):
                z, valid = refs[m]
                world = (gr*z[:, None]-v['t']) @ v['R']
                cam = world @ src['R'].T+src['t']
                front = cam[:, 2] > 1e-8
                ray = cam/np.maximum(cam[:, 2:3], 1e-8)
                sz, support = samples(sid, m, ray)
                available = valid & support & front
                back_world = (ray*sz[:, None]-src['t']) @ src['R']
                back = back_world @ v['R'].T+v['t']
                proj = back @ v['K'].T
                px = proj[:, 0]/np.maximum(proj[:, 2], 1e-8)
                py = proj[:, 1]/np.maximum(proj[:, 2], 1e-8)
                # Pixel threshold defined in the common PM grid for both estimators.
                good = available & (back[:, 2] > 0) & (np.hypot(px-gx, py-gy) <= 1)
                good &= np.abs(back[:, 2]-z)/np.maximum(np.maximum(back[:, 2], z), 1e-8) <= .01
                measured.append(available); agrees.append(good)
                obs_counts[m] += available; good_counts[m] += good
            paired = measured[0] & measured[1]; paired_obs += paired
            for m in range(2):
                paired_good[m] += paired & agrees[m]
        eligible = paired_obs >= 2; totals['geo_common_eligible'] += int(eligible.sum())
        for m, name in enumerate(['app', 'pm']):
            totals[name+'_geo'] += int((good_counts[m] >= 2).sum())
            totals[name+'_geo_common'] += int((eligible & (paired_good[m] >= 2)).sum())
            row[name+'_geometry_yield_pct'] = 100*float((good_counts[m] >= 2).mean()) if len(gr) else None
        rows.append(row)
        if idx in selected:
            panels.append((idx, xx.shape, roi.copy(), [e[0].copy() for e in estimates], common.copy()))
        if (index+1) % 10 == 0 or index == len(ids)-1:
            print(f'Compared {index+1}/{len(ids)} views', flush=True)
    for entry in manifest:
        st = Path(entry['path']).stat()
        if st.st_size != entry['bytes'] or st.st_mtime_ns != entry['mtime_ns']:
            raise RuntimeError(f'Input changed during comparison: {entry["path"]}')
    summary = dict(views=len(ids), ground_truth=False, totals=totals,
                   disagreement_symmetric_relative=stats(np.concatenate(disagreements)), methods={})
    for m, name in enumerate(['app', 'pm']):
        summary['methods'][name] = dict(coverage_pct=100*totals[name+'_valid']/max(totals['roi'], 1),
            paired_sparse=stats(np.concatenate(errors[m])), own_sparse=stats(np.concatenate(own_errors[m])),
            sparse_coverage_pct=100*totals[name+'_sparse_valid']/max(totals['sparse_roi'], 1),
            geometry_verified_yield_pct=100*totals[name+'_geo']/max(totals['grid_roi'], 1),
            paired_geometry_agreement_pct=100*totals[name+'_geo_common']/max(totals['geo_common_eligible'], 1))
    summary['audit'] = dict(max_pose_error=max(v['pose_error'] for v in views.values()),
        app_shape=views[ids[0]]['app_shape'], pm_shape=views[ids[0]]['shape'],
        paired_sparse_views_app_lower=sum(r['app_paired_sparse_median_pct'] < r['pm_paired_sparse_median_pct'] for r in rows if r['app_paired_sparse_median_pct'] is not None),
        parameters={k: str(val) if isinstance(val, Path) else val for k, val in vars(args).items()},
        caveats=['Existing saved outputs; estimator settings and source-view selections are not proven identical.',
                 'App maps are post matching acceptance but before cross-view correction and fusion; PM depth_est is before confidence/geometric fusion.',
                 'Foreground is the fixed Mask_shrink, mapped to the common undistorted view.',
                 'COLMAP sparse points also support calibration/depth bounds: reference proxy, not independent truth.',
                 'Cross-view consistency measures self-consistency, not absolute accuracy.',
                 'App pixel centers use +0.5; PM uses integer centers, following each local implementation.',
                 'Threshold: >=2 of first requested pair.txt sources, <=1 common-grid pixel and <=1% relative camera-Z.',
                 'No metric units inferred from COLMAP scale. Coverage requires four valid bilinear neighbors.'])
    (args.output/'summary.json').write_text(json.dumps(summary, indent=2))
    (args.output/'input_manifest.json').write_text(json.dumps(manifest, indent=2))
    with (args.output/'per_view.csv').open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=rows[0]); writer.writeheader(); writer.writerows(rows)
    report = ['frame_00000_full: pre-fusion depth comparison', f'Views: {len(ids)}',
              'No independent dense ground truth: these results measure reference agreement and self-consistency.', '',
              'Metric | GraphicsApp | PatchmatchNet']
    a, b = summary['methods']['app'], summary['methods']['pm']
    for label, val_a, val_b in [('Foreground coverage (%)', a['coverage_pct'], b['coverage_pct']),
        ('Paired sparse median relative error (%)', a['paired_sparse']['median_pct'], b['paired_sparse']['median_pct']),
        ('Paired sparse p90 relative error (%)', a['paired_sparse']['p90_pct'], b['paired_sparse']['p90_pct']),
        ('Sparse reference coverage (%)', a['sparse_coverage_pct'], b['sparse_coverage_pct']),
        ('Paired cross-view agreement (%)', a['paired_geometry_agreement_pct'], b['paired_geometry_agreement_pct']),
        ('Verified foreground grid yield (%)', a['geometry_verified_yield_pct'], b['geometry_verified_yield_pct'])]:
        report.append(f'{label} | {val_a:.4f} | {val_b:.4f}')
    report += ['', f'Paired sparse samples: {totals["sparse_common"]}',
               f'Paired geometry eligible samples: {totals["geo_common_eligible"]}',
               f'Depth disagreement (symmetric relative): {summary["disagreement_symmetric_relative"]}',
               '', 'Limits / method:', *summary['audit']['caveats']]
    (args.output/'report.txt').write_text('\n'.join(report))
    fig, axes = plt.subplots(len(panels), 4, figsize=(16, 4*len(panels)), squeeze=False, layout='constrained')
    for row, (idx, shape, roi, ds, common) in enumerate(panels):
        im = Image.open(args.pm_input/'images'/f'{idx:08d}.png')
        axes[row, 0].imshow(im); axes[row, 0].set_title(views[idx]['name'])
        valid_values = np.concatenate([d[roi & (d > 0)] for d in ds])
        near, far = np.quantile(valid_values, [.02, .98])
        for m, label in enumerate(['GraphicsApp', 'PatchmatchNet']):
            pic = np.ma.masked_where(~roi | (ds[m] <= 0), ds[m]).reshape(shape)
            artist = axes[row, m+1].imshow(pic, vmin=near, vmax=far, cmap='viridis')
            axes[row, m+1].set_title(label+' camera-Z (shared scale)')
        fig.colorbar(artist, ax=list(axes[row, 1:3]), shrink=.7)
        rel = 100*np.abs(ds[0]-ds[1])/np.maximum(np.maximum(ds[0], ds[1]), 1e-8)
        artist = axes[row, 3].imshow(np.ma.masked_where(~common, rel).reshape(shape), vmin=0, vmax=5, cmap='magma')
        axes[row, 3].set_title('Depth disagreement (%)'); fig.colorbar(artist, ax=axes[row, 3], shrink=.7)
        for ax in axes[row]: ax.axis('off')
    fig.suptitle('Existing saved maps before fusion | fixed foreground | no dense ground truth')
    fig.savefig(args.output/'depth_comparison.png', dpi=150); plt.close(fig)
    fig, axes = plt.subplots(2, 3, figsize=(13, 7), layout='constrained')
    metrics = [('Foreground coverage (%)', 'coverage_pct', None),
               ('Paired sparse median error (%) - lower is better', 'paired_sparse', 'median_pct'),
               ('Paired sparse p90 error (%) - lower is better', 'paired_sparse', 'p90_pct'),
               ('Sparse reference coverage (%)', 'sparse_coverage_pct', None),
               ('Paired cross-view agreement (%)', 'paired_geometry_agreement_pct', None),
               ('Verified foreground grid yield (%)', 'geometry_verified_yield_pct', None)]
    for ax, (title, key, subkey) in zip(axes.ravel(), metrics):
        values = [m[key][subkey] if subkey else m[key] for m in (a, b)]
        ax.bar(['GraphicsApp', 'PatchmatchNet'], values, color=['#617ea8', '#209989'])
        ax.set_title(title, fontsize=10); ax.set_ylim(0, max(values)*1.2)
        ax.spines[['top', 'right']].set_visible(False)
        for j, value in enumerate(values): ax.text(j, value, f'{value:.2f}', ha='center', va='bottom')
    fig.suptitle(f'{len(ids)} views | depth before fusion | reference proxies, no dense ground truth')
    fig.savefig(args.output/'metrics.png', dpi=150); plt.close(fig)
    print('\n'.join(report), flush=True)


if __name__ == '__main__':
    main()

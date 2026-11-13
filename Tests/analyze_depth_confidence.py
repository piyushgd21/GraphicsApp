"""Evaluate confidence rejection using a fixed GPU ablation and sparse references.

Both rankings see exactly the same unfiltered candidate locations. Confidence
curves rank the same sparse-reference pool at equal retained sample counts;
this isolates score ranking from the different estimators' baseline errors.
Foreground curves additionally report density at operational thresholds.
"""
import argparse
import csv
import json
from pathlib import Path

import cv2
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from PIL import Image
from compare_patchmatchnet_depth import read_depth, sample, stats


def min_score(data, x, y):
    """Minimum confidence in the four pixels required by bilinear depth sampling."""
    h, w = data.shape
    ix = np.clip(np.nan_to_num(x, nan=0, posinf=0, neginf=0), 0, w-1.001).astype(int)
    iy = np.clip(np.nan_to_num(y, nan=0, posinf=0, neginf=0), 0, h-1.001).astype(int)
    return np.minimum.reduce([data[iy, ix], data[iy+1, ix], data[iy, ix+1], data[iy+1, ix+1]])


def ranked_mask(score, fraction):
    """Retain an exact count; deterministic index ordering resolves confidence ties."""
    order = np.argsort(-score, kind='stable')
    keep = np.zeros(len(score), bool)
    keep[order[:max(1, int(round(fraction*len(score))))]] = True
    return keep


def cutoff_weights(reference_scores, fraction, query_scores):
    """Expected retention for uniform random tie breaking at an exact coverage.

    A sparse query tied at the cutoff has the same inclusion probability as a
    foreground pixel tied at the cutoff. Avoid selecting spatially ordered ties.
    """
    if fraction >= 1:
        return np.ones(len(query_scores)), float(np.finfo(float).min), 1., len(reference_scores)
    target = max(1, int(round(len(reference_scores)*fraction)))
    cutoff = np.sort(reference_scores)[-target]
    above = int((reference_scores > cutoff).sum())
    tied = int((reference_scores == cutoff).sum())
    tie_weight = (target-above)/tied
    weights = (query_scores > cutoff).astype(float) + (query_scores == cutoff)*tie_weight
    return weights, float(cutoff), float(tie_weight), target


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('benchmark', type=Path)
    parser.add_argument('--capture', type=Path, required=True)
    parser.add_argument('--mapping', type=Path, required=True)
    parser.add_argument('--pm-input', type=Path, required=True)
    parser.add_argument('--pm-output', type=Path, required=True)
    args = parser.parse_args()
    root = args.benchmark
    views = [json.loads(line) for line in (root/'views.jsonl').read_text().splitlines()]
    mapping = {int(r['colmap_image_id']): int(r['cvp_index']) for r in csv.DictReader(args.mapping.open())}
    sparse = np.atleast_1d(np.genfromtxt(root/'sparse_samples.csv', delimiter=',', names=True))
    settings = dict(line.split('=', 1) for line in (root/'effective_settings.txt').read_text().splitlines() if '=' in line)
    threshold = float(settings['confidence_gap'])*.015625
    all_error = [[], []]; all_score = [[], []]; rows = []; curves = []; foreground_curves = []
    sweep = {factor: dict(grid_kept=0, native_kept=0, errors=[]) for factor in [0, .125, .25, .5, 1., 2., 4.]}
    totals = dict(roi=0, raw=0, on=0, restored=0, native_rejected=0,
                  pm_conf_05=0, pm_conf_08=0, app_sparse_kept=0, sparse=0,
                  restoration_good_1pct=0, restoration_sparse=0, paired_planes_changed=0,
                  native_roi=0, native_raw=0, native_on=0)
    rejected_error = []; kept_error = []; paired_pm_on_error = []
    preview = None
    for v in views:
        iid = v['id']; idx = mapping[iid]
        off = read_depth(root/'confidence_off'/f'{iid}.depth.bin')
        on = read_depth(root/'confidence_on'/f'{iid}.depth.bin')
        gap = read_depth(root/'confidence_on'/f'{iid}.gap.bin')
        status = read_depth(root/'confidence_on'/f'{iid}.status.bin')
        cost_on = read_depth(root/'confidence_on'/f'{iid}.cost.bin')
        cost_off = read_depth(root/'confidence_off'/f'{iid}.cost.bin')
        if not np.array_equal(cost_on, cost_off):
            raise AssertionError(f'Confidence changed matching costs in view {iid}')
        accepted = np.isfinite(on) & (on > 0)
        if not np.array_equal(on[accepted], off[accepted]):
            raise AssertionError(f'Confidence changed accepted depths in view {iid}')
        if np.any((status == 2) & (off <= 0)):
            raise AssertionError('Confidence-only rejected pixels were not restored')
        totals['native_rejected'] += int((status == 2).sum())
        pm = read_depth(args.pm_output/'depth_est'/f'{idx:08d}.pfm')
        conf = read_depth(args.pm_output/'confidence'/f'{idx:08d}.pfm')
        if conf.shape != pm.shape or not np.isfinite(conf).all():
            raise AssertionError('Invalid PM confidence map')
        lines = (args.pm_input/'cams'/f'{idx:08d}_cam.txt').read_text().splitlines()
        k = np.loadtxt(lines[7:10])
        with Image.open(args.pm_input/'images'/f'{idx:08d}.png') as im:
            iw, ih = im.size
        h, w = pm.shape; k[0] *= w/iw; k[1] *= h/ih
        ow, oh = v['image_size']; ah, aw = off.shape
        fx, fy, cx, cy, *dist = v['intrinsics']
        raw_k = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
        with Image.open(args.capture/'Mask_shrink'/v['name']) as im:
            mask = np.asarray(im.convert('L')) > 127
        native_mask = mask[np.minimum(((np.arange(ah)+.5)*oh/ah).astype(int), oh-1)[:, None],
                           np.minimum(((np.arange(aw)+.5)*ow/aw).astype(int), ow-1)[None, :]]
        totals['native_roi'] += int(native_mask.sum())
        totals['native_raw'] += int((native_mask & (off > 0)).sum())
        totals['native_on'] += int((native_mask & (on > 0)).sum())

        def warp(rays):
            uv, _ = cv2.projectPoints(rays, np.zeros(3), np.zeros(3), raw_k, np.array(dist))
            uv = uv.reshape(-1, 2)
            return uv[:, 0]*aw/ow-.5, uv[:, 1]*ah/oh-.5, uv

        def roi_at(uv):
            u, vv = uv.T
            return ((u >= 0) & (vv >= 0) & (u < ow) & (vv < oh) &
                    mask[np.clip(vv.astype(int), 0, oh-1), np.clip(u.astype(int), 0, ow-1)])

        yy, xx = np.mgrid[0:h:4, 0:w:4]
        rays = np.column_stack([xx.ravel(), yy.ravel(), np.ones(xx.size)]) @ np.linalg.inv(k).T
        ax, ay, uv = warp(rays); roi = roi_at(uv)
        zraw, raw_ok = sample(off, ax, ay); zon, on_ok = sample(on, ax, ay)
        zp, pm_ok = sample(pm, xx.ravel().astype(float), yy.ravel().astype(float))
        common = roi & raw_ok & pm_ok
        gs = min_score(gap, ax, ay); ps = min_score(conf, xx.ravel(), yy.ravel())
        if not np.array_equal(on_ok[common], (gs[common] >= np.float32(threshold))):
            raise AssertionError('Exported confidence gap did not reproduce the actual GPU acceptance')
        totals['roi'] += int(roi.sum()); totals['raw'] += int((roi & raw_ok).sum())
        totals['on'] += int((roi & on_ok).sum()); totals['restored'] += int((roi & raw_ok & ~on_ok).sum())
        totals['pm_conf_05'] += int((roi & pm_ok & (ps > .5)).sum())
        totals['pm_conf_08'] += int((roi & pm_ok & (ps > .8)).sum())
        s = sparse[sparse['image_id'] == iid]
        native = np.column_stack([s['x']+.5, s['y']+.5])
        sr2 = cv2.undistortPoints(native.reshape(-1, 1, 2), raw_k, np.array(dist)).reshape(-1, 2)
        sr = np.column_stack([sr2, np.ones(len(sr2))])
        pxy = sr @ k.T; sx = native[:, 0]*aw/ow-.5; sy = native[:, 1]*ah/oh-.5
        sr_raw, valid_raw = sample(off, sx, sy); sr_on, valid_on = sample(on, sx, sy)
        sr_pm, valid_pm = sample(pm, pxy[:, 0], pxy[:, 1])
        eligible = roi_at(native) & valid_raw & valid_pm & (s['depth'] > 0)
        er = [np.abs(sr_raw-s['depth'])/s['depth'], np.abs(sr_pm-s['depth'])/s['depth']]
        scores = [min_score(gap, sx, sy), min_score(conf, pxy[:, 0], pxy[:, 1])]
        n = int(eligible.sum()); totals['sparse'] += n
        totals['app_sparse_kept'] += int((eligible & valid_on).sum())
        restored = eligible & ~valid_on
        totals['restoration_sparse'] += int(restored.sum())
        totals['restoration_good_1pct'] += int((restored & (er[0] <= .01)).sum())
        rejected_error.extend(er[0][restored]); kept_error.extend(er[0][eligible & valid_on])
        paired_pm_on_error.extend(er[1][eligible & valid_on])
        for m in range(2):
            all_error[m].extend(er[m][eligible]); all_score[m].extend(scores[m][eligible])
        for factor, data in sweep.items():
            keep_grid = np.ones(len(gs), bool) if factor == 0 else gs >= np.float32(threshold*factor)
            keep_sparse = np.ones(len(scores[0]), bool) if factor == 0 else scores[0] >= np.float32(threshold*factor)
            data['grid_kept'] += int((common & keep_grid).sum())
            keep_native = np.ones(gap.shape, bool) if factor == 0 else gap >= np.float32(threshold*factor)
            data['native_kept'] += int((native_mask & (off > 0) & keep_native).sum())
            data['errors'].extend(er[0][eligible & keep_sparse])
        row = dict(id=iid, image=v['name'], sparse_candidates=n,
                   raw_coverage_pct=100*float((roi & raw_ok).sum())/max(int(roi.sum()), 1),
                   confidence_on_coverage_pct=100*float((roi & on_ok).sum())/max(int(roi.sum()), 1),
                   confidence_removed_pct_of_raw=100*float((common & ~on_ok).sum())/max(int(common.sum()), 1),
                   removed_sparse=int(restored.sum()), removed_sparse_good_1pct=int((restored & (er[0] <= .01)).sum()))
        rows.append(row)
        # Equal retained sparse counts per view prevents easy camera views dominating rankings.
        for f in [.1, .25, .5, .75, .9, 1.]:
            for m, name in enumerate(['app', 'pm']):
                e, score = er[m][eligible], scores[m][eligible]
                keep = ranked_mask(score, f)
                curves.append(dict(id=iid, method=name, fraction=f, n=int(keep.sum()),
                    error_sum=float(e[keep].sum()), bad_1pct=int((e[keep] > .01).sum()),
                    # Fraction of candidate foreground retained using the same score cutoff.
                    foreground_retained_pct=100*float((np.array([gs, ps])[m][common] >= score[keep].min()).mean()),
                    tied_at_cutoff=int((score == score[keep].min()).sum())))
                grid_score = (gs if m == 0 else ps)[common]
                weights, cutoff, tie_weight, target = cutoff_weights(grid_score, f, score)
                foreground_curves.append(dict(id=iid, method=name, fraction=f,
                    grid_target=target, roi_n=int(roi.sum()), sparse_weight=float(weights.sum()),
                    error_sum=float((weights*e).sum()), bad_1pct=float((weights*(e > .01)).sum()),
                    cutoff=cutoff, cutoff_tie_weight=tie_weight))
        if preview is None:
            preview = (xx.shape, roi, raw_ok, on_ok, zraw, gs, ps)
        print(f'Confidence analyzed id={iid}: {n} paired sparse samples', flush=True)
    errors = [np.array(e) for e in all_error]; scores = [np.array(s) for s in all_score]
    summary = dict(views=len(views), totals=totals, app_gap_threshold=threshold,
        app_no_confidence_error=stats(errors[0]), pm_same_candidates_error=stats(errors[1]),
        app_current_confidence_kept_error=stats(kept_error), pm_same_app_kept_locations_error=stats(paired_pm_on_error),
        app_confidence_removed_error=stats(rejected_error), curves=[], foreground_curves=[], threshold_sweep=[],
        caveats=['Sparse COLMAP reference is not independent ground truth.',
                 'Only 12 uniformly spaced GPU views, using current setup, not all 299 views.',
                 'App raw here retains cost/support/mask rules; only confidence is removed.',
                 'Identical app costs and accepted depths verified exactly between ON/OFF runs.',
                 'PM scores and depths come from its existing run; source selection and preprocessing differ.',
                 'Both confidence rankings use the same unfiltered app/PM sparse-reference candidate domain.',
                 'Equal retained counts are set per view; ties use stable index ordering.',
                 'Confidence quality is measured by error reduction relative to each estimator unfiltered error.',
                 'All four bilinear neighbors must pass confidence for a sampled location to survive.',
                 'Foreground coverage is matched per view on a common grid; boundary ties have fractional expected retention.'])
    for f in [.1, .25, .5, .75, .9, 1.]:
        for m, name in enumerate(['app', 'pm']):
            relevant = [r for r in curves if r['fraction'] == f and r['method'] == name]
            n = sum(r['n'] for r in relevant)
            mean = sum(r['error_sum'] for r in relevant)/n
            summary['curves'].append(dict(method=name, retained_pct=100*f, n=n,
                mean_error_pct=100*mean, bad_1pct_pct=100*sum(r['bad_1pct'] for r in relevant)/n,
                error_reduction_pct=100*(1-mean/errors[m].mean()),
                max_view_tie_at_cutoff=max(r['tied_at_cutoff'] for r in relevant)))
            fc = [r for r in foreground_curves if r['fraction'] == f and r['method'] == name]
            weight = sum(r['sparse_weight'] for r in fc)
            mean = sum(r['error_sum'] for r in fc)/weight if weight else None
            summary['foreground_curves'].append(dict(method=name, retained_candidate_pct=100*f,
                foreground_coverage_pct=100*sum(r['grid_target'] for r in fc)/sum(r['roi_n'] for r in fc),
                sparse_weight=weight, mean_error_pct=100*mean if mean is not None else None,
                bad_1pct_pct=100*sum(r['bad_1pct'] for r in fc)/weight if weight else None,
                error_reduction_pct=100*(1-mean/errors[m].mean()) if mean is not None else None))
    for factor, data in sweep.items():
        summary['threshold_sweep'].append(dict(factor=factor, configured_minimum=float(settings['confidence_gap'])*factor,
            foreground_coverage_pct=100*data['grid_kept']/totals['roi'],
            native_foreground_coverage_pct=100*data['native_kept']/totals['native_roi'], errors=stats(data['errors'])))
    (root/'confidence_summary.json').write_text(json.dumps(summary, indent=2))
    for file, data in [('confidence_per_view.csv', rows), ('confidence_curves_per_view.csv', curves),
                       ('confidence_foreground_curves_per_view.csv', foreground_curves)]:
        with (root/file).open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=data[0]); writer.writeheader(); writer.writerows(data)
    t = totals
    report = [f'Confidence comparison: {len(views)} controlled views',
        f'App foreground coverage with confidence: {100*t["on"]/t["roi"]:.3f}%',
        f'App foreground coverage without confidence: {100*t["raw"]/t["roi"]:.3f}%',
        f'Confidence removes {100*(t["raw"]-t["on"])/t["raw"]:.3f}% of otherwise usable app samples.',
        f'Native-pixel app foreground coverage ON/OFF: {100*t["native_on"]/t["native_roi"]:.3f}% / {100*t["native_raw"]/t["native_roi"]:.3f}%',
        f'PM foreground coverage at confidence >0.5: {100*t["pm_conf_05"]/t["roi"]:.3f}%',
        f'PM foreground coverage at confidence >0.8: {100*t["pm_conf_08"]/t["roi"]:.3f}%',
        f'Restored sparse locations within 1% of reference: {t["restoration_good_1pct"]}/{t["restoration_sparse"]}',
        '', 'App confidence-kept errors: '+str(summary['app_current_confidence_kept_error']),
        'App confidence-removed errors: '+str(summary['app_confidence_removed_error']),
        '', 'Equal retained sparse-reference counts (per view):',
        'Method | retained % | mean error % | errors over 1% (%) | error reduction vs unfiltered (%)']
    report += [f'{r["method"]} | {r["retained_pct"]:.0f} | {r["mean_error_pct"]:.4f} | {r["bad_1pct_pct"]:.3f} | {r["error_reduction_pct"]:.3f}' for r in summary['curves']]
    report += ['', 'Matched foreground coverage (same candidate grid and retained counts per view):',
               'Method | foreground coverage % | mean reference error % | errors over 1% (%) | error reduction (%)']
    report += [f'{r["method"]} | {r["foreground_coverage_pct"]:.3f} | {r["mean_error_pct"]:.4f} | {r["bad_1pct_pct"]:.3f} | {r["error_reduction_pct"]:.3f}' for r in summary['foreground_curves']]
    report += ['', 'App threshold sweep (same estimated depths, only confidence changes):',
               'Threshold factor | sampled foreground coverage % | native foreground coverage % | sparse mean error % | sparse p90 error % | sparse samples']
    report += [f'{r["factor"]:.3f} | {r["foreground_coverage_pct"]:.3f} | {r["native_foreground_coverage_pct"]:.3f} | {r["errors"]["mean_pct"]:.4f} | {r["errors"]["p90_pct"]:.4f} | {r["errors"]["n"]}' for r in summary['threshold_sweep']]
    report += ['', 'Limits:', *summary['caveats']]
    (root/'confidence_report.txt').write_text('\n'.join(report))
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.5), layout='constrained')
    for name, label, color in [('app', 'GraphicsApp', '#617ea8'), ('pm', 'PatchmatchNet', '#209989')]:
        c = [r for r in summary['curves'] if r['method'] == name]
        for ax, key in zip(axes, ['mean_error_pct', 'bad_1pct_pct', 'error_reduction_pct']):
            ax.plot([r['retained_pct'] for r in c], [r[key] for r in c], 'o-', label=label, color=color)
            ax.set_xlabel('Retained sparse samples (%)'); ax.spines[['top', 'right']].set_visible(False)
    axes[0].set_title('Mean reference error (%) - lower is better')
    axes[1].set_title('Errors over 1% (%) - lower is better')
    axes[2].set_title('Error reduction vs unfiltered (%) - higher is better')
    axes[2].axhline(0, color='#999999', linestyle='--'); axes[0].legend()
    fig.suptitle(f'{len(views)} views | same candidate locations | sparse reference, no dense ground truth')
    fig.savefig(root/'confidence_quality.png', dpi=160); plt.close(fig)
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.5), layout='constrained')
    for name, label, color in [('app', 'GraphicsApp', '#617ea8'), ('pm', 'PatchmatchNet', '#209989')]:
        c = [r for r in summary['foreground_curves'] if r['method'] == name]
        for ax, key in zip(axes, ['mean_error_pct', 'bad_1pct_pct', 'error_reduction_pct']):
            ax.plot([r['foreground_coverage_pct'] for r in c], [r[key] for r in c], 'o-', label=label, color=color)
            ax.set_xlabel('Foreground coverage (%)'); ax.spines[['top', 'right']].set_visible(False)
    axes[0].set_title('Mean reference error (%) - lower is better')
    axes[1].set_title('Errors over 1% (%) - lower is better')
    axes[2].set_title('Error reduction vs unfiltered (%) - higher is better')
    axes[2].axhline(0, color='#999999', linestyle='--'); axes[0].legend()
    fig.suptitle(f'{len(views)} views | matched foreground coverage per view | sparse reference')
    fig.savefig(root/'confidence_foreground_quality.png', dpi=160); plt.close(fig)
    print('\n'.join(report))


if __name__ == '__main__':
    main()

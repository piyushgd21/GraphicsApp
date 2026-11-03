"""Analyze saved GPU-only tilt ablations; no reconstruction or depth search runs here."""
import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.path import Path as PlotPath
import numpy as np
from PIL import Image
from scipy.spatial import ConvexHull


def read_array(path):
    with open(path, "rb") as stream:
        w, h = np.fromfile(stream, dtype="<u4", count=2)
        return np.fromfile(stream, dtype="<f4").reshape(int(h), int(w))


def intrinsics(view, shape):
    h, w = shape
    ow, oh = view["image_size"]
    fx, fy, cx, cy, k1, k2, p1, p2 = view["intrinsics"]
    return fx*w/ow, fy*h/oh, cx*w/ow, cy*h/oh, k1, k2, p1, p2


def rays(view, shape, x, y):
    fx, fy, cx, cy, k1, k2, p1, p2 = intrinsics(view, shape)
    tx, ty = (x + .5-cx)/fx, (y + .5-cy)/fy
    lx, ly = tx.copy(), ty.copy()
    for _ in range(5):
        r2 = lx*lx+ly*ly
        radial = 1+k1*r2+k2*r2*r2
        dx = lx*radial+2*p1*lx*ly+p2*(r2+2*lx*lx)
        dy = ly*radial+p1*(r2+2*ly*ly)+2*p2*lx*ly
        lx += tx-dx
        ly += ty-dy
    return (lx[:, None]*view["right"]-ly[:, None]*view["up"]+np.asarray(view["forward"]))


def project(view, shape, points):
    fx, fy, cx, cy, k1, k2, p1, p2 = intrinsics(view, shape)
    delta = points-np.asarray(view["position"])
    z = delta@np.asarray(view["forward"])
    safe = np.where(z > 1e-8, z, 1)
    lx, ly = delta@np.asarray(view["right"])/safe, -delta@np.asarray(view["up"])/safe
    r2 = lx*lx+ly*ly
    radial = 1+k1*r2+k2*r2*r2
    x = fx*(lx*radial+2*p1*lx*ly+p2*(r2+2*lx*lx))+cx-.5
    y = fy*(ly*radial+p1*(r2+2*ly*ly)+2*p2*lx*ly)+cy-.5
    return x, y, z


def sample(depth, x, y):
    h, w = depth.shape
    finite = np.isfinite(x)&np.isfinite(y)
    valid = finite&(x >= 0)&(y >= 0)&(x < w-1)&(y < h-1)
    xx = np.clip(np.nan_to_num(x, nan=0, posinf=0, neginf=0), 0, w-1.00001)
    yy = np.clip(np.nan_to_num(y, nan=0, posinf=0, neginf=0), 0, h-1.00001)
    ix, iy = xx.astype(int), yy.astype(int)
    a, b, c, d = depth[iy, ix], depth[iy, ix+1], depth[iy+1, ix], depth[iy+1, ix+1]
    valid &= (a > 0)&(b > 0)&(c > 0)&(d > 0)
    sx, sy = xx-ix, yy-iy
    z = (a*(1-sx)+b*sx)*(1-sy)+(c*(1-sx)+d*sx)*sy
    return z, valid


def stats(values):
    values = np.asarray(values)
    if not values.size:
        return {"n": 0}
    return {"n": int(values.size), "mean": float(np.mean(values)), "median": float(np.median(values)),
            "p90": float(np.quantile(values, .9)), "within_1pct": float(np.mean(values < .01)),
            "within_2pct": float(np.mean(values < .02)), "within_5pct": float(np.mean(values < .05))}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("folder", type=Path)
    args = parser.parse_args()
    root = args.folder
    views = [json.loads(line) for line in (root/"views.jsonl").read_text().splitlines()]
    by_id = {v["id"]: v for v in views}
    names = ["tilt_off", "tilt_on"]
    maps = {name: {v["id"]: read_array(root/name/f'{v["id"]}.depth.bin') for v in views} for name in names}
    sparse = np.genfromtxt(root/"sparse_samples.csv", delimiter=",", names=True)
    all_sparse = [[], []]
    per_view = []
    total = {"valid": [0, 0], "pixels": 0, "roi_valid": [0, 0], "roi_pixels": 0,
             "geo_tested": [0, 0], "geo_verified": [0, 0], "grid_roi": 0,
             "paired_tested": 0, "paired_verified": [0, 0], "common_cost_sum": [0., 0.], "common_cost_n": 0}
    sparse_by_id = {}
    for view in views:
        vid = view["id"]
        off, on = [maps[name][vid] for name in names]
        shape = off.shape
        h, w = shape
        s = sparse[sparse["image_id"] == vid]
        x = (s["x"]+.5)*w/view["image_size"][0]-.5
        y = (s["y"]+.5)*h/view["image_size"][1]-.5
        inside = (x >= 0)&(y >= 0)&(x < w)&(y < h)&(s["depth"] > 0)
        x, y, gt = x[inside], y[inside], s["depth"][inside]
        predictions = [sample(d, x, y) for d in [off, on]]
        common = predictions[0][1]&predictions[1][1]
        errors = [np.abs(p[0][common]-gt[common])/gt[common] for p in predictions]
        sparse_by_id[vid] = (x, y, gt, common, errors)
        for i in range(2):
            all_sparse[i].extend(errors[i].tolist())
        # An approximate object ROI from the observed sparse points, fixed for both modes.
        limits = np.quantile(np.column_stack([x, y]), [.01, .99], axis=0)
        retained = (x >= limits[0, 0])&(x <= limits[1, 0])&(y >= limits[0, 1])&(y <= limits[1, 1])
        points2d = np.column_stack([x[retained], y[retained]])
        hull = PlotPath(points2d[ConvexHull(points2d).vertices])
        yy, xx = np.indices(shape)
        roi = hull.contains_points(np.column_stack([xx.ravel(), yy.ravel()])).reshape(shape)
        valid = [off > 0, on > 0]
        total["pixels"] += off.size
        total["roi_pixels"] += int(roi.sum())
        for i in range(2):
            total["valid"][i] += int(valid[i].sum())
            total["roi_valid"][i] += int((valid[i]&roi).sum())
        c = valid[0]&valid[1]&roi
        costs = [read_array(root/name/f"{vid}.cost.bin") for name in names]
        for i in range(2):
            total["common_cost_sum"][i] += float(costs[i][c].sum(dtype=np.float64))
        total["common_cost_n"] += int(c.sum())
        # Fixed image grid, same cameras and thresholds; no depth is estimated here.
        gy, gx = np.mgrid[2:h-2:4, 2:w-2:4]
        gx, gy = gx.ravel(), gy.ravel()
        grid_roi = roi[gy, gx]
        gx, gy = gx[grid_roi].astype(float), gy[grid_roi].astype(float)
        total["grid_roi"] += gx.size
        zref = [d[gy.astype(int), gx.astype(int)] for d in [off, on]]
        reference_valid = [z > 0 for z in zref]
        world_ray = rays(view, shape, gx, gy)
        world = [np.asarray(view["position"])+world_ray*z[:, None] for z in zref]
        observations = [np.zeros(gx.size, dtype=int), np.zeros(gx.size, dtype=int)]
        agreements = [np.zeros(gx.size, dtype=int), np.zeros(gx.size, dtype=int)]
        paired_observations = np.zeros(gx.size, dtype=int)
        paired_agreements = [np.zeros(gx.size, dtype=int), np.zeros(gx.size, dtype=int)]
        for sid in view["sources"]:
            if sid not in by_id:
                continue
            source = by_id[sid]
            measured, good = [], []
            for i, name in enumerate(names):
                target = maps[name][sid]
                sx, sy, sz = project(source, target.shape, world[i])
                source_depth, supported = sample(target, sx, sy)
                measured.append(reference_valid[i]&supported&(sz > 0))
                # Do not undistort invalid/out-of-image projections: these observations are excluded.
                safe_x = np.where(measured[i], sx, target.shape[1]*.5)
                safe_y = np.where(measured[i], sy, target.shape[0]*.5)
                safe_depth = np.where(measured[i], source_depth, 1.)
                back = np.asarray(source["position"])+rays(source, target.shape, safe_x, safe_y)*safe_depth[:, None]
                bx, by, bz = project(view, shape, back)
                pixel_error = np.hypot(bx-gx, by-gy)
                depth_error = np.abs(bz-zref[i])/np.maximum(np.maximum(bz, zref[i]), 1e-9)
                good.append(measured[i]&(pixel_error <= 1)&(depth_error <= .01))
                observations[i] += measured[i]
                agreements[i] += good[i]
            paired = measured[0]&measured[1]
            paired_observations += paired
            for i in range(2):
                paired_agreements[i] += good[i]&paired
        paired_eligible = paired_observations >= 2
        total["paired_tested"] += int(paired_eligible.sum())
        row = {"image_id": vid, "sparse_common_n": int(common.sum()), "paired_geometry_n": int(paired_eligible.sum())}
        for i, name in enumerate(names):
            eligible = observations[i] >= 2
            verified = agreements[i] >= 2
            total["geo_tested"][i] += int(eligible.sum())
            total["geo_verified"][i] += int(verified.sum())
            total["paired_verified"][i] += int((paired_eligible&(paired_agreements[i] >= 2)).sum())
            row[name+"_coverage"] = float(valid[i].mean())
            row[name+"_sparse_median"] = float(np.median(errors[i])) if errors[i].size else None
            row[name+"_sparse_p90"] = float(np.quantile(errors[i], .9)) if errors[i].size else None
            row[name+"_paired_geometry_rate"] = float(np.mean(paired_agreements[i][paired_eligible] >= 2)) if paired_eligible.any() else None
        per_view.append(row)
        print(f'Analyzed {len(per_view)}/{len(views)} id={vid}', flush=True)
    summary = {"views": len(views), "sparse_reference_is_ground_truth": False,
               "geometric_rule": "At least two selected source views; round-trip <=1 matching-level pixel and relative depth <=1%.",
               "sampling_stride": 4, "roi": "Fixed per-view convex hull of central 98% of observed sparse points; approximate object ROI.",
               "totals": total}
    for i, name in enumerate(names):
        summary[name] = {"coverage_pct": 100*total["valid"][i]/total["pixels"],
                         "roi_coverage_pct": 100*total["roi_valid"][i]/total["roi_pixels"],
                         "sparse_common": stats(all_sparse[i]),
                         "crossview_paired_verified_pct": 100*total["paired_verified"][i]/max(total["paired_tested"], 1),
                         "crossview_verified_yield_pct": 100*total["geo_verified"][i]/total["grid_roi"],
                         "crossview_verified_among_measured_pct": 100*total["geo_verified"][i]/max(total["geo_tested"][i], 1),
                         "common_roi_mean_cost": total["common_cost_sum"][i]/max(total["common_cost_n"], 1)}
    summary["views_tilt_improved_sparse_median"] = sum(r["tilt_on_sparse_median"] < r["tilt_off_sparse_median"] for r in per_view)
    summary["views_tilt_improved_geometric_agreement"] = sum(r["tilt_on_paired_geometry_rate"] > r["tilt_off_paired_geometry_rate"] for r in per_view if r["tilt_on_paired_geometry_rate"] is not None)
    (root/"summary.json").write_text(json.dumps(summary, indent=2))
    with open(root/"per_view_analysis.csv", "w", newline="") as out:
        writer = csv.DictWriter(out, fieldnames=per_view[0].keys()); writer.writeheader(); writer.writerows(per_view)
    # Publish matched-pixel errors for further inspection, without rounding the measurements.
    np.savez_compressed(root/"common_sparse_errors.npz", tilt_off=np.asarray(all_sparse[0]), tilt_on=np.asarray(all_sparse[1]))
    fig, axes = plt.subplots(1, 4, figsize=(15, 4), layout="constrained")
    metrics = [("Object ROI coverage (%)", "roi_coverage_pct", 1),
               ("Sparse-reference median error (%) ↓", "sparse_common", 100),
               ("Paired cross-view agreement (%) ↑", "crossview_paired_verified_pct", 1),
               ("Verified ROI yield (%) ↑", "crossview_verified_yield_pct", 1)]
    for ax, (title, key, factor) in zip(axes, metrics):
        vals = [summary[n][key]["median"]*factor if key == "sparse_common" else summary[n][key]*factor for n in names]
        ax.bar(["Tilt OFF", "Tilt ON"], vals, color=["#667b96", "#179579"])
        ax.set_title(title, fontsize=10); ax.spines[["top", "right"]].set_visible(False)
        ax.set_ylim(0, max(vals)*1.25)
        for j, val in enumerate(vals): ax.text(j, val, f"{val:.3f}", ha="center", va="bottom")
    fig.suptitle(f"House dataset: {len(views)} views, GPU reconstruction, only tilt permission changed", fontsize=12)
    fig.savefig(root/"metrics.png", dpi=160); plt.close(fig)
    # Identical color/depth ranges per row, so density differences are not a display rescaling effect.
    selected = [views[0], views[len(views)//3], views[2*len(views)//3]]
    settings = dict(line.split("=", 1) for line in (root/"effective_settings.txt").read_text().splitlines() if "=" in line)
    fig, axes = plt.subplots(len(selected), 4, figsize=(15, 3.7*len(selected)), layout="constrained")
    for row, view in enumerate(selected):
        vid = view["id"]; a, b = [maps[name][vid] for name in names]
        image = Image.open(Path(settings["dataset"])/view["name"]).convert("RGB")
        axes[row, 0].imshow(image); axes[row, 0].set_title(f"Reference {vid}: {view['name']}")
        near, far = view["range"]
        cmap = plt.colormaps["viridis"].copy(); cmap.set_bad("#20252d")
        for j, data in enumerate([a, b]):
            axes[row, j+1].imshow(np.ma.masked_where(data <= 0, data), cmap=cmap, vmin=near, vmax=far)
            axes[row, j+1].set_title("Tilt OFF" if j == 0 else "Tilt ON")
        common = (a > 0)&(b > 0)
        diff = np.ma.masked_where(~common, 100*(b-a)/np.maximum(a, 1e-8))
        axes[row, 3].imshow(diff, cmap="coolwarm", vmin=-.2, vmax=.2)
        axes[row, 3].set_title("Common-pixel depth change (±0.2%)")
        for ax in axes[row]: ax.axis("off")
    fig.savefig(root/"depth_comparison.png", dpi=145); plt.close(fig)
    print(json.dumps(summary, indent=2), flush=True)


if __name__ == "__main__":
    main()

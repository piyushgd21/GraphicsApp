"""Compare saved GPU matching modes with paired sparse and cross-view checks."""
import argparse
import csv
import json
from itertools import combinations
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.path import Path as PlotPath
import numpy as np
from PIL import Image
from scipy.spatial import ConvexHull

from analyze_tilt_benchmark import read_array, rays, project, sample, stats


NAMES = ["planesweep", "slanted_front", "slanted"]
LABELS = ["PlaneSweep", "Weighted front-facing", "Slanted"]


def fraction(numerator, denominator):
    return 100.0 * numerator / denominator if denominator else None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("folder", type=Path)
    args = parser.parse_args()
    root = args.folder
    views = [json.loads(line) for line in (root / "views.jsonl").read_text().splitlines()]
    by_id = {view["id"]: view for view in views}
    settings = dict(line.split("=", 1) for line in (root / "effective_settings.txt").read_text().splitlines() if "=" in line)
    threshold = np.float32(settings["cost_threshold"])
    maps = {name: {v["id"]: read_array(root / name / f'{v["id"]}.depth.bin') for v in views} for name in NAMES}
    sparse = np.atleast_1d(np.genfromtxt(root / "sparse_samples.csv", delimiter=",", names=True))
    totals = {name: dict(valid=0, roi_valid=0, usable_cost=0, cost_passing=0, confidence_rejected=0,
                         tilted_over_5deg=0, geo_tested=0, geo_verified=0) for name in NAMES}
    pairs = {}
    for a, b in combinations(NAMES, 2):
        pairs[a, b] = dict(common=0, gained=0, lost=0, both_invalid=0, depth_change=[],
                           sparse_a=[], sparse_b=[], gained_sparse=[], lost_sparse=[],
                           sparse_improved=0, sparse_worse=0, sparse_tied=0,
                           paired_tested=0, paired_verified_a=0, paired_verified_b=0,
                           gained_grid=0, gained_geo_measured=0, gained_geo_verified=0,
                           gained_over_5deg=0, gained_from_no_plane=0, gained_from_cost=0, gained_from_confidence=0)
    all_sparse = {name: [] for name in NAMES}
    all_common_sparse = {name: [] for name in NAMES}
    rows = []
    pixels = roi_pixels = grid_pixels = 0
    for index, view in enumerate(views):
        vid = view["id"]
        depth = {name: maps[name][vid] for name in NAMES}
        shape = depth[NAMES[0]].shape
        h, w = shape
        cost = {name: read_array(root / name / f"{vid}.cost.bin") for name in NAMES}
        tilt = {name: read_array(root / name / f"{vid}.tilt.bin") for name in NAMES}
        valid = {name: np.isfinite(depth[name]) & (depth[name] > 0) for name in NAMES}
        usable = {name: np.isfinite(cost[name]) & (cost[name] < np.finfo(np.float32).max / 2) for name in NAMES}
        passing = {name: usable[name] & (cost[name] <= threshold) for name in NAMES}
        s = sparse[sparse["image_id"] == vid]
        x = (s["x"] + .5) * w / view["image_size"][0] - .5
        y = (s["y"] + .5) * h / view["image_size"][1] - .5
        inside = (x >= 0) & (y >= 0) & (x < w) & (y < h) & (s["depth"] > 0)
        x, y, reference = x[inside], y[inside], s["depth"][inside]
        if x.size < 3:
            raise ValueError(f"View {vid} has too few sparse observations for a fixed ROI")
        predictions = {name: sample(depth[name], x, y) for name in NAMES}
        errors = {name: np.abs(predictions[name][0] - reference) / reference for name in NAMES}
        common_sparse = np.logical_and.reduce([predictions[name][1] for name in NAMES])
        for name in NAMES:
            all_sparse[name].extend(errors[name][predictions[name][1]])
            all_common_sparse[name].extend(errors[name][common_sparse])
        limits = np.quantile(np.column_stack([x, y]), [.01, .99], axis=0)
        central = (x >= limits[0, 0]) & (x <= limits[1, 0]) & (y >= limits[0, 1]) & (y <= limits[1, 1])
        points = np.column_stack([x[central], y[central]])
        hull = PlotPath(points[ConvexHull(points).vertices])
        yy, xx = np.indices(shape)
        roi = hull.contains_points(np.column_stack([xx.ravel(), yy.ravel()])).reshape(shape)
        pixels += h * w
        roi_pixels += int(roi.sum())
        gy, gx = np.mgrid[2:h-2:4, 2:w-2:4]
        gx, gy = gx.ravel(), gy.ravel()
        in_roi = roi[gy, gx]
        gx, gy = gx[in_roi].astype(float), gy[in_roi].astype(float)
        grid_pixels += gx.size
        depths_grid = {name: depth[name][gy.astype(int), gx.astype(int)] for name in NAMES}
        ray = rays(view, shape, gx, gy)
        sources = [by_id[sid] for sid in view["sources"] if sid in by_id]
        measured, agreed = {}, {}
        row = {"image_id": vid}
        for name in NAMES:
            t = totals[name]
            t["valid"] += int(valid[name].sum())
            t["roi_valid"] += int((valid[name] & roi).sum())
            t["usable_cost"] += int(usable[name].sum())
            t["cost_passing"] += int(passing[name].sum())
            t["confidence_rejected"] += int((passing[name] & ~valid[name]).sum())
            t["tilted_over_5deg"] += int((valid[name] & (tilt[name] > 5)).sum())
            z = depths_grid[name]
            world = np.asarray(view["position"]) + ray * z[:, None]
            measured[name] = np.zeros((len(sources), gx.size), dtype=bool)
            agreed[name] = np.zeros_like(measured[name])
            for j, source in enumerate(sources):
                target = maps[name][source["id"]]
                sx, sy, sz = project(source, target.shape, world)
                source_depth, supported = sample(target, sx, sy)
                observable = (z > 0) & supported & (sz > 0)
                back = np.asarray(source["position"]) + rays(source, target.shape,
                    np.where(observable, sx, target.shape[1] * .5), np.where(observable, sy, target.shape[0] * .5)) * np.where(observable, source_depth, 1.)[:, None]
                bx, by, bz = project(view, shape, back)
                measured[name][j] = observable
                agreed[name][j] = observable & (np.hypot(bx-gx, by-gy) <= 1) & (np.abs(bz-z) / np.maximum(np.maximum(bz, z), 1e-9) <= .01)
            eligible = measured[name].sum(axis=0) >= 2
            verified = agreed[name].sum(axis=0) >= 2
            t["geo_tested"] += int(eligible.sum())
            t["geo_verified"] += int(verified.sum())
            row[name + "_valid"] = int(valid[name].sum())
            row[name + "_sparse_median"] = float(np.median(errors[name][common_sparse])) if common_sparse.any() else None
        for (a, b), p in pairs.items():
            common = valid[a] & valid[b]
            gained = valid[b] & ~valid[a]
            lost = valid[a] & ~valid[b]
            p["common"] += int(common.sum())
            p["gained"] += int(gained.sum())
            p["lost"] += int(lost.sum())
            p["both_invalid"] += int((~valid[a] & ~valid[b]).sum())
            p["depth_change"].extend((np.abs(depth[b][common] - depth[a][common]) / depth[a][common]).tolist())
            p["gained_over_5deg"] += int((gained & (tilt[b] > 5)).sum())
            p["gained_from_no_plane"] += int((gained & ~usable[a]).sum())
            p["gained_from_cost"] += int((gained & usable[a] & ~passing[a]).sum())
            p["gained_from_confidence"] += int((gained & passing[a]).sum())
            paired_sparse = predictions[a][1] & predictions[b][1]
            ea, eb = errors[a][paired_sparse], errors[b][paired_sparse]
            p["sparse_a"].extend(ea.tolist()); p["sparse_b"].extend(eb.tolist())
            p["gained_sparse"].extend(errors[b][predictions[b][1] & ~predictions[a][1]].tolist())
            p["lost_sparse"].extend(errors[a][predictions[a][1] & ~predictions[b][1]].tolist())
            p["sparse_improved"] += int((eb < ea).sum())
            p["sparse_worse"] += int((eb > ea).sum())
            p["sparse_tied"] += int((eb == ea).sum())
            paired_sources = measured[a] & measured[b]
            paired_eligible = paired_sources.sum(axis=0) >= 2
            p["paired_tested"] += int(paired_eligible.sum())
            for suffix, name in [("a", a), ("b", b)]:
                p["paired_verified_" + suffix] += int((paired_eligible & ((agreed[name] & paired_sources).sum(axis=0) >= 2)).sum())
            gained_grid = (depths_grid[b] > 0) & (depths_grid[a] <= 0)
            p["gained_grid"] += int(gained_grid.sum())
            p["gained_geo_measured"] += int((gained_grid & (measured[b].sum(axis=0) >= 2)).sum())
            p["gained_geo_verified"] += int((gained_grid & (agreed[b].sum(axis=0) >= 2)).sum())
        rows.append(row)
        print(f"Analyzed {index + 1}/{len(views)}", flush=True)
    summary = dict(views=len(views), ground_truth=False, pixels=pixels, roi_pixels=roi_pixels, grid_pixels=grid_pixels,
                   geometric_rule="Fixed stride-4 ROI grid; >=2 selected source views, round-trip <=1 matching pixel and relative depth <=1%.",
                   roi="Convex hull of central 98% of sparse projections; fixed across modes, not an object ground-truth mask.", variants={}, pairs={})
    for name, t in totals.items():
        summary["variants"][name] = {**t, "coverage_pct": fraction(t["valid"], pixels),
            "roi_coverage_pct": fraction(t["roi_valid"], roi_pixels), "verified_yield_pct": fraction(t["geo_verified"], grid_pixels),
            "agreement_among_measurable_pct": fraction(t["geo_verified"], t["geo_tested"]),
            "accepted_tilt_over_5deg_pct": fraction(t["tilted_over_5deg"], t["valid"]),
            "sparse_individual": stats(all_sparse[name]), "sparse_common_all_modes": stats(all_common_sparse[name])}
    for (a, b), p in pairs.items():
        result = {k: v for k, v in p.items() if k not in ["depth_change", "sparse_a", "sparse_b", "gained_sparse", "lost_sparse"]}
        result.update(depth_change=stats(p["depth_change"]), sparse_a=stats(p["sparse_a"]), sparse_b=stats(p["sparse_b"]),
            gained_sparse=stats(p["gained_sparse"]), lost_sparse=stats(p["lost_sparse"]),
            paired_agreement_a_pct=fraction(p["paired_verified_a"], p["paired_tested"]),
            paired_agreement_b_pct=fraction(p["paired_verified_b"], p["paired_tested"]),
            gained_verified_among_measurable_pct=fraction(p["gained_geo_verified"], p["gained_geo_measured"]))
        summary["pairs"][a + "_vs_" + b] = result
    (root / "mode_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    report = ["scan9 matching-mode comparison", "Run date: November 3, 2026", "",
        f"{len(views)} registered views; GPU-only reconstruction; identical input cameras, sources, depth ranges and numeric settings.",
        "Raw per-view matching acceptance, before cross-view filtering, fusion and outlier removal.",
        "Counts are depth samples across views, not unique fused 3D points.", "",
        "Variant | Accepted samples | Fixed ROI coverage | Common sparse median error | Verified ROI grid samples"]
    for name in NAMES:
        v = summary["variants"][name]
        report.append(f'{name} | {v["valid"]:,} | {v["roi_coverage_pct"]:.5f}% | '
            f'{100*v["sparse_common_all_modes"]["median"]:.5f}% | {v["geo_verified"]:,}')
    report += ["", f'Common sparse table uses {summary["variants"][NAMES[0]]["sparse_common_all_modes"]["n"]:,} identical observations.',
        "Full mode comparison includes changes in weighting, search/propagation and confidence evaluation.",
        "Only slanted_front versus slanted isolates permission to tilt the planes.", ""]
    for (a, b), p in pairs.items():
        r = summary["pairs"][a + "_vs_" + b]
        report += [f"{a} -> {b}",
            f'Newly accepted: {p["gained"]:,}; lost: {p["lost"]:,}; net: {p["gained"]-p["lost"]:,}.',
            f'Previously no usable plane/support: {p["gained_from_no_plane"]:,}; cost-rejected: {p["gained_from_cost"]:,}; confidence-rejected: {p["gained_from_confidence"]:,}.',
            f'Median relative depth change at common accepted pixels: {100*r["depth_change"]["median"]:.5f}%.',
            f'Paired cross-view agreement: {r["paired_agreement_a_pct"]:.5f}% -> {r["paired_agreement_b_pct"]:.5f}%.',
            f'Paired sparse-reference median error: {100*r["sparse_a"]["median"]:.5f}% -> {100*r["sparse_b"]["median"]:.5f}%.',
            f'Paired sparse-reference p90 error: {100*r["sparse_a"]["p90"]:.5f}% -> {100*r["sparse_b"]["p90"]:.5f}%.', ""]
    report += ["Interpretation: substantially greater retained coverage and verified yield, with modest median sparse-reference gains.",
        "Common-point cross-view agreement and tail errors do not show a uniform accuracy improvement.",
        "The tilt-only comparison adds coverage with essentially unchanged paired agreement.", "",
        "Limits: no dense ground truth. COLMAP points are a texture-biased proxy also used for source selection and depth ranges.",
        "The fixed ROI is an approximate sparse-projection hull, not a true foreground mask.",
        "Geometric checks require two source maps with bilinear depth support; points without that support remain unverified.",
        "PlaneSweep and slanted use different matching and confidence formulations; matching costs are not comparable accuracy scores.",
        "This run compares current modes; it does not isolate every-level refinement versus the former finest-only refinement.",
        "Timing is not treated as a controlled speed benchmark; synthetic tests ran during the initial real-data views.", "",
        "Reproduce: PlaneSweepTiltBenchmark setup.ini NEW_OUTPUT_FOLDER 48 modes",
        "Analyze: python Tests/analyze_matching_modes.py NEW_OUTPUT_FOLDER", ""]
    (root / "report.txt").write_text("\n".join(report), encoding="utf-8")
    with open(root / "mode_per_view.csv", "w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys()); writer.writeheader(); writer.writerows(rows)
    fig, axes = plt.subplots(1, 3, figsize=(13, 4), layout="constrained")
    for ax, title, values in [
        (axes[0], "Accepted samples in fixed ROI (%)", [summary["variants"][n]["roi_coverage_pct"] for n in NAMES]),
        (axes[1], "Common sparse median relative error (%)", [100*summary["variants"][n]["sparse_common_all_modes"]["median"] for n in NAMES]),
        (axes[2], "Cross-view verified ROI yield (%)", [summary["variants"][n]["verified_yield_pct"] for n in NAMES])]:
        ax.bar(LABELS, values, color=["#667b96", "#c79432", "#179579"])
        ax.set_title(title, fontsize=10); ax.tick_params(axis="x", labelrotation=15)
        ax.spines[["top", "right"]].set_visible(False)
        ax.set_ylim(0, max(values)*1.2)
        for j, value in enumerate(values): ax.text(j, value, f"{value:.4f}", ha="center", va="bottom")
    fig.suptitle(f"scan9: {len(views)} views, current GPU matching modes; geometric proxies, no dense ground truth")
    fig.savefig(root / "mode_metrics.png", dpi=160); plt.close(fig)
    selected = [views[0], views[len(views)//3], views[2*len(views)//3]]
    fig, axes = plt.subplots(3, 5, figsize=(17, 10), layout="constrained")
    for row, view in enumerate(selected):
        vid = view["id"]
        axes[row, 0].imshow(Image.open(Path(settings["dataset"]) / view["name"]))
        axes[row, 0].set_title(f"Reference {vid}")
        cmap = plt.colormaps["viridis"].copy(); cmap.set_bad("#20252d")
        for j, name in enumerate(NAMES):
            data = maps[name][vid]
            axes[row, j+1].imshow(np.ma.masked_where(data <= 0, data), cmap=cmap, vmin=view["range"][0], vmax=view["range"][1])
            axes[row, j+1].set_title(LABELS[j])
        a, b = maps["planesweep"][vid] > 0, maps["slanted"][vid] > 0
        acceptance = np.zeros((*a.shape, 3), dtype=np.uint8) + 24
        acceptance[a & b] = [150, 150, 150]
        acceptance[b & ~a] = [30, 190, 130]
        acceptance[a & ~b] = [225, 90, 65]
        axes[row, 4].imshow(acceptance)
        axes[row, 4].set_title("Slanted gains: green; losses: red")
        for ax in axes[row]: ax.axis("off")
    fig.savefig(root / "mode_depth_comparison.png", dpi=145); plt.close(fig)
    print(json.dumps(summary, indent=2), flush=True)


if __name__ == "__main__":
    main()

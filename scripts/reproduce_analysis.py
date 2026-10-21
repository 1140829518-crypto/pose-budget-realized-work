#!/usr/bin/env python3
"""Recompute Paper2 summary tables and public figures from frozen CSV files.

This script does not run SLAM and does not require RGB-D image data.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from collections import Counter, defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


SCENES = ["sitting_static", "walking_xyz", "sitting_rpy", "walking_halfsphere"]
BUDGETS = [1, 3, 4, 5, 10]
COLORS = ["#0072B2", "#D55E00", "#009E73", "#CC79A7"]


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def mean(values: list[float]) -> float:
    return statistics.fmean(values)


def sd(values: list[float]) -> float:
    return statistics.stdev(values) if len(values) > 1 else math.nan


def pearson(x: list[float], y: list[float]) -> float:
    if len(x) != len(y) or len(x) < 2:
        return math.nan
    xa = np.asarray(x, dtype=float)
    ya = np.asarray(y, dtype=float)
    if np.std(xa) == 0 or np.std(ya) == 0:
        return math.nan
    return float(np.corrcoef(xa, ya)[0, 1])


def write_csv(path: Path, fieldnames: list[str], rows: list[dict[str, object]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def equal_float(a: object, b: object, tolerance: float = 1e-10) -> bool:
    def parse(value: object) -> float:
        text = str(value).strip().lower()
        return math.nan if text in {"", "nan"} else float(value)

    x, y = parse(a), parse(b)
    if math.isnan(x) and math.isnan(y):
        return True
    return math.isclose(x, y, rel_tol=tolerance, abs_tol=tolerance)


def style_axes(ax) -> None:
    ax.set_facecolor("white")
    ax.grid(False)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)


def save_figure(fig, path: Path) -> None:
    fig.patch.set_facecolor("white")
    fig.savefig(path, dpi=600, bbox_inches="tight", facecolor="white")
    plt.close(fig)


def main() -> None:
    package_root = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", type=Path, default=package_root / "data")
    parser.add_argument("--output", type=Path, default=package_root / "generated")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)

    formal = args.data_root / "formal"
    diagnostic = args.data_root / "diagnostic"
    runs = read_csv(formal / "FINAL_RESULTS.csv")
    invocations = read_csv(formal / "INVOCATION_RESULTS.csv")

    assert len(runs) == 60, f"expected 60 formal runs, found {len(runs)}"
    assert len(invocations) == 9480, f"expected 9480 invocations, found {len(invocations)}"
    assert sorted({r["sequence"] for r in runs}) == sorted(SCENES)
    assert sorted({int(r["budget"]) for r in runs}) == BUDGETS
    assert all(int(r["tracking_gaps"]) == 0 for r in runs)

    grouped: dict[tuple[str, int], list[dict[str, str]]] = defaultdict(list)
    for row in runs:
        grouped[(row["sequence"], int(row["budget"]))].append(row)

    summary_rows: list[dict[str, object]] = []
    for scene in SCENES:
        for budget in BUDGETS:
            group = grouped[(scene, budget)]
            realized = [float(r["realized_total"]) for r in group]
            ratio = [float(r["realized_requested_ratio"]) for r in group]
            cap = [float(r["cap_hit_rate"]) for r in group]
            pose = [float(r["poseopt_mean_ms"]) for r in group]
            ate = [float(r["ate_rmse_m"]) for r in group]
            summary_rows.append(
                {
                    "sequence": scene,
                    "budget": budget,
                    "n_runs": len(group),
                    "realized_mean": mean(realized),
                    "realized_sd": sd(realized),
                    "realization_ratio_mean": mean(ratio),
                    "cap_hit_rate_mean": mean(cap),
                    "poseopt_mean_ms": mean(pose),
                    "poseopt_sd_ms": sd(pose),
                    "ate_mean_m": mean(ate),
                    "ate_sd_m": sd(ate),
                }
            )

    summary_fields = list(summary_rows[0].keys())
    write_csv(args.output / "scene_budget_recomputed.csv", summary_fields, summary_rows)

    # Cross-check recomputation against the frozen published aggregation.
    frozen_summary = {
        (r["sequence"], int(r["budget"])): r
        for r in read_csv(formal / "SCENE_BUDGET_SUMMARY.csv")
    }
    comparisons = {
        "n_runs": "n",
        "realized_mean": "realized_mean",
        "realized_sd": "realized_sd",
        "cap_hit_rate_mean": "cap_hit_rate",
        "poseopt_mean_ms": "poseopt_mean_ms",
        "poseopt_sd_ms": "poseopt_sd_ms",
        "ate_mean_m": "ate_mean",
        "ate_sd_m": "ate_sd",
    }
    for row in summary_rows:
        frozen = frozen_summary[(str(row["sequence"]), int(row["budget"]))]
        for rebuilt_field, frozen_field in comparisons.items():
            assert equal_float(row[rebuilt_field], frozen[frozen_field]), (
                row["sequence"], row["budget"], rebuilt_field,
                row[rebuilt_field], frozen[frozen_field]
            )

    by_summary = {(r["sequence"], int(r["budget"])): r for r in summary_rows}
    response_rows: list[dict[str, object]] = []
    for scene in SCENES:
        b5 = float(by_summary[(scene, 5)]["realized_mean"])
        b10 = float(by_summary[(scene, 10)]["realized_mean"])
        response_rows.append(
            {
                "sequence": scene,
                "b5_realized_mean": b5,
                "b10_realized_mean": b10,
                "absolute_change": b10 - b5,
                "relative_change_percent": 100.0 * (b10 - b5) / b5,
                "elasticity": ((b10 - b5) / b5) / ((10 - 5) / 5),
            }
        )
    write_csv(args.output / "b5_b10_response.csv", list(response_rows[0].keys()), response_rows)

    corr_rows: list[dict[str, object]] = []
    inv_by_scene: dict[str, list[dict[str, str]]] = defaultdict(list)
    for row in invocations:
        inv_by_scene[row["sequence"]].append(row)
    for scene in SCENES:
        group = inv_by_scene[scene]
        requested = [float(r["requested_per_round"]) for r in group]
        realized = [float(r["realized_iterations"]) for r in group]
        core = [float(r["optimization_core_ms"]) for r in group]
        corr_rows.append(
            {
                "sequence": scene,
                "n_invocations": len(group),
                "pearson_requested_vs_core": pearson(requested, core),
                "pearson_realized_vs_core": pearson(realized, core),
            }
        )
    write_csv(args.output / "runtime_correlations.csv", list(corr_rows[0].keys()), corr_rows)

    frozen_generalization = {
        r["sequence"]: r for r in read_csv(formal / "SCENE_GENERALIZATION.csv")
    }
    for row in corr_rows:
        frozen = frozen_generalization[str(row["sequence"])]
        assert equal_float(row["pearson_requested_vs_core"], frozen["corr_requested_runtime"])
        assert equal_float(row["pearson_realized_vs_core"], frozen["corr_realized_runtime"])

    # Figure 1: realization, cap hits, and marginal realized-work response.
    fig, axes = plt.subplots(1, 3, figsize=(10.8, 3.25))
    for idx, scene in enumerate(SCENES):
        vals = [by_summary[(scene, b)] for b in BUDGETS]
        axes[0].plot(BUDGETS, [v["realization_ratio_mean"] for v in vals], "o-", color=COLORS[idx], label=scene)
        axes[1].plot(BUDGETS, [v["cap_hit_rate_mean"] for v in vals], "o-", color=COLORS[idx])
        changes = []
        for b0, b1 in zip(BUDGETS[:-1], BUDGETS[1:]):
            w0 = float(by_summary[(scene, b0)]["realized_mean"])
            w1 = float(by_summary[(scene, b1)]["realized_mean"])
            changes.append(100.0 * (w1 - w0) / w0)
        axes[2].plot(range(4), changes, "o-", color=COLORS[idx])
    axes[0].set(xlabel="Requested budget", ylabel="Budget realization ratio", xticks=BUDGETS)
    axes[1].set(xlabel="Requested budget", ylabel="Cap-hit rate", xticks=BUDGETS)
    axes[2].set(xlabel="Budget transition", ylabel="Realized-work change (%)", xticks=range(4))
    axes[2].set_xticklabels(["1→3", "3→4", "4→5", "5→10"])
    for ax in axes:
        style_axes(ax)
    axes[0].legend(frameon=False, fontsize=7)
    fig.tight_layout()
    save_figure(fig, args.output / "work_response.png")

    # Figure 2: routine runtime versus requested budget.
    fig, ax = plt.subplots(figsize=(5.5, 3.7))
    for idx, scene in enumerate(SCENES):
        vals = [by_summary[(scene, b)] for b in BUDGETS]
        ax.plot(BUDGETS, [v["poseopt_mean_ms"] for v in vals], "o-", color=COLORS[idx], label=scene)
    ax.set(xlabel="Requested budget per round", ylabel="PoseOptimization mean time (ms)", xticks=BUDGETS)
    style_axes(ax)
    ax.legend(frameon=False, fontsize=7)
    fig.tight_layout()
    save_figure(fig, args.output / "budget_vs_poseopt_runtime.png")

    # Figure 3: invocation realized work versus optimization-core time.
    fig, ax = plt.subplots(figsize=(5.5, 3.7))
    for idx, scene in enumerate(SCENES):
        group = inv_by_scene[scene]
        ax.scatter(
            [float(r["realized_iterations"]) for r in group],
            [float(r["optimization_core_ms"]) for r in group],
            s=6,
            alpha=0.25,
            color=COLORS[idx],
            label=scene,
        )
    ax.set(xlabel="Realized LM iterations per invocation", ylabel="Optimization-core time (ms)")
    style_axes(ax)
    ax.legend(frameon=False, fontsize=7)
    fig.tight_layout()
    save_figure(fig, args.output / "realized_work_vs_runtime.png")

    # Figure 4: trajectory accuracy. No monotonic-improvement claim is implied.
    fig, axes = plt.subplots(1, 2, figsize=(8.0, 3.2))
    for idx, scene in enumerate(SCENES):
        vals = grouped
        axes[0].plot(BUDGETS, [mean([float(r["ate_rmse_m"]) for r in vals[(scene, b)]]) for b in BUDGETS], "o-", color=COLORS[idx], label=scene)
        axes[1].plot(BUDGETS, [mean([float(r["rpe_translation_rmse_m"]) for r in vals[(scene, b)]]) for b in BUDGETS], "o-", color=COLORS[idx])
    axes[0].set(xlabel="Requested budget", ylabel="ATE RMSE (m)", xticks=BUDGETS)
    axes[1].set(xlabel="Requested budget", ylabel="RPE translation RMSE (m)", xticks=BUDGETS)
    for ax in axes:
        style_axes(ax)
    axes[0].legend(frameon=False, fontsize=7)
    fig.tight_layout()
    save_figure(fig, args.output / "accuracy_vs_budget.png")

    # Figure 5: separate diagnostic batch; never pooled with the 60 formal runs.
    stops = read_csv(diagnostic / "STOP_REASONS.csv")
    contributions = read_csv(diagnostic / "ITERATION_CONTRIBUTIONS.csv")
    stop_counts: dict[str, Counter[str]] = defaultdict(Counter)
    for row in stops:
        stop_counts[row["sequence"]][row["stop_reason"]] += 1
    reasons = sorted({r["stop_reason"] for r in stops})
    fig, axes = plt.subplots(1, 2, figsize=(9.0, 3.4))
    x = np.arange(len(reasons))
    width = 0.35
    for idx, scene in enumerate(sorted(stop_counts)):
        axes[0].bar(x + (idx - 0.5) * width, [stop_counts[scene][r] for r in reasons], width, label=scene, color=COLORS[idx])
    axes[0].set_xticks(x, [r.replace("LM_", "") for r in reasons], rotation=20, ha="right")
    axes[0].set_ylabel("Diagnostic round count")
    axes[0].legend(frameon=False, fontsize=7)
    for idx, scene in enumerate(sorted({r["sequence"] for r in contributions})):
        group = sorted((r for r in contributions if r["sequence"] == scene), key=lambda r: int(r["iteration"]))
        axes[1].semilogy([int(r["iteration"]) for r in group], [float(r["pose_update_norm_median"]) for r in group], "o-", color=COLORS[idx], label=scene)
    axes[1].set(xlabel="LM iteration", ylabel="Median pose-update norm")
    axes[1].legend(frameon=False, fontsize=7)
    for ax in axes:
        style_axes(ax)
    fig.tight_layout()
    save_figure(fig, args.output / "diagnostic_stop_and_decay.png")

    report = {
        "formal_runs": len(runs),
        "invocations": len(invocations),
        "scenes": SCENES,
        "budgets": BUDGETS,
        "tracking_gaps_total": sum(int(r["tracking_gaps"]) for r in runs),
        "generated_files": sorted(p.name for p in args.output.iterdir() if p.is_file()),
        "scope": "offline reconstruction only; no SLAM execution",
    }
    (args.output / "reproduction_summary.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()

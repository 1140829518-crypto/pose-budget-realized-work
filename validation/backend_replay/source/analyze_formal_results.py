#!/usr/bin/env python3
"""Aggregate and analyze the frozen g2o--Ceres formal replay batch."""

from __future__ import annotations

import csv
import hashlib
import json
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "artifacts/CERES_VALIDATION_NEW_20261111"
RECORDS = OUT / "formal_records"
BUDGETS = [1, 3, 4, 5, 10]
BACKENDS = ["g2o", "ceres"]
SEED = 20261111


def digest(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def bootstrap_mean_ci(values: np.ndarray, draws: int = 10000) -> tuple[float, float]:
    values = np.asarray(values, dtype=float)
    rng = np.random.default_rng(SEED + len(values) + int(abs(values.sum()) * 1000) % 100003)
    samples = rng.choice(values, size=(draws, len(values)), replace=True).mean(axis=1)
    return tuple(np.quantile(samples, [0.025, 0.975]))


def paired_dz(values: np.ndarray) -> float:
    values = np.asarray(values, dtype=float)
    sd = values.std(ddof=1)
    return float(values.mean() / sd) if sd > 0 else math.nan


def elasticity(w0: float, w1: float, k0: int, k1: int) -> float:
    return ((w1 - w0) / w0) / ((k1 - k0) / k0)


def quat_angle_deg(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    left = left / np.linalg.norm(left, axis=1, keepdims=True)
    right = right / np.linalg.norm(right, axis=1, keepdims=True)
    dot = np.clip(np.abs(np.sum(left * right, axis=1)), 0.0, 1.0)
    return np.degrees(2.0 * np.arccos(dot))


def main() -> int:
    order = pd.read_csv(OUT / "FINAL_RUN_ORDER.csv")
    execution = pd.read_csv(OUT / "FORMAL_EXECUTION_LOG.csv")
    run_rows: list[dict[str, object]] = []
    call_frames = []
    round_frames = []
    failure_frames = []

    for item in order.to_dict("records"):
        record_id = item["record_id"]
        directory = RECORDS / record_id
        aggregate = json.loads((directory / "run_aggregate.json").read_text(encoding="utf-8"))
        aggregate.update({
            "order_id": item["order_id"],
            "independent_run_id": item["independent_run_id"],
            "scene_id": item["scene_id"], "dataset": item["dataset"],
            "expected_calls": item["expected_calls"],
            "expected_rounds": item["expected_rounds"],
        })
        run_rows.append(aggregate)
        calls = pd.read_csv(directory / "call_results.csv")
        calls.insert(0, "record_id", record_id)
        calls.insert(1, "independent_run_id", item["independent_run_id"])
        calls.insert(2, "scene_id", item["scene_id"])
        call_frames.append(calls)
        rounds = pd.read_csv(directory / "round_results.csv")
        rounds.insert(0, "record_id", record_id)
        rounds.insert(1, "independent_run_id", item["independent_run_id"])
        rounds.insert(2, "backend", item["backend"])
        rounds.insert(3, "budget", item["requested_budget"])
        round_frames.append(rounds)
        failures = pd.read_csv(directory / "errors.csv")
        if not failures.empty:
            failures.insert(0, "record_id", record_id)
            failure_frames.append(failures)

    runs = pd.DataFrame(run_rows)
    runs["independent_run_id"] = runs["independent_run_id"].astype(str)
    calls = pd.concat(call_frames, ignore_index=True)
    rounds = pd.concat(round_frames, ignore_index=True)
    failures = (pd.concat(failure_frames, ignore_index=True) if failure_frames else
                pd.DataFrame(columns=["record_id", "call_id", "stage", "returncode", "message"]))

    expected_cells = len(order) == 50 and order[["independent_run_id", "backend", "requested_budget"]].drop_duplicates().shape[0] == 50
    run_count_ok = len(runs) == 50 and runs["record_id"].nunique() == 50
    valid_ok = bool(runs["aggregate_valid"].all())
    calls_ok = bool((runs["call_total"] == runs["expected_calls"]).all() and
                    (runs["call_success"] == runs["expected_calls"]).all() and
                    (runs["call_failure"] == 0).all())
    rounds_ok = len(rounds) == int(runs["expected_rounds"].sum())
    counter_ok = bool((rounds["attempted_steps"] ==
                       rounds["accepted_steps"] + rounds["rejected_steps"]).all())
    duplicates_ok = (not calls.duplicated(["record_id", "fixture_id"]).any() and
                     not rounds.duplicated(["record_id", "call_id", "round_id"]).any())
    numeric_call = [
        "logical_outer_work", "attempted_steps", "accepted_steps", "rejected_steps",
        "residual_evaluations", "jacobian_evaluations", "linear_solver_iterations",
        "initial_cost", "final_cost", "solver_core_time_ms", "tx", "ty", "tz",
        "qx", "qy", "qz", "qw",
    ]
    finite_ok = bool(np.isfinite(calls[numeric_call].to_numpy(dtype=float)).all())
    backup_ok = bool((execution["record_status"] == "SUCCESS").all() and
                     (execution["linux_archive_sha256"] == execution["windows_archive_sha256"]).all())
    quality = {
        "formal_records": len(runs), "successful_records": int(runs["aggregate_valid"].sum()),
        "failed_records": int((~runs["aggregate_valid"]).sum()),
        "call_rows": len(calls), "round_rows": len(rounds), "failure_rows": len(failures),
        "frozen_design_complete": expected_cells, "run_count_unique": run_count_ok,
        "all_aggregates_valid": valid_ok, "call_counts_complete": calls_ok,
        "round_counts_complete": rounds_ok, "attempted_equals_accepted_plus_rejected": counter_ok,
        "no_duplicate_call_or_round_keys": duplicates_ok, "all_numeric_outputs_finite": finite_ok,
        "all_linux_windows_backups_match": backup_ok,
    }
    if not all(value is True for key, value in quality.items() if isinstance(value, bool)):
        raise RuntimeError("formal integrity gate failed: " + json.dumps(quality, sort_keys=True))

    runs = runs.sort_values(["backend", "independent_run_id", "budget"]).reset_index(drop=True)
    runs.to_csv(OUT / "FORMAL_RUN_RESULTS.csv", index=False)
    calls.to_csv(OUT / "FORMAL_CALL_RESULTS.csv", index=False)
    rounds.to_csv(OUT / "FORMAL_ROUND_RESULTS.csv", index=False)
    failures.to_csv(OUT / "FORMAL_FAILURES.csv", index=False)

    elastic_rows = []
    for (backend, run_id), group in runs.groupby(["backend", "independent_run_id"]):
        by_budget = group.set_index("budget")
        for k0, k1 in zip(BUDGETS[:-1], BUDGETS[1:]):
            w0 = float(by_budget.loc[k0, "logical_outer_work"])
            w1 = float(by_budget.loc[k1, "logical_outer_work"])
            elastic_rows.append({
                "backend": backend, "independent_run_id": run_id,
                "scene_id": by_budget.loc[k0, "scene_id"], "k0": k0, "k1": k1,
                "W_k0": w0, "W_k1": w1, "absolute_change": w1 - w0,
                "E_W": elasticity(w0, w1, k0, k1),
            })
    elastic = pd.DataFrame(elastic_rows)
    elastic.to_csv(OUT / "FORMAL_ELASTICITY.csv", index=False)

    # Pair exactly matched backend calls before any call-level descriptive summaries.
    key = ["independent_run_id", "budget", "frame_id", "caller_id"]
    pose_cols = key + ["initial_cost", "final_cost", "solver_core_time_ms",
                       "logical_outer_work", "tx", "ty", "tz", "qx", "qy", "qz", "qw"]
    g = calls[calls["backend"] == "g2o"][pose_cols].copy()
    c = calls[calls["backend"] == "ceres"][pose_cols].copy()
    pairs = g.merge(c, on=key, suffixes=("_g2o", "_ceres"), validate="one_to_one")
    if len(pairs) != sum(int(value) for value in runs[runs["backend"] == "g2o"]["call_total"]):
        raise RuntimeError("backend call pairing is incomplete")
    pairs["initial_cost_abs_difference"] = np.abs(pairs["initial_cost_g2o"] - pairs["initial_cost_ceres"])
    pairs["final_cost_abs_difference"] = np.abs(pairs["final_cost_g2o"] - pairs["final_cost_ceres"])
    denominator = np.maximum(np.abs(pairs["final_cost_g2o"]), 1e-15)
    pairs["final_cost_relative_difference"] = pairs["final_cost_abs_difference"] / denominator
    pairs["translation_difference_m"] = np.linalg.norm(
        pairs[["tx_g2o", "ty_g2o", "tz_g2o"]].to_numpy() -
        pairs[["tx_ceres", "ty_ceres", "tz_ceres"]].to_numpy(), axis=1)
    pairs["rotation_difference_deg"] = quat_angle_deg(
        pairs[["qx_g2o", "qy_g2o", "qz_g2o", "qw_g2o"]].to_numpy(),
        pairs[["qx_ceres", "qy_ceres", "qz_ceres", "qw_ceres"]].to_numpy())
    pairs.to_csv(OUT / "FORMAL_PAIRWISE_CALL_DIFFERENCES.csv", index=False)

    pose_run = pairs.groupby(["independent_run_id", "budget"], as_index=False).agg(
        paired_calls=("caller_id", "size"),
        initial_cost_abs_difference_max=("initial_cost_abs_difference", "max"),
        final_cost_relative_difference_mean=("final_cost_relative_difference", "mean"),
        final_cost_relative_difference_max=("final_cost_relative_difference", "max"),
        translation_difference_m_mean=("translation_difference_m", "mean"),
        translation_difference_m_max=("translation_difference_m", "max"),
        rotation_difference_deg_mean=("rotation_difference_deg", "mean"),
        rotation_difference_deg_max=("rotation_difference_deg", "max"),
    )
    pose_run.to_csv(OUT / "FORMAL_POSE_COST_PAIRING_SUMMARY.csv", index=False)

    metric_specs = [
        ("BRR", "BRR"), ("cap_hit_rate", "cap_hit_rate"),
        ("logical_outer_work", "logical_outer_work"),
        ("solver_core_time_ms", "solver_core_time_ms"),
        ("residual_evaluations", "residual_evaluations"),
        ("jacobian_evaluations", "jacobian_evaluations"),
        ("linear_solver_iterations", "linear_solver_iterations"),
    ]
    summary_rows = []
    paired_rows = []
    for budget in BUDGETS:
        for backend in BACKENDS:
            subset = runs[(runs["backend"] == backend) & (runs["budget"] == budget)]
            for metric, label in metric_specs:
                values = subset[metric].to_numpy(dtype=float)
                lo, hi = bootstrap_mean_ci(values)
                summary_rows.append({
                    "backend": backend, "budget": budget, "metric": label,
                    "n_independent_runs": len(values), "mean": values.mean(),
                    "sd": values.std(ddof=1), "ci95_low": lo, "ci95_high": hi,
                })
        left = runs[(runs["backend"] == "g2o") & (runs["budget"] == budget)].set_index("independent_run_id")
        right = runs[(runs["backend"] == "ceres") & (runs["budget"] == budget)].set_index("independent_run_id")
        if list(left.index) != list(right.index):
            right = right.reindex(left.index)
        for metric, label in metric_specs:
            diff = left[metric].to_numpy(dtype=float) - right[metric].to_numpy(dtype=float)
            lo, hi = bootstrap_mean_ci(diff)
            paired_rows.append({
                "budget": budget, "metric": label, "direction": "g2o_minus_ceres",
                "n_paired_runs": len(diff), "mean_difference": diff.mean(),
                "sd_difference": diff.std(ddof=1), "paired_dz": paired_dz(diff),
                "bootstrap_ci95_low": lo, "bootstrap_ci95_high": hi,
            })
    aggregate = pd.DataFrame(summary_rows)
    aggregate.to_csv(OUT / "FORMAL_RUN_AGGREGATION.csv", index=False)
    paired = pd.DataFrame(paired_rows)
    paired.to_csv(OUT / "FORMAL_PAIRED_BACKEND_DIFFERENCES.csv", index=False)

    elastic_summary_rows = []
    for (backend, k0, k1), group in elastic.groupby(["backend", "k0", "k1"]):
        values = group["E_W"].to_numpy(dtype=float)
        lo, hi = bootstrap_mean_ci(values)
        elastic_summary_rows.append({
            "backend": backend, "k0": k0, "k1": k1,
            "n_independent_runs": len(values), "mean_E_W": values.mean(),
            "sd_E_W": values.std(ddof=1), "ci95_low": lo, "ci95_high": hi,
        })
    elastic_summary = pd.DataFrame(elastic_summary_rows)
    elastic_summary.to_csv(OUT / "FORMAL_ELASTICITY_AGGREGATION.csv", index=False)

    figures = OUT / "figures"
    figures.mkdir(exist_ok=True)
    colors = {"g2o": "#1f77b4", "ceres": "#d62728"}
    labels = {"g2o": "g2o", "ceres": "Ceres 2.2.0"}
    for metric, ylabel, filename in [
        ("BRR", "Budget realization ratio (BRR)", "FIGURE_BRR_RESPONSE"),
        ("cap_hit_rate", "Cap-hit rate", "FIGURE_CAP_HIT_RESPONSE"),
    ]:
        fig, ax = plt.subplots(figsize=(6.6, 4.2))
        for backend in BACKENDS:
            view = aggregate[(aggregate["backend"] == backend) & (aggregate["metric"] == metric)].sort_values("budget")
            y = view["mean"].to_numpy(); lo = view["ci95_low"].to_numpy(); hi = view["ci95_high"].to_numpy()
            ax.errorbar(BUDGETS, y, yerr=[y - lo, hi - y], marker="o", capsize=3,
                        color=colors[backend], label=labels[backend])
        ax.set_xlabel("Requested outer-iteration budget k")
        ax.set_ylabel(ylabel)
        ax.set_xticks(BUDGETS); ax.grid(alpha=0.25); ax.legend(); fig.tight_layout()
        fig.savefig(figures / f"{filename}.png", dpi=300)
        fig.savefig(figures / f"{filename}.pdf")
        plt.close(fig)

    fig, ax = plt.subplots(figsize=(7.0, 4.2))
    transitions = ["1–3", "3–4", "4–5", "5–10"]
    x = np.arange(len(transitions)); width = 0.34
    for index, backend in enumerate(BACKENDS):
        view = elastic_summary[elastic_summary["backend"] == backend].sort_values(["k0", "k1"])
        y = view["mean_E_W"].to_numpy(); lo = view["ci95_low"].to_numpy(); hi = view["ci95_high"].to_numpy()
        ax.bar(x + (index - 0.5) * width, y, width, label=labels[backend], color=colors[backend], alpha=0.85)
        ax.errorbar(x + (index - 0.5) * width, y, yerr=[y - lo, hi - y], fmt="none", color="black", capsize=3)
    ax.axhline(0, color="black", linewidth=0.8); ax.set_xticks(x, transitions)
    ax.set_xlabel("Adjacent budget interval"); ax.set_ylabel("Marginal realized-work elasticity $E_W$")
    ax.grid(axis="y", alpha=0.25); ax.legend(); fig.tight_layout()
    fig.savefig(figures / "FIGURE_ELASTICITY_RESPONSE.png", dpi=300)
    fig.savefig(figures / "FIGURE_ELASTICITY_RESPONSE.pdf")
    plt.close(fig)

    fig, axes = plt.subplots(1, 2, figsize=(10.5, 4.0))
    for backend in BACKENDS:
        work = aggregate[(aggregate["backend"] == backend) & (aggregate["metric"] == "logical_outer_work")].sort_values("budget")
        timing = aggregate[(aggregate["backend"] == backend) & (aggregate["metric"] == "solver_core_time_ms")].sort_values("budget")
        axes[0].plot(BUDGETS, work["mean"], marker="o", color=colors[backend], label=labels[backend])
        axes[1].plot(BUDGETS, timing["mean"], marker="o", color=colors[backend], label=labels[backend])
    axes[0].set_ylabel("Run-level realized outer iterations")
    axes[1].set_ylabel("Run-level solver-core time (ms)")
    for ax in axes:
        ax.set_xlabel("Requested budget k"); ax.set_xticks(BUDGETS); ax.grid(alpha=0.25); ax.legend()
    fig.tight_layout(); fig.savefig(figures / "FIGURE_WORK_TIME_RESPONSE.png", dpi=300)
    fig.savefig(figures / "FIGURE_WORK_TIME_RESPONSE.pdf"); plt.close(fig)

    quality["initial_cost_abs_difference_max"] = float(pairs["initial_cost_abs_difference"].max())
    quality["paired_call_rows"] = len(pairs)
    quality["main_tracked_diff_sha256"] = execution["main_tracked_diff_sha256"].iloc[0]
    (OUT / "FORMAL_INTEGRITY_VALIDATION.json").write_text(
        json.dumps(quality, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )

    def metric_table(metric: str) -> str:
        view = aggregate[aggregate["metric"] == metric]
        rows = ["|后端|预算|均值|95% Bootstrap CI|", "|---|---:|---:|---:|"]
        for backend in BACKENDS:
            for budget in BUDGETS:
                row = view[(view["backend"] == backend) & (view["budget"] == budget)].iloc[0]
                rows.append(f"|{labels[backend]}|B{budget}|{row['mean']:.6g}|[{row['ci95_low']:.6g}, {row['ci95_high']:.6g}]|")
        return "\n".join(rows)

    def elasticity_table() -> str:
        rows = [
            "|后端|预算区间|Run数|E_W均值|标准差|95% Bootstrap CI|",
            "|---|---:|---:|---:|---:|---:|",
        ]
        for item in elastic_summary.to_dict("records"):
            rows.append(
                f"|{labels[item['backend']]}|B{item['k0']}–B{item['k1']}|"
                f"{item['n_independent_runs']}|{item['mean_E_W']:.6g}|"
                f"{item['sd_E_W']:.6g}|[{item['ci95_low']:.6g}, {item['ci95_high']:.6g}]|"
            )
        return "\n".join(rows)

    report = f"""# 正式独立后端统计分析

统计独立单位为5个诊断Run；预算是Run内重复测量因素，g2o与Ceres按同一Run配对。调用和轮次仅用于Run内汇总。Bootstrap区间按Run成组重采样（10 000次，固定种子{SEED}），不把{len(calls):,}条调用或{len(rounds):,}个轮次当作独立重复。样本量仅5个Run，区间用于描述不确定性，不作等效性声明。

## 完整性

- 正式记录：{len(runs)}，成功{int(runs['aggregate_valid'].sum())}，失败{int((~runs['aggregate_valid']).sum())}。
- 调用结果：{len(calls):,}条；轮次结果：{len(rounds):,}条；双后端配对调用：{len(pairs):,}对。
- 五个独立Run共759个真实调用、3 036个输入快照；每个后端×预算组合读取对应Run的完整调用集合。
- 两后端初始目标函数最大绝对差为{pairs['initial_cost_abs_difference'].max():.3e}。
- 所有数值有限，调用和轮次主键无重复，attempted=accepted+rejected，Linux/F盘正式归档哈希全部一致。

## BRR

{metric_table('BRR')}

## 触顶率

{metric_table('cap_hit_rate')}

## 边际实际工作量弹性

{elasticity_table()}

## 配对后端差异

`FORMAL_PAIRED_BACKEND_DIFFERENCES.csv`给出每个预算下5个Run的g2o−Ceres配对均值差、配对标准化效应量dz与Run级Bootstrap区间。结果描述后端在本回放协议和固定配置下的预算响应差异，不推断到其他硬件、线性求解器或SLAM系统。

## 成本与位姿

`FORMAL_PAIRWISE_CALL_DIFFERENCES.csv`保存每个同输入调用的成本和最终位姿差；`FORMAL_POSE_COST_PAIRING_SUMMARY.csv`仅在Run层汇总。最终位姿差并非ATE/RPE，也不与历史120次SLAM端到端精度结果混合。
"""
    (OUT / "FORMAL_STATISTICAL_ANALYSIS.md").write_text(report, encoding="utf-8")
    print(json.dumps(quality, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

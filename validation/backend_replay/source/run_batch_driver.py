#!/usr/bin/env python3
"""Execute one run-level chained replay record without silently dropping calls."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import subprocess
import sys
from collections import Counter, defaultdict
from pathlib import Path


FIELDS = [
    "fixture_id", "backend", "snapshot_file_sha256", "snapshot_input_sha256",
    "budget", "executed_rounds", "logical_outer_work", "attempted_steps",
    "accepted_steps", "rejected_steps", "residual_evaluations",
    "jacobian_evaluations", "linear_solver_iterations", "initial_cost",
    "final_cost", "solver_core_time_ms", "active_before_by_round",
    "active_after_by_round", "native_iterations_by_round", "attempted_by_round",
    "accepted_by_round", "rejected_by_round", "residual_by_round",
    "jacobian_by_round", "linear_by_round", "termination_by_round",
    "tx", "ty", "tz", "qx", "qy", "qz", "qw",
]


def digest(path: Path) -> str:
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(chunk)
    return value.hexdigest()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--record-id", required=True)
    parser.add_argument("--backend", choices=["g2o", "ceres"], required=True)
    parser.add_argument("--budget", type=int, choices=[1, 3, 4, 5, 10], required=True)
    parser.add_argument("--snapshot-manifest", type=Path, required=True)
    parser.add_argument("--replay-binary", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--role", choices=["NONFORMAL_DRIVER_TEST", "FORMAL"],
                        default="FORMAL")
    return parser.parse_args()


def split_ints(value: str) -> list[int]:
    return [int(item) for item in value.split("|") if item != ""]


def main() -> int:
    args = parse_args()
    manifest = args.snapshot_manifest.resolve()
    binary = args.replay_binary.resolve()
    output = args.output_dir.resolve()
    if output.exists():
        raise FileExistsError(f"refusing to overwrite output directory: {output}")
    if not manifest.is_file() or not binary.is_file():
        raise FileNotFoundError("snapshot manifest or replay binary is missing")
    output.mkdir(parents=True)

    with manifest.open(newline="", encoding="utf-8") as stream:
        manifest_rows = list(csv.DictReader(stream))
    groups: dict[tuple[str, str, str], list[dict[str, str]]] = defaultdict(list)
    preflight_errors: list[dict[str, str]] = []
    for row in manifest_rows:
        path = manifest.parent / "snapshots" / row["relative_path"]
        if not path.is_file():
            preflight_errors.append({"call_id": row.get("caller_id", ""),
                                     "stage": "PREFLIGHT", "message": f"missing {path}"})
            continue
        observed = digest(path)
        if observed != row["file_sha256"]:
            preflight_errors.append({"call_id": row.get("caller_id", ""),
                                     "stage": "PREFLIGHT", "message":
                                     f"file hash mismatch {path.name}"})
            continue
        groups[(row["source_run_id"], row["frame_id"], row["caller_id"])].append(row)
    for key, rows in groups.items():
        rounds = sorted(int(row["round_id"]) for row in rows)
        if rounds != [1, 2, 3, 4]:
            preflight_errors.append({"call_id": key[2], "stage": "PREFLIGHT",
                                     "message": f"round set is {rounds}"})
    sources = sorted({key[0] for key in groups})
    if len(sources) != 1:
        preflight_errors.append({"call_id": "", "stage": "PREFLIGHT",
                                 "message": f"expected one source_run_id, got {sources}"})

    error_fields = ["call_id", "stage", "returncode", "message"]
    errors: list[dict[str, str]] = [dict(row, returncode="") for row in preflight_errors]
    calls: list[dict[str, str]] = []
    rounds_out: list[dict[str, str | int]] = []
    if not preflight_errors:
        ordered = sorted(groups.items(), key=lambda item:
                         (int(item[0][1]), item[0][2]))
        for index, (key, rows) in enumerate(ordered, start=1):
            round1 = next(row for row in rows if int(row["round_id"]) == 1)
            snapshot = manifest.parent / "snapshots" / round1["relative_path"]
            call_id = f"{args.record_id}:call{index:04d}:{key[2]}"
            command = [str(binary), call_id, str(snapshot), str(args.budget)]
            completed = subprocess.run(command, text=True, capture_output=True,
                                       check=False, env=os.environ.copy())
            if completed.returncode != 0:
                errors.append({"call_id": call_id, "stage": "REPLAY",
                               "returncode": str(completed.returncode),
                               "message": completed.stderr.strip() or completed.stdout.strip()})
                continue
            parsed = next(csv.reader([completed.stdout.strip()]))
            if len(parsed) != len(FIELDS):
                errors.append({"call_id": call_id, "stage": "PARSE", "returncode": "0",
                               "message": f"expected {len(FIELDS)} fields, got {len(parsed)}"})
                continue
            row = dict(zip(FIELDS, parsed))
            if row["backend"] != args.backend or int(row["budget"]) != args.budget:
                errors.append({"call_id": call_id, "stage": "SEMANTICS", "returncode": "0",
                               "message": "backend or budget echo mismatch"})
                continue
            native = split_ints(row["native_iterations_by_round"])
            attempted = split_ints(row["attempted_by_round"])
            accepted = split_ints(row["accepted_by_round"])
            rejected = split_ints(row["rejected_by_round"])
            reasons = row["termination_by_round"].split("|")
            if not (len(native) == len(attempted) == len(accepted) ==
                    len(rejected) == len(reasons) == int(row["executed_rounds"])):
                errors.append({"call_id": call_id, "stage": "SEMANTICS", "returncode": "0",
                               "message": "round vector length mismatch"})
                continue
            if any(a != b + c for a, b, c in zip(attempted, accepted, rejected)):
                errors.append({"call_id": call_id, "stage": "SEMANTICS", "returncode": "0",
                               "message": "attempted != accepted + rejected"})
                continue
            if any(value < 0 or value > args.budget for value in native):
                errors.append({"call_id": call_id, "stage": "SEMANTICS", "returncode": "0",
                               "message": "native iteration outside [0,k]"})
                continue
            if sum(native) != int(row["logical_outer_work"]):
                errors.append({"call_id": call_id, "stage": "SEMANTICS", "returncode": "0",
                               "message": "logical work does not equal round sum"})
                continue
            row["source_run_id"] = key[0]
            row["frame_id"] = key[1]
            row["caller_id"] = key[2]
            calls.append(row)
            before = split_ints(row["active_before_by_round"])
            after = split_ints(row["active_after_by_round"])
            residual = split_ints(row["residual_by_round"])
            jacobian = split_ints(row["jacobian_by_round"])
            linear = split_ints(row["linear_by_round"])
            for round_index in range(len(native)):
                cap_reason = (args.backend == "g2o" and reasons[round_index] == "REQUESTED_MAXIMUM") or (
                    args.backend == "ceres" and reasons[round_index] == "NO_CONVERGENCE")
                rounds_out.append({
                    "call_id": call_id, "round_id": round_index + 1,
                    "native_iterations": native[round_index],
                    "attempted_steps": attempted[round_index],
                    "accepted_steps": accepted[round_index],
                    "rejected_steps": rejected[round_index],
                    "residual_evaluations": residual[round_index],
                    "jacobian_evaluations": jacobian[round_index],
                    "linear_solver_iterations": linear[round_index],
                    "active_before": before[round_index], "active_after": after[round_index],
                    "termination_reason": reasons[round_index],
                    "cap_hit": int(native[round_index] == args.budget and cap_reason),
                })

    call_fields = FIELDS + ["source_run_id", "frame_id", "caller_id"]
    with (output / "call_results.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=call_fields, extrasaction="ignore")
        writer.writeheader(); writer.writerows(calls)
    round_fields = ["call_id", "round_id", "native_iterations", "attempted_steps",
                    "accepted_steps", "rejected_steps", "residual_evaluations",
                    "jacobian_evaluations", "linear_solver_iterations", "active_before",
                    "active_after", "termination_reason", "cap_hit"]
    with (output / "round_results.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=round_fields)
        writer.writeheader(); writer.writerows(rounds_out)
    with (output / "errors.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=error_fields)
        writer.writeheader(); writer.writerows(errors)

    valid = len(errors) == 0 and len(calls) == len(groups)
    requested_capacity = len(groups) * 4 * args.budget
    logical_work = sum(int(row["logical_outer_work"]) for row in calls)
    aggregate = {
        "record_id": args.record_id, "role": args.role, "backend": args.backend,
        "budget": args.budget, "source_run_id": sources[0] if len(sources) == 1 else None,
        "snapshot_manifest": str(manifest), "snapshot_manifest_sha256": digest(manifest),
        "replay_binary": str(binary), "replay_binary_sha256": digest(binary),
        "call_total": len(groups), "call_success": len(calls),
        "call_failure": len(groups) - len(calls), "aggregate_valid": valid,
        "requested_capacity": requested_capacity, "logical_outer_work": logical_work,
        "BRR": logical_work / requested_capacity if valid and requested_capacity else None,
        "cap_hit_rounds": sum(int(row["cap_hit"]) for row in rounds_out),
        "executed_rounds": len(rounds_out),
        "cap_hit_rate": (sum(int(row["cap_hit"]) for row in rounds_out) / len(rounds_out))
                        if valid and rounds_out else None,
        "E_W": None, "E_W_status": "REQUIRES_ADJACENT_BUDGET_RUN_RECORDS",
        "accepted_steps": sum(int(row["accepted_steps"]) for row in calls),
        "rejected_steps": sum(int(row["rejected_steps"]) for row in calls),
        "residual_evaluations": sum(int(row["residual_evaluations"]) for row in calls),
        "jacobian_evaluations": sum(int(row["jacobian_evaluations"]) for row in calls),
        "linear_solver_iterations": sum(int(row["linear_solver_iterations"]) for row in calls),
        "solver_core_time_ms": sum(float(row["solver_core_time_ms"]) for row in calls),
        "termination_reason_counts": dict(sorted(Counter(
            row["termination_reason"] for row in rounds_out).items())),
    }
    (output / "run_aggregate.json").write_text(
        json.dumps(aggregate, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    with (output / "SHA256SUMS").open("w", encoding="utf-8") as stream:
        for path in sorted(output.glob("*")):
            if path.is_file() and path.name != "SHA256SUMS":
                stream.write(f"{digest(path)}  {path.name}\n")
    print(json.dumps(aggregate, sort_keys=True))
    return 0 if valid else 1


if __name__ == "__main__":
    raise SystemExit(main())

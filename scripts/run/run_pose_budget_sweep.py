#!/usr/bin/env python3
"""Run independent budget records from an explicit, public JSON configuration.

This helper never deletes or overwrites an output directory. It is a portable
execution aid, not the frozen historical randomisation schedule.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import subprocess
import time
from pathlib import Path


def digest(path: Path) -> str:
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(chunk)
    return value.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("config", type=Path)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    config = json.loads(args.config.read_text(encoding="utf-8"))
    required = ["slam_binary", "vocabulary", "settings", "dataset",
                "association", "output_root", "scene_id", "budgets", "repeats"]
    missing = [key for key in required if key not in config]
    if missing:
        raise ValueError("missing configuration fields: " + ", ".join(missing))
    budgets = [int(value) for value in config["budgets"]]
    if budgets != [1, 3, 4, 5, 10]:
        raise ValueError("this release expects budgets [1, 3, 4, 5, 10]")
    paths = {name: Path(config[name]).expanduser().resolve() for name in
             ("slam_binary", "vocabulary", "settings", "dataset", "association")}
    if not args.dry_run:
        absent = [str(path) for path in paths.values() if not path.exists()]
        if absent:
            raise FileNotFoundError("missing inputs: " + ", ".join(absent))
    output_root = Path(config["output_root"]).expanduser().resolve()
    if output_root.exists():
        raise FileExistsError(f"refusing to reuse output root: {output_root}")
    records = []
    for budget in budgets:
        for repeat in range(1, int(config["repeats"]) + 1):
            run_id = f"{config['scene_id']}_B{budget}_run{repeat:02d}"
            run_dir = output_root / run_id
            command = [str(paths["slam_binary"]), str(paths["vocabulary"]),
                       str(paths["settings"]), str(paths["dataset"]),
                       str(paths["association"])]
            records.append((run_id, budget, repeat, run_dir, command))
    if args.dry_run:
        print(json.dumps({"records": len(records), "run_ids": [r[0] for r in records]},
                         indent=2))
        return 0
    output_root.mkdir(parents=True)
    ledger = output_root / "RUN_LEDGER.csv"
    fields = ["run_id", "budget", "repeat", "returncode", "wall_seconds",
              "binary_sha256", "settings_sha256", "association_sha256"]
    with ledger.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for run_id, budget, repeat, run_dir, command in records:
            run_dir.mkdir()
            env = os.environ.copy()
            env.update({
                "ORB_SLAM2_POSE_BUDGET_MODE": f"B{budget}",
                "ORB_SLAM2_POSE_BUDGET_LOG": str(run_dir / "pose_budget.csv"),
                "ORB_SLAM2_POSE_ITERATION_LOG": str(run_dir / "pose_iterations.csv"),
                "ORB_SLAM2_OUTPUT_DIR": str(run_dir),
                "ORB_SLAM2_PILOT_MAX_FRAMES": str(config.get("max_frames", 80)),
            })
            wrapped = command
            affinity = str(config.get("cpu_affinity", "")).strip()
            if affinity:
                wrapped = ["taskset", "-c", affinity] + command
            started = time.monotonic()
            with (run_dir / "stdout_stderr.log").open("w", encoding="utf-8") as log:
                result = subprocess.run(wrapped, cwd=run_dir, env=env, stdout=log,
                                        stderr=subprocess.STDOUT, check=False)
            writer.writerow({
                "run_id": run_id, "budget": budget, "repeat": repeat,
                "returncode": result.returncode,
                "wall_seconds": f"{time.monotonic() - started:.6f}",
                "binary_sha256": digest(paths["slam_binary"]),
                "settings_sha256": digest(paths["settings"]),
                "association_sha256": digest(paths["association"]),
            })
            stream.flush()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Capture one audited RGB-D run as a BSRW diagnostic snapshot collection.

This is a diagnostic data-acquisition driver, not a formal replay runner.  It
keeps the historical raw-input identity and belief-active configuration while
writing every mutable artifact under the caller-provided output directory.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import socket
import subprocess
import sys
import time
from pathlib import Path


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def wait_for_socket(path: Path, process: subprocess.Popen[str], timeout: float) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if path.exists():
            return
        if process.poll() is not None:
            raise RuntimeError(f"YOLO process exited before socket readiness: {process.returncode}")
        time.sleep(0.2)
    raise TimeoutError(f"socket was not ready within {timeout}s: {path}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--association", type=Path, required=True)
    parser.add_argument("--settings", type=Path, required=True)
    parser.add_argument("--vocabulary", type=Path, required=True)
    parser.add_argument("--slam-binary", type=Path, required=True)
    parser.add_argument("--yolo-workdir", type=Path, required=True)
    parser.add_argument("--yolo-source", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--socket-path", type=Path, required=True)
    parser.add_argument("--max-frames", type=int, default=80)
    parser.add_argument("--socket-timeout", type=float, default=120.0)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--profile", choices=["geometry-diagnostic", "belief-active"],
                        default="geometry-diagnostic")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    args.output_dir = args.output_dir.resolve()
    args.socket_path = args.socket_path.resolve()
    for name in ("dataset", "association", "settings", "vocabulary",
                 "slam_binary", "yolo_workdir", "yolo_source"):
        setattr(args, name, getattr(args, name).resolve())
    paths = [args.dataset, args.association, args.settings, args.vocabulary,
             args.slam_binary, args.yolo_workdir, args.yolo_source]
    missing = [str(path) for path in paths if not path.exists()]
    if missing:
        raise FileNotFoundError("missing required inputs: " + ", ".join(missing))
    if args.socket_path.exists():
        raise FileExistsError(f"refusing to remove pre-existing socket: {args.socket_path}")

    args.output_dir.mkdir(parents=True, exist_ok=False)
    snapshots = args.output_dir / "snapshots"
    snapshots.mkdir()
    cache = args.output_dir / "matplotlib_cache"
    cache.mkdir()
    yolo_project = args.output_dir / "yolo_runs"

    env = os.environ.copy()
    env.update({
        "MPLCONFIGDIR": str(cache),
        "PYTHONDONTWRITEBYTECODE": "1",
        "ORB_SLAM2_ACTIVE_MODE_ENABLED": "0",
        "ORB_SLAM2_BELIEF_ENABLED": "1",
        "ORB_SLAM2_RELIABILITY_ENABLED": "1",
        "ORB_SLAM2_LEGACY_TEMPORAL_WEIGHT_ENABLED": "1",
        "ORB_SLAM2_LEGACY_TEMPORAL_HARD_REJECTION_ENABLED": "1",
        "ORB_SLAM2_MEASUREMENT_POLICY": "BELIEF_ONLY",
        "ORB_SLAM2_OBJECT_MAP": "0",
        "ORB_SLAM2_RUNTIME_LIGHTWEIGHT": "1",
        "ORB_SLAM2_SEMANTIC_MODE": "0",
        "ORB_SLAM2_SOCKET_PATH": str(args.socket_path),
        "ORB_SLAM2_OUTPUT_DIR": str(args.output_dir),
        "ORB_SLAM2_POSE_BUDGET_MODE": "B10",
        "ORB_SLAM2_POSE_BUDGET_LOG": str(args.output_dir / "pose_budget.csv"),
        "ORB_SLAM2_POSE_ITERATION_LOG": str(args.output_dir / "pose_iterations.csv"),
        "ORB_SLAM2_BSRW_SNAPSHOT_DIR": str(snapshots),
        "ORB_SLAM2_BSRW_SOURCE_RUN_ID": args.run_id,
        "ORB_SLAM2_PILOT_MAX_FRAMES": str(args.max_frames),
    })
    env.pop("ORB_SLAM2_MEMORY_CARRIER", None)
    if args.profile == "belief-active":
        env.update({
            "ORB_SLAM2_ACTIVE_MODE_ENABLED": "1",
            "ORB_SLAM2_LEGACY_TEMPORAL_WEIGHT_ENABLED": "0",
            "ORB_SLAM2_LEGACY_TEMPORAL_HARD_REJECTION_ENABLED": "0",
            "ORB_SLAM2_MEASUREMENT_POLICY": "GEOMETRY_PROTECTED",
            "ORB_SLAM2_MEMORY_CARRIER": "mappoint",
            "ORB_SLAM2_SEMANTIC_MODE": "2",
        })
    yolo_command = [
        sys.executable, "detect_speedup_send.py",
        "--source", str(args.yolo_source),
        "--weights", "./weights/yolov5s.pt",
        "--conf-thres", "0.4", "--save-txt",
        "--img-size", "224", "--device", args.device,
        "--project", str(yolo_project), "--name", args.run_id,
        "--exist-ok",
    ]
    slam_command = [str(args.slam_binary), str(args.vocabulary), str(args.settings),
                    str(args.dataset), str(args.association)]
    metadata = {
        "capture_role": "NONFORMAL_DIAGNOSTIC_SNAPSHOT_ACQUISITION",
        "capture_profile": args.profile,
        "historical_equivalence_claim": False,
        "run_id": args.run_id,
        "commands": {"yolo": yolo_command, "slam": slam_command},
        "inputs": {
            "association": str(args.association),
            "association_sha256": sha256_file(args.association),
            "settings": str(args.settings),
            "settings_sha256": sha256_file(args.settings),
            "vocabulary": str(args.vocabulary),
            "vocabulary_sha256": sha256_file(args.vocabulary),
            "slam_binary": str(args.slam_binary),
            "slam_binary_sha256": sha256_file(args.slam_binary),
            "yolo_weights": str(args.yolo_workdir / "weights/yolov5s.pt"),
            "yolo_weights_sha256": sha256_file(args.yolo_workdir / "weights/yolov5s.pt"),
            "yolo_source": str(args.yolo_source),
        },
        "environment": {key: env[key] for key in sorted(env)
                        if key.startswith("ORB_SLAM2_") or key in
                        {"MPLCONFIGDIR", "PYTHONDONTWRITEBYTECODE"}},
    }
    (args.output_dir / "capture_environment.json").write_text(
        json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    yolo_log = (args.output_dir / "yolo.log").open("w", encoding="utf-8")
    yolo: subprocess.Popen[str] | None = None
    try:
        if args.profile == "belief-active":
            yolo = subprocess.Popen(yolo_command, cwd=args.yolo_workdir, env=env,
                                    stdout=yolo_log, stderr=subprocess.STDOUT, text=True)
            wait_for_socket(args.socket_path, yolo, args.socket_timeout)
        else:
            yolo_log.write("YOLO_NOT_STARTED profile=geometry-diagnostic\n")
            yolo_log.flush()
        with (args.output_dir / "slam.log").open("w", encoding="utf-8") as slam_log:
            completed = subprocess.run(slam_command, cwd=args.output_dir, env=env,
                                       stdout=slam_log, stderr=subprocess.STDOUT,
                                       text=True, check=False)
        (args.output_dir / "capture_exit_status.json").write_text(
            json.dumps({"slam_returncode": completed.returncode,
                        "yolo_returncode_before_shutdown":
                            yolo.poll() if yolo is not None else None},
                       indent=2, sort_keys=True) + "\n", encoding="utf-8")
        return completed.returncode
    finally:
        if yolo is not None and yolo.poll() is None:
            yolo.terminate()
            try:
                yolo.wait(timeout=10)
            except subprocess.TimeoutExpired:
                yolo.kill()
                yolo.wait(timeout=5)
        yolo_log.close()
        if args.socket_path.exists() and stat_is_socket(args.socket_path):
            args.socket_path.unlink()


def stat_is_socket(path: Path) -> bool:
    return socket.S_ISSOCK(path.stat().st_mode) if hasattr(socket, "S_ISSOCK") else path.is_socket()


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Validate build capability without starting SLAM Tracking."""

import argparse
import subprocess
import sys
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--active", type=int, choices=(0, 1), required=True)
    parser.add_argument("--shadow", type=int, choices=(0, 1), required=True)
    parser.add_argument("--counters", type=int, choices=(0, 1), required=True)
    args = parser.parse_args()

    completed = subprocess.run(
        [str(args.binary)], text=True, stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT, check=False)
    expected = (
        f"[BuildCapability] ACTIVE_MODE={args.active} "
        f"SHADOW_MODE={args.shadow} REALIZED_WORK_COUNTERS={args.counters}")
    if completed.returncode != 1:
        print(f"unexpected no-argument exit code: {completed.returncode}", file=sys.stderr)
        return 1
    if expected not in completed.stdout:
        print(f"missing capability banner: {expected!r}", file=sys.stderr)
        print(completed.stdout, file=sys.stderr)
        return 1
    if "Usage: ./rgbd_tum" not in completed.stdout:
        print("missing expected usage text", file=sys.stderr)
        return 1
    print(expected)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

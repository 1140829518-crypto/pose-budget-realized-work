#!/usr/bin/env python3
"""Build deterministic input and snapshot manifests for one diagnostic run."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from collections import defaultdict
from pathlib import Path


def digest(path: Path) -> str:
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(chunk)
    return value.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--association", type=Path, required=True)
    args = parser.parse_args()
    run_dir = args.run_dir.resolve()
    dataset = args.dataset.resolve()
    association = args.association.resolve()

    input_rows: list[tuple[str, str, str]] = []
    for line in association.read_text(encoding="utf-8").splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        fields = stripped.split()
        if len(fields) < 4:
            raise ValueError(f"invalid association row: {line}")
        for role, relative in (("rgb", fields[1]), ("depth", fields[3])):
            path = (dataset / relative).resolve()
            if not path.is_file():
                raise FileNotFoundError(path)
            input_rows.append((role, str(path), digest(path)))
    input_path = run_dir / "INPUT_RGBD_SHA256SUMS.csv"
    with input_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["role", "resolved_path", "sha256"])
        writer.writerows(input_rows)

    snapshots = sorted((run_dir / "snapshots").glob("*.json"))
    if not snapshots:
        raise ValueError("empty snapshot directory")
    rows = []
    groups: dict[tuple[str, int, str], list[int]] = defaultdict(list)
    source_ids = set()
    budgets = set()
    for path in snapshots:
        payload = json.loads(path.read_text(encoding="utf-8"))
        source_ids.add(payload["source_run_id"])
        budgets.add(int(payload["budget"]))
        key = (payload["source_run_id"], int(payload["frame_id"]), payload["caller_id"])
        groups[key].append(int(payload["round_id"]))
        rows.append([
            path.name, digest(path), payload["input_sha256"], payload["snapshot_id"],
            payload["source_run_id"], payload["frame_id"], payload["caller_id"],
            payload["round_id"], payload["budget"], len(payload["observations"]),
            len(payload["active_observations"]),
        ])
    incomplete = [key for key, rounds in groups.items() if sorted(rounds) != [1, 2, 3, 4]]
    manifest = run_dir / "SNAPSHOT_MANIFEST.csv"
    with manifest.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(["relative_path", "file_sha256", "input_sha256", "snapshot_id",
                         "source_run_id", "frame_id", "caller_id", "round_id", "budget",
                         "observation_count", "active_observation_count"])
        writer.writerows(rows)
    sums = run_dir / "SNAPSHOT_SHA256SUMS"
    sums.write_text("".join(f"{row[1]}  snapshots/{row[0]}\n" for row in rows),
                    encoding="utf-8")
    status = {
        "snapshot_count": len(rows),
        "call_count": len(groups),
        "complete_four_round_calls": len(groups) - len(incomplete),
        "incomplete_call_count": len(incomplete),
        "source_run_ids": sorted(source_ids),
        "budgets": sorted(budgets),
        "association_sha256": digest(association),
        "input_file_count": len(input_rows),
        "input_manifest_sha256": digest(input_path),
        "snapshot_manifest_sha256": digest(manifest),
        "snapshot_sums_sha256": digest(sums),
    }
    (run_dir / "COLLECTION_VALIDATION.json").write_text(
        json.dumps(status, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(status, sort_keys=True))
    return 0 if not incomplete and len(source_ids) == 1 and budgets == {10} else 1


if __name__ == "__main__":
    raise SystemExit(main())

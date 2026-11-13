#!/usr/bin/env python3
"""Validate frozen row counts, hierarchy and publication-safe paths."""

from __future__ import annotations

import csv
import math
import re
from collections import defaultdict
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def rows(relative: str) -> list[dict[str, str]]:
    with (ROOT / relative).open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def mean(values: list[float]) -> float:
    return sum(values) / len(values)


def main() -> int:
    historical_runs = rows("data/formal/FINAL_RESULTS.csv")
    historical_calls = rows("data/formal/INVOCATION_RESULTS.csv")
    extension_runs = rows("data/extension60/EFFECTIVE_60_RUN_RESULTS.csv")
    extension_calls = rows("data/extension60/EFFECTIVE_60_INVOCATION_RECORDS.csv")
    cross = rows("data/cross_batch/BATCH_SEPARATED_120_RUN_INDEX.csv")
    backend_runs = rows("data/backend_validation/FORMAL_RUN_RESULTS.csv")
    backend_calls = rows("data/backend_validation/FORMAL_CALL_RESULTS.csv")
    backend_rounds = rows("data/backend_validation/FORMAL_ROUND_RESULTS.csv")
    paired_calls = rows("data/backend_validation/FORMAL_PAIRWISE_CALL_DIFFERENCES.csv")
    failures = rows("data/backend_validation/FORMAL_FAILURES.csv")

    require(len(historical_runs) == 60, "historical run count")
    require(len(historical_calls) == 9480, "historical invocation count")
    require(len(extension_runs) == 60, "extension run count")
    require(len(extension_calls) == 9480, "extension invocation count")
    require(all(row["status"] == "SUCCESS" for row in extension_runs),
            "extension includes non-success rows")
    require(len(cross) == 120, "cross-batch index count")
    require({row["batch"] for row in cross} ==
            {"historical_frozen60", "independent_extension60"},
            "cross-batch identities")
    require(len(backend_runs) == 50, "backend run-record count")
    require(len(backend_calls) == 7590, "backend call-result count")
    require(len(backend_rounds) == 30360, "backend round-result count")
    require(len(paired_calls) == 3795, "paired backend call count")
    require(len(failures) == 0, "formal backend failures are present")
    require(len({row["independent_run_id"] for row in backend_runs}) == 5,
            "backend independent-run count")
    require({int(row["budget"]) for row in backend_runs} == {1, 3, 4, 5, 10},
            "backend budgets")
    require({row["backend"] for row in backend_runs} == {"g2o", "ceres"},
            "backend identities")

    expected = {
        "g2o": {1: 1.0000, 3: 0.9409, 4: 0.8599, 5: 0.7075, 10: 0.3578},
        "ceres": {1: 0.8035, 3: 0.4562, 4: 0.3653, 5: 0.3044, 10: 0.1618},
    }
    grouped: dict[tuple[str, int], list[float]] = defaultdict(list)
    for row in backend_runs:
        grouped[(row["backend"], int(row["budget"]))].append(float(row["BRR"]))
    for backend, budgets in expected.items():
        for budget, target in budgets.items():
            observed = mean(grouped[(backend, budget)])
            require(math.isclose(observed, target, abs_tol=5e-5),
                    f"{backend} B{budget} mean BRR {observed} != {target}")

    forbidden = re.compile(
        "/" + "home/" + "|/" + "mnt/[a-zA-Z]/|[A-Za-z]:" + r"\\" +
        "|github_" + "pat_|g" + "hp_"
    )
    offenders = []
    for path in ROOT.rglob("*"):
        if (not path.is_file() or path.resolve() == Path(__file__).resolve() or
                ".git" in path.parts or path.suffix.lower() in {".png"}):
            continue
        try:
            text = path.read_text(encoding="utf-8")
        except UnicodeDecodeError:
            continue
        if forbidden.search(text):
            offenders.append(str(path.relative_to(ROOT)))
    require(not offenders, "private path/token patterns: " + ", ".join(offenders))
    print("PASS: release hierarchy, counts, BRR means, failures and path scan")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

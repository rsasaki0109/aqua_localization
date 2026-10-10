#!/usr/bin/env python3
"""Compare offline_replay summaries and report whether the runs are identical.

Each ``replay_summary.yaml`` carries per-output-topic message counts and an
FNV-1a digest of the serialized messages in delivery order. Two runs of the
same scenario on the same bag must match exactly; any difference means the
pipeline (not the replay) is nondeterministic, and the per-topic table shows
where the divergence starts (e.g. sonar odometry differs but IMU input does
not).

Exit status: 0 when every compared topic matches, 1 otherwise.
"""

import argparse
import sys
from pathlib import Path

import yaml


def load(path: Path) -> dict:
    p = Path(path)
    if p.is_dir():
        p = p / "replay_summary.yaml"
    with p.open() as f:
        return yaml.safe_load(f)


def compare(summaries, topics=None):
    ref = summaries[0]
    names = topics or sorted(ref.get("outputs", {}))
    rows = []
    all_match = True
    for topic in names:
        cells = []
        for s in summaries:
            out = s.get("outputs", {}).get(topic)
            cells.append(None if out is None else (out["count"], out["fnv1a64"]))
        match = all(c == cells[0] and c is not None for c in cells)
        all_match &= match
        rows.append((topic, cells, match))
    return rows, all_match


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("summaries", nargs="+",
                        help="replay_summary.yaml files or output bag directories (>= 2)")
    parser.add_argument("--topic", action="append", dest="topics",
                        help="Only compare these output topics (repeatable).")
    args = parser.parse_args(argv)
    if len(args.summaries) < 2:
        parser.error("need at least two summaries")
    summaries = [load(p) for p in args.summaries]
    rows, all_match = compare(summaries, args.topics)

    print(f"{'topic':40s} " + " ".join(f"{'run' + str(i):>28s}" for i in range(len(summaries)))
          + "  status")
    for topic, cells, match in rows:
        rendered = " ".join(
            f"{'missing':>28s}" if c is None else f"{c[0]:>9d} {c[1]:>18s}" for c in cells)
        print(f"{topic:40s} {rendered}  {'MATCH' if match else 'DIFF'}")
    prints = [s.get("fingerprint") for s in summaries]
    print("fingerprints: " + ", ".join(str(p) for p in prints))
    print("RESULT: " + ("IDENTICAL" if all_match else "DIFFERENT"))
    return 0 if all_match else 1


if __name__ == "__main__":
    sys.exit(main())

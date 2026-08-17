"""Reading the results, and applying the rule that was written before the run.

The decision rule comes from [docs/omni-build.md], and it is quoted rather than
paraphrased because the document is explicit that it exists to bind whoever runs
the measurement -- who will, it says, be motivated to continue:

    a spoken total within ten points of the text column (out of 90) proceeds;
    a drop of more than twenty ends the project; in between, the failures are
    read one by one before any hardware is ordered.

One deliberate departure. The document compares against "the text column we
already have", meaning the 66/90 measured on `Qwen3-VL-4B`. Comparing an omni
model's spoken score against a different model's typed score confounds the change
of modality with the change of model, so the text column here is re-measured on
the same weights in the same process, and the old number is carried alongside as
background rather than as the comparator. The rule's arithmetic is unchanged.

    python score.py runs/minicpm-text.jsonl runs/minicpm-audio.jsonl
"""

from __future__ import annotations

import argparse
import json
from collections import defaultdict
from pathlib import Path

from corpus import DENOMINATOR, PRIMARY, SAMPLES

PROCEED, READ, STOP = "proceed", "read the failures", "stop"


def load(paths: list[Path]) -> list[dict]:
    records = []
    for path in paths:
        for line in path.read_text(encoding="utf-8").splitlines():
            if line.strip():
                records.append(json.loads(line))
    return records


def tally(records: list[dict]) -> dict[str, dict[str, int]]:
    """passed counts, per arm, per case."""
    counts: dict[str, dict[str, int]] = defaultdict(lambda: defaultdict(int))
    for record in records:
        counts[record["arm"]][record["case"]] += int(record["passed"])
    return counts


def verdict(text_total: int, audio_total: int) -> tuple[str, str]:
    drop = text_total - audio_total
    if drop <= 10:
        return PROCEED, f"spoken is {drop} points under typed, within the ten-point band"
    if drop > 20:
        return STOP, f"spoken is {drop} points under typed, past the twenty-point limit"
    return READ, f"spoken is {drop} points under typed, between ten and twenty"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("results", nargs="+", type=Path)
    parser.add_argument("--verbose", action="store_true", help="every failing attempt")
    args = parser.parse_args()

    records = load(args.results)
    counts = tally(records)
    primary = {case.key: case for case in PRIMARY}
    arms = [arm for arm in ("text", "audio") if arm in counts]

    header = "request".ljust(42) + "".join(a.rjust(8) for a in arms) + "   was"
    print(header)
    print("-" * len(header))
    totals = {arm: 0 for arm in arms}
    for case in PRIMARY:
        row = case.text.ljust(42)
        for arm in arms:
            got = counts[arm].get(case.key, 0)
            totals[arm] += got
            row += f"{got}/{SAMPLES}".rjust(8)
        print(row + "   " + case.baseline)
    print("-" * len(header))
    print("total".ljust(42) + "".join(f"{totals[a]}/{DENOMINATOR}".rjust(8) for a in arms))

    # Anything outside the fifteen is reported separately, never folded in.
    extra = {k: v for arm in arms for k, v in counts[arm].items() if k not in primary}
    if extra:
        print("\ncoverage cases, not counted in the total:")
        for arm in arms:
            for key, got in sorted(counts[arm].items()):
                if key not in primary:
                    print(f"  {arm:<6} {key:<36} {got}/{SAMPLES}")

    if "text" in counts and "audio" in counts:
        call, why = verdict(totals["text"], totals["audio"])
        print(f"\ndecision rule: {call.upper()} -- {why}")
        print("(the 66/90 and 75/90 in voice_chat/README.md were Qwen3-VL-4B, typed, "
              "and are background here rather than the comparator)")

    spoken_only = [
        case for case in PRIMARY
        if counts.get("text", {}).get(case.key, 0) - counts.get("audio", {}).get(case.key, 0) >= 3
    ]
    if spoken_only:
        print("\ncells that speech cost at least three of six:")
        for case in spoken_only:
            print(f"  {case.text}")

    timed = [r for r in records if r.get("marker_at_time") is not None]
    if timed:
        chars = sorted(r["marker_at_char"] for r in timed)
        print(f"\ntool call surfaced after {chars[0]}-{chars[-1]} characters of reply "
              f"(median {chars[len(chars) // 2]}), across {len(timed)} calls")

    if args.verbose:
        print("\nfailures:")
        for record in records:
            if not record["passed"]:
                print(f"  [{record['arm']}] {record['text']:<40} {record['why']}")
                if record["prose"]:
                    print(f"      said: {record['prose'][:110]}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

"""Why chosen observations ended unassigned: every resolver decision about them.

The store records a match with its reason and records nothing for a region left
waiting, so "unassigned" on its own cannot say whether nothing fitted or too
much did. This reruns a recording's actual calls, as `replay_call_recording`
does, and prints each decision the resolver makes about the chosen observations
-- ambiguous ones included, with every candidate it weighed. `--new-visibility-only`
runs the visibility candidate instead of the control, exactly as
`replay_visibility_rival_veto.py --new-visibility-only` does.

    python experiments/entity_association/explain_ambiguity.py --directory <recording>
        --output <new-directory> --observations 76267,76277 [--new-visibility-only]

Read-only on the recording; writes only the replay stores under `--output`.
On the identity trial of 2026-10-07 it showed the clear views of the painting
left waiting because seven placed things, five of them records of that one
painting, fitted equally well.
"""
import argparse
import functools
import json
import runpy
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from world_state import resolve  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--directory", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--observations", required=True,
                        help="comma-separated observation identifiers to explain")
    parser.add_argument("--new-visibility-only", action="store_true")
    args = parser.parse_args()
    watch = {int(one) for one in args.observations.split(",")}
    original = resolve.resolve

    @functools.wraps(original)
    def explained(*a, **k):
        result = original(*a, **k)
        for one in result.get("decisions", []):
            if one["observation_id"] in watch:
                print(json.dumps({key: one.get(key) for key in
                                  ("observation_id", "outcome", "entity_id", "why",
                                   "candidates")}), flush=True)
        return result

    resolve.resolve = explained
    if args.new_visibility_only:
        script = "replay_visibility_rival_veto.py"
        sys.argv = [script, "--directory", args.directory, "--output", args.output,
                    "--new-visibility-only"]
    else:
        script = "replay_call_recording.py"
        sys.argv = [script, "--directory", args.directory, "--output", args.output]
    runpy.run_path(str(Path(__file__).with_name(script)), run_name="__main__")


if __name__ == "__main__":
    main()

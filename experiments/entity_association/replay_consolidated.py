"""Counterfactual: a recording replayed with chosen records already one thing.

Which records are one object is decided beforehand from their photographs and
given here; nothing in this script chooses them. The control arm reproduces the
recording exactly, as `replay_call_recording.py` checks. The counterfactual arm
starts from the same checked store, joins the records with the world state's
own merge (`merging.apply`, which refuses two records that share a look and
re-places the kept one from everything it then holds), and replays the same
calls against the archived maps. `--new-visibility-only` runs the visibility
candidate on the consolidated store as well, as
`replay_visibility_rival_veto.py --new-visibility-only` does on the original.

    python experiments/entity_association/replay_consolidated.py --directory <recording>
        --output <new-directory> --merge object:375=object:301,object:328
        [--merge ...] [--new-visibility-only]

A diagnostic only: the real store is opened read-only and never merged, and a
good result here does not justify merging it.
"""
import argparse
import contextlib
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from world_state import merging, resolve  # noqa: E402
from experiments.entity_association.replay_call_recording import run  # noqa: E402
from experiments.entity_association.replay_measured_visibility import (  # noqa: E402
    matching_scope, matching_visibility)
from experiments.entity_association.replay_visibility_rival_veto import (  # noqa: E402
    frame_rival_veto)


def parse_merges(values):
    """`keep=gone,gone` strings as (keep, [gone, ...]) pairs."""
    out = []
    for value in values:
        keep, gone = value.split("=", 1)
        out.append((keep.strip(), [one.strip() for one in gone.split(",") if one.strip()]))
    return out


def preparer(merges):
    def prepare(store, reach):
        done = []
        for keep, gones in merges:
            for gone in gones:
                answer = merging.apply(store, [[keep, gone]], reach=reach)
                done.append({"keep": keep, "gone": gone, "ok": answer.get("ok"),
                             "joined": answer.get("joined"),
                             "refused": answer.get("refused") or answer.get("error")})
        return done
    return prepare


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--directory", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--merge", action="append", required=True,
                   help="keep=gone[,gone...]: records judged one object from their photographs")
    p.add_argument("--new-visibility-only", action="store_true")
    a = p.parse_args()
    prepare = preparer(parse_merges(a.merge))
    if not a.new_visibility_only:
        result = run(a.directory, a.output, map_invariant=True, prepare=prepare)
    else:
        active, original_rays = [False], {}

        def transform(ray):
            active[0] = True
            original_rays[ray["observation_id"]] = dict(ray)
            return matching_visibility(ray)

        with frame_rival_veto(lambda: active[0], original_rays) as refusals, matching_scope():
            result = run(a.directory, a.output, map_invariant=True,
                         candidate_ray_transform=transform, prepare=prepare)
        (a.output / "rival-veto.json").write_text(json.dumps(
            {"threshold": resolve.OUTCLASSED_LEAD, "refusals": refusals}, indent=2) + "\n")
    print(json.dumps(result.get("prepared"), indent=1))


if __name__ == "__main__":
    main()

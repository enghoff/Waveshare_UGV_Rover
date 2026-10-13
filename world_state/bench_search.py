#!/usr/bin/env python3
"""Does a search find what is in the flat and refuse what cannot be?

    ssh orin 'python3 ~/ugv/world_state/bench_search.py'
    ssh orin 'python3 ~/ugv/world_state/bench_search.py --floors 0.0975,0.1,0.105'
    ssh orin 'python3 ~/ugv/world_state/bench_search.py --looks'
    ssh orin 'python3 ~/ugv/world_state/bench_search.py --looks --floors 0.09 --keep-slivers'

Read-only: the store is copied through sqlite into a temporary directory first,
and the phrases go to the running perception sidecar for their vectors, which is
the one thing that has to be the rover's own -- a phrase embedded anywhere else
would not compare.

Each phrase is judged the way `find_thing` judges it (`rover_recall._recall`):
since 2026-10-13 that is every thing ranked by its average look
(`search.rank_things`), found if the best clears `THING_MATCHES`. `--looks` judges
the way it did before -- rank the newest 2,000 looks, walk the top ten, take the
first that belongs to a thing if it clears `MATCHES` -- and adding
`--floors 0.09 --keep-slivers` to that is the search before 2026-10-05. What is
counted is whether a phrase was found, not whether the right thing was; the
second is judged by eye, and `--sheets DIR` writes a contact sheet per phrase of
the thing it settled on to judge it from.

**The phrases are frozen, in two sets.** The first was what the fix of
2026-10-05 was chosen on and the second was written afterwards to check it. The
real things come from the labelled drive of 2026-09-08
(`labels/m0-2026-09-08.json`) and the absent ones are things that cannot be in
the flat. Change a list and the numbers stop being comparable with
[the progress entry](../docs/progress/2026-10-05-search-floor.md).
"""
from __future__ import annotations

import argparse
import os
import shutil
import sqlite3
import sys
import tempfile

if __package__ in (None, ""):                       # run as a script
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    __package__ = "world_state"

from . import search                                # noqa: E402
from .perception_client import SidecarEyes          # noqa: E402
from .store import WorldStore, world_dir            # noqa: E402

#: How far down the look ranking `find_thing` used to walk for a thing, before it
#: ranked things (`--looks`).
RECALL_LIMIT = 10
#: Contact sheets: how many crops of the settled thing, and how big.
SHEET_CROPS = 6
SHEET_TILE = 100

SETS = {
    "tuning": (
        ["the rug", "a dining chair", "the dining table", "a framed picture",
         "a pendant lamp", "the sofa", "the blue armchair", "the teal door",
         "a green landscape painting", "a white door", "a ceiling fan",
         "the office chair", "the keyboard", "the wardrobe", "the bed",
         "the floor lamp", "the black cabinet", "the chest of drawers",
         "the front door", "a lampshade", "the cow painting",
         "a green tissue box", "a shoe", "a mirror", "a spray bottle",
         "a cardboard box", "a blue bin", "the desk", "a gold-framed painting",
         "a window", "a doorway", "a person", "a dark armchair",
         "a portrait painting"],
        ["a purple elephant", "a giraffe", "a grand piano", "a bicycle",
         "a motorcycle", "a surfboard", "a christmas tree", "a fire truck",
         "a horse", "an aquarium with fish", "a swimming pool", "a skateboard",
         "a bunch of bananas", "a traffic light", "a snowman", "a red sports car",
         "a palm tree", "a canoe", "a tractor", "a pizza", "a parrot",
         "a stack of car tyres", "a lion", "a hot air balloon", "a pumpkin",
         "a kangaroo", "a cactus", "a shark", "a drum kit", "a wedding cake"]),
    "fresh": (
        ["a chair", "an armchair", "a door", "a lamp", "a painting", "a carpet",
         "a cupboard", "a table", "a couch", "a picture frame", "a light fitting",
         "a computer keyboard", "a bin", "a box", "a tissue box", "a sneaker",
         "a cabinet", "a bed", "a ceiling light", "a painting of a cow",
         "a teal painted door", "an office chair", "a sofa", "a rug on the floor",
         "a black cupboard", "a picture on the wall"],
        ["a zebra", "a violin", "a lighthouse", "an anchor", "a rocket ship",
         "a penguin", "a submarine", "a dinosaur", "a windmill", "a tennis racket",
         "a fire hydrant", "a helicopter", "a camel", "a volcano", "a sailboat",
         "a hamburger", "a beehive", "a scarecrow", "a gorilla", "an igloo",
         "a jet ski", "a gondola", "a tiger", "a pineapple", "a harp",
         "a totem pole", "a lobster", "a goat", "an excavator", "a waterfall"]),
}


def recalled(answer: dict) -> dict | None:
    """`rover_recall._recall`'s walk: the first look of a thing above the floor."""
    floor = answer.get("floor")
    for match in answer.get("matches") or []:
        if not match.get("entity_id"):
            continue
        if floor is not None and float(match.get("score") or 0.0) < floor:
            break
        return match
    return None


def copied_store(source: str) -> tuple[WorldStore, str]:
    """A private copy of the store, taken through sqlite so a write the daemon
    is in the middle of arrives whole or not at all."""
    directory = tempfile.mkdtemp(prefix="bench-search-")
    live = sqlite3.connect(f"file:{os.path.join(source, 'world.db')}?mode=ro", uri=True)
    copy = sqlite3.connect(os.path.join(directory, "world.db"))
    live.backup(copy)
    copy.close()
    live.close()
    return WorldStore(directory), directory


def sheet(store, frames: str, entity_id: str, title: str, path: str) -> None:
    """A row of crops of one thing, spread across its looks, to judge it by eye."""
    from PIL import Image, ImageDraw

    looks = store.observations(entity_id, limit=10 ** 6)
    step = max(1, len(looks) // SHEET_CROPS)
    page = Image.new("RGB", (SHEET_CROPS * SHEET_TILE, SHEET_TILE + 16), (20, 20, 20))
    ImageDraw.Draw(page).text((4, 2), title, fill=(255, 255, 0))
    for slot, look in enumerate(looks[::step][:SHEET_CROPS]):
        try:
            image = Image.open(os.path.join(frames, f"{look['frame_id']}.jpg"))
        except OSError:
            continue
        width, height = image.size
        left, top, right, bottom = look.get("bbox") or (0.0, 0.0, 1.0, 1.0)
        crop = image.convert("RGB").crop((
            int(left * width), int(top * height),
            max(int(left * width) + 2, int(right * width)),
            max(int(top * height) + 2, int(bottom * height))))
        crop.thumbnail((SHEET_TILE, SHEET_TILE))
        page.paste(crop, (slot * SHEET_TILE, 16))
    page.save(path, quality=80)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--store", default=world_dir(),
                        help="directory holding world.db (default: the rover's)")
    parser.add_argument("--looks", action="store_true",
                        help="judge the look search find_thing used before 2026-10-13")
    parser.add_argument("--floors", default=None,
                        help="comma-separated floors to judge at (default: the "
                             "floor of the search being judged)")
    parser.add_argument("--keep-slivers", action="store_true",
                        help="rank slivers as the search did before 2026-10-05")
    parser.add_argument("--sheets", default=None,
                        help="write a contact sheet per phrase into this directory")
    arguments = parser.parse_args()
    if arguments.keep_slivers:
        search.sliver = lambda bbox: False
    floors = [float(one) for one in (arguments.floors or str(
        search.MATCHES if arguments.looks else search.THING_MATCHES)).split(",")]

    store, directory = copied_store(arguments.store)
    try:
        if arguments.looks:
            rows = store.searchable()
            print(f"{len(rows)} looks ranked, as a search is handed them")
            judge = lambda vector: recalled(search.rank(vector, rows,  # noqa: E731
                                                        limit=RECALL_LIMIT))
        else:
            index = search.ThingIndex(store)
            index.refresh()
            print(f"{len(index.averages())} things ranked by their average look")

            def judge(vector):
                answer = search.rank_things(vector, index, limit=1)
                best = (answer.get("matches") or [None])[0]
                return best if best and best["score"] >= search.THING_MATCHES else None
        return _judged(arguments, floors, judge, store)
    finally:
        store.close()
        shutil.rmtree(directory, ignore_errors=True)


def _judged(arguments, floors, judge, store) -> int:
    eyes = SidecarEyes()
    vectors = {}
    for name, (present, absent) in SETS.items():
        phrases = [phrase.lower() for phrase in present + absent]
        got, error = eyes.embed(phrases)
        if error or len(got) != len(phrases):
            print(f"the sidecar would not embed the phrases: {error}", file=sys.stderr)
            return 1
        vectors[name] = dict(zip(phrases, got))

    measured = search.MATCHES, search.THING_MATCHES
    try:
        for floor in floors:
            # The judge reads whichever of the two its search uses.
            search.MATCHES = search.THING_MATCHES = floor
            print(f"\nfloor {floor:.4f}, slivers "
                  f"{'ranked' if arguments.keep_slivers else 'counted out'}")
            found_all = missed_all = 0
            for name, (present, absent) in SETS.items():
                found, missed = [], []
                for phrase in present + absent:
                    match = judge(vectors[name][phrase.lower()])
                    if match and phrase in absent:
                        found.append(f"{phrase} ({match['score']:.3f}, "
                                     f"{match['entity_id']})")
                    if not match and phrase in present:
                        missed.append(phrase)
                    if arguments.sheets and match and floor == floors[0]:
                        os.makedirs(arguments.sheets, exist_ok=True)
                        sheet(store, os.path.join(arguments.store, "frames"),
                              match["entity_id"],
                              f"{phrase}: {match['entity_id']} {match['score']:.3f}",
                              os.path.join(arguments.sheets,
                                           phrase.replace(" ", "_") + ".jpg"))
                found_all += len(found)
                missed_all += len(missed)
                print(f"  {name}: {len(found)} of {len(absent)} absent found, "
                      f"{len(missed)} of {len(present)} present missed")
                if found:
                    print(f"    found:  {'; '.join(found)}")
                if missed:
                    print(f"    missed: {'; '.join(missed)}")
            print(f"  together: {found_all} absent found, {missed_all} present missed")
    finally:
        search.MATCHES, search.THING_MATCHES = measured
    return 0


if __name__ == "__main__":
    sys.exit(main())

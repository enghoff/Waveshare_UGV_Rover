"""A look taken to test a hypothesis: recorded fresh, its depth kept, read back whole.

Three small changes the check in `hypothesis_check.py` depends on. A look is
recorded even when the picture matches the last one, because the question is
about this moment; its depth map is kept even when nothing was ranged, because
depth past an empty place is the only absence the check accepts; and the map
carries the lens it was taken through, so it can be read without the camera.
"""
from __future__ import annotations

import tempfile
from types import SimpleNamespace

from test_harness import SKIP, check
from test_fakes import (JPEG, a_camera_showing, a_seeing_inspector, a_sighting,
                        a_store)
from test_inspect import _room
from world_state.depth_client import DepthMap, Ranged
from world_state.inspection_ranges import InspectionRanges

LENS = SimpleNamespace(fx=500.3, fy=500.17, cx=321.23, cy=190.69,
                       width=640, height=360)


def test_a_fresh_look_is_recorded_even_when_the_room_has_not_changed() -> None:
    same = _room()
    if same is None:
        SKIP.append("a fresh look past the unchanged gate (no numpy or OpenCV)")
        return
    with tempfile.TemporaryDirectory() as directory:
        store, eyes, inspector_ = a_seeing_inspector(
            directory, [[a_sighting()], [a_sighting()]],
            capture=a_camera_showing([same]))
        inspector_.inspect()
        again = inspector_.inspect(fresh=True)
        check("a look asked for fresh is recorded though the picture is the same",
              (again.get("unchanged"), again["stored"]), (None, 1))
        check("...and names the frame it kept", bool(again.get("frame_id")), True)
        store.close()


class _Keeper(InspectionRanges):
    def __init__(self, store, ranger):
        self.store, self.ranger = store, ranger


class _Ranger:
    def depth_map(self):
        return DepthMap(millimetres=bytes(8), width=2, height=2, dtype="uint16")

    def lens(self):
        return LENS


def test_a_check_look_keeps_its_depth_with_nothing_ranged() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        frame_id = store.save_frame(JPEG, 640, 480)
        keeper = _Keeper(store, _Ranger())
        nothing = [Ranged(absent="outside the depth camera's view")]
        check("an ordinary look with nothing ranged keeps no depth",
              keeper._keep_depth(frame_id, nothing), 0)
        check("...a look taken to test a hypothesis keeps it anyway",
              keeper._keep_depth(frame_id, nothing, always=True) > 0, True)
        _body, described = store.depth(frame_id)
        check("...with the lens it was taken through",
              described.get("lens"), {"fx": 500.3, "fy": 500.17, "cx": 321.23,
                                      "cy": 190.69, "width": 640.0,
                                      "height": 360.0})
        store.close()


def test_one_look_is_read_back_whole_with_its_vectors() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        store.record([a_sighting(), a_sighting()], capture={"frame_id": "f1"},
                     region_source="yoloe")
        store.record([a_sighting()], capture={"frame_id": "f2"},
                     region_source="yoloe")
        rows = store.observations(frame_id="f1", vectors=True)
        check("every region of the one look comes back", len(rows), 2)
        check("...with its appearance vector, for a caller that compares them",
              all(isinstance(row.get("dino_blob"), bytes) for row in rows), True)
        check("...while an ordinary read still leaves the bytes out",
              "dino_blob" in store.observations(frame_id="f1")[0], False)
        store.close()


TESTS = (test_a_fresh_look_is_recorded_even_when_the_room_has_not_changed,
         test_a_check_look_keeps_its_depth_with_nothing_ranged,
         test_one_look_is_read_back_whole_with_its_vectors)

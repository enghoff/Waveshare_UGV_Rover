"""Whether one look shows something where a claim says it stands.

Driven with a made-up depth map of a single distance everywhere, which is enough
to put the place in front of a wall, on a surface or behind something nearer,
and with looks whose regions are placed by hand. What the made-up map cannot
test is whether real depth looks like this; that is the replay's job.
"""
from __future__ import annotations

from types import SimpleNamespace

from test_harness import check
from world_state import hypothesis_check as hc
from world_state import oak

LENS = SimpleNamespace(fx=500.3, fy=500.17, cx=321.23, cy=190.69,
                       width=640, height=360)
WIDTH, HEIGHT = 320, 180
#: After the OAK went onto the gimbal rail, so the projection takes that mount.
AT = oak.RAIL_SINCE + 1000.0


def depth_of(metres: float | None, width: int = WIDTH, height: int = HEIGHT):
    """A depth map reading `metres` everywhere, or nothing (zero) everywhere."""
    value = 0 if metres is None else int(round(metres * 1000))
    body = value.to_bytes(2, "little") * (width * height)
    return body, {"width": width, "height": height, "unit": "mm",
                  "dtype": "uint16"}


def region(bearing: float, *, range_m: float | None = None,
           elevation: float = 0.0, span: float = 8.0, tilt: float = 0.0,
           at: float = AT, ident: int = 1) -> dict:
    return {"id": ident, "frame_id": "f1", "bearing_deg": bearing,
            "span_deg": span, "bearing_sigma_deg": 1.5,
            "elevation_deg": elevation, "elevation_span_deg": 8.0,
            "range_m": range_m, "range_sigma_m": 0.05,
            "pose": {"x_m": 0.0, "y_m": 0.0, "heading_deg": 0.0},
            "observer_pan_deg": 0.0, "observer_tilt_deg": tilt,
            "observed_at": at, "camera": "gimbal"}


CLAIM = {"x_m": 1.5, "y_m": 0.0, "height_m": 0.0, "uncertainty_m": 0.2,
         "height_sigma_m": 0.1}
POSE = {"x_m": 0.0, "y_m": 0.0, "heading_deg": 0.0, "pan_deg": 0.0,
        "tilt_deg": 0.0, "observed_at": AT}


def test_a_ranged_region_on_the_line_supports_the_claim() -> None:
    got = hc.check([region(1.0, range_m=1.52)], CLAIM, depth_of(1.5), LENS)
    check("a region on the line, ranged where the place is, supports it",
          (got["outcome"], got["code"]), (hc.SUPPORTED, "ranged"))
    got = hc.check([region(1.0, range_m=2.4)], CLAIM, depth_of(1.5), LENS)
    check("...ranged a metre further on, it does not",
          got["outcome"], hc.UNRESOLVED)
    got = hc.check([region(25.0, range_m=1.5)], CLAIM, depth_of(1.5), LENS)
    check("...nor does one at the right range in another direction",
          got["outcome"], hc.UNRESOLVED)
    got = hc.check([region(1.0, range_m=1.6, elevation=30.0)], CLAIM,
                   depth_of(1.6), LENS)
    check("...nor one at the right range far above the place",
          got["outcome"], hc.UNRESOLVED)


def test_depth_past_the_place_contradicts_it() -> None:
    got = hc.check([], CLAIM, depth_of(3.5), LENS, pose=POSE)
    check("a wall two metres behind the place, and nothing nearer, is a "
          "contradiction", (got["outcome"], got["code"]),
          (hc.CONTRADICTED, "seen through"))
    check("...and the evidence says how far the nearest surface was",
          got["evidence"]["depth"]["nearest_m"], 3.5)
    got = hc.check([region(1.0)], CLAIM, depth_of(3.5), LENS)
    check("an unranged region on the line does not stop it",
          got["outcome"], hc.CONTRADICTED)


def test_what_cannot_be_answered_is_unresolved() -> None:
    cases = {
        "a surface where the place is, with no region on it":
            (hc.check([], CLAIM, depth_of(1.5), LENS, pose=POSE), "surface"),
        "something nearer in the way":
            (hc.check([], CLAIM, depth_of(0.8), LENS, pose=POSE), "occluded"),
        "no depth measured at all":
            (hc.check([], CLAIM, depth_of(None), LENS, pose=POSE), "sparse"),
        "no depth kept with the look":
            (hc.check([], CLAIM, (None, "no depth map kept"), LENS, pose=POSE),
             "none"),
        "a place off to the side, outside the depth camera's view":
            (hc.check([], dict(CLAIM, y_m=1.5), depth_of(3.5), LENS, pose=POSE),
             "outside"),
        "a place below the camera seen at twenty up":
            (hc.check([], dict(CLAIM, height_m=-0.2), depth_of(3.5), LENS,
                      pose=dict(POSE, tilt_deg=20.0)), "outside"),
        "a claim placed too loosely to test":
            (hc.check([], dict(CLAIM, uncertainty_m=0.8), depth_of(3.5), LENS,
                      pose=POSE), "too loose"),
    }
    for name, (got, code) in cases.items():
        check(name, (got["outcome"], got["code"]), (hc.UNRESOLVED, code))


def test_a_look_with_no_direction_cannot_answer() -> None:
    withheld = region(1.0, range_m=1.5)
    withheld["bearing_deg"] = None
    got = hc.check([withheld], CLAIM, depth_of(3.5), LENS)
    check("a look whose bearings were withheld is unresolved, not contradicted",
          (got["outcome"], got["code"]), (hc.UNRESOLVED, "direction"))
    got = hc.check([], CLAIM, depth_of(3.5), LENS, pose=None,
                   withheld="the heading could not be checked against the map")
    check("...and so is one the inspector said why for",
          (got["outcome"], got["code"]), (hc.UNRESOLVED, "direction"))
    check("...with the reason in the sentence",
          "checked against the map" in got["why"], True)


def test_a_claim_with_no_height_can_be_supported_but_not_refuted() -> None:
    claim = {"x_m": 1.5, "y_m": 0.0, "uncertainty_m": 0.2}
    got = hc.check([], claim, depth_of(3.5), LENS, pose=POSE)
    check("with no height the patch reaches the floor, and depth past the place "
          "is not enough", got["outcome"], hc.UNRESOLVED)
    got = hc.check([region(1.0, range_m=1.5)], claim, depth_of(3.5), LENS)
    check("...while a ranged region still supports it", got["outcome"],
          hc.SUPPORTED)


def test_the_lens_can_come_with_the_depth() -> None:
    body, described = depth_of(3.5)
    described["lens"] = {"fx": 500.3, "fy": 500.17, "cx": 321.23,
                         "cy": 190.69, "width": 640, "height": 360}
    got = hc.check([], CLAIM, (body, described), None, pose=POSE)
    check("a depth map saved with its lens needs no camera to be read",
          got["outcome"], hc.CONTRADICTED)
    got = hc.check([], CLAIM, depth_of(3.5), None, pose=POSE)
    check("...and one saved without it, and no lens given, abstains",
          (got["outcome"], got["code"]), (hc.UNRESOLVED, "none"))


def test_appearance_is_recorded_and_decides_nothing() -> None:
    import struct

    def vector(*values):
        return struct.pack(f"<{len(values)}f", *values)

    look = [dict(region(1.0, range_m=1.5), dino_blob=vector(1.0, 0.0))]
    source = [{"id": 9, "frame_id": "f0", "dino_blob": vector(0.0, 1.0)}]
    got = hc.check(look, CLAIM, depth_of(1.5), LENS, source=source)
    check("a region that looks nothing like the claim's looks still supports "
          "the place", got["outcome"], hc.SUPPORTED)
    check("...and how unlike it is is written down",
          got["evidence"]["on_the_line"][0]["looks_like_claim"], 0.0)


TESTS = (test_a_ranged_region_on_the_line_supports_the_claim,
         test_depth_past_the_place_contradicts_it,
         test_what_cannot_be_answered_is_unresolved,
         test_a_look_with_no_direction_cannot_answer,
         test_a_claim_with_no_height_can_be_supported_but_not_refuted,
         test_the_lens_can_come_with_the_depth,
         test_appearance_is_recorded_and_decides_nothing)

"""Attach OAK ranges to regions, including cross-camera geometry and frame age."""
from __future__ import annotations

import math
from typing import Any

from . import oak, view

# Redraw projected boxes when the first measured range differs by over 40%.
REASK_RANGE_FRAC = 0.40


class InspectionRanges:
    """Range methods for an inspector with a ranger supplied by its caller."""

    def _ranges(self, capture: dict[str, Any], regions: list):
        """How far away each region is, and one clause for the diagnostics line.

        `([], "")` whenever the question cannot be asked, which is most of the
        time and is not a failure: no depth camera on this rover, a mount nobody
        has measured, a service that is restarting, or a gimbal turned to look at
        something the OAK cannot see. Everything downstream treats a missing
        range as abstention.

        **Two shapes, because the two cameras stand differently to the depth
        map.** A look taken through the OAK is already in the depth map's own
        frame -- the depth is warped into the colour camera's geometry on the
        device -- so a box goes straight across. A look taken through the gimbal
        is a box on a different lens on a mount that turns, so each box becomes
        four directions in the rover's frame and `oak.box_for` finds where those
        land in the OAK's picture, if they land in it at all. About half of a
        centred gimbal frame does, and a look taken over the rover's shoulder
        does not.
        """
        if self.ranger is None or not regions:
            return [], ""
        camera = capture.get("camera") or oak.GIMBAL
        try:
            if camera == oak.OAK:
                return self._ranges_here(capture, regions)
            return self._ranges_across(capture, regions)
        except Exception as error:                 # never past here
            return [], f"no ranges ({type(error).__name__}: {error})"

    def _ranges_here(self, capture: dict[str, Any], regions: list):
        """Ranges for boxes already drawn on the depth camera's own picture."""
        answers, error = self.ranger.ranges([list(region.bbox)
                                             for region in regions])
        if error:
            return [], f"no ranges ({error})"
        speed = capture.get("speed_mps") or 0.0
        got = 0
        for one in answers:
            if one is None or one.range_m is None:
                continue
            one.sigma_m = self._aged_sigma(one, speed)
            got += 1
        return answers, (f"{got} of {len(regions)} ranged"
                         if got else "nothing in the frame could be ranged")

    @staticmethod
    def _aged_sigma(one, speed_mps: float) -> float:
        """What a range is worth once its own staleness is charged to it.

        **A range is true of where the camera was when the frame was taken.** The
        depth camera holds each frame back until the picture it belongs with has
        come through the encoder, so a reading is always older than the moment it
        is read at, and on a rover exploring at 0.47 m/s that age is distance
        against a stereo error of two to seven centimetres at these ranges.
        Ignoring it would make the world state trust a stale range far more than
        a fresh one deserves.

        The age itself is read off the reply and never assumed here, which is
        what let the camera's rate go from 2 fps to 15 on 2026-09-04 without
        touching this: the hold-back was half a second of it at the old rate and
        is 67 ms at the new one, and this arithmetic did not need to know.

        Added in quadrature with what the camera said the reading was worth, the
        same way `Inspector._where` adds the turn to the bearing: the two are
        independent, one is the camera's and one is the rover's.

        It is a *widening* and never a narrowing -- a rover standing still adds
        nothing -- so being wrong optimistic about the speed only costs precision.
        """
        camera = float(one.sigma_m or 0.0)
        stale = max(0.0, float(speed_mps)) * max(0.0, float(one.age_s or 0.0))
        return round(math.hypot(camera, stale), 3)

    def _ranges_across(self, capture: dict[str, Any], regions: list):
        """Ranges for boxes drawn on the gimbal camera, found in the OAK's picture.

        **The offset between the two cameras is what makes this more than a
        rotation.** They sit a few centimetres apart, so they see a thing two
        metres away in slightly different directions and how different depends on
        how far away it is -- which is the thing being asked. The box is worked
        out at `oak.GUESS_RANGE_M`, and any answer that comes back a long way
        from that guess is asked again from where it now appears to be. One extra
        loopback call, and only for the near things where the parallax is worth
        correcting: at two metres a wrong guess of half a metre moves the box by
        about four pixels, and at sixty centimetres it moves it by forty.
        """
        try:
            lens = self.ranger.lens()
        except Exception:                          # never past here
            lens = None
        if lens is None or not oak.MEASURED:
            return [], ""
        size = capture.get("frame_size")
        pan = capture.get("pan") or 0.0
        tilt = capture.get("tilt") or 0.0
        corners: list[Any] = []
        boxes: list[Any] = []
        for region in regions:
            found = self._corners_of(region.bbox, pan, tilt, size)
            corners.append(found)
            boxes.append(None if found is None else oak.box_for(found, lens))
        asked = [index for index, box in enumerate(boxes) if box is not None]
        if not asked:
            return [], "none of it was in the depth camera's picture"
        answers, error = self.ranger.ranges([boxes[index] for index in asked])
        if error:
            return [], f"no ranges ({error})"
        found: list[Any] = [None] * len(regions)
        again: list[int] = []
        for slot, index in enumerate(asked):
            if slot >= len(answers):
                break
            one = answers[slot]
            found[index] = one
            if one is not None and one.range_m is not None and (
                    abs(one.range_m - oak.GUESS_RANGE_M)
                    > REASK_RANGE_FRAC * oak.GUESS_RANGE_M):
                again.append(index)
        if again:
            self._reask(again, corners, found, lens)
        return self._as_gimbal(corners, found, len(regions),
                               capture.get("speed_mps") or 0.0)

    def _reask(self, again, corners, found, lens) -> None:
        """Ask a second time for the boxes whose range was nothing like the guess.

        Silent on failure and deliberately so: the first answer is already in
        hand and is at worst a few pixels off, so a second call that does not
        come back leaves a slightly worse number rather than none at all.
        """
        redrawn = []
        for index in again:
            placed = oak.box_for(corners[index], lens, found[index].range_m)
            if placed is not None:
                redrawn.append((index, placed))
        if not redrawn:
            return
        answers, error = self.ranger.ranges([box for _index, box in redrawn])
        if error:
            return
        for slot, (index, _box) in enumerate(redrawn):
            if slot >= len(answers):
                break
            one = answers[slot]
            if one is not None and one.range_m is not None:
                found[index] = one

    def _as_gimbal(self, corners, found, total: int, speed_mps: float = 0.0):
        """The OAK's ranges, as lengths along the rays they will be stored against.

        **A range is a length along a particular ray from a particular point**,
        and these were measured from the other camera. The observation's ray
        starts at the gimbal camera, so an OAK range put on it unchanged would be
        a few centimetres wrong in a way that grows as things get closer -- and
        `locate` would then spend it against a crossing measured from somewhere
        else. `oak.range_from_gimbal` is the correction, run with the range that
        actually came back rather than with the guess the box was drawn at.
        """
        ranged = 0
        for index, one in enumerate(found):
            if one is None or one.range_m is None:
                continue
            corrected = oak.range_from_gimbal(corners[index], one.range_m)
            if corrected is None or corrected <= 0.0:
                found[index] = None
                continue
            one.range_m = round(corrected, 3)
            one.sigma_m = self._aged_sigma(one, speed_mps)
            ranged += 1
        return found, (f"{ranged} of {total} ranged by the depth camera"
                       if ranged else
                       "the depth camera saw none of it well enough to range")

    @staticmethod
    def _corners_of(bbox, pan_deg: float, tilt_deg: float, size):
        """A box on the gimbal camera as four directions in the rover's frame.

        None when the box is unusable or the lens cannot be reached, which is the
        same silence everything else here keeps. Four corners rather than a
        centre because what the depth camera is asked for is an area of its own
        picture, and the two lenses do not agree about shape: a box near the edge
        of a 130-degree fisheye maps to a very different rectangle on a pinhole.
        """
        if not isinstance(bbox, (list, tuple)) or len(bbox) != 4:
            return None
        try:
            left, top, right, bottom = (float(value) for value in bbox)
        except (TypeError, ValueError):
            return None
        found = []
        for x_frac, y_frac in ((left, top), (right, top),
                               (left, bottom), (right, bottom)):
            direction = view.chassis_direction(x_frac, y_frac, pan_deg, tilt_deg,
                                               size)
            if direction is None:
                return None
            found.append(direction)
        return found

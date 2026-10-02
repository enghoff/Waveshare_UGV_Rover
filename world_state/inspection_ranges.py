"""Attach OAK ranges to regions, including cross-camera geometry and frame age."""
from __future__ import annotations

import math
import time
from typing import Any

from . import oak, outline
from .depth_client import (NO_DEPTH_ANSWER, NOTHING_TO_MEASURE, OUTSIDE_VIEW,
                           TURNING, Ranged)

#: What the store's `range_from` calls a range the depth service read from a box.
SERVICE = "service"

# Redraw projected boxes when the first measured range differs by over 40%.
REASK_RANGE_FRAC = 0.40

#: How far the rover may have turned between the picture and the depth frame
#: before a range is dropped. The depth frame is fetched after the encoders,
#: a third of a second or more after the shutter. On 2026-10-01 a tissue box 0.95 m
#: away read 2.61 m from a look taken while the rover turned 54.5 degrees across
#: the shutter: about 17 degrees of turning separated the two frames, and the box
#: landed on the furniture behind it, which then placed the box from that one look
#: 1.77 m out. A degree is a fifth of the narrowest box that is ever ranged.
RANGE_TURN_LIMIT_DEG = 1.0

#: How far from the picture a depth frame may have been taken and still count as of
#: the picture's moment, in seconds: a frame and a half at the 15 fps it runs at.
#: Further than that the service's history did not reach, and the old rule applies.
MATCHED_WITHIN_S = 0.10

#: How well the moment of the gimbal camera's picture is known, in seconds -- the
#: same figure as `inspector.FRAME_TIME_SIGMA_S`, and for the same reason: its bias
#: is one frame interval at most and nothing on this rover can measure it. **This is
#: what is left once the depth frame is asked for at the shutter and the region is
#: turned by the rover's turn between the two**: a range read that way is dropped
#: only when the turn rate times this exceeds `RANGE_TURN_LIMIT_DEG`, which is 33
#: degrees a second rather than any turning at all. On 2026-10-02 the old rule
#: dropped 347 regions' ranges in a six-minute drive.
SHUTTER_UNKNOWN_S = 0.03


class InspectionRanges:
    """Range methods for an inspector with a ranger supplied by its caller."""

    def _ranges(self, capture: dict[str, Any], regions: list):
        """How far away each region is, and one clause for the diagnostics line.

        `([], "")` whenever the question cannot be asked, which is most of the
        time and is not a failure: no depth camera on this rover, a mount nobody
        has measured, a service that is restarting, or a region out near the
        fisheye's edge where the OAK cannot see. Everything downstream treats a
        missing range as abstention.

        **Two shapes, because the two cameras stand differently to the depth
        map.** A look taken through the OAK is already in the depth map's own
        frame -- the depth is warped into the colour camera's geometry on the
        device -- so a box goes straight across. A look taken through the gimbal
        is a box on a different lens a few centimetres away, so each box becomes
        four directions in the gimbal camera's own frame and `oak.box_for` finds
        where those land in the OAK's picture, if they land in it at all. The OAK
        rides the same platform, so the middle of every fisheye picture has depth
        behind it wherever the gimbal points.
        """
        # The depth map a look's ranges were read from, when it read one itself, so
        # that the map kept beside the frame is that one rather than the next.
        self._depth_read = None
        if self.ranger is None or not regions:
            return [], ""
        camera = capture.get("camera") or oak.GIMBAL
        try:
            if camera == oak.OAK:
                found, note = self._ranges_here(capture, regions)
            else:
                found, note = self._ranges_across(capture, regions)
        except Exception as error:                 # never past here
            return [], f"no ranges ({type(error).__name__}: {error})"
        dropped = self._drop_turned(found, capture)
        if dropped:
            note = (f"{note}; {dropped} dropped because the rover was turning"
                    if note else f"{dropped} ranges dropped because the rover "
                                 f"was turning")
        return found, note

    def _drop_turned(self, found, capture, now=None) -> int:
        """Drop the ranges whose depth frame the rover had turned away from.

        Two measures of the turn, and the larger counts. One is how far the
        rover's heading moved between the shutter and now, which bounds any depth
        frame read in between. The other is the turn rate across the shutter
        times how far the depth frame stood from the picture, which covers a frame
        older than the shutter. A rover standing still loses nothing.
        """
        ranged = [i for i, one in enumerate(found or [])
                  if one is not None and one.range_m is not None]
        if not ranged:
            return 0
        rate = float(capture.get("turn_dps") or 0.0)
        # A frame asked for at the shutter, with the region turned by the turn
        # between the two: what is left is the shutter's own unknown moment.
        matched = [index for index in ranged
                   if getattr(found[index], "off_s", None) is not None]
        ranged = [index for index in ranged if index not in matched]
        dropped = 0
        if rate * SHUTTER_UNKNOWN_S > RANGE_TURN_LIMIT_DEG:
            for index in matched:
                found[index] = Ranged(absent=TURNING, age_s=found[index].age_s)
                dropped += 1
        if not ranged:
            return dropped
        now = time.time() if now is None else now
        turned = 0.0
        then = capture.get("shutter_heading_deg")
        if then is not None:
            pose = self._pose() if hasattr(self, "_pose") else None
            if isinstance(pose, dict) and pose.get("heading_deg") is not None:
                turned = abs((float(pose["heading_deg"]) - float(then) + 180.0)
                             % 360.0 - 180.0)
        taken = capture.get("taken_at")
        for index in ranged:
            one = found[index]
            swung = turned
            if rate and taken is not None and one.age_s is not None:
                apart = abs((now - float(one.age_s)) - float(taken))
                swung = max(swung, rate * apart)
            if swung > RANGE_TURN_LIMIT_DEG:
                found[index] = Ranged(absent=TURNING, age_s=one.age_s)
                dropped += 1
        return dropped

    def _ranges_here(self, capture: dict[str, Any], regions: list):
        """Ranges for boxes already drawn on the depth camera's own picture."""
        answers, error = self.ranger.ranges([list(region.bbox)
                                             for region in regions])
        if error:
            return [], f"no ranges ({error})"
        speed = capture.get("speed_mps") or 0.0
        got = 0
        for index, one in enumerate(answers):
            if one is None:
                answers[index] = Ranged(absent=NO_DEPTH_ANSWER)
                continue
            if one.range_m is None:
                # A box on this camera's own picture is inside its view by
                # construction, so the only silence available here is a surface
                # it could not measure.
                one.absent = one.absent or NOTHING_TO_MEASURE
                continue
            one.sigma_m = self._aged_sigma(one, speed)
            one.method = SERVICE
            got += 1
        return answers, (f"{got} of {len(regions)} ranged"
                         if got else "nothing in the frame could be ranged")

    def _keep_depth(self, frame_id: str, ranges, always: bool = False) -> int:
        """Save the depth map behind this look, where there was one.

        **Only where a range was actually measured**, which is the cheap and
        correct condition: a parked rover with the camera switched off, a look
        taken over the rover's shoulder, and a board with no depth camera at all
        each ask for nothing and store nothing. What it costs when it does fire
        is one loopback fetch of half a megabyte and a gzip, against a look
        already spending half a second.

        `always` is a look taken to test a hypothesis, which keeps its depth
        whether or not anything was ranged: depth measured past an empty place is
        the only evidence of absence the check will accept. The lens goes in with
        it, so that the map can be projected into later without the camera.

        Returns the bytes written, zero for every ordinary reason. Nothing here
        may raise: a look whose evidence could not be kept is still a look.
        """
        if self.ranger is None or not frame_id:
            return 0
        if not always and not any(one is not None and one.range_m is not None
                                  for one in (ranges or [])):
            return 0
        try:
            try:
                lens = self.ranger.lens()
            except Exception:                      # never past here
                lens = None
            # The map the ranges were read from, where the look read one; the
            # newest otherwise, which is the frame after the one the service read.
            depth = getattr(self, "_depth_read", None) or self.ranger.depth_map()
            return self.store.save_depth(frame_id, depth, lens=lens)
        except Exception:                          # never past here
            return 0

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
        correcting: with the OAK five centimetres above the fisheye, a guess of
        2.5 m for a thing at two metres puts the box about three pixels out, and
        for one at sixty centimetres about thirty.
        """
        try:
            lens = self.ranger.lens()
        except Exception:                          # never past here
            lens = None
        if lens is None or not oak.MEASURED:
            return [], ""
        size = capture.get("frame_size")
        read = self._ranges_read(capture, regions, lens, size)
        if read is not None:
            return read
        corners: list[Any] = []
        boxes: list[Any] = []
        for region in regions:
            found = self._corners_of(region.bbox, size)
            corners.append(found)
            boxes.append(None if found is None else oak.box_for(found, lens))
        asked = [index for index, box in enumerate(boxes) if box is not None]
        # Every region that never reached the camera says so on its own row,
        # rather than being indistinguishable from one the camera looked at and
        # found nothing in. A thing only ever seen out here can never be ranged
        # however often the rover looks, and that is worth being able to report.
        outside = [Ranged(absent=OUTSIDE_VIEW) if box is None else None
                   for box in boxes]
        if not asked:
            return outside, "none of it was in the depth camera's picture"
        answers, error = self.ranger.ranges([boxes[index] for index in asked])
        if error:
            return ([one or Ranged(absent=NO_DEPTH_ANSWER) for one in outside],
                    f"no ranges ({error})")
        found: list[Any] = list(outside)
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

    def _ranges_read(self, capture: dict[str, Any], regions: list, lens, size):
        """Every region's range read here from one depth map, or None to ask the service.

        **Under the region's own outline, and from its box only where the outline
        cannot say.** The service answers a box with its nearest surface, which is a
        chair whenever a chair stands in front of the painting the box is drawn
        round: on 2026-10-02 that read 1.22 m for a painting 2.54 m away. The
        outline is the region finder's own record of which pixels are the painting.
        See `outline` for what the replay of four drives measured.

        One depth map for the whole look, fetched once and kept beside the frame by
        `_keep_depth`, so the evidence kept is the map the ranges came from -- the
        service's answers came from whatever frame was newest when it was asked,
        and the map kept was the one after it.

        None when there is no numpy on this host, no frame size, or no depth map to
        be had; the service's own box reading is asked for then, as before.
        """
        if not size:
            return None
        try:
            import numpy as np
        except ImportError:                        # a bench without numpy
            return None
        # The frame taken nearest the picture, where the service keeps a history and
        # the camera stamped its picture; the newest otherwise.
        taken = capture.get("taken_at")
        try:
            try:
                depth = (self.ranger.depth_map(at=float(taken)) if taken
                         else self.ranger.depth_map())
            except TypeError:                      # a ranger that takes no moment
                depth = self.ranger.depth_map()
        except Exception:                          # never past here
            return None
        if not depth.ok or (depth.dtype and depth.dtype != "uint16"):
            return None
        # A map is projected into as the lens's own picture, scaled, so one of another
        # shape is not this lens's picture and nothing here can say where in it a
        # region lands.
        if abs(depth.width / float(depth.height)
               - float(lens.width) / float(lens.height)) > 0.05:
            return None
        # How far the rover turned between the picture and that frame, from the turn
        # rate across the shutter, when the frame is of the picture's moment.
        off = getattr(depth, "off_s", None)
        if off is not None and (not taken or abs(off) > MATCHED_WITHIN_S):
            off = None
        turn_deg = float(capture.get("turn_rate_dps") or 0.0) * off if off else 0.0
        try:
            answers = outline.read(np, depth.millimetres, depth.width, depth.height,
                                   lens, [(region.bbox, getattr(region, "outline", b""))
                                          for region in regions],
                                   tuple(size), turn_deg=turn_deg,
                                   pan_deg=capture.get("pan"),
                                   tilt_deg=capture.get("tilt"))
        except Exception:                          # never past here
            return None
        self._depth_read = depth
        speed = capture.get("speed_mps") or 0.0
        found, ranged, outlined, blind = [], 0, 0, 0
        for answer in answers:
            if answer.get("range_m") is None:
                found.append(Ranged(absent=answer.get("absent") or NOTHING_TO_MEASURE,
                                    age_s=depth.age_s))
                blind += answer.get("absent") == OUTSIDE_VIEW
                continue
            one = Ranged(range_m=answer["range_m"], sigma_m=answer["sigma_m"],
                         valid=answer["valid"], pixels=answer["pixels"],
                         age_s=depth.age_s, apart_s=depth.apart_s,
                         method=answer["method"], off_s=off)
            one.sigma_m = self._aged_sigma(one, speed)
            found.append(one)
            ranged += 1
            outlined += answer["method"] == outline.OUTLINE
        if ranged:
            note = f"{ranged} of {len(regions)} ranged by the depth camera"
            if outlined < ranged:
                note += f" ({ranged - outlined} from the box)"
            if blind:
                note += f", {blind} outside its view"
        else:
            note = "the depth camera saw none of it well enough to range"
            if blind:
                note += f" ({blind} of them outside its view)"
        return found, note

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
            if one is None:
                found[index] = Ranged(absent=OUTSIDE_VIEW)
                continue
            if one.range_m is None:
                if not one.absent:
                    one.absent = NOTHING_TO_MEASURE
                continue
            corrected = oak.range_from_gimbal(corners[index], one.range_m)
            if corrected is None or corrected <= 0.0:
                # A range shorter than the two lenses are apart describes nothing
                # the gimbal camera could have been looking at, so it is dropped
                # -- and saying which of the three silences this is matters as
                # much here as anywhere.
                found[index] = Ranged(absent=NOTHING_TO_MEASURE)
                continue
            one.range_m = round(corrected, 3)
            one.sigma_m = self._aged_sigma(one, speed_mps)
            one.method = SERVICE
            ranged += 1
        blind = sum(1 for one in found if one.absent == OUTSIDE_VIEW)
        note = (f"{ranged} of {total} ranged by the depth camera"
                if ranged else
                "the depth camera saw none of it well enough to range")
        if blind:
            # The one number the acceptance drive of 2026-09-07 wanted and could
            # not have: how much of what the rover just looked at was somewhere
            # this camera cannot see at all.
            note += (f", {blind} outside its view"
                     if ranged else f" ({blind} of them outside its view)")
        return found, note

    @staticmethod
    def _corners_of(bbox, size):
        """A box on the gimbal camera as four directions in that camera's frame.

        None when the box is unusable or the lens cannot be reached, which is the
        same silence everything else here keeps. Four corners rather than a
        centre because what the depth camera is asked for is an area of its own
        picture, and the two lenses do not agree about shape: a box near the edge
        of a 130-degree fisheye maps to a very different rectangle on a pinhole.

        **At pan 0 and tilt 0 whatever the gimbal was doing**, which is what makes
        these directions in the camera's own frame rather than the rover's: the
        OAK turns with the gimbal, so where a fisheye pixel lands in its picture
        does not depend on the servos -- and their pointing errors cannot move it.
        """
        return outline.corners_of(bbox, size)

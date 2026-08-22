#!/usr/bin/env python3
"""The driver board's gyro and wheel counts, read two ways.

The board has streamed a 9-DoF IMU and wheel odometry down `/dev/ttyAMA0` this
whole time and nothing has ever read them. This is what reads them, and it does
two quite different jobs with the same numbers -- worth separating up front,
because they have opposite requirements and only one of them is blocked on a
measurement nobody has taken.

**As a prior** the gyro needs its scale factor. `slam2d_set_prior` centres the
search window on where the rover thinks it went, which matters exactly when the
rover is moving faster than the window is wide: measured in `selftest`, driving
30 cm a revolution, the matcher ends 3 m out with no prior and exact with one.
But a scale factor that is wrong drags the window off true on *every* revolution,
which is worse than a window that is merely centred on standing still. So the
prior stays at zero until the factors have been measured, and this refuses to
guess them.

**As a witness** the gyro needs no scale at all, and this is the more valuable of
the two. The match score cannot detect a scan that has snapped onto the
wrong-but-self-consistent alignment -- scoring high is precisely why that pose
won -- and the map then gets stamped from the bad pose, after which the map
agrees with the error and the score recovers. That is the failure that welds a
second copy of the room in at an angle. An uncalibrated gyro still says,
unambiguously, whether the chassis physically rotated at all, and a matcher
claiming nine degrees of yaw while the chassis sat still is a claim with an
independent witness against it. It is the only witness on this rover that is not
the thing under suspicion.

What the witness *does* need is a noise floor: how much the resting gyro wanders,
in the board's own units, over a span the length of one lidar revolution. That is
measurable with the rover standing still, so it costs nothing and needs no drive.
The sign convention -- whether a positive `gz` is a left turn or a right one --
is not known either, and cannot be learnt standing still, so the sign test stays
dark until the first confirmed turn establishes it. The magnitude test, which is
the one that catches the dangerous case, works from the first stationary second.

Both scale factors calibrate themselves out of moves the rover makes anyway. A
confirmed `turn_in_place` knows how far the matcher says the rover turned, and
this knows what the gyro integrated over the same span; the ratio is the scale.
A confirmed straight drive does the same for the wheels. So the numbers arrive by
driving rather than by ceremony, and `estimate` says how well they agree before
anything is committed.
"""
from __future__ import annotations

import json
import math
import os
import time

#: A span shorter than this is not evidence of anything -- two telemetry lines
#: arrive about 50 ms apart, so a span of a few milliseconds holds no samples and
#: dividing by it turns rounding into a rate.
MIN_SPAN_S = 0.04
#: Longer than this and the loop stalled rather than ran slowly. The gyro integral
#: is still honest across it -- the reader keeps its own account of gaps -- but it
#: is not a revolution's worth of motion and should not teach the noise floor.
MAX_SPAN_S = 1.0

#: How many resting spans before the bias is worth using. At 10 Hz this is about
#: three seconds of standing still, which the rover does constantly.
REST_SPANS_WANTED = 30
#: The bias and the noise floor are both exponential averages rather than windows,
#: because the thing they track drifts with temperature over minutes and a window
#: long enough to be quiet is long enough to be stale.
REST_ALPHA = 0.02

#: How many noise floors away from rest counts as the chassis having rotated. Six
#: is deliberately far: the cost of a false "it turned" is a wrongly-trusted match,
#: the cost of a false "it sat still" is a wrongly-*distrusted* one, and the second
#: only costs a recovery search. The floor itself is an RMS over resting spans, so
#: six of them is a rate the resting sensor essentially never reaches.
ROTATION_SIGMAS = 6.0
#: Below this the noise floor is not believed -- a sensor reading identically at
#: rest would otherwise make every twitch significant. One LSB is the quantisation,
#: so this is the smallest spread that is physically meaningful.
MIN_NOISE_LSB = 1.0

#: How much heading the matcher may accumulate across revolutions where the gyro
#: said the chassis never turned, before that is a contradiction in itself. This is
#: the slow half of the witness and the harder one to see: a match that creeps two
#: degrees a revolution never trips the per-revolution bar and is twenty degrees
#: wrong in ten seconds. Well clear of the random walk of an honest match, whose
#: revolution-to-revolution noise is a few tenths of a degree and cancels.
QUIET_CREEP_DEG = 15.0
#: And not from a handful of revolutions, where that walk has not averaged out.
QUIET_CREEP_SPANS = 8

#: Turns and drives smaller than these teach the scale factors nothing worth
#: having: the matcher's own few-millimetre, few-tenths-of-a-degree noise is a
#: large fraction of them, and a ratio of two small noisy numbers is noise.
MIN_TURN_DEG = 20.0
MIN_DRIVE_M = 0.30
#: Enough agreeing moves to hand a scale factor over.
CALIBRATION_WANTED = 3
#: And enough *variety* among them, which turned out to matter more than the count.
#: Three 175-degree turns in the same direction fitted a scale of 14.66 with a 2%
#: internal spread, against 16.37 from fifteen turns of mixed size and direction --
#: so the spread was measuring how much three near-identical moves agree with each
#: other, which is not the same question as whether they are right. A fit is only
#: published once its moves either go both ways or differ in size by this much.
CALIBRATION_SPAN_RATIO = 1.8


def _round(value, places=3):
    return None if value is None else round(value, places)


def _default_store():
    """Where the measured scale factors live between runs.

    The parent of this directory, which is `~/ugv` on the rover and the repository
    root at a desk. Deliberately *not* inside `lidar_slam/`, because the deploy in
    CLAUDE.md is `scp lidar_slam/*.py`, and a measurement that a deploy can
    overwrite is a measurement that will be taken twice.
    """
    return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        "odometry.json")


class Span:
    """What the board reported between two marks, in the board's own units.

    `intact` is the one to read before believing any of it. The reader counts
    every interval it could not integrate -- a thread starved of the core, a board
    that restarted its counters -- and a span across one of those has a hole in it
    where motion should be. Better to have no prior for a revolution than a prior
    missing a tenth of a second of turning.
    """

    __slots__ = ("dt", "gz_lsb_s", "ticks", "intact", "samples")

    def __init__(self, dt, gz_lsb_s, ticks, intact, samples):
        self.dt = dt
        self.gz_lsb_s = gz_lsb_s      # raw integral of yaw rate, LSB-seconds
        self.ticks = ticks            # raw wheel count change, or None
        self.intact = intact
        self.samples = samples

    @property
    def usable(self):
        return self.intact and MIN_SPAN_S <= self.dt <= MAX_SPAN_S

    def __repr__(self):
        return (f"Span(dt={self.dt:.3f}, gz_lsb_s={self.gz_lsb_s:+.2f}, "
                f"ticks={self.ticks}, intact={self.intact})")


class _Fit:
    """A least-squares slope through the origin, and how much it is trusted.

    Through the origin because both scales are physical ratios with no offset:
    no rotation is no gyro integral, and no travel is no ticks. Weighted by the
    size of the move, which `sum(x*y)/sum(x*x)` does for free and which averaging
    ratios would not -- a 180 degree turn measures the gyro far better than a 20
    degree one, and should count for more rather than the same.
    """

    def __init__(self):
        self.sxy = 0.0
        self.sxx = 0.0
        self.n = 0
        self.samples = []      # (x, y) pairs, kept for the residual spread

    def add(self, x, y):
        self.sxy += x * y
        self.sxx += x * x
        self.n += 1
        self.samples.append((x, y))
        if len(self.samples) > 40:
            del self.samples[0]

    @property
    def slope(self):
        return None if self.sxx <= 0.0 else self.sxy / self.sxx

    @property
    def diverse(self):
        """Whether the moves in here differ enough to have measured anything.

        Both directions, or a decent range of sizes. Same-sized moves the same way
        round share whatever systematic error that particular manoeuvre has, and a
        slope fitted through them repeats it back with a small residual -- which
        reads as confidence and is not. See CALIBRATION_SPAN_RATIO.
        """
        if len(self.samples) < 2:
            return False
        xs = [x for x, _ in self.samples]
        if any(x > 0 for x in xs) and any(x < 0 for x in xs):
            return True
        sizes = [abs(x) for x in xs if x]
        if not sizes:
            return False
        return max(sizes) >= CALIBRATION_SPAN_RATIO * min(sizes)

    def restore(self, samples):
        """Take back a fit measured before the last restart.

        Kept across runs because these are constants of the rover, not of the
        session: gearbox, wheel and the gyro's full-scale range do not change when
        the daemon does. Starting the fit again every restart meant every run
        published whatever its first three moves happened to say.
        """
        for pair in samples or ():
            try:
                x, y = float(pair[0]), float(pair[1])
            except (TypeError, ValueError, IndexError):
                continue
            self.add(x, y)

    def spread(self):
        """Worst residual as a fraction of the fitted value, over the kept samples.

        A fraction rather than an absolute, because the two things this fits are
        in different units and the question is the same for both: does the next
        move agree with the ones before it. None until there is a fit and a
        sample big enough to divide by.
        """
        slope = self.slope
        if slope is None or not self.samples:
            return None
        worst = 0.0
        for x, y in self.samples:
            predicted = slope * x
            if abs(predicted) < 1e-9:
                continue
            worst = max(worst, abs(y - predicted) / abs(predicted))
        return worst

    def asdict(self):
        return {"value": self.slope, "moves": self.n, "spread": self.spread(),
                "diverse": self.diverse, "samples": list(self.samples)}


class Odometry:
    """Raw telemetry in, a motion prior and a rotation witness out.

    The source is anything with a `motion()` returning the reader's running
    totals -- the daemon's `SerialLink`, or `run_slam.py`'s standalone reader. A
    source without one is not an error: the rover drove for months on the lidar
    alone, and everything here degrades to "unknown" rather than to an exception,
    because a board on WiFi answers requests and has no stream to integrate.
    """

    def __init__(self, source, store=None, load=True):
        self.source = source
        self.store = _default_store() if store is None else store
        self._mark = None
        # Bias in raw LSB, and the spread of the resting rate around it. Both are
        # learnt while the rover is known to be standing still; neither can be
        # learnt while it moves, which is why `learn_rest` is called by the
        # navigator rather than from in here.
        self.gz_bias_lsb = None
        self.gz_noise_lsb = None
        self.rest_spans = 0
        #: Signed, and the sign is the point: it carries whether a positive `gz`
        #: is a counter-clockwise turn in the rover's frame, which nothing on this
        #: rover documents. Until a turn has established it, the witness cannot
        #: compare directions and says so.
        self.gyro_lsb_per_dps = None
        self.ticks_per_metre = None
        self._gyro_fit = _Fit()
        self._ticks_fit = _Fit()
        # Heading the matcher has claimed across an unbroken run of revolutions
        # where the gyro said the chassis was still, and how many of them.
        self._quiet_yaw_deg = 0.0
        self._quiet_spans = 0
        self.last_prior = (0.0, 0.0)
        self.priors_used = 0
        self.priors_asked = 0
        self.source_ok = hasattr(source, "motion")
        if load:
            self.load()

    # --- the stream -------------------------------------------------------

    def mark(self):
        """The board's running totals as they stand, for `between` to difference.

        Two ways of reading the same stream, and they must not share a mark. The
        prior wants each lidar revolution on its own and so consumes a mark every
        time; a turn wants the whole turn as one span and lasts many revolutions.
        So a caller measuring a move holds its own mark from here, and the
        revolution-by-revolution one in `span` goes on being consumed underneath
        it without disturbing anything.
        """
        return self._snapshot()

    def reset(self):
        """Forget the running mark, so the next `span` starts here.

        For the caller that has just moved the pose itself: a re-seed is not
        motion, and a span straddling one would ask the witness to explain a jump
        that never happened.
        """
        self._mark = self._snapshot()
        self.forget_quiet()

    def forget_quiet(self):
        """Start the creep test again.

        For the caller that has just done something the matcher's heading is
        entitled to jump over -- a recovery sweep, a re-seed, a map that has just
        been re-agreed. Carrying a stale accumulation across one of those is how a
        healthy rover gets accused of drifting.
        """
        self._quiet_yaw_deg = 0.0
        self._quiet_spans = 0

    def _snapshot(self):
        if not self.source_ok:
            return None
        try:
            return self.source.motion()
        except Exception:
            return None

    @staticmethod
    def between(was, now):
        """What the board reported between two marks. None if either is missing."""
        if was is None or now is None:
            return None
        if was["at"] is None or now["at"] is None:
            return None
        ticks = None
        if was["ticks"] is not None and now["ticks"] is not None:
            ticks = now["ticks"] - was["ticks"]
        return Span(dt=now["at"] - was["at"],
                    gz_lsb_s=now["gz_lsb_s"] - was["gz_lsb_s"],
                    ticks=ticks,
                    intact=now["breaks"] == was["breaks"],
                    samples=now["samples"] - was["samples"])

    def span(self):
        """What the board reported since the running mark, and move it here.

        None when there is nothing to compare against yet -- the first call after
        a start or a reset establishes the mark and returns nothing, which is the
        honest answer rather than a span from zero.
        """
        now = self._snapshot()
        if now is None:
            return None
        was, self._mark = self._mark, now
        return self.between(was, now)

    # --- rest, which is where the witness gets its bar ---------------------

    def learn_rest(self, span):
        """Fold a span the rover spent standing still into bias and noise floor.

        The caller has to be sure of the standing still. A span with real rotation
        in it teaches the bias to expect that rotation, and from then on the
        witness will not see the thing it exists to see.
        """
        if span is None or not span.usable or span.samples < 1:
            return
        rate = span.gz_lsb_s / span.dt          # mean raw gz over the span
        if self.gz_bias_lsb is None:
            self.gz_bias_lsb = rate
            self.gz_noise_lsb = 0.0
        else:
            residual = rate - self.gz_bias_lsb
            self.gz_bias_lsb += REST_ALPHA * residual
            # RMS rather than a mean absolute, tracked the same exponential way,
            # because it is a spread being compared against and the sigmas above
            # are sigmas.
            noise = self.gz_noise_lsb or 0.0
            self.gz_noise_lsb = math.sqrt(
                noise * noise + REST_ALPHA * (residual * residual - noise * noise))
        self.rest_spans += 1

    @property
    def rest_known(self):
        return (self.gz_bias_lsb is not None
                and self.rest_spans >= REST_SPANS_WANTED)

    def _rate(self, span):
        """The span's mean yaw rate with the resting bias taken out, raw LSB."""
        if self.gz_bias_lsb is None or not span.usable:
            return None
        return span.gz_lsb_s / span.dt - self.gz_bias_lsb

    def threshold_lsb(self):
        """How fast the chassis must be turning before the gyro will vouch for it."""
        if self.gz_noise_lsb is None:
            return None
        return ROTATION_SIGMAS * max(self.gz_noise_lsb, MIN_NOISE_LSB)

    # --- the witness ------------------------------------------------------

    def rotated(self, span):
        """Whether the chassis physically turned over this span.

        Three answers, and the third is not a failure. "unknown" means the gyro
        has nothing to say -- no bias yet, a hole in the span, a source that does
        not stream -- and a caller that treats it as "still" would manufacture the
        very disagreement this exists to detect.
        """
        if span is None or not span.usable or not self.rest_known:
            return "unknown"
        rate = self._rate(span)
        if rate is None:
            return "unknown"
        return "turning" if abs(rate) >= self.threshold_lsb() else "still"

    def disagreement(self, span, matcher_yaw_rad, min_claim_deg=5.0):
        """Why the gyro and the scan match cannot both be right, or None.

        Only the two comparisons that hold without a calibrated scale. The
        magnitude one -- the matcher says the rover swung round and the chassis
        says it did not move -- is the one that catches a scan snapping onto the
        wrong alignment, and it works from the first stationary second. The sign
        one needs to know which way a positive `gz` points, which only a turn can
        establish, so it stays dark until one has.
        """
        verdict = self.rotated(span)
        if verdict == "unknown":
            # Not "still". A caller that read it that way would manufacture the
            # very disagreement this exists to detect, on every revolution, from
            # a cold start.
            self.forget_quiet()
            return None
        claim_deg = math.degrees(matcher_yaw_rad)

        # The slow half. A run of revolutions the chassis spent still is a run the
        # heading should have spent still too, and heading that accumulates across
        # one is wrong however little of it arrives at a time.
        if verdict == "still":
            self._quiet_yaw_deg += claim_deg
            self._quiet_spans += 1
            if (self._quiet_spans >= QUIET_CREEP_SPANS
                    and abs(self._quiet_yaw_deg) >= QUIET_CREEP_DEG):
                drifted, spans = self._quiet_yaw_deg, self._quiet_spans
                self.forget_quiet()
                return (f"the match has drifted the heading {drifted:+.1f} degrees "
                        f"over {spans} revolutions the gyro says the chassis spent "
                        f"standing still")
        else:
            self.forget_quiet()

        if abs(claim_deg) < min_claim_deg:
            # The matcher is not claiming anything worth contradicting outright. A
            # gyro that says "turning" here is the rover being nudged or the
            # chassis flexing, neither of which impugns the match.
            return None
        if verdict == "still":
            return (f"the match moved the heading {claim_deg:+.1f} degrees while "
                    f"the gyro says the chassis did not turn")
        if self.gyro_lsb_per_dps:
            gyro_deg = self.gyro_degrees(span)
            if gyro_deg is not None and gyro_deg * claim_deg < 0:
                return (f"the match turned the heading {claim_deg:+.1f} degrees "
                        f"and the gyro turned {gyro_deg:+.1f} the other way")
        return None

    # --- the prior --------------------------------------------------------

    def gyro_degrees(self, span):
        """The span's rotation in degrees, or None while the scale is unmeasured."""
        if not self.gyro_lsb_per_dps or span is None or not span.usable:
            return None
        if self.gz_bias_lsb is None:
            return None
        return (span.gz_lsb_s - self.gz_bias_lsb * span.dt) / self.gyro_lsb_per_dps

    def wheel_metres(self, span):
        """The span's travel in metres, or None while the scale is unmeasured."""
        if not self.ticks_per_metre or span is None or not span.usable:
            return None
        if span.ticks is None:
            return None
        return span.ticks / self.ticks_per_metre

    def prior(self, span):
        """(forward metres, yaw radians) for `slam2d_set_prior`.

        Zero for whichever half has no scale factor yet, which is a legitimate
        prior and not a fallback: a constant-position guess is inside the coarse
        window at anything short of a run, and a half-known prior is better than
        none. Zero for both is what the rover has always driven on.
        """
        forward = self.wheel_metres(span)
        yaw_deg = self.gyro_degrees(span)
        # Kept so that a prior which is quietly always zero can be told from one
        # that is working. That is not a hypothetical failure: every gate here
        # returns zero rather than raising, by design, so "wired up" and "having
        # any effect" look identical from outside without this.
        self.last_prior = (round(forward or 0.0, 4), round(yaw_deg or 0.0, 2))
        self.priors_asked += 1
        if forward or yaw_deg:
            self.priors_used += 1
        return (forward or 0.0, math.radians(yaw_deg or 0.0))

    # --- calibrating, out of moves the rover makes anyway ------------------

    def note_turn(self, degrees, span):
        """A confirmed turn of `degrees`, against what the gyro integrated.

        Confirmed is the word doing the work. The matcher's heading after a
        dead-reckoned burst is a hypothesis until a recovery sweep and the
        revolution after it agree, and calibrating against an unconfirmed one
        would fit the gyro to the rover's guess about itself.

        Returns (taken, why). The reason is there whether or not it was taken,
        because a calibration that quietly declines every move looks exactly like
        one that is not wired up, and the difference took a drive on the floor to
        find out once already.
        """
        if span is None:
            return False, "the board said nothing over the turn"
        # `intact` rather than `usable`: a whole turn is seconds long and would
        # fail the per-revolution ceiling on span length, which exists to keep a
        # stalled loop out of the noise floor and has nothing to say here.
        if not span.intact:
            return False, "the telemetry has a hole in it over that turn"
        if abs(degrees) < MIN_TURN_DEG:
            return False, f"{degrees:+.0f} deg is too small to measure a gyro by"
        if self.gz_bias_lsb is None:
            return False, "the resting gyro has not been measured yet"
        corrected = span.gz_lsb_s - self.gz_bias_lsb * span.dt
        self._gyro_fit.add(degrees, corrected)
        self.save()
        if self._gyro_fit.n < CALIBRATION_WANTED:
            return True, ""
        if not self._gyro_fit.diverse:
            return True, ("kept, but the turns so far are all much the same size "
                          "and direction, so no scale is published from them yet")
        self.gyro_lsb_per_dps = self._gyro_fit.slope
        self.save()
        return True, ""

    def note_drive(self, metres, span):
        """A confirmed straight drive of `metres`, against the wheel counts.

        Returns (taken, why), for the reason `note_turn` gives.
        """
        if span is None:
            return False, "the board said nothing over the drive"
        if not span.intact:
            return False, "the telemetry has a hole in it over that drive"
        if abs(metres) < MIN_DRIVE_M:
            return False, f"{metres:.2f} m is too short to measure wheels by"
        if span.ticks is None:
            return False, "the board reported no wheel counts"
        self._ticks_fit.add(metres, span.ticks)
        self.save()
        if self._ticks_fit.n < CALIBRATION_WANTED:
            return True, ""
        if not self._ticks_fit.diverse:
            return True, ("kept, but the drives so far are all much the same "
                          "length, so no scale is published from them yet")
        self.ticks_per_metre = self._ticks_fit.slope
        self.save()
        return True, ""

    def estimate(self):
        """What has been measured so far, and how well the moves agree."""
        return {
            "gyro_lsb_per_dps": self._gyro_fit.asdict(),
            "ticks_per_metre": self._ticks_fit.asdict(),
            "in_use": {"gyro_lsb_per_dps": self.gyro_lsb_per_dps,
                       "ticks_per_metre": self.ticks_per_metre},
        }

    def status(self):
        """The short form, for `nav_status`."""
        return {
            "source": self.source_ok,
            "quiet_drift_deg": round(self._quiet_yaw_deg, 1),
            # Every interval the reader could not integrate since the daemon
            # started. It should stay put; one that climbs means the loop is being
            # starved, and the spans it lands in are refused by both jobs here.
            "telemetry_holes": (None if self._mark is None
                                else self._mark.get("breaks")),
            "gyro_bias_lsb": None if self.gz_bias_lsb is None
                             else round(self.gz_bias_lsb, 2),
            "gyro_noise_lsb": None if self.gz_noise_lsb is None
                              else round(self.gz_noise_lsb, 2),
            "rest_spans": self.rest_spans,
            "witness": self.rest_known,
            "gyro_lsb_per_dps": None if self.gyro_lsb_per_dps is None
                                else round(self.gyro_lsb_per_dps, 3),
            "ticks_per_metre": None if self.ticks_per_metre is None
                               else round(self.ticks_per_metre, 1),
            "turns_measured": self._gyro_fit.n,
            "drives_measured": self._ticks_fit.n,
            # Do the moves agree with each other? The worst residual as a fraction
            # of the fitted value, which is the only thing here that says whether a
            # scale factor is worth using or merely exists. A fit nothing checks is
            # a number with a decimal point and no claim behind it.
            "gyro_spread": _round(self._gyro_fit.spread()),
            "ticks_spread": _round(self._ticks_fit.spread()),
            # Whether the moves behind each fit differ enough to be measuring the
            # rover rather than agreeing with themselves.
            "gyro_varied": self._gyro_fit.diverse,
            "ticks_varied": self._ticks_fit.diverse,
            "prior": bool(self.gyro_lsb_per_dps or self.ticks_per_metre),
            "last_prior_m_deg": self.last_prior,
            # What fraction of revolutions actually got one. A scale factor measured
            # but a fraction near zero means every span is being refused, which is a
            # different problem wearing the same face.
            "priors_used": (None if not self.priors_asked else
                            round(self.priors_used / self.priors_asked, 3)),
        }

    # --- keeping the measurement ------------------------------------------

    def load(self):
        try:
            with open(self.store, encoding="utf-8") as handle:
                saved = json.load(handle)
        except (OSError, ValueError):
            return False
        if not isinstance(saved, dict):
            return False
        self._gyro_fit.restore(saved.get("gyro_samples"))
        self._ticks_fit.restore(saved.get("ticks_samples"))
        gyro = saved.get("gyro_lsb_per_dps")
        ticks = saved.get("ticks_per_metre")
        if isinstance(gyro, (int, float)) and gyro:
            self.gyro_lsb_per_dps = float(gyro)
        if isinstance(ticks, (int, float)) and ticks:
            self.ticks_per_metre = float(ticks)
        # A file from before the samples were kept, or one hand-written: the values
        # stand, and the next move that lands joins a fit starting from them.
        if self._gyro_fit.n and self.gyro_lsb_per_dps is None:
            if self._gyro_fit.diverse and self._gyro_fit.n >= CALIBRATION_WANTED:
                self.gyro_lsb_per_dps = self._gyro_fit.slope
        if self._ticks_fit.n and self.ticks_per_metre is None:
            if self._ticks_fit.diverse and self._ticks_fit.n >= CALIBRATION_WANTED:
                self.ticks_per_metre = self._ticks_fit.slope
        # The bias is deliberately *not* restored. It drifts with temperature and
        # the rover is standing still at every start anyway, so a stale one would
        # only be believed until the first few resting spans replaced it -- and
        # believed is exactly the wrong thing to be about a stale bias.
        return True

    def save(self):
        payload = {"gyro_lsb_per_dps": self.gyro_lsb_per_dps,
                   "ticks_per_metre": self.ticks_per_metre,
                   "turns_measured": self._gyro_fit.n,
                   "drives_measured": self._ticks_fit.n,
                   # The moves themselves, not just what they came to. A restart
                   # then continues the measurement instead of starting a new one
                   # from whatever the next three moves happen to be.
                   "gyro_samples": [[round(x, 3), round(y, 3)]
                                    for x, y in self._gyro_fit.samples],
                   "ticks_samples": [[round(x, 4), round(y, 2)]
                                     for x, y in self._ticks_fit.samples],
                   "measured_at": time.strftime("%Y-%m-%dT%H:%M:%S")}
        try:
            temporary = self.store + ".new"
            with open(temporary, "w", encoding="utf-8") as handle:
                json.dump(payload, handle, indent=1)
                handle.write("\n")
            os.replace(temporary, self.store)
            return True
        except OSError:
            return False


# --- self-test ------------------------------------------------------------

class _Board:
    """A driver board that can be told what to do, for testing without one."""

    def __init__(self, bias=6.9, lsb_per_dps=16.4, ticks_per_metre=1000.0):
        self.bias = bias
        self.lsb_per_dps = lsb_per_dps
        self.ticks_per_metre = ticks_per_metre
        self.t = 100.0
        self.gz_lsb_s = 0.0
        self.ticks = 5000.0
        self.samples = 0
        self.breaks = 0

    def advance(self, seconds, dps=0.0, ms=0.0, noise=0.0):
        steps = max(1, int(seconds / 0.05))
        step = seconds / steps
        for i in range(steps):
            wobble = noise * (1 if i % 2 else -1)
            self.gz_lsb_s += (self.bias + wobble + dps * self.lsb_per_dps) * step
            self.ticks += ms * step * self.ticks_per_metre
            self.t += step
            self.samples += 1

    def motion(self):
        return {"at": self.t, "gz_lsb_s": self.gz_lsb_s, "ticks": self.ticks,
                "samples": self.samples, "breaks": self.breaks}


def _selftest():
    import tempfile

    ok = [True]

    def check(what, got, want, tol=None):
        good = (abs(got - want) <= tol) if tol is not None else (got == want)
        ok[0] = ok[0] and good
        shown = f"{got:.4g}" if isinstance(got, float) else repr(got)
        wanted = f"{want:.4g}" if isinstance(want, float) else repr(want)
        pad = "" if tol is None else f" +/- {tol:.4g}"
        print(f"  {what:<52} {shown:>10}  (want {wanted}{pad})  "
              f"{'ok' if good else 'FAILED'}")

    store = os.path.join(tempfile.mkdtemp(), "odometry.json")

    print("resting: the bias and the noise floor come out of standing still")
    board = _Board()
    odo = Odometry(board, store=store, load=False)
    odo.reset()
    for _ in range(80):
        board.advance(0.1, dps=0.0, noise=1.5)
        odo.learn_rest(odo.span())
    check("bias, LSB", odo.gz_bias_lsb, 6.9, 0.6)
    check("the witness has a bar to judge by", odo.rest_known, True)
    board.advance(0.1, noise=1.5)
    check("a resting span reads as still", odo.rotated(odo.span()), "still")

    print("\nturning: the same span reads as rotation, with no scale factor")
    board.advance(1.0, dps=45.0)
    span = odo.span()
    check("a 45 deg/s span reads as turning", odo.rotated(span), "turning")
    check("and the matcher agreeing draws no complaint",
          odo.disagreement(span, math.radians(45.0)), None)

    print("\nthe failure this exists to catch")
    board.advance(0.1)                      # the chassis sat still
    span = odo.span()
    claim = odo.disagreement(span, math.radians(9.0))
    check("a match that swung 9 deg over a still chassis is caught",
          claim is not None, True)
    print(f"    -> {claim}")
    board.advance(0.1)
    check("a match claiming nothing much is not second-guessed",
          odo.disagreement(odo.span(), math.radians(0.4)), None)

    print("\nthe prior stays zero until the scale factors are measured")
    board.advance(0.1, dps=30.0, ms=0.3)
    span = odo.span()
    check("forward metres", odo.prior(span)[0], 0.0)
    check("yaw radians", odo.prior(span)[1], 0.0)

    print("\ncalibrating out of confirmed moves")
    for degrees in (90.0, -90.0, 180.0):
        board.advance(0.1)
        odo.span()                          # mark, so the turn spans only the turn
        board.advance(abs(degrees) / 60.0, dps=60.0 * (1 if degrees > 0 else -1))
        odo.note_turn(degrees, odo.span())
    check("gyro scale, LSB per deg/s", odo.gyro_lsb_per_dps, 16.4, 0.3)
    for metres in (0.5, 1.0, 0.8):
        board.advance(0.1)
        odo.span()
        board.advance(metres / 0.25, ms=0.25)
        odo.note_drive(metres, odo.span())
    check("wheel scale, ticks per metre", odo.ticks_per_metre, 1000.0, 20.0)

    print("\nand then the prior is real")
    board.advance(0.1)
    odo.span()
    board.advance(0.1, dps=30.0, ms=0.3)
    span = odo.span()
    forward, yaw = odo.prior(span)
    check("forward metres over 0.1 s at 0.3 m/s", forward, 0.03, 0.004)
    check("yaw degrees over 0.1 s at 30 deg/s", math.degrees(yaw), 3.0, 0.4)

    print()
    print("the slow half: a creep no single revolution would trip")
    creeper = Odometry(_Board(), store=store, load=False)
    creeper.reset()
    for _ in range(80):
        creeper.source.advance(0.1, noise=1.5)
        creeper.learn_rest(creeper.span())
    complaint = None
    for _ in range(12):
        creeper.source.advance(0.1, noise=1.5)      # the chassis never moves
        complaint = complaint or creeper.disagreement(
            creeper.span(), math.radians(2.0))      # two degrees a revolution
    check("a two-degree-a-revolution creep is caught", complaint is not None, True)
    print(f"    -> {complaint}")

    steady = Odometry(_Board(), store=store, load=False)
    steady.reset()
    for _ in range(80):
        steady.source.advance(0.1, noise=1.5)
        steady.learn_rest(steady.span())
    alleged = None
    for step in [0.3, -0.4, 0.2, -0.2, 0.35, -0.3, 0.25, -0.35] * 8:
        steady.source.advance(0.1, noise=1.5)
        alleged = alleged or steady.disagreement(steady.span(), math.radians(step))
    check("an honest match's own noise is not called a creep", alleged, None)

    print("\nthe sign test, which needed the turn to light up")
    board.advance(0.1)
    odo.span()
    board.advance(0.3, dps=-40.0)           # the chassis turned right
    span = odo.span()
    claim = odo.disagreement(span, math.radians(12.0))   # the match says left
    check("a match turning the wrong way is caught", claim is not None, True)
    print(f"    -> {claim}")

    print("\nspans this cannot vouch for say so rather than guessing")
    board.advance(0.1)
    odo.span()
    board.breaks += 1
    board.advance(0.1, dps=90.0)
    span = odo.span()
    check("a span with a hole in it is not usable", span.usable, False)
    check("and the witness declines to judge it", odo.rotated(span), "unknown")
    check("so nothing is alleged from it",
          odo.disagreement(span, math.radians(30.0)), None)

    print("\na source with no stream degrades rather than raising")
    blind = Odometry(object(), store=store, load=False)
    check("no span", blind.span(), None)
    check("no verdict", blind.rotated(None), "unknown")
    check("no prior", blind.prior(None), (0.0, 0.0))

    print("\nmoves that are all alike are kept but not published")
    samey = Odometry(_Board(), store=os.path.join(os.path.dirname(store),
                                                  "samey.json"), load=False)
    samey.reset()
    for _ in range(80):
        samey.source.advance(0.1, noise=1.5)
        samey.learn_rest(samey.span())
    for _ in range(4):
        samey.source.advance(0.1)
        samey.span()
        samey.source.advance(175.0 / 60.0, dps=60.0)
        taken, why = samey.note_turn(175.0, samey.span())
        check("the turn is kept", taken, True)
    check("four identical turns publish no scale", samey.gyro_lsb_per_dps, None)
    check("and the reason is on the record", "much the same size" in why, True)
    samey.source.advance(0.1)
    samey.span()
    samey.source.advance(40.0 / 60.0, dps=-60.0)     # a smaller one, the other way
    samey.note_turn(-40.0, samey.span())
    check("one turn that differs unlocks it", samey.gyro_lsb_per_dps is not None,
          True)

    print("\nthe measurement survives a restart, and goes on being measured")
    odo.save()
    again = Odometry(_Board(), store=store)
    check("gyro scale reloaded", again.gyro_lsb_per_dps, 16.4, 0.3)
    check("wheel scale reloaded", again.ticks_per_metre, 1000.0, 20.0)
    check("and so did the moves behind it, not just the answer",
          again._gyro_fit.n, odo._gyro_fit.n)
    check("but the bias is not, because it drifts", again.gz_bias_lsb, None)

    print("\nPASS" if ok[0] else "\nFAILED")
    return 0 if ok[0] else 1


if __name__ == "__main__":
    raise SystemExit(_selftest())

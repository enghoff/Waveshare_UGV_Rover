"""A recording session for Perception, on the daemon's side.

Perception records the rover's streams on one clock for its training and
evaluation (docs/plans/recording-for-perception.md), to a schema agreed with
it (`rover-schema-v1.json` in that project). The daemon owns a session: it
opens the directory, announces it so that the depth service, the lidar node and
the ROS recorder open their own streams (session_record.Follower), and closes
it with the manifest. Its own streams are the ones only it can see:

- every write to the board, from the board link's own write lock
  (`daemon_commands`, and the gimbal and motor commands among them);
- every feedback line the board sends, as it is drained;
- the gimbal camera, from a feed held open for the whole session, every frame
  with its own driver stamp;
- the scripted gimbal schedules it runs, the operator's annotations, the stills,
  and health once a second.

**While a session is open the session holds the camera.** The camera cannot be
opened twice, so the daemon's own pictures -- the console's, a look's -- are
the feed's newest frame instead of a grab, face tracking is refused (it would
open the camera and move the gimbal by a policy of its own), and a picture at
another size is refused. The OAK is held on (R-CTL-6), and a manual exposure,
if the session asks for one, is held for the session and given back at the end.

Sessions in the held-out bedroom are evaluation or its truth only
(docs/decisions/the-bedroom-is-held-out-for-perception.md), so a session of any
other kind that names it is refused.

    recording_start    {kind, rooms, operator, configuration?: "base"|"full",
                        exposure_ms?: null|number}
    recording_stop     {}
    recording_status   {}
    recording_annotate {kind, unit?, room?, object?, change?, stratum?, manoeuvre?,
                        schedule_id?, point?, door_state?, footprint_before?,
                        footprint_after?, note?, t_unc_s?, pressed_ago_s?}
    recording_still    {view, unit, when: "before"|"after"}
    recording_schedule {schedule_id, steps: [{step, t_offset_ns, cmd, pan_deg,
                        tilt_deg, speed_raw, accel_raw, pan_speed_raw,
                        tilt_speed_raw}], lead_s?}
    recording_schedule_cancel {}

None of these is a model tool: a session is the owner's and the agents'.
"""
from __future__ import annotations

import glob
import hashlib
import json
import os
import re
import shutil
import threading
import time
from datetime import datetime
from typing import Any

import session_record as rec

RECORDINGS_ENV = "UGV_RECORDINGS"
KINDS = ("commissioning", "stationary_trial", "reference_capture", "validation",
         "training", "evaluation")
#: The kinds a session in the held-out room may be.
HELD_OUT_KINDS = ("evaluation", "reference_capture", "validation")
HELD_OUT = {"room": "bedroom", "has_door": True, "visibility_rule": None}
#: The rooms' outlines on the map, by room, kept on the rover outside git
#: (experiments/rooms/bedroom_outline.py draws the bedroom's).
ROOMS_FILE = os.path.expanduser("~/.ugv/recording/rooms.json")
#: The deployer's record of each component's commit, by name (deploy/deploy.py).
DEPLOY_STATE = os.path.expanduser("~/.ugv/deploy-state.json")
#: The ROS half of a session, started with it (ros_nav/recorder_node.py).
ROS_RECORDER = os.path.expanduser("~/ugv/ros_nav/run_recorder.sh")
#: Camera and depth rates a second; commissioning chooses one configuration and
#: every scored session uses it. The camera runs at 30 and the depth pipeline at
#: 15, so the lower rates keep every third.
CONFIGURATIONS = {"base": {"camera": 10.0, "depth": 5.0},
                  "full": {"camera": 30.0, "depth": 15.0}}
CAMERA_HZ = 30.0
#: The other periodic streams' nominal rates: the board as measured (111,484
#: lines in 5,613 s), the lidar's packets as measured (417 a second, 2026-08-11),
#: the rest as configured.
NOMINAL_HZ = {"oak_imu": 200.0, "board_feedback": 20.0, "odometry": 20.0, "lidar": 417.0,
              "lidar_scan": 10.0, "map_pose": 5.0, "clock": 1.0, "health": 1.0}
#: The streams that flow all session, which the session's start waits for.
STEADY = tuple(NOMINAL_HZ) + ("camera", "depth")
#: Which process writes which stream; the closing step waits for each to close.
WRITERS = {"depth": ("depth", "oak_imu"), "lidar": ("lidar",),
           "ros": ("odometry", "lidar_scan", "map_pose", "clock")}
CLOSE_WAIT_S = 15.0
#: The frame the daemon hands its own callers is stale after this.
FRAME_STALE_S = 2.0
#: What a schedule's first step waits by default, so that its "fixed" record is
#: written well before the step it describes.
SCHEDULE_LEAD_S = 1.0
STAMP_KINDS = {"start_of_exposure": "v4l2_labelled_start_of_exposure",
               "end_of_frame": "v4l2_labelled_end_of_frame"}
ANNOTATION_KINDS = ("session_start", "session_end", "timing_start", "timing_end",
                    "trial_start", "trial_end", "change_start", "change_end",
                    "revisit_start", "revisit_end", "stratum_start", "stratum_end",
                    "capture_start", "capture_end", "carry_start", "carry_end",
                    "floor_point_stop", "door", "note")
ANNOTATION_FIELDS = ("unit", "room", "object", "change", "stratum", "manoeuvre",
                     "schedule_id", "point", "door_state", "footprint_before",
                     "footprint_after", "note")
NAME = re.compile(r"^[A-Za-z0-9._-]+$")

#: Who is asking, for the commands a call ends up writing: set by Rover.call for
#: the length of a call, and by the schedule runner for its steps.
CALLER = threading.local()


def recordings_dir() -> str:
    return os.environ.get(RECORDINGS_ENV) or os.path.expanduser("~/recordings")


def _int_or_none(value):
    return None if value is None else int(value)


def _cpu_times() -> list[tuple[int, int]]:
    """(busy, total) jiffies for each core, from /proc/stat."""
    out = []
    try:
        with open("/proc/stat") as f:
            for line in f:
                if re.match(r"cpu\d", line):
                    n = [int(x) for x in line.split()[1:]]
                    idle = n[3] + (n[4] if len(n) > 4 else 0)
                    out.append((sum(n) - idle, sum(n)))
    except OSError:
        pass
    return out


def _temperatures() -> dict[str, float]:
    temps = {}
    for zone in glob.glob("/sys/class/thermal/thermal_zone*"):
        try:
            with open(os.path.join(zone, "type")) as f:
                name = f.read().strip()
            with open(os.path.join(zone, "temp")) as f:
                temps[name] = int(f.read().strip()) / 1000.0
        except (OSError, ValueError):
            pass
    return temps


def _sha256(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


class Recording:
    """One open session: the daemon's streams, and the tap the board link calls."""

    def __init__(self, rover, directory: str, session: str, kind: str, config: dict[str, Any]):
        self.rover = rover
        self.dir = directory
        self.session = session
        self.kind = kind
        self.config = config
        self.started_ns = rec.now_ns()
        self.streams = {
            "daemon_commands": rec.Stream(directory, "daemon_commands", "daemon_commands.jsonl",
                                          flush_each=True),
            "gimbal_cmd": rec.Stream(directory, "gimbal_cmd", "gimbal_cmd.jsonl"),
            "chassis_cmd": rec.Stream(directory, "chassis_cmd", "chassis_cmd.jsonl"),
            "board_feedback": rec.Stream(directory, "board_feedback", "board_feedback.jsonl"),
            "gimbal_schedule": rec.Stream(directory, "gimbal_schedule", "gimbal_schedule.jsonl"),
            "annotations": rec.Stream(directory, "annotations", "annotations.jsonl",
                                      flush_each=True),
            "stills": rec.Stream(directory, "stills", "stills/index.jsonl"),
            "health": rec.Stream(directory, "health", "health.jsonl"),
            "camera": rec.Stream(directory, "camera", "camera/index.jsonl",
                                 "camera/frames.mjpeg"),
        }
        self.first_command_ns: int | None = None
        self.last_command_ns: int | None = None
        self._keep_every = max(1, round(CAMERA_HZ / config["camera_hz"]))
        self._frames_seen = 0
        self.newest: tuple[bytes, float] | None = None
        self.camera = None
        self.camera_error: str | None = None
        self._volts: int | None = None
        self._units: dict[str, dict[str, Any]] = {}
        self._marks: set[str] = set()  # session_start and session_end, once each
        self._units_lock = threading.Lock()
        self._run: dict[str, Any] | None = None
        self._runs = 0
        self._run_cancel = threading.Event()
        self._stopping = threading.Event()
        self._threads = [threading.Thread(target=self._hold_camera, name="record-camera",
                                          daemon=True),
                         threading.Thread(target=self._health, name="record-health",
                                          daemon=True)]

    # --- the board link's tap -----------------------------------------------

    def command(self, seq: int, t: int, ok: bool, client: str, line: str,
                command: dict[str, Any], note: dict[str, Any] | None) -> None:
        if self.first_command_ns is None:
            self.first_command_ns = t
        self.last_command_ns = t
        self.streams["daemon_commands"].put(
            {"seq": seq, "t_arr": t, "ok": ok, "client": client, "line": line})
        if not ok:
            return
        code = command.get("T")
        common = {"t_arr": t, "daemon_epoch": 0, "daemon_seq": seq, "client": client,
                  "line": line}
        if code in (133, 134):
            run = getattr(CALLER, "run", None)
            t133 = code == 133
            self.streams["gimbal_cmd"].put(dict(
                common, cmd="T133" if t133 else "T134",
                pan_deg=float(command.get("X", 0)), tilt_deg=float(command.get("Y", 0)),
                speed_raw=_int_or_none(command.get("SPD")) if t133 else None,
                accel_raw=_int_or_none(command.get("ACC")) if t133 else None,
                pan_speed_raw=None if t133 else _int_or_none(command.get("SX")),
                tilt_speed_raw=None if t133 else _int_or_none(command.get("SY")),
                run_id=run[0] if run else None, schedule_step=run[1] if run else None,
                caller=getattr(CALLER, "name", None)))
        elif code == 11:
            note = note or {}
            self.streams["chassis_cmd"].put(dict(
                common, left_raw=int(round(command.get("L", 0))),
                right_raw=int(round(command.get("R", 0))),
                linear=note.get("linear"), angular=note.get("angular"),
                keepalive=bool(note.get("keepalive")),
                caller=note.get("caller") or getattr(CALLER, "name", None)))

    def feedback(self, lines: list[bytes], t: int) -> None:
        stream = self.streams["board_feedback"]
        for raw in lines:
            text = raw.decode("utf-8", errors="replace").strip()
            try:
                f = json.loads(text)
            except ValueError:
                f = {}

            def three(*keys):
                got = [f.get(k) for k in keys]
                return [int(v) for v in got] if all(isinstance(v, (int, float)) for v in got) else None

            volts = f.get("v")
            if isinstance(volts, (int, float)):
                self._volts = int(volts)
            stream.put({"t_arr": t, "enc_left": _int_or_none(f.get("odl")),
                        "enc_right": _int_or_none(f.get("odr")),
                        "accel_raw": three("ax", "ay", "az"), "gyro_raw": three("gx", "gy", "gz"),
                        "mag_raw": three("mx", "my", "mz"),
                        "volt_raw": _int_or_none(volts), "raw": text})

    # --- the camera -----------------------------------------------------------

    def _frame(self, jpeg: bytes, stamp: float | None, arrived: int) -> None:
        """Every frame the feed reassembles, on its reader thread: keep it for the
        daemon's own callers, and record every `_keep_every`th."""
        self.newest = (jpeg, stamp if stamp is not None else arrived * 1e-9)
        self._frames_seen += 1
        if (self._frames_seen - 1) % self._keep_every:
            return
        manual = self.config["exposure_s"]
        kind = getattr(self.camera, "stamp_kind", None)
        self.streams["camera"].put(
            {"t_arr": arrived,
             "t_dev": None if stamp is None else int(round(stamp * 1e9)),
             "t_dev_kind": None if stamp is None else STAMP_KINDS.get(kind, "instant_unknown"),
             "offset": 0, "length": 0, "exposure_s": manual, "gain": None,
             "auto_exposure": manual is None},
            blob=jpeg)

    def _hold_camera(self) -> None:
        """Open the feed and keep it for the session, on this thread: the kernel
        ends v4l2-ctl with the thread that started it (uvc_camera.die_with_parent).
        A feed that dies is reopened, behind a restart record."""
        import uvc_camera
        from track_face_pi import Camera

        rover = self.rover
        opened = 0
        while not self._stopping.is_set():
            try:
                with rover._camera_lock:
                    camera = Camera(rover.device, rover.size, "MJPG")
                camera.listener = self._frame
                if self.config["exposure_s"] is not None:
                    units = round(self.config["exposure_s"] / 100e-6)
                    if not uvc_camera.hold_manual(rover.device, units):
                        self.camera_error = "the camera would not take the manual exposure"
                if opened:
                    self.streams["camera"].restart()
                opened += 1
                self.camera = camera
                while not self._stopping.wait(1.0):
                    if not camera.alive():
                        self.camera_error = "the feed stopped: " + "; ".join(camera.complaints[-2:])
                        break
                camera.listener = None
                camera.close()
            except Exception as exc:
                self.camera_error = "%s: %s" % (type(exc).__name__, exc)
                self._stopping.wait(2.0)
        if self.config["exposure_s"] is not None:
            uvc_camera.restore_automatic(rover.device)

    def snapshot(self, size) -> tuple[list, str]:
        """What the daemon's own picture-takers get while the session holds the camera."""
        if size is not None and tuple(size) != tuple(self.rover.size):
            return [], "a recording session holds the camera at %dx%d" % tuple(self.rover.size)
        newest = self.newest
        if newest is None:
            return [], "the recording's camera feed has not delivered a frame yet"
        if time.monotonic() - newest[1] > FRAME_STALE_S:
            return [], "the recording's camera feed has stalled"
        return [newest], ""

    # --- health ---------------------------------------------------------------

    def _reported(self) -> dict[str, dict]:
        """What the other processes last reported of their streams, by stream."""
        reported = {}
        for path in glob.glob(os.path.join(self.dir, ".status-*.json")):
            try:
                with open(path, encoding="utf-8") as f:
                    reported.update(json.load(f))
            except (OSError, ValueError):
                continue
        return reported

    def counts(self) -> tuple[dict[str, int], dict[str, int]]:
        """Queue depths and drops for every stream: the daemon's own, and what the
        other processes last reported in their status files. A process that has
        not reported yet has queued and dropped nothing, and its streams are
        named all the same, so that every health record covers every stream."""
        queues = {name: 0 for names in WRITERS.values() for name in names}
        queues.update({name: s.depth() for name, s in self.streams.items()})
        dropped = dict.fromkeys(queues, 0)
        dropped.update({name: s.dropped for name, s in self.streams.items()})
        for name, c in self._reported().items():
            queues[name] = int(c.get("queue", 0))
            dropped[name] = int(c.get("dropped", 0))
        return queues, dropped

    def waiting_for(self) -> list[str]:
        """The steady streams that have not yet given the session a record. The
        OAK boots and the ROS recorder starts for seconds after a session opens,
        and a start marked before them is a gap in every one of their streams
        (2026-10-28: 2 to 4 s)."""
        offered = {name: s.queued + s.dropped for name, s in self.streams.items()}
        for name, c in self._reported().items():
            offered[name] = int(c.get("queued", 0)) + int(c.get("dropped", 0))
        return sorted(name for name in STEADY if not offered.get(name))

    def _health(self) -> None:
        before = _cpu_times()
        # On a fixed beat rather than a second after the last, which lost one
        # record in sixty to the work in between.
        beat = time.monotonic()
        while True:
            beat += 1.0
            if self._stopping.wait(max(0.0, beat - time.monotonic())):
                return
            now = _cpu_times()
            load = [round((b1 - b0) / (t1 - t0), 4) if t1 > t0 else 0.0
                    for (b0, t0), (b1, t1) in zip(before, now)]
            before = now
            queues, dropped = self.counts()
            self.streams["health"].put(
                {"battery_v": None if self._volts is None else self._volts / 100.0,
                 "cpu_load": load or None, "temps_c": _temperatures(),
                 "queues": queues, "dropped": dropped})

    # --- annotations, stills and schedules -------------------------------------

    def annotate(self, args: dict[str, Any]) -> dict[str, Any]:
        kind = args.get("kind")
        if kind not in ANNOTATION_KINDS:
            raise ValueError("`kind` is one of " + ", ".join(ANNOTATION_KINDS))
        record = {k: args.get(k) for k in ANNOTATION_FIELDS}
        unit = record["unit"]
        if unit is not None and not NAME.match(str(unit)):
            raise ValueError("a unit id is letters, digits, '.', '_' and '-'")
        with self._units_lock:
            if kind in ("session_start", "session_end"):
                # The session's own markers, which the counts run between: once
                # each, in order, and the start only once every stream flows.
                if kind in self._marks:
                    raise ValueError("this session already has its %s" % kind)
                if kind == "session_end" and "session_start" not in self._marks:
                    raise ValueError("the session has no session_start to end")
                waiting = self.waiting_for() if kind == "session_start" else []
                if waiting:
                    raise ValueError("not every stream has begun yet (%s): mark the start "
                                     "once they have" % ", ".join(waiting))
                self._marks.add(kind)
            elif kind.endswith("_start"):
                if unit is None or unit in self._units:
                    raise ValueError("a start needs a unit id this session has not used")
                self._units[unit] = {"kind": kind[:-6], "open": True}
            elif kind.endswith("_end"):
                held = self._units.get(unit)
                if held is None or not held["open"] or held["kind"] != kind[:-4]:
                    raise ValueError("an end needs the unit id of an open %s" % kind[:-4])
                held["open"] = False
            elif unit is not None:
                if unit in self._units:
                    raise ValueError("unit ids name one unit only in the session")
                self._units[unit] = {"kind": kind, "open": False}
        now = rec.now_ns()
        ago = float(args.get("pressed_ago_s") or 0.0)
        record.update(t_arr=now, t_press=now - int(ago * 1e9),
                      t_unc_s=float(args.get("t_unc_s", 2.0)), kind=kind)
        self.streams["annotations"].put(record)
        return {"ok": True, "kind": kind, "unit": unit}

    def still(self, args: dict[str, Any]) -> dict[str, Any]:
        view, unit, when = args.get("view"), args.get("unit"), args.get("when")
        if when not in ("before", "after"):
            raise ValueError("`when` is before or after")
        for value, what in ((view, "view"), (unit, "unit")):
            if not value or not NAME.match(str(value)):
                raise ValueError("`%s` is a name of letters, digits, '.', '_' and '-'" % what)
        with self._units_lock:
            held = self._units.get(unit)
        if held is not None and held["kind"] not in ("change", "capture"):
            raise ValueError("a still belongs to a change or capture unit, and %s is a %s"
                             % (unit, held["kind"]))
        got, why = self.snapshot(None)
        if not got:
            raise ValueError(why)
        name = "%s-%s-%s.jpg" % (unit, when, view)
        path = os.path.join(self.dir, "stills", name)
        if os.path.exists(path):
            raise ValueError("that unit already has its %s still from %s" % (when, view))
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "wb") as f:
            f.write(got[0][0])
        self.streams["stills"].put({"file": name, "source": "gimbal_camera", "view": view,
                                    "unit": unit, "when": when})
        if held is None:
            # A before still may come ahead of its unit's start, so it is kept.
            return {"ok": True, "file": name,
                    "warning": "no change or capture unit %s has started yet; one must, "
                               "in this session" % unit}
        return {"ok": True, "file": name}

    def schedule(self, args: dict[str, Any]) -> dict[str, Any]:
        """Fix a scripted gimbal schedule, store it, and run it on its own thread."""
        if self._run is not None:
            raise ValueError("a schedule is already running: cancel it first")
        sid = args.get("schedule_id")
        if not sid or not NAME.match(str(sid)):
            raise ValueError("`schedule_id` is a name of letters, digits, '.', '_' and '-'")
        steps = args.get("steps")
        if not isinstance(steps, list) or not steps:
            raise ValueError("`steps` is a list of at least one step")
        fields = ("step", "t_offset_ns", "cmd", "pan_deg", "tilt_deg", "speed_raw",
                  "accel_raw", "pan_speed_raw", "tilt_speed_raw")
        clean, last = [], -1
        for i, s in enumerate(steps):
            step = {k: s.get(k) for k in fields}
            if step["cmd"] not in ("T133", "T134"):
                raise ValueError("step %d: `cmd` is T133 or T134" % i)
            offset = int(step["t_offset_ns"])
            if (i == 0 and offset != 0) or offset < last:
                raise ValueError("step offsets start at 0 and never go back")
            last = step["t_offset_ns"] = offset
            for k in ("pan_deg", "tilt_deg"):
                if float(step[k]) != round(float(step[k])):
                    raise ValueError("step %d: the board is told whole degrees" % i)
                step[k] = float(step[k])
            if not -180 <= step["pan_deg"] <= 180 or not -30 <= step["tilt_deg"] <= 90:
                raise ValueError("step %d is outside the gimbal's travel" % i)
            for k in ("speed_raw", "accel_raw", "pan_speed_raw", "tilt_speed_raw"):
                step[k] = _int_or_none(step[k])
            clean.append(step)
        path = os.path.join(self.dir, "schedules", sid + ".json")
        if os.path.exists(path):
            raise ValueError("schedule %s is already stored in this session" % sid)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        body = {"schema": "rover-recording", "version": 1, "file": "schedule",
                "schedule_id": sid, "steps": clean}
        with open(path, "w", encoding="utf-8") as f:
            json.dump(body, f, indent=1)
        digest = _sha256(path)
        self._runs += 1
        run_id = "%s-run%d" % (sid, self._runs)
        t_fixed = rec.now_ns()
        first = t_fixed + int(float(args.get("lead_s", SCHEDULE_LEAD_S)) * 1e9)
        planned = [dict({k: v for k, v in s.items() if k != "t_offset_ns"},
                        t_plan=first + s["t_offset_ns"]) for s in clean]
        self.streams["gimbal_schedule"].put(
            {"run_id": run_id, "event": "fixed", "schedule_id": sid,
             "schedule_sha256": digest, "t_fixed": t_fixed, "steps": planned})
        self._run = {"run_id": run_id, "schedule_id": sid, "sha256": digest}
        self._run_cancel.clear()
        threading.Thread(target=self._run_schedule, args=(planned,), name="record-schedule",
                         daemon=True).start()
        return {"ok": True, "run_id": run_id, "steps": len(planned)}

    def _schedule_event(self, event: str) -> None:
        run = self._run
        self.streams["gimbal_schedule"].put(
            {"run_id": run["run_id"], "event": event, "schedule_id": run["schedule_id"],
             "schedule_sha256": run["sha256"], "t_fixed": None, "steps": None})

    def _run_schedule(self, planned: list[dict[str, Any]]) -> None:
        rover = self.rover
        run_id = self._run["run_id"]
        cancelled = False
        try:
            self._sleep_until(planned[0]["t_plan"])
            if self._run_cancel.is_set():
                cancelled = True
                return
            self._schedule_event("started")
            for step in planned:
                self._sleep_until(step["t_plan"])
                if self._run_cancel.is_set():
                    cancelled = True
                    return
                if step["cmd"] == "T133":
                    command = {"T": 133, "X": int(step["pan_deg"]), "Y": int(step["tilt_deg"]),
                               "SPD": step["speed_raw"] or 0, "ACC": step["accel_raw"] or 0}
                else:
                    command = {"T": 134, "X": int(step["pan_deg"]), "Y": int(step["tilt_deg"]),
                               "SX": step["pan_speed_raw"] or 0, "SY": step["tilt_speed_raw"] or 0}
                CALLER.name, CALLER.run = "schedule", (run_id, step["step"])
                try:
                    with rover._lock:
                        sent = rover._pan_sent
                        if rover.link.send(command):
                            pan = float(command["X"])
                            if sent is not None and pan != sent:
                                rover.pan_approach = 1 if pan > sent else -1
                            rover._pan_sent = pan
                            rover.pan, rover.tilt = pan, float(command["Y"])
                finally:
                    CALLER.name, CALLER.run = None, None
        finally:
            self._schedule_event("cancelled" if cancelled else "completed")
            self._run = None

    def _sleep_until(self, t_ns: int) -> None:
        while not self._run_cancel.is_set():
            left = (t_ns - rec.now_ns()) * 1e-9
            if left <= 0:
                return
            self._run_cancel.wait(min(left, 0.05))

    def cancel_schedule(self) -> bool:
        if self._run is None:
            return False
        self._run_cancel.set()
        return True

    # --- the session's ends ----------------------------------------------------

    def start(self) -> None:
        for thread in self._threads:
            thread.start()
        self.rover.link.tap = self
        rec.announce({"dir": self.dir, "session": self.session, "kind": self.kind,
                      "camera_hz": self.config["camera_hz"],
                      "depth_hz": self.config["depth_hz"], "started_ns": self.started_ns})

    def stop(self) -> dict[str, Any]:
        """Stop recording, wait for the other processes to close, write the manifest."""
        self.cancel_schedule()
        deadline = time.monotonic() + 5.0
        while self._run is not None and time.monotonic() < deadline:
            time.sleep(0.05)
        rec.announce(None)
        self.rover.link.tap = None
        self._stopping.set()
        for thread in self._threads:
            thread.join(5.0)
        counts = {name: s.close() for name, s in self.streams.items()}
        deadline = time.monotonic() + CLOSE_WAIT_S
        missing = []
        for who in WRITERS:
            path = os.path.join(self.dir, ".closed-%s.json" % who)
            while not os.path.exists(path) and time.monotonic() < deadline:
                time.sleep(0.2)
            if not os.path.exists(path):
                missing.append(who)
                continue
            with open(path, encoding="utf-8") as f:
                for name, c in json.load(f).items():
                    if isinstance(c, dict) and "written" in c:
                        counts[name] = {"written": int(c["written"]),
                                        "dropped": int(c.get("dropped", 0))}
        return {"counts": counts, "missing": missing}


class RoverRecording:
    """The calls, mixed into Rover."""

    recording: Recording | None = None

    def _recording_snapshot(self, size):
        """For RoverCamera._snapshot: the session's frame instead of a grab, or None
        when no session holds the camera."""
        recording = self.recording
        return None if recording is None else recording.snapshot(size)

    def _tool_recording_start(self, arguments: dict[str, Any]) -> dict[str, Any]:
        if self.recording is not None:
            raise ValueError("a session is already open: %s" % self.recording.session)
        kind = arguments.get("kind")
        if kind not in KINDS:
            raise ValueError("`kind` is one of " + ", ".join(KINDS))
        rooms = arguments.get("rooms")
        if (not isinstance(rooms, list) or not rooms
                or not all(isinstance(r, str) and NAME.match(r) for r in rooms)):
            raise ValueError("`rooms` is a list of room names")
        if HELD_OUT["room"] in rooms and kind not in HELD_OUT_KINDS:
            raise ValueError("the bedroom is held out: a session there is %s only"
                             % ", ".join(HELD_OUT_KINDS))
        operator = arguments.get("operator")
        if not operator or not isinstance(operator, str):
            raise ValueError("`operator` says who is running the session")
        name = arguments.get("configuration") or "base"
        if name not in CONFIGURATIONS:
            raise ValueError("`configuration` is one of " + ", ".join(CONFIGURATIONS))
        exposure_ms = arguments.get("exposure_ms")
        if exposure_ms is not None and not 5 <= float(exposure_ms) <= 1000:
            raise ValueError("a manual exposure is 5 to 1000 ms")
        if time.time() < 1.7e9:
            raise ValueError("the rover's clock is not set yet; try again in a minute")
        if self._tracking.is_set():
            raise ValueError("face tracking holds the camera: stop it first")
        stamp = datetime.now().strftime("%Y-%m-%dT%H%M")
        session = "%s-%s" % (stamp, kind)
        directory = os.path.join(recordings_dir(), session)
        if os.path.exists(directory):
            raise ValueError("%s already exists: wait a minute" % session)
        config = {"name": name, "camera_hz": CONFIGURATIONS[name]["camera"],
                  "depth_hz": CONFIGURATIONS[name]["depth"],
                  "exposure_s": None if exposure_ms is None else float(exposure_ms) / 1000.0}
        os.makedirs(directory)
        import recording_inputs
        try:
            inputs = recording_inputs.build(self._recording_colour_lens())
        except Exception as exc:
            shutil.rmtree(directory, ignore_errors=True)
            raise ValueError("the calibration could not be read, so no session: %s" % exc)
        with open(os.path.join(directory, "inputs.json"), "w", encoding="utf-8") as f:
            json.dump(inputs, f, indent=1)
        recording = Recording(self, directory, session, kind, config)
        recording.meta = {"rooms": rooms, "operator": operator,
                          "clock_start": self._recording_clock(),
                          "services": self._recording_services(),
                          "map": self._recording_map()}
        self.recording = recording
        recording.start()
        # Held on while the session lasts (rover_depth.depth_tick), and switched on
        # now rather than at the next tick.
        threading.Thread(target=self.depth_wake, args=(10.0,), daemon=True).start()
        # The ROS half, which runs for sessions only and leaves when this one ends.
        if os.path.exists(ROS_RECORDER):
            import subprocess
            subprocess.Popen(["bash", ROS_RECORDER], stdin=subprocess.DEVNULL,
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                             start_new_session=True)
        return {"ok": True, "session": session, "dir": directory}

    def _tool_recording_stop(self, _arguments: dict[str, Any]) -> dict[str, Any]:
        recording = self.recording
        if recording is None:
            raise ValueError("no session is open")
        self.recording = None
        closed = recording.stop()
        manifest = write_manifest(recording, closed["counts"], self._recording_clock())
        return {"ok": True, "session": recording.session, "dir": recording.dir,
                "missing_writers": closed["missing"],
                "counts": manifest["final_counts"]}

    def _tool_recording_status(self, _arguments: dict[str, Any]) -> dict[str, Any]:
        recording = self.recording
        if recording is None:
            return {"ok": True, "open": False}
        queues, dropped = recording.counts()
        return {"ok": True, "open": True, "session": recording.session,
                "seconds": round((rec.now_ns() - recording.started_ns) * 1e-9, 1),
                "written": {n: s.written for n, s in recording.streams.items()},
                "queues": queues, "dropped": dropped,
                "waiting_for": recording.waiting_for(),
                "camera_error": recording.camera_error,
                "schedule": None if recording._run is None else recording._run["run_id"]}

    def _tool_recording_annotate(self, arguments: dict[str, Any]) -> dict[str, Any]:
        if self.recording is None:
            raise ValueError("no session is open")
        return self.recording.annotate(arguments)

    def _tool_recording_still(self, arguments: dict[str, Any]) -> dict[str, Any]:
        if self.recording is None:
            raise ValueError("no session is open")
        return self.recording.still(arguments)

    def _tool_recording_schedule(self, arguments: dict[str, Any]) -> dict[str, Any]:
        if self.recording is None:
            raise ValueError("no session is open")
        return self.recording.schedule(arguments)

    def _tool_recording_schedule_cancel(self, _arguments: dict[str, Any]) -> dict[str, Any]:
        if self.recording is None:
            raise ValueError("no session is open")
        return {"ok": True, "cancelled": self.recording.cancel_schedule()}

    # --- what the manifest needs from the rest of the daemon -----------------

    def _recording_colour_lens(self) -> dict[str, Any] | None:
        try:
            ranger = self._depth_ranger()
            lens = ranger.lens() if ranger is not None else None
        except Exception:
            lens = None
        if lens is None:
            return None
        return {"width_px": int(lens.width), "height_px": int(lens.height), "fx": lens.fx,
                "fy": lens.fy, "cx": lens.cx, "cy": lens.cy}

    def _recording_clock(self) -> dict[str, Any]:
        mono = rec.now_ns()
        wall = datetime.now().astimezone().isoformat()
        # ROS on this rover runs on the system's wall clock (2026-10-28).
        return {"mono_ns": mono, "wall": wall, "ros_ns": time.time_ns()}

    def _recording_services(self) -> list[str]:
        names = {"rover_daemon": "rover_daemon.py", "lidar_node": "lidar_node.py",
                 "base_node": "base_node.py", "localizer": "localizer_node.py",
                 "mapper": "async_slam_toolbox", "nav2": "controller_server",
                 "nav_bridge": "nav_bridge.py", "change_watch": "change_node.py",
                 "depth_service": "depth_server.py", "ros_recorder": "recorder_node.py",
                 "console": "drive_web.py", "voice": "session.py",
                 "world_state_perception": "run_perception", "visit_model": "scan_log.py"}
        running = []
        try:
            cmdlines = []
            for path in glob.glob("/proc/[0-9]*/cmdline"):
                try:
                    with open(path, "rb") as f:
                        cmdlines.append(f.read().replace(b"\0", b" ").decode(errors="replace"))
                except OSError:
                    pass
            for name, marker in names.items():
                if any(marker in c for c in cmdlines):
                    running.append(name)
        except Exception:
            pass
        return running

    def _recording_map(self) -> dict[str, Any]:
        map_id, digest = None, None
        try:
            status = self.nav.status() if getattr(self, "nav", None) else {}
            map_id = status.get("map_id")
        except Exception:
            pass
        graph = os.path.expanduser("~/.ugv/map/current.posegraph")
        if os.path.exists(graph):
            digest = _sha256(graph)
        return {"id": map_id or "unknown", "sha256": digest or "0" * 64}


def write_manifest(recording: Recording, counts: dict[str, Any], clock_end: dict[str, Any]) -> dict[str, Any]:
    """The closing step: final counts, the clock, every file's size and hash."""
    meta = recording.meta
    directory = recording.dir
    for path in glob.glob(os.path.join(directory, ".closed-*.json")) + \
            glob.glob(os.path.join(directory, ".status-*.json")):
        os.remove(path)
    try:
        with open(ROOMS_FILE, encoding="utf-8") as f:
            outlines = json.load(f).get("outlines", {})
    except (OSError, ValueError):
        outlines = {}
    try:
        with open(DEPLOY_STATE, encoding="utf-8") as f:
            state = json.load(f)
        commit = state["components"]["rover_daemon"]
        if not isinstance(commit, str):
            raise ValueError(commit)
    except Exception:
        commit = "0000000"
    deployed = os.path.expanduser("~/ugv/manifest.json")
    config = recording.config
    rates = dict(NOMINAL_HZ, camera=config["camera_hz"], depth=config["depth_hz"])
    acquired = {n for n in rates if n in counts}
    files = []
    for root, _dirs, names in os.walk(directory):
        for name in sorted(names):
            path = os.path.join(root, name)
            rel = os.path.relpath(path, directory).replace(os.sep, "/")
            if rel == "manifest.json":
                continue
            files.append({"path": rel, "size": os.path.getsize(path), "sha256": _sha256(path)})
    log = os.path.join(directory, "daemon_commands.jsonl")
    manifest = {
        "schema": "rover-recording", "version": 1, "file": "manifest",
        "session": recording.session, "kind": recording.kind,
        "day": recording.session[:10], "rooms": meta["rooms"],
        "rover_commit": commit,
        "deployment_manifest": {"path": "deploy/manifest.json",
                                "sha256": _sha256(deployed) if os.path.exists(deployed) else "0" * 64},
        "configuration": {
            "name": config["name"],
            "streams": {n: {"acquired": n in acquired, "nominal_hz": rates[n] if n in acquired else None}
                        for n in ("camera", "depth", "oak_imu", "board_feedback", "odometry",
                                  "lidar", "lidar_scan", "map_pose", "clock", "health")},
            "auto_exposure": config["exposure_s"] is None, "exposure_s": config["exposure_s"]},
        "map": meta["map"], "room_outlines": outlines, "held_out": HELD_OUT,
        "clock": {"start": meta["clock_start"], "end": clock_end},
        "services": meta["services"], "operator": meta["operator"],
        "daemon_log": {"path": "daemon_commands.jsonl",
                       "t_from": recording.first_command_ns or recording.started_ns,
                       "t_to": recording.last_command_ns or recording.started_ns,
                       "sha256": _sha256(log)},
        "final_counts": {n: {"written": int(c.get("written", 0)), "dropped": int(c.get("dropped", 0))}
                         for n, c in counts.items() if isinstance(c, dict)},
        "files": files,
    }
    with open(os.path.join(directory, "manifest.json"), "w", encoding="utf-8") as f:
        json.dump(manifest, f, indent=1)
    return manifest

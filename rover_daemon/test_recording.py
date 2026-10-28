"""Perception's session recordings: the writer every recording process shares.

What matters is what the checker on Perception's side will hold the files to:
a header on every file, sequence numbers that leave a gap exactly where a record
was dropped, restart records before a new epoch, frames' offsets that are the
bytes' real places, no NaN in a line -- and that a full queue drops rather than
waits.
"""
from __future__ import annotations

import json
import os
import tempfile
import threading
import time

from test_harness import check


def _lines(path):
    with open(path, encoding="utf-8") as f:
        return [json.loads(line) for line in f]


def test_a_stream_writes_its_header_records_and_frames():
    import session_record

    with tempfile.TemporaryDirectory() as root:
        s = session_record.Stream(root, "camera", "camera/index.jsonl", "camera/frames.mjpeg")
        s.put({"t_arr": 5, "t_dev": 4, "length": 0, "offset": 0}, blob=b"abc")
        s.put({"t_arr": 6, "t_dev": float("nan"), "length": 0, "offset": 0}, blob=b"defgh")
        counts = s.close()
        lines = _lines(os.path.join(root, "camera", "index.jsonl"))
        check("a stream's file opens with the schema's header",
              lines[0], {"schema": "rover-recording", "version": 1, "stream": "camera"})
        check("...then its records, numbered from 0 in order",
              [(r["seq"], r["t_arr"]) for r in lines[1:]], [(0, 5), (1, 6)])
        check("...each frame's offset and length its bytes' real place",
              [(r["offset"], r["length"]) for r in lines[1:]], [(0, 3), (3, 5)])
        with open(os.path.join(root, "camera", "frames.mjpeg"), "rb") as f:
            check("...and the bytes back to back", f.read(), b"abcdefgh")
        check("...a NaN written as null, never as NaN", lines[2]["t_dev"], None)
        check("...and the counts it says it wrote", counts, {"written": 2, "dropped": 0})


def test_a_full_queue_drops_counts_and_leaves_a_gap():
    import session_record

    class StuckDisk:
        """A binary file whose first write does not return until released."""

        def __init__(self):
            self.go = threading.Event()

        def tell(self):
            return 0

        def write(self, blob):
            self.go.wait(5.0)

        def flush(self):
            pass

        def close(self):
            pass

    with tempfile.TemporaryDirectory() as root:
        s = session_record.Stream(root, "depth", "depth/index.jsonl", "depth/frames.u16",
                                  capacity=2)
        disk = s._binary = StuckDisk()
        s.put({"n": -1, "offset": 0}, blob=b"x")
        time.sleep(0.2)  # the writer takes it and sticks
        took = time.monotonic()
        results = [s.put({"n": n}) for n in range(50)]
        check("put never waits on a full queue", time.monotonic() - took < 0.5, True)
        disk.go.set()
        counts = s.close()
        lines = _lines(os.path.join(root, "depth", "index.jsonl"))[1:]
        check("a stuck disk fills the queue and the rest are dropped and counted",
              (results.count(True), counts["dropped"]), (2, 48))
        check("...and the records written keep the numbers they were put with, a gap "
              "where the dropped ones were", [r["seq"] for r in lines], [0, 1, 2])


def test_a_restart_record_comes_before_the_new_epoch():
    import session_record

    with tempfile.TemporaryDirectory() as root:
        s = session_record.Stream(root, "depth", "depth/index.jsonl")
        s.put({"t_arr": 1})
        s.restart(t_arr=2, seq=0)
        s.put({"t_arr": 3})
        s.close()
        lines = _lines(os.path.join(root, "depth", "index.jsonl"))[1:]
        check("a restart record in line, its epoch 1 and the new epoch's first seq",
              lines, [{"t_arr": 1, "seq": 0},
                      {"restart": True, "epoch": 1, "seq": 0, "t_arr": 2},
                      {"t_arr": 3, "seq": 0}])


def test_a_follower_opens_and_closes_with_the_session():
    import session_record

    with tempfile.TemporaryDirectory() as root:
        was = os.environ.get(session_record.CURRENT_ENV)
        os.environ[session_record.CURRENT_ENV] = os.path.join(root, "current.json")
        try:
            seen = []
            follower = session_record.Follower(
                "lidar", lambda session: seen.append(("open", session["dir"])) or "handle",
                lambda handle: seen.append(("close", handle)) or {"lidar": {"written": 3}},
                poll_s=3600)
            follower.poll()
            check("no announcement opens nothing", seen, [])
            directory = os.path.join(root, "2026-10-28T1200-commissioning")
            os.makedirs(directory)
            session_record.announce({"dir": directory, "session": "x"})
            follower.poll()
            follower.poll()
            check("an announcement opens the streams once", seen, [("open", directory)])
            session_record.announce(None)
            follower.poll()
            check("...and its removal closes them", seen[-1], ("close", "handle"))
            with open(os.path.join(directory, ".closed-lidar.json")) as f:
                check("...leaving word of what was written", json.load(f), {"lidar": {"written": 3}})
            follower.stop()
        finally:
            if was is None:
                os.environ.pop(session_record.CURRENT_ENV, None)
            else:
                os.environ[session_record.CURRENT_ENV] = was


class TappingLink:
    """A board link that tells a session's tap of each write, the way
    board_link.SerialLink does, numbering them as it goes."""

    def __init__(self):
        self.sent = []
        self.tap = None
        self.writes = 0

    def send(self, command, client="daemon", note=None):
        self.sent.append(command)
        seq, self.writes = self.writes, self.writes + 1
        if self.tap is not None:
            self.tap.command(seq, time.monotonic_ns(), True, client,
                             json.dumps(command, separators=(",", ":")), command, note)
        return True

    def telemetry(self):
        return {"T": 1001, "v": 1153}

    def describe(self):
        return "a tapping fake"


def test_a_session_records_commands_feedback_a_schedule_and_its_manifest():
    import rover_daemon
    import recording_inputs
    import rover_recording
    import session_record

    saved = {k: os.environ.get(k) for k in (session_record.CURRENT_ENV,
                                            rover_recording.RECORDINGS_ENV)}
    odometry_was = recording_inputs.ODOMETRY_FILE
    # Nothing else is writing here, so waiting the rover's quarter-minute for the
    # other processes to close would only slow the suite.
    wait_was, rover_recording.CLOSE_WAIT_S = rover_recording.CLOSE_WAIT_S, 0.5
    with tempfile.TemporaryDirectory() as root:
        os.environ[session_record.CURRENT_ENV] = os.path.join(root, "current.json")
        os.environ[rover_recording.RECORDINGS_ENV] = os.path.join(root, "recordings")
        recording_inputs.ODOMETRY_FILE = os.path.join(root, "odometry.json")
        with open(recording_inputs.ODOMETRY_FILE, "w") as f:
            json.dump({"gyro_lsb_per_dps": 16.14613, "ticks_per_metre": 107.206}, f)
        try:
            link = TappingLink()
            rover = rover_daemon.Rover(link, "unused", device="/dev/null")
            refused = rover.call("recording_start", {"kind": "training", "rooms": ["bedroom"],
                                                      "operator": "test"})
            check("a training session in the held-out bedroom is refused",
                  (refused["ok"], "held out" in refused.get("error", "")), (False, True))
            got = rover.call("recording_start", {"kind": "commissioning",
                                                  "rooms": ["living"], "operator": "test"})
            check("a commissioning session starts", got["ok"], True)
            directory = got["dir"]
            check("...and is announced to the other processes",
                  session_record.announced()["dir"], directory)
            check("...with the calibration written at its start",
                  json.load(open(os.path.join(directory, "inputs.json")))["fisheye"]["values"]["b1"],
                  -0.0578)
            rover.call("look_at", {"pan": 10, "tilt": 20})
            link.send({"T": 11, "L": 90, "R": 90}, client="bridge",
                      note={"linear": 0.2, "angular": 0.0, "keepalive": False,
                            "caller": "cmd_vel"})
            rover.recording.feedback(
                [b'{"T":1001,"L":0,"R":0,"ax":112,"ay":-64,"az":8482,"gx":-7,"gy":6,'
                 b'"gz":8,"mx":-245,"my":803,"mz":1301,"odl":10,"odr":12,"v":1137}'],
                time.monotonic_ns())
            check("an annotation starts a unit",
                  rover.call("recording_annotate", {"kind": "change_start", "unit": "c1",
                                                    "object": "mug-3", "change": "moved"})["ok"],
                  True)
            check("...and a second start of the same unit is refused",
                  rover.call("recording_annotate", {"kind": "trial_start", "unit": "c1"})["ok"],
                  False)
            rover.call("recording_annotate", {"kind": "change_end", "unit": "c1",
                                              "object": "mug-3", "change": "moved"})
            steps = [{"step": 0, "t_offset_ns": 0, "cmd": "T133", "pan_deg": 5, "tilt_deg": 20,
                      "speed_raw": 0, "accel_raw": 0, "pan_speed_raw": None,
                      "tilt_speed_raw": None},
                     {"step": 1, "t_offset_ns": 100_000_000, "cmd": "T133", "pan_deg": -5,
                      "tilt_deg": 20, "speed_raw": 0, "accel_raw": 0, "pan_speed_raw": None,
                      "tilt_speed_raw": None}]
            run = rover.call("recording_schedule", {"schedule_id": "sweep1", "steps": steps,
                                                    "lead_s": 0.1})
            check("a schedule is fixed and run", run["ok"], True)
            deadline = time.monotonic() + 5
            while rover.recording._run is not None and time.monotonic() < deadline:
                time.sleep(0.05)
            stopped = rover.call("recording_stop", {})
            check("the session stops", stopped["ok"], True)
            check("...and nothing is announced any more", session_record.announced(), None)

            def read(name):
                return _lines(os.path.join(directory, name))[1:]

            log = read("daemon_commands.jsonl")
            gimbal = read("gimbal_cmd.jsonl")
            chassis = read("chassis_cmd.jsonl")
            by_seq = {r["seq"]: r for r in log}
            check("every gimbal and motor command is in the log under its own daemon_seq, "
                  "with the same stamp and line",
                  all(by_seq[r["daemon_seq"]]["line"] == r["line"]
                      and by_seq[r["daemon_seq"]]["t_arr"] == r["t_arr"]
                      for r in gimbal + chassis), True)
            check("a command from a call names the call",
                  [r["caller"] for r in gimbal if r["run_id"] is None][:1], ["look_at"])
            check("the schedule's commands name its run and steps",
                  [(r["run_id"], r["schedule_step"], r["pan_deg"]) for r in gimbal
                   if r["run_id"] is not None],
                  [("sweep1-run1", 0, 5.0), ("sweep1-run1", 1, -5.0)])
            check("a motor command carries the velocity it was mixed from",
                  [(r["left_raw"], r["linear"], r["keepalive"], r["caller"]) for r in chassis],
                  [(90, 0.2, False, "cmd_vel")])
            events = read("gimbal_schedule.jsonl")
            check("the run is recorded fixed, started and completed",
                  [e["event"] for e in events], ["fixed", "started", "completed"])
            stored = json.load(open(os.path.join(directory, "schedules", "sweep1.json")))
            check("...its steps' plans are the first plan plus each stored offset",
                  [s["t_plan"] - events[0]["steps"][0]["t_plan"] for s in events[0]["steps"]],
                  [s["t_offset_ns"] for s in stored["steps"]])
            feedback = read("board_feedback.jsonl")
            check("a feedback line keeps its raw integers",
                  (feedback[0]["enc_left"], feedback[0]["accel_raw"], feedback[0]["volt_raw"]),
                  (10, [112, -64, 8482], 1137))
            manifest = json.load(open(os.path.join(directory, "manifest.json")))
            check("the manifest lists every file with its hash, itself excepted",
                  sorted(f["path"] for f in manifest["files"])[:3],
                  ["annotations.jsonl", "board_feedback.jsonl", "camera/frames.mjpeg"])
            check("...holds the bedroom out, with its door",
                  manifest["held_out"], {"room": "bedroom", "has_door": True,
                                         "visibility_rule": None})
            check("...and says the other processes' streams never closed, here",
                  stopped["missing_writers"], ["depth", "lidar", "ros"])
        finally:
            recording_inputs.ODOMETRY_FILE = odometry_was
            rover_recording.CLOSE_WAIT_S = wait_was
            for k, v in saved.items():
                if v is None:
                    os.environ.pop(k, None)
                else:
                    os.environ[k] = v


TESTS = (
    test_a_stream_writes_its_header_records_and_frames,
    test_a_full_queue_drops_counts_and_leaves_a_gap,
    test_a_restart_record_comes_before_the_new_epoch,
    test_a_follower_opens_and_closes_with_the_session,
    test_a_session_records_commands_feedback_a_schedule_and_its_manifest,
)

"""Track a face from the rover itself: the Pi holds the loop, the OAK does the seeing.

The rover's own machine is a Pi 1 Model B -- one ARM1176 at 700 MHz with no NEON
-- and it cannot run a face detector at any useful rate. What it *can* do is hold
a control loop, and it is the only machine wired to the ESP32, so this is where
the loop belongs. The picture goes out to a detector and the boxes come back; the
servo commands never leave the board.

That detector used to be YuNet on MEDIA's CPU, across the network. It is now
oak_detect/server.py on this same Pi, running the graph on the Myriad X inside the
OAK camera -- so the rover sees without MEDIA, or any network, being up at all.
The protocol did not change, which is why this script did not have to.

    python3 track_face_pi.py                          # camera, the OAK, ttyAMA0
    python3 track_face_pi.py --service 192.168.1.3:8768   # or a detector elsewhere
    python3 track_face_pi.py --host 192.168.1.22      # command the board over WiFi
    python3 track_face_pi.py --no-move                # detect and report, command nothing
    python3 track_face_pi.py --no-scan                # stay put when there is nobody about

Headless, unlike track_face.py: this host has no display by choice, so there is a
status line rather than a window. The aiming, the thresholds and the sweep are the
same in both -- they come from aiming.py, which exists so that these two scripts
cannot drift into being two different robots.

**Nothing here decodes a picture.** The camera is asked for MJPEG and those exact
bytes are forwarded unopened; the Pi never sees a pixel. That is not an
optimisation, it is the design: decoding one 640x480 JPEG costs 93 ms on this
machine, measured, which is three frames' worth of work to produce something
nothing here needs. Forwarding them costs 30% of the core at 30 fps.

MEASURED, on this rover, over WiFi (an RTL8188FTV dongle on wlan0):

    what                                    320x240    640x480    1280x720
    exposure to a whole frame in hand        39 ms      41 ms      43 ms
    the picture to MEDIA, raw TCP             4 ms       9 ms      20 ms
    sustained rate, forwarded                30.0 fps   30.0 fps   29.8 fps
    of the Pi's one core                      ~15%       ~30%       52%

640x480 is the default, and the reason is the last row: 720p costs half the
machine and buys nothing, because the detector works at DETECT_WIDTH = 640 and
throws the rest away. The link is not the constraint either way -- the WiFi
ceiling here measured 38 Mbit/s, against 8.4 for this stream.

The rest of the round trip, measured from the Pi against the service on MEDIA:
**22.6 ms** for a 35 kB POST, of which about 6 is the detection and 13 this
machine's own HTTP stack -- a 700 MHz ARM11 is not fast at anything, including
sockets. So a box was in hand roughly 65 ms after the light that made it, against
the 266 ms of dead time measured for track_face.py with the camera on the
workstation's own USB. Moving the camera onto the rover made the loop faster, not
slower, and the command path is now a wire rather than a radio.

**Against the OAK on this Pi it is slower than that, and the reason is JPEG.**
Measured 2026-08-18, one frame end to end over loopback: 85 ms to decode a 640x480
MJPEG frame, 41 ms for the detection itself, and 41 ms of this machine's HTTP
stack, for about 190 ms a frame -- and with the scan matcher also running, the
tracking loop settles at **2.3 fps**. Nothing there is the camera's fault: the
inference is the cheapest part, and what costs is that the picture now has to be
decoded on the rover instead of being handed to a 5700G. The way to get the rate
back is to stop sending JPEG at all -- this camera offers uncompressed YUYV, and
the graph would take those pixels directly -- which is a change to how the frame
is captured rather than to anything about the detector.

That figure was 73 ms until the detector stopped writing its replies in two
sends: see the Nagle note in face_detect/server.py. Worth recording how that hid,
because the wrong answer was reached twice. The service was first reached through
an SSH tunnel, which measured 85 ms, and a tunnel is an easy thing to blame --
but the direct path measured 73, so the tunnel was never more than a few
milliseconds of it. What made the tunnel look guilty was comparing it against a
stub on the Pi's own loopback, which was quick for a reason that had nothing to
do with the network: that stub set disable_nagle_algorithm and the real service
did not. A control that differs from the thing it is controlling for is not a
control. The link itself is 3-4 ms, and always was.

**The dead time is measured per frame, not assumed.** V4L2 stamps every buffer
with the start of its exposure (`ts-monotonic, ts-src-soe`), v4l2-ctl prints that
on stderr, and this pairs it back with the frame it belongs to and sends it along
as an opaque `ts` that the detector echoes. Gimbal.track() is then told exactly
when the picture it is answering was taken, rather than assuming DEAD_TIME_S --
a constant that was measured once and varied by +-50 ms while being measured.
Nothing has to agree about clocks: the stamp only ever means something here.

**Frames are dropped on purpose.** The camera runs at 30 fps and a round trip
takes longer than a frame, so a queue would form -- and a queue here is not
slowness, it is a rover aiming at where somebody was a second ago, which looks
exactly like the divergence aiming.py's dead time compensation exists to prevent.
So the reader keeps only the newest complete frame and the loop sends one at a
time, waiting for the boxes before sending the next. The rate falls out of the
round trip and the staleness cannot accumulate. Dropped frames are counted and
shown, because a silently decimated stream would look identical to a healthy one.

**If the detector goes away, the rover stops.** That was written when the
detector was a desktop that reboots, and it still holds now that it is a service
on this same Pi: it drives a USB device that can be unplugged or brown out, and it
spends about 6 s uploading firmware and a graph before it answers again after a
restart. LOST_GRACE_S covers a blink, not that. After SERVICE_GRACE_S of failures
the camera is centred and left alone
rather than sweeping blind, and the loop keeps retrying quietly until the service
answers again.
"""

import argparse
import json
import os
import re
import signal
import socket
import subprocess
import sys
import threading
import time

from aiming import (
    DEADBAND, DETECT_WIDTH, GAIN, KEEP_SCORE, MAX_DT, SCAN_AFTER_S, SCAN_RATE,
    Gimbal, Scan, Target, clamp,
)

# --- the camera -----------------------------------------------------------

DEFAULT_DEVICE = "/dev/video0"
# The rover's USB module: 0abd:8050, MJPG at 30 fps from 160x120 to 2592x1944.
# 640x480 because the detector works at DETECT_WIDTH and anything more is thrown
# away after costing the Pi real CPU to forward -- see the table in the docstring.
DEFAULT_SIZE = (DETECT_WIDTH, 480)
# What one frame is worth waiting for before giving up on the camera entirely.
FRAME_TIMEOUT_S = 2.0
# How long to wait for a frame's exposure stamp, which arrives on a different pipe
# and can lose the race with the frame's own bytes -- see Camera._stamp_for(). Six
# milliseconds is the measured p90; this is the ceiling before giving up on it.
STAMP_WAIT_S = 0.03
# Fallback for a frame that could not be paired with a V4L2 stamp: how old a frame
# already is when this process has all of it. MEASURED, over 60 frames each, with
# `stdbuf -o0` and this pairing:
#
#     320x240   39.0 ms (p90 39.3)        buffered: 65.5
#     640x480   40.9      (p90 41.4)                65.6
#    1280x720   43.2      (p90 44.6)                61.7
#
# Barely a function of the picture at all, which is what says it is the camera's
# own pipeline rather than anything on this machine. The second column is what
# v4l2-ctl does without `stdbuf`: its stdout is a pipe, so libc buffers it, and
# the tail of every frame waits for the next one to push it out -- 25 ms of pure
# stdio, in the one number the whole control law is most sensitive to. That is
# also why an earlier pass here recorded 98 ms and called it the camera.
CAMERA_LAG_S = 0.041
# A stamp that would make the frame this old is not this frame's. The pairing is
# by content and resynchronises itself, so this is a backstop rather than the
# mechanism -- but a wrong answer here is a lie to the controller about when it
# was looking, which is worse than no answer at all.
MAX_FRAME_AGE_S = 0.5
# JPEG's end-of-image marker. Safe to scan for: 0xFF inside entropy-coded data is
# byte-stuffed as FF 00, so FFD9 only appears where the picture ends. Its opposite
# number is what tells a whole picture from the tail of one, which is the only
# check either camera path here makes on what it hands back.
JPEG_EOI = b"\xff\xd9"
JPEG_SOI = b"\xff\xd8"
READ_CHUNK = 4096
# Buffers whose bytes may still be in flight. Deeper than anything measured (the
# pairing runs one behind at most), shallow enough that a stamp cannot be matched
# against a frame from seconds ago.
STAMP_BACKLOG = 16

# How many frames a one-shot capture asks for, and how long to allow it. Three
# rather than one because the first frame off a camera that was closed a moment
# ago is exposed for whatever the room was last time: auto-exposure needs a couple
# of frames to settle, and on this camera they cost 30 ms each against a fixed
# 0.56 s of opening the device. The last frame is the one worth having.
SNAPSHOT_FRAMES = 3
# Generous, because the failure it is guarding is a camera another process holds
# open -- v4l2-ctl then sits there rather than refusing -- and the honest answer to
# that is a complaint, not a wait that never ends.
SNAPSHOT_TIMEOUT_S = 8.0
# Where camera work sits relative to everything else on this host. Positive is
# *down*: it asks the kernel to prefer the thread driving the rover whenever the
# two want the core at the same moment. See stand_aside().
CAMERA_NICE = 10

# --- the detector ---------------------------------------------------------

# By address, not by name. The rover is reached by name because it has two
# addresses and which one is live varies; MEDIA has one fixed address, so a
# name buys no agility here and costs mDNS. Measured from the Pi, three
# lookups of `media.local` in a row: 344 ms, **5193 ms**, 194 ms. This sits
# in a control loop with a 1 s service timeout, so that outlier is a stall,
# and a transient resolver failure is a frame nobody looked at. Pass
# --service media.local:8768 to point it at a detector on another host instead.
DEFAULT_SERVICE = "127.0.0.1:8768"  # oak_detect/server.py, on this Pi
# One round trip is ~15 ms of network and detection. This is long enough that a
# service busy with another client is waited for and short enough that a dead one
# is noticed within a frame or two.
SERVICE_TIMEOUT_S = 1.0
# How long the detector may be unreachable before the rover gives up and centres.
# Longer than a dropped packet, far shorter than a service restart.
SERVICE_GRACE_S = 3.0
# Between retries once it has been declared gone. The service takes seconds to
# come back at best, and a Pi 1 has better things to do than knock every 40 ms.
SERVICE_RETRY_S = 1.0

# --- the board ------------------------------------------------------------

DEFAULT_SERIAL = "/dev/ttyAMA0"
BAUD = 115200
SERIAL_CONSOLE_HINT = ("Is a getty still on it? `systemctl status serial-getty@"
                       + DEFAULT_SERIAL.rsplit("/", 1)[-1] + "`")

STATUS_HZ = 5


def stand_aside():
    """Drop this thread's claim on the core, so that driving outranks looking.

    Linux schedules threads, not processes, and `nice` here applies to whichever
    thread calls it -- which is what makes this usable from inside a process whose
    other threads must *not* be slowed down. Nothing needs privileges: asking to
    matter less always succeeds, and asking to matter more is the call that would
    have needed them.

    It is a preference and not a guarantee, and on this rover it is the weaker half
    of the answer. Within one interpreter the GIL decides who runs, not the
    scheduler, so a thread that has the GIL and work to do keeps the core whatever
    its niceness -- which is why the camera paths below are built to stop *asking*
    for frames nobody wants rather than merely to ask politely.

    So this is for the long-lived work only: the threads that reassemble the 30 fps
    feed while face tracking runs, and the v4l2-ctl behind it. It is deliberately
    *not* used on a one-shot capture, where the caller is waiting on the result and
    the cost of arranging it exceeds anything it saves -- see snapshot().
    """
    try:
        os.nice(CAMERA_NICE)
    except Exception:
        pass  # not Linux, or already at the floor: a preference, not a requirement


def die_with_parent():
    """Ask the kernel to kill this child when its parent goes, and stand aside.

    v4l2-ctl holds /dev/video0 exclusively, so an orphan is not a stray process
    but a camera nothing else can open until somebody finds it with `fuser`. The
    ordinary paths -- Ctrl-C, SIGTERM -- are handled below and clean up properly;
    this covers the ones that cannot be, a SIGKILL or the interpreter dying under
    the process. PR_SET_PDEATHSIG is 1, and the signal it should send is SIGTERM.

    **"Parent" here means the thread that started the child, not the process.** So
    a camera opened on a short-lived thread loses v4l2-ctl the moment that thread
    finishes, which is why every caller in this file opens one from a thread that
    lives as long as the camera should: the tracking loop, or `main`. A one-shot
    picture does not use this class at all -- see snapshot().
    """
    import ctypes

    stand_aside()
    try:
        ctypes.CDLL("libc.so.6", use_errno=True).prctl(1, signal.SIGTERM, 0, 0, 0)
    except Exception:
        pass  # not Linux, or no libc under that name: the explicit close still runs


class Stopping(Exception):
    """SIGTERM arrived. Raised in the main thread so the `finally` block runs."""


def stop_on_sigterm():
    """Make `kill`, `timeout` and systemd stop this the way Ctrl-C does.

    Not a nicety. The camera's angles are a model kept true by centring on the
    way out, so a run that is terminated rather than interrupted leaves the servos
    somewhere the next run will not know about -- and the next run's first
    correction is then wrong by however far they were left. See aiming.Gimbal.
    """
    def handler(signum, frame):
        raise Stopping()

    signal.signal(signal.SIGTERM, handler)
    signal.signal(signal.SIGHUP, handler)


class Camera:
    """The camera as a stream of MJPEG frames, newest only.

    v4l2-ctl does the capturing -- there is no OpenCV on this host and no reason
    to want one, since nothing here looks inside a frame. Its `--verbose` output
    carries each buffer's start-of-exposure timestamp, which is the whole reason
    for reading its stderr rather than discarding it.

    Two threads, because the two pipes have to be drained independently: stderr
    filling while only stdout is read would deadlock v4l2-ctl, and stdout filling
    while only stderr is read would stall the camera. What they produce is a
    single slot holding the most recent complete frame. Older frames are dropped
    where they lie -- see the docstring: a queue of frames is a queue of stale
    aiming errors, and the loop is better served by the newest picture than by
    every picture.

    `stdbuf -o0` is load bearing. v4l2-ctl writes frames with stdio, and stdio
    buffers when its output is a pipe, so the tail of each frame sits in libc
    waiting for the next frame to push it out. That is 25 ms added to every
    measurement of when the picture was taken -- see CAMERA_LAG_S, where both
    numbers are recorded.
    """

    def __init__(self, device=DEFAULT_DEVICE, size=DEFAULT_SIZE):
        self.device = device
        self.size = size
        self.dropped = 0
        self.unpaired = 0
        self.complaints = []                # what v4l2-ctl said, if it went wrong
        self._lock = threading.Lock()
        self._fresh = threading.Event()
        self._latest = None
        self._pending = []                  # (bytesused, exposed_at) awaiting frames
        self._stamps = threading.Condition()
        self._stop = threading.Event()
        argv = ["v4l2-ctl", "-d", device,
                "--set-fmt-video=width=%d,height=%d,pixelformat=MJPG" % size,
                "--stream-mmap", "--stream-to=-", "--verbose"]
        if os.path.exists("/usr/bin/stdbuf"):
            argv = ["stdbuf", "-o0"] + argv
        self.proc = subprocess.Popen(
            argv, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0,
            preexec_fn=die_with_parent)
        self._readers = [
            threading.Thread(target=self._read_frames, daemon=True),
            threading.Thread(target=self._read_stamps, daemon=True),
        ]
        for thread in self._readers:
            thread.start()

    def _read_stamps(self):
        """Buffer sizes and exposure times, off v4l2-ctl's stderr, as printed.

        One line per delivered buffer, carrying both the timestamp and how many
        bytes that buffer holds -- and the byte count is what makes the pairing
        honest, since it identifies *which* frame the stamp belongs to rather
        than merely how many have gone by.
        """
        stand_aside()
        pattern = re.compile(rb"bytesused: (\d+) ts: (\d+\.\d+)")
        for line in self.proc.stderr:
            if self._stop.is_set():
                return
            found = pattern.search(line)
            if found:
                with self._stamps:
                    self._pending.append((int(found.group(1)), float(found.group(2))))
                    del self._pending[:-STAMP_BACKLOG]
                    self._stamps.notify_all()
            elif b"ailed" in line or b"rror" in line:
                # Everything v4l2-ctl says that is not a buffer. Kept because the
                # one that matters -- "Device or resource busy" -- otherwise
                # surfaces as a camera that delivers nothing, which reads like
                # broken hardware rather than a device somebody else has open.
                self.complaints.append(line.decode(errors="replace").strip())
                del self.complaints[:-6]

    def _read_frames(self):
        # Reassembling 30 frames a second is the expensive half of holding this
        # camera open, and on this one core it is expensive enough to matter to the
        # rover's control loop. Whoever wants a picture waits; the wheels do not.
        stand_aside()
        buf = b""
        scan = 0
        while not self._stop.is_set():
            chunk = self.proc.stdout.read(READ_CHUNK)
            if not chunk:
                return
            buf += chunk
            while True:
                end = buf.find(JPEG_EOI, scan)
                if end < 0:
                    # Only the last byte can be half of a marker; keep it in view.
                    scan = max(0, len(buf) - 1)
                    break
                frame, buf, scan = buf[:end + 2], buf[end + 2:], 0
                self._offer(frame, self._stamp_for(len(frame)))

    def _stamp_for(self, length):
        """When the frame now in hand was exposed, on time.monotonic()'s clock.

        The stamps are CLOCK_MONOTONIC already -- the same clock, on the same
        machine -- so nothing is converted, only matched. Matching is by the
        buffer's own byte count rather than by arrival order, and the difference
        matters: order alone stays permanently offset after a single hiccup, and
        an offset of even one frame is 33 ms of lie about when the camera was
        looking. Everything older than the match is dropped, since those are
        buffers whose bytes never arrived.

        Unbuffered, the frame usually beats its own stderr line here -- measured
        p90 6 ms -- so this waits rather than giving up. Waiting is free: the
        next thing this frame does is cross a network.
        """
        deadline = time.monotonic() + STAMP_WAIT_S
        with self._stamps:
            while True:
                hit = next((i for i, s in enumerate(self._pending) if s[0] == length),
                           None)
                if hit is not None:
                    stamp = self._pending[hit][1]
                    del self._pending[:hit + 1]
                    if 0 <= time.monotonic() - stamp <= MAX_FRAME_AGE_S:
                        return stamp
                    break
                if not self._stamps.wait(max(0.0, deadline - time.monotonic())):
                    break
        self.unpaired += 1
        return time.monotonic() - CAMERA_LAG_S

    def _offer(self, frame, exposed_at):
        with self._lock:
            if self._latest is not None:
                self.dropped += 1
            self._latest = (frame, exposed_at)
            self._fresh.set()

    def latest(self, timeout=FRAME_TIMEOUT_S):
        """The newest complete frame and when it was exposed, or None."""
        if not self._fresh.wait(timeout):
            return None
        with self._lock:
            frame, self._latest = self._latest, None
            self._fresh.clear()
        return frame

    def alive(self):
        return self.proc.poll() is None

    def close(self):
        self._stop.set()
        try:
            self.proc.terminate()
            self.proc.wait(timeout=2)
        except Exception:
            self.proc.kill()


def split_jpegs(buf):
    """The complete pictures in a run of concatenated JPEGs, in order.

    v4l2-ctl writes one buffer after another with nothing between them, so the
    boundaries are the markers themselves. Both are scanned for, and requiring the
    start as well as the end is what makes a fragment fall out rather than be
    handed back: a run that begins mid-picture has no start-of-image before its
    first end-of-image, and that first end is skipped instead of terminating
    something that was never a whole frame.
    """
    frames, at = [], 0
    while True:
        start = buf.find(JPEG_SOI, at)
        if start < 0:
            return frames
        end = buf.find(JPEG_EOI, start + 2)
        if end < 0:
            return frames          # the tail of a capture that was cut short
        frames.append(buf[start:end + 2])
        at = end + 2


def snapshot(device=DEFAULT_DEVICE, size=DEFAULT_SIZE, frames=SNAPSHOT_FRAMES,
             timeout=SNAPSHOT_TIMEOUT_S):
    """A few complete frames from a camera that is opened for them and then shut.

    Returns `(frames, complaint)`: a list of `(jpeg, exposed_at)` oldest first, and
    a sentence about why the list is short or empty. A short list is not an error on
    its own -- one frame is enough to look at something.

    **This exists because of what holding the camera open costs the rover, not
    because of what opening it costs.** `Camera` above is a feed: v4l2-ctl streams
    at 30 fps for as long as it lives and the reader threads reassemble every frame
    whether or not anybody wants one. Measured on the Pi against the 10 Hz control
    loop in [lidar_slam/navigator.py](../lidar_slam/navigator.py), with the rover
    standing still and one picture taken:

        camera closed      9.94 revolutions/s matched,  0.0% dropped, replies  16 ms
        camera streaming   7.52                      , 22.1% dropped, replies 109 ms

    A quarter of the lidar revolutions, and it lasted the whole twenty seconds the
    daemon kept the camera warm for -- long after the picture had been sent. Since
    the scan matcher is the only odometer this rover has, those revolutions are the
    measurement that `drive` closes its loop on, so a photograph taken on the move
    was quietly corrupting the thing keeping the rover off the walls.

    A bounded capture is 0.56 s on this host, start to exit, and leaves nothing
    behind to compete with anything. It is strictly the better way to take one
    picture, so it is what every one-shot caller uses; the feed is now only for
    tracking a face, which genuinely needs 30 fps.

    The stamps here are honest but coarse -- one clock reading as v4l2-ctl exits,
    less the camera's own pipeline lag, rather than the per-buffer V4L2 timestamps
    `Camera._stamp_for` pairs up. Nothing steers on them: they reach the detector as
    an opaque identity token and come back untouched. Aiming a gimbal from a stamp
    this rough would not be safe, and nothing that aims uses this path.
    """
    argv = ["v4l2-ctl", "-d", device,
            "--set-fmt-video=width=%d,height=%d,pixelformat=MJPG" % size,
            "--stream-mmap", "--stream-count=%d" % max(1, int(frames)),
            "--stream-to=-"]
    # **No preexec_fn here, deliberately.** Neither of the two things one would
    # reach for is worth what it costs on this path. There is nothing to nice down:
    # v4l2-ctl is about 3% of the core and this call is waiting on it, so standing
    # it aside only makes the caller wait. And PR_SET_PDEATHSIG buys nothing either,
    # because the capture is bounded and the timeout below already covers a v4l2-ctl
    # that hangs -- it is the *feed* that can be orphaned, not this.
    #
    # The cost is not small and it is not obvious. Any preexec_fn drops CPython onto
    # a fork-and-run-Python-in-the-child path, and in a process whose GIL is as busy
    # as this daemon's the child's wait for the interpreter dominates everything
    # else. Measured on the Pi with two GIL-hungry threads running: 0.82 s without a
    # preexec_fn against 3.13 s with one, for a capture that is 0.6 s on its own. A
    # photograph is on a tool call the voice service gives 12 s in total, so those
    # seconds are the difference between an answer and a timeout.
    try:
        done = subprocess.run(argv, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                              timeout=timeout)
    except subprocess.TimeoutExpired:
        return [], (f"the camera at {device} did not deliver a frame in "
                    f"{timeout:.0f}s; something else may be holding it open")
    except OSError as error:
        return [], f"could not run v4l2-ctl for {device}: {error}"
    got = split_jpegs(done.stdout)
    if not got:
        # v4l2-ctl's own words, which is the difference between "broken hardware"
        # and "somebody else has the camera" -- the second says "Device or
        # resource busy" and is the one that actually happens here.
        said = " ".join(line for line in done.stderr.decode(errors="replace").split("\n")
                        if "ailed" in line or "rror" in line or "usy" in line).strip()
        return [], (f"the camera at {device} gave no whole picture"
                    + (f": {said}" if said else ""))
    at = time.monotonic() - CAMERA_LAG_S
    return [(jpeg, at) for jpeg in got], ""


class Detector:
    """The face detector, over one kept-open connection.

    Strictly one request at a time, which is the backpressure: the next frame is
    not sent until this one's boxes are back, so nothing can queue anywhere and
    the loop rate is simply whatever the round trip allows.

    A failed request is reported rather than raised. The service being away is an
    expected state on a rover -- see the docstring -- and the loop handles it by
    stopping, not by dying.
    """

    def __init__(self, address, score=KEEP_SCORE, width=DETECT_WIDTH,
                 timeout=SERVICE_TIMEOUT_S):
        import http.client

        self._client = http.client
        host, _, port = address.partition(":")
        self.host = host
        self.port = int(port) if port else 8768
        self.timeout = timeout
        self.query = f"?score={score}&width={width}&ts="
        self.connection = None
        self.rtt_ms = 0.0
        self.detect_ms = 0.0

    def describe(self):
        return f"http://{self.host}:{self.port}/detect"

    def detect(self, jpeg, exposed_at):
        """Faces for this frame, or None if the service did not answer.

        The exposure time goes out as an opaque string and comes back untouched.
        It is checked on return: a reply carrying somebody else's stamp would be
        a reply to a different frame, which the controller must not act on.
        """
        stamp = repr(exposed_at)
        for attempt in (1, 2):  # a stale keep-alive costs one retry, not a frame
            started = time.monotonic()
            try:
                if self.connection is None:
                    self.connection = self._client.HTTPConnection(
                        self.host, self.port, timeout=self.timeout)
                    # Connecting here rather than letting request() do it, so the
                    # socket exists to be configured -- and inside the try, because
                    # a detector that is simply not there must come back as None
                    # like any other failure. It is the expected state on a rover.
                    self.connection.connect()
                    # http.client writes the headers and then the body, so Nagle
                    # can hold the second until the first is acknowledged -- the
                    # classic 40 ms. Measured, it changes nothing here; the stall
                    # that mattered was the same mistake on the server's side.
                    self.connection.sock.setsockopt(
                        socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.connection.request(
                    "POST", "/detect" + self.query + stamp, body=jpeg,
                    headers={"Content-Type": "image/jpeg",
                             "Content-Length": str(len(jpeg))})
                payload = json.loads(self.connection.getresponse().read())
            except Exception:
                self.close()
                if attempt == 2:
                    return None
                continue
            if payload.get("ts") != stamp or "faces" not in payload:
                return None
            self.rtt_ms = (time.monotonic() - started) * 1e3
            self.detect_ms = payload.get("detect_ms", 0.0)
            return [tuple(face) for face in payload["faces"]]
        return None

    def close(self):
        if self.connection is not None:
            try:
                self.connection.close()
            except Exception:
                pass
            self.connection = None


class SerialLink:
    """JSON commands down the GPIO UART to the ESP32.

    The wire the whole arrangement is built around: the detector may be a room
    away, but the servos are commanded over a cable that cannot drop out.
    """

    def __init__(self, port):
        import serial

        self.port = port
        self.link = serial.Serial(port, BAUD, timeout=0.1)

    def describe(self):
        return f"{self.port} at {BAUD}"

    def send(self, command):
        try:
            self.link.write(json.dumps(command, separators=(",", ":")).encode() + b"\n")
            # The board streams T:1001 telemetry continuously and nothing here
            # reads it; left alone it would fill the buffer within seconds.
            self.link.reset_input_buffer()
            return True
        except Exception:
            return False

    def close(self):
        try:
            self.link.close()
        except Exception:
            pass


class HttpLink:
    """JSON commands over the ESP32's own `/js` endpoint, for a board on WiFi."""

    def __init__(self, host, timeout=0.5):
        import http.client
        from urllib.parse import quote

        self._client = http.client
        self._quote = quote
        self.host = host
        self.timeout = timeout
        self.connection = None

    def describe(self):
        return f"http://{self.host}/js"

    def send(self, command):
        path = "/js?json=" + self._quote(
            json.dumps(command, separators=(",", ":")), safe="")
        for attempt in (1, 2):
            if self.connection is None:
                self.connection = self._client.HTTPConnection(
                    self.host, timeout=self.timeout)
            try:
                self.connection.request("GET", path)
                self.connection.getresponse().read()
                return True
            except Exception:
                self.close()
                if attempt == 2:
                    return False
        return False

    def close(self):
        if self.connection is not None:
            try:
                self.connection.close()
            except Exception:
                pass
            self.connection = None


class NoLink:
    """--no-move: everything runs, nothing is commanded."""

    def describe(self):
        return "nothing (--no-move)"

    def send(self, command):
        return True

    def close(self):
        pass


def open_link(args):
    if not args.move:
        return NoLink()
    if args.host:
        return HttpLink(args.host)
    try:
        return SerialLink(args.serial)
    except Exception as error:
        sys.exit(f"Cannot open {args.serial}: {error}\n{SERIAL_CONSOLE_HINT}")


def parse_size(text):
    try:
        width, _, height = text.lower().partition("x")
        return int(width), int(height)
    except ValueError:
        raise argparse.ArgumentTypeError(f"not a size: {text!r} (try 640x480)")


def main():
    parser = argparse.ArgumentParser(
        description="Track a face from the rover, detecting on the OAK camera's VPU.")
    parser.add_argument(
        "--service", default=DEFAULT_SERVICE, metavar="HOST[:PORT]",
        help=f"the face detector (default {DEFAULT_SERVICE})")
    parser.add_argument(
        "--serial", default=DEFAULT_SERIAL, metavar="PORT",
        help=f"the ESP32's serial port (default {DEFAULT_SERIAL})")
    parser.add_argument(
        "--host", default=None, metavar="ADDRESS",
        help="command the board over WiFi at this address instead of the UART")
    parser.add_argument(
        "--device", default=DEFAULT_DEVICE, metavar="PATH",
        help=f"the camera (default {DEFAULT_DEVICE})")
    parser.add_argument(
        "--size", type=parse_size, default=DEFAULT_SIZE, metavar="WxH",
        help="capture size (default %dx%d); larger costs the Pi and buys nothing"
             % DEFAULT_SIZE)
    parser.add_argument(
        "--no-move", dest="move", action="store_false",
        help="detect and report, but command nothing")
    parser.add_argument(
        "--no-scan", dest="scan", action="store_false",
        help="stay put when there is no face, instead of sweeping to look for one")
    parser.add_argument(
        "--scan-rate", type=float, default=SCAN_RATE, metavar="DEG",
        help=f"sweep speed in degrees per second (default {SCAN_RATE})")
    parser.add_argument(
        "--gain", type=float, default=GAIN, metavar="G",
        help=f"fraction of the error corrected per frame (default {GAIN})")
    parser.add_argument(
        "--quiet", action="store_true", help="no status line")
    args = parser.parse_args()

    if not os.path.exists(args.device):
        sys.exit(f"No camera at {args.device}.")
    stop_on_sigterm()
    camera = Camera(args.device, args.size)
    width, height = args.size
    detector = Detector(args.service)
    link = open_link(args)
    print(f"camera {args.device} {width}x{height} MJPG -> {detector.describe()}, "
          f"commanding {link.describe()}; Ctrl-C to stop")

    gimbal = Gimbal(clamp(args.gain, 0.05, 1.0), args.size)
    # The one thing that moves before a face is seen: the angles are a model, and
    # this is what makes the model true.
    if not link.send(gimbal.command()):
        camera.close()
        link.close()
        sys.exit(f"No answer from the driver board on {link.describe()}. Is it powered?")
    gimbal.changed()  # the centring above counts as sent; do not repeat it

    target = Target()
    scan = None
    scan_rate = clamp(args.scan_rate, 1.0, 200.0)
    failures = 0
    frames = 0
    rate = 0.0
    lag_ms = 0.0
    last_tick = time.monotonic()
    service_ok_at = time.monotonic()
    stalled = False  # the detector has been gone long enough to give up on
    last_status = 0.0

    try:
        while True:
            got = camera.latest()
            now = time.monotonic()
            if got is None:
                if not camera.alive():
                    why = "; ".join(camera.complaints) or "it stopped without saying why"
                    print(f"\nno frames from {args.device} -- {why}", file=sys.stderr)
                    if any("busy" in line.lower() for line in camera.complaints):
                        print("Something else has the camera open -- find it with "
                              "`fuser -v /dev/video0`. A v4l2-ctl left behind by a "
                              "killed run is the usual one.", file=sys.stderr)
                    break
                continue
            frame, exposed_at = got
            # Clamped, not raw: a frame that took a second to arrive must not be
            # answered with a second's worth of sweep. See MAX_DT.
            dt, last_tick = min(now - last_tick, MAX_DT), now

            faces = detector.detect(frame, exposed_at)
            if faces is None:
                # The detector did not answer. Not a frame without faces in it --
                # a frame nobody looked at -- so the target is left exactly as it
                # was and the grace period decides what happens next.
                if now - service_ok_at > SERVICE_GRACE_S:
                    if not stalled:
                        print(f"\nno answer from {detector.describe()} for "
                              f"{SERVICE_GRACE_S:.0f}s -- centring and waiting",
                              file=sys.stderr)
                        target.drop()
                        scan = None
                        gimbal.centre()
                        gimbal.changed()
                        link.send(gimbal.command())
                        stalled = True
                    time.sleep(SERVICE_RETRY_S)
                continue
            if stalled:
                print(f"\n{detector.describe()} is answering again", file=sys.stderr)
                stalled = False
                last_tick = now
                dt = 0.0
            service_ok_at = now
            lag_ms = (now - exposed_at) * 1e3

            tracking = target.update(faces, now)

            scanning = False
            if tracking:
                # A face again: the sweep is abandoned, and the next one will be
                # built afresh from wherever tracking has left the camera pointing.
                scan = None
                # Positive x is right of centre and positive y is *above* it, which
                # is not the picture's own row order -- see Gimbal.track().
                error_x = (target.centre[0] - width / 2) / (width / 2)
                error_y = (height / 2 - target.centre[1]) / (height / 2)
                # The measured exposure time, not DEAD_TIME_S: this is the whole
                # reason the stamp is carried out to the detector and back.
                gimbal.track(error_x, error_y, dt, now, exposed_at=exposed_at)
            else:
                if target.centre is not None:
                    target.drop()
                if args.scan and now - target.seen_at > SCAN_AFTER_S:
                    if scan is None:
                        scan = Scan(gimbal)
                    scan.step(gimbal, scan_rate, dt)
                    scanning = True

            # Every frame, moved or not: track() reads this back to find where the
            # camera was when a frame was exposed.
            gimbal.record(now)

            if gimbal.changed():
                failures = 0 if link.send(gimbal.command()) else failures + 1

            frames += 1
            rate = rate + 0.1 * (1.0 / max(dt, 1e-3) - rate) if frames > 1 else 0.0

            if not args.quiet and now - last_status > 1.0 / STATUS_HZ:
                last_status = now
                state = ("tracking" if tracking else
                         scan.state() if scanning else "no face")
                offset = ""
                if tracking:
                    offset = (f"  err {error_x:+.2f},{error_y:+.2f}")
                print(f"\r {state:<20} faces {len(faces)}{offset}   "
                      f"pan {gimbal.pan:+4.0f} tilt {gimbal.tilt:+3.0f}   "
                      f"{rate:4.1f} fps  lag {lag_ms:3.0f} ms  "
                      f"(rtt {detector.rtt_ms:3.0f}, det {detector.detect_ms:4.1f})  "
                      f"dropped {camera.dropped}"
                      + ("" if not failures else f"  link {failures} lost")
                      + "   ", end="", flush=True)
    except (KeyboardInterrupt, Stopping):
        pass
    finally:
        print()
        # Back to centre, which is where the next run will assume it is. Nothing
        # else needs undoing: the wheels were never touched and the heartbeat was
        # left at the firmware's own default throughout -- T:134 does not feed it.
        gimbal.centre()
        link.send(gimbal.command())
        link.close()
        detector.close()
        camera.close()
        print(f"centred, stopped -- {frames} frames tracked, {camera.dropped} dropped, "
              f"{camera.unpaired} without a V4L2 stamp")
    return 0


if __name__ == "__main__":
    sys.exit(main())

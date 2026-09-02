"""The gimbal camera as a stream of MJPEG frames, newest only.

v4l2-ctl does the capturing -- there is no OpenCV on this host and no reason to
want one, because nothing here looks inside a frame. Frames arrive as bytes, are
split on their own markers, and the newest complete one is kept; older ones are
dropped where they lie, since a queue of frames is a queue of stale aiming errors.

Two threads, because the two pipes have to be drained independently: stderr
filling while only stdout is read would deadlock v4l2-ctl, and stdout filling
while only stderr is read would stall the camera. stderr is read rather than
discarded because `--verbose` carries each buffer's start-of-exposure timestamp,
which is how a frame's age is known at all.

Split out of track_face_pi.py because three other callers want the camera without
the tracking loop around it -- the daemon opens `Camera` for its own loop, takes
stills through `snapshot`, and its checks use `split_jpegs`. All three still
import them from track_face_pi, which re-exports what is here.
"""

import os
import re
import signal
import subprocess
import threading
import time

from aiming import DETECT_WIDTH


# --- the camera -----------------------------------------------------------

DEFAULT_DEVICE = "/dev/video0"


# The rover's USB module: 0abd:8050, MJPG at 30 fps from 160x120 to 2592x1944.
# 640x480 because the detector works at DETECT_WIDTH and anything more is thrown
# away after costing the Pi 1 real CPU to forward -- see the table in the docstring.
DEFAULT_SIZE = (DETECT_WIDTH, 480)


# What one frame is worth waiting for before giving up on the camera entirely.
FRAME_TIMEOUT_S = 2.0


# How long to wait for a frame's exposure stamp, which arrives on a different pipe
# and can lose the race with the frame's own bytes -- see Camera._stamp_for().
#
# Six milliseconds is the p90 when this process has the core to itself, and 0.03
# was set from that. On the rover it is not the p90 that matters but the tail: the
# stderr thread is one of a dozen competing for one core, and when it does not get
# scheduled in time the frame is used with a *guessed* exposure time instead. That
# guess was 41 ms against a real age of 227, and the whole dead-time compensation
# is built on it -- so the camera under-subtracted its own motion and hunted.
# Measured 2026-08-18 on the rover: **106 of about 70 frames unpaired**, which is
# to say all of them, several times over. Waiting is much the cheaper mistake:
# a late stamp still describes the frame correctly, where a missing one is a lie
# about when the camera was looking.
STAMP_WAIT_S = 0.25


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
#
# **Sized for the reader, not for the camera.** v4l2-ctl delivers 30 frames a
# second and this host reassembles about four, so a frame really is most of a
# second old by the time it is in hand -- and at 0.5 this backstop was rejecting
# every correctly matched stamp as implausible, leaving the loop to guess 41 ms.
# The guess is what made it hunt: a frame believed fresh is a frame whose
# correction has not been applied yet, so the same correction went out again. An
# old frame with an honest stamp is entirely usable, because Gimbal.was_at() looks
# up where the camera was pointing then; an old frame with a *fresh* stamp is not
# usable at all. So this is now sized to admit the truth rather than to flatter it.
MAX_FRAME_AGE_S = 1.5


# JPEG's end-of-image marker. Safe to scan for: 0xFF inside entropy-coded data is
# byte-stuffed as FF 00, so FFD9 only appears where the picture ends. Its opposite
# number is what tells a whole picture from the tail of one, which is the only
# check either camera path here makes on what it hands back.
JPEG_EOI = b"\xff\xd9"


JPEG_SOI = b"\xff\xd8"


# How much of v4l2-ctl's stdout to take at a time. Sized for the *reader*, not for
# the frame: this host cannot reassemble 30 frames a second while the tracking loop
# has the core, so the pipe backs up and every picture the loop acts on is old --
# measured live on the rover at a median 1.4 s, against a control loop whose whole
# frame period is 0.43 s. Reading in 4 kB bites cost ten trips through the
# interpreter per frame and it was losing the race. Measured on the Pi with one
# core kept busy, frames reassembled per second:
#
#                          4 kB reads    64 kB reads
#     stamping each frame     1.2            3.2
#     stamping only the kept  1.8           10.4
#
# Both halves are needed and neither is sufficient: with a stamp looked up for every
# frame the read size makes no difference, and with 4 kB reads dropping the stamps
# makes almost none.
READ_CHUNK = 65536


# Buffers whose bytes may still be in flight. This is deep because the reader is
# slow, not because the pairing is: v4l2-ctl delivers 30 frames a second and this
# host reassembles about four, so the stamps of every frame still queued have to
# stay in hand. At 16 they were being discarded before their frames arrived, which
# is why the pairing failed on the rover while passing standalone.
STAMP_BACKLOG = 96


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

    **Dropping happens after the read, so it only helps if the reader keeps up.**
    The pipe is a queue whatever this slot does, and a reader slower than the
    camera reaches the newest frame only by reading every stale one first, in
    full, to discard it. That turned an 85 ms decode saved into half a second of
    waiting bought -- see `_open_camera` in rover_daemon.py. Choosing a format the
    reader can drain is therefore not a bandwidth question but a latency one.

    `stdbuf -o0` is load bearing. v4l2-ctl writes frames with stdio, and stdio
    buffers when its output is a pipe, so the tail of each frame sits in libc
    waiting for the next frame to push it out. That is 25 ms added to every
    measurement of when the picture was taken -- see CAMERA_LAG_S, where both
    numbers are recorded.
    """

    def __init__(self, device=DEFAULT_DEVICE, size=DEFAULT_SIZE, pixelformat="MJPG"):
        self.device = device
        self.size = size
        # MJPG or YUYV. Uncompressed exists for a detector that takes pixels
        # directly rather than decoding a JPEG -- which the OAK's did, and which
        # is worth nothing now that decoding one costs 7 ms here rather than 85.
        #
        # **On this host this bus does not carry it, and tracking uses MJPG.** 614 kB
        # a frame at 30 fps is 18 MB/s where the reader below drains about 7, so
        # the frames pile up in the pipe and `latest` hands out half-second-old
        # pictures; and the traffic starved the wlan adapter off the same USB
        # controller. The earlier note here claimed 12.3 MB/s at 20 fps and that
        # the bus carried it -- measured with nothing else on the bus, which is
        # not the condition it runs in. Kept because the format itself is right
        # for a host whose reader can keep up.
        self.pixelformat = pixelformat
        self.frame_bytes = size[0] * size[1] * 2 if pixelformat == "YUYV" else 0
        self.dropped = 0
        self.unpaired = 0
        # How old the frames that *did* pair have lately turned out to be, in
        # seconds. The fallback for a frame whose stamp cannot be found -- see
        # _stamp_for, where guessing CAMERA_LAG_S instead was actively harmful.
        self.typical_age = CAMERA_LAG_S
        self.complaints = []                # what v4l2-ctl said, if it went wrong
        self._lock = threading.Lock()
        self._fresh = threading.Event()
        self._latest = None
        self._pending = []                  # (bytesused, exposed_at) awaiting frames
        self._stamps = threading.Condition()
        self._stop = threading.Event()
        # The format is set by its own call, and the streaming call is left with
        # nothing to say. v4l2-ctl echoes the negotiated format to *stdout* --
        # "Format Video Capture:" and so on -- before the first buffer, and for
        # MJPEG that is harmless, because the reader syncs on the end-of-image
        # marker and swallows the text as a prefix. A raw stream has no marker to
        # sync on, so those bytes offset every frame that follows, for ever: the
        # picture tears into displaced bands and, since a one-byte shift lands U
        # where V should be, the colours invert with it. Detection still worked,
        # which is what made it hard to see -- luma survives a chroma swap, so
        # faces were still found in a picture that plainly looked wrong.
        # --silent would take the text away and the exposure timestamps with it.
        # **Full auto, on every open, because v4l2 controls do not survive the
        # device.** They are the driver's state, not the daemon's: a reboot or a
        # re-plug puts them back to whatever the camera powers up with, and
        # anything set by hand in between is gone. Set here rather than left
        # alone because the alternative was measured -- this camera's exposure
        # wound itself up during a spell in a dark room, stayed there when
        # daylight returned, and every frame came back pure white. Nothing
        # noticed: a blank frame reads exactly like an empty room, and a whole
        # drive was recorded off white pictures before anyone looked at one.
        #
        # These are the camera's own defaults rather than a policy of ours, so
        # this restores documented behaviour rather than imposing any. One call
        # each, so a camera that lacks one of them still gets the other, and
        # failures are ignored for the same reason: a different lens on a bench
        # must not stop the daemon opening it.
        for control in ("auto_exposure=3", "white_balance_automatic=1"):
            subprocess.run(["v4l2-ctl", "-d", device, "-c", control],
                           capture_output=True, check=False)
        subprocess.run(
            ["v4l2-ctl", "-d", device,
             "--set-fmt-video=width=%d,height=%d,pixelformat=%s"
             % (size + (pixelformat,))],
            capture_output=True, check=False)
        argv = ["v4l2-ctl", "-d", device,
                "--stream-mmap", "--stream-to=-", "--verbose"]
        if os.path.exists("/usr/bin/stdbuf"):
            argv = ["stdbuf", "-o0"] + argv
        self.proc = subprocess.Popen(
            argv, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0,
            preexec_fn=die_with_parent)
        self._frames = self.proc.stdout
        self._readers = [
            threading.Thread(
                target=self._read_raw if self.frame_bytes else self._read_frames,
                daemon=True),
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
        """Whole JPEGs off the pipe, of which only the newest is ever handed on.

        Reassembling 30 frames a second is the expensive half of holding this camera
        open, and on this one core it is expensive enough to matter to the rover's
        control loop. Whoever wants a picture waits; the wheels do not.

        **But falling behind is not free, and that is what this used to do.** Only
        the newest frame is ever taken -- `_offer` overwrites the slot -- so the
        work spent looking up an exposure stamp for each of the others bought
        nothing, and while it was being spent the pipe filled and v4l2-ctl's own
        buffers filled behind it. What the loop then got was a *queue*, drained at
        the reader's rate, so the picture it steered by was over a second old. A
        control loop cannot be given a second of dead time and be expected to
        settle; that showed up as the camera swinging past faces and coming back.
        So everything in the pipe is drained each pass, the pictures nobody will
        look at are dropped by their length alone, and only the survivor is stamped.
        """
        stand_aside()
        buf = b""
        scan = 0
        while not self._stop.is_set():
            chunk = self.proc.stdout.read(READ_CHUNK)
            if not chunk:
                return
            buf += chunk
            newest, skipped = None, []
            while True:
                end = buf.find(JPEG_EOI, scan)
                if end < 0:
                    # Only the last byte can be half of a marker; keep it in view.
                    scan = max(0, len(buf) - 1)
                    break
                if newest is not None:
                    skipped.append(len(newest))
                newest, buf, scan = buf[:end + 2], buf[end + 2:], 0
            if newest is None:
                continue
            # The stamps of the frames being dropped are consumed rather than left
            # behind, because _stamp_for matches by byte count from the oldest
            # pending stamp forwards: an abandoned stamp of the same length would
            # be handed to a later frame, which is a wrong exposure time rather
            # than a missing one, and a wrong one is not detectable afterwards.
            for length in skipped:
                self._discard_stamp(length)
            self.dropped += len(skipped)
            self._offer(newest, self._stamp_for(len(newest)))

    def _read_raw(self):
        """Fixed-size frames, for an uncompressed format.

        Simpler than reassembling JPEG and cheaper: there is no marker to scan
        for, so this is a byte count and a slice. The pairing in `_stamp_for` is
        weaker here and cannot be helped -- every buffer is the same size, so
        matching a stamp by its byte count degenerates into taking the oldest
        pending one, which is the arrival-order matching that docstring warns
        about. With frames this large the camera either delivers one whole or
        drops it, so the failure it guards against does not arise the same way.
        """
        stand_aside()
        need = self.frame_bytes
        # readinto a buffer that is reused, rather than read() and concatenate.
        # A frame is 614 kB here and the naive form copies it three times -- once
        # for the bytes read() returns, once to append, once to freeze -- which is
        # 13 MB/s of pure memcpy at any useful rate, on a host whose whole USB
        # budget is about 12. Only the freeze survives, and it has to: the buffer
        # is handed on while the next frame is already being read into this one.
        buf = bytearray(need)
        view = memoryview(buf)
        got = 0
        stream = self._frames
        while not self._stop.is_set():
            read = stream.readinto(view[got:])
            if not read:
                return
            got += read
            if got == need:
                self._offer(bytes(buf), self._stamp_for(need))
                got = 0

    def _discard_stamp(self, length):
        """Drop the pending stamp for a frame nobody is going to look at.

        Never waits: a frame being thrown away is not worth a quarter of a second,
        and if its stamp has not arrived yet the byte count will simply not match
        anything, which leaves the list as it was and costs one scan.
        """
        with self._stamps:
            hit = next((i for i, s in enumerate(self._pending) if s[0] == length),
                       None)
            if hit is not None:
                del self._pending[:hit + 1]

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
                    age = time.monotonic() - stamp
                    if 0 <= age <= MAX_FRAME_AGE_S:
                        # Slowly, because one late frame is not a new normal and
                        # this is what an unpaired frame will be given.
                        self.typical_age += 0.1 * (age - self.typical_age)
                        return stamp
                    break
                if not self._stamps.wait(max(0.0, deadline - time.monotonic())):
                    break
        self.unpaired += 1
        # **Not CAMERA_LAG_S.** That is how long this camera takes to hand over a
        # frame once it has been exposed, and using it here amounts to claiming the
        # picture is fresh -- which is the one claim that switches off the whole
        # dead-time compensation, on exactly the frames where the pairing has
        # already shown something is wrong. On this host the difference is not
        # academic: frames run over a second old, so an unpaired one was being
        # declared 41 ms old and its full error corrected a second time.
        #
        # What frames have lately turned out to be is a guess, but it is a guess
        # made of measurements, and it degrades towards the truth rather than away
        # from it. Until something pairs it is CAMERA_LAG_S, which is right for a
        # host that is keeping up.
        return time.monotonic() - self.typical_age

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
    loop in [ros_nav/nav_bridge.py](../ros_nav/nav_bridge.py), with the rover
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

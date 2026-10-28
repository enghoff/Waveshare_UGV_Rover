"""Writing a recording session's streams, without ever making the caller wait.

Perception's sessions (docs/plans/recording-for-perception.md) record every
stream the rover has on one clock, to a contract of Perception's
(`rover-schema-v1.json` in that project): one directory a session, one
`.jsonl` file a stream, each opening with a header line, then one record a
line with the source's own sequence number and its arrival stamp on the
Orin's monotonic clock. Several processes write into the same directory -- the
daemon, the depth service, the lidar node and a ROS subscriber -- each its own
files, so this module is deployed into each of them.

**A recording never slows the rover (R-CTL-13).** The places that produce
records are the board link's write lock, the camera's reader, the lidar node's
serial loop: the things that stop the wheels and keep the rover's place on the
map. So `Stream.put` never blocks. It hands the record to a bounded queue and
returns; a thread of the stream's own does the encoding and the writing; and a
full queue drops the record and counts it, which the session's health stream
reports. The sequence number is taken before the queue, so a dropped record
leaves a gap where it would have been: the source did produce it, and the
checker counts it missing.

Which session is open is one small file, `~/.ugv/recording/current.json`,
written by the daemon when a session starts and removed when it ends. Every
other process follows it with a `Follower`, which opens that process's streams
when the file appears and closes them when it goes.
"""
from __future__ import annotations

import json
import math
import os
import queue
import threading
import time

SCHEMA = "rover-recording"
VERSION = 1
#: Where the open session is announced; an environment variable moves it for tests.
CURRENT_ENV = "UGV_RECORDING_CURRENT"
#: What each stream's queue holds before it starts dropping. A tenth of a second
#: of 30 frames is three; this is several seconds of the busiest stream, so only a
#: disk that has stopped keeping up drops anything.
CAPACITY = 2048
#: How often a stream's file is flushed, when its records do not ask for it.
FLUSH_S = 0.5


def current_path() -> str:
    return os.environ.get(CURRENT_ENV) or os.path.expanduser("~/.ugv/recording/current.json")


def now_ns() -> int:
    """The Orin's monotonic clock in integer nanoseconds: every arrival stamp."""
    return time.monotonic_ns()


def clean(value):
    """A record's numbers as JSON can hold them: no NaN and no infinity, which
    the schema writes as null ("a value the source does not give")."""
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {k: clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(v) for v in value]
    return value


def encode(record: dict) -> str:
    return json.dumps(clean(record), separators=(",", ":"), allow_nan=False)


class Stream:
    """One stream's file (and binary file, for frames), written by its own thread.

    `put(record)` fills in `seq` and `t_arr` if the caller has not, queues the
    record and returns whether it was queued. A record given a `blob` has the
    blob appended to the binary file and its `offset` (and, where `length` is in
    the record, its length) filled in by the writer, so the offsets are the bytes'
    real places whatever was dropped.
    """

    def __init__(self, directory: str, stream: str, path: str, binary: str | None = None,
                 capacity: int = CAPACITY, flush_each: bool = False):
        self.stream = stream
        self.path = os.path.join(directory, path)
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        self._file = open(self.path, "w", encoding="utf-8")
        self._file.write(encode({"schema": SCHEMA, "version": VERSION, "stream": stream}) + "\n")
        self._binary = None
        if binary:
            self.binary_path = os.path.join(directory, binary)
            os.makedirs(os.path.dirname(self.binary_path), exist_ok=True)
            self._binary = open(self.binary_path, "wb")
        self._flush_each = flush_each
        self._queue: queue.Queue = queue.Queue(maxsize=capacity)
        self._lock = threading.Lock()
        self._seq = 0
        self.epoch = 0
        self.queued = 0
        self.dropped = 0
        self.written = 0
        self.error: str | None = None
        self._thread = threading.Thread(target=self._write_forever, name="record-" + stream,
                                        daemon=True)
        self._thread.start()

    def put(self, record: dict, blob: bytes | None = None) -> bool:
        with self._lock:
            if "seq" not in record:
                record["seq"] = self._seq
            self._seq = record["seq"] + 1
            record.setdefault("t_arr", now_ns())
            try:
                self._queue.put_nowait((record, blob))
                self.queued += 1
                return True
            except queue.Full:
                self.dropped += 1
                return False

    def restart(self, t_arr: int | None = None, seq: int | None = None) -> None:
        """The source restarted: its next records are a new epoch. Queued in line
        with them, so it lands before the first of them. A restart record is never
        dropped: without it the epoch's records would be misread."""
        with self._lock:
            self.epoch += 1
            if seq is not None:
                self._seq = seq
            marker = {"restart": True, "epoch": self.epoch, "seq": self._seq,
                      "t_arr": t_arr if t_arr is not None else now_ns()}
            self._queue.put((marker, None))

    def depth(self) -> int:
        return self._queue.qsize()

    def close(self, timeout: float = 10.0) -> dict:
        """Write what is queued, close the files, and say what was written and dropped."""
        self._queue.put((None, None))
        self._thread.join(timeout)
        return {"written": self.written, "dropped": self.dropped}

    def _write_forever(self) -> None:
        last_flush = time.monotonic()
        try:
            while True:
                try:
                    record, blob = self._queue.get(timeout=FLUSH_S)
                except queue.Empty:
                    record, blob = False, None
                if record is None:
                    break
                if record:
                    if blob is not None and self._binary is not None:
                        record["offset"] = self._binary.tell()
                        if "length" in record:
                            record["length"] = len(blob)
                        self._binary.write(blob)
                    self._file.write(encode(record) + "\n")
                    if "restart" not in record:
                        self.written += 1
                if self._flush_each or time.monotonic() - last_flush >= FLUSH_S:
                    self._file.flush()
                    if self._binary is not None:
                        self._binary.flush()
                    last_flush = time.monotonic()
        except Exception as exc:  # a full disk, mostly: say so rather than die quietly
            self.error = "%s: %s" % (type(exc).__name__, exc)
        finally:
            for f in (self._file, self._binary):
                if f is not None:
                    try:
                        f.close()
                    except Exception:
                        pass


def announce(session: dict | None) -> None:
    """Say which session is open, or (None) that none is. Written whole and
    renamed into place, so a follower never reads half of it."""
    path = current_path()
    if session is None:
        try:
            os.remove(path)
        except FileNotFoundError:
            pass
        return
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(session, f)
    os.replace(tmp, path)


def announced() -> dict | None:
    try:
        with open(current_path(), encoding="utf-8") as f:
            return json.load(f)
    except (FileNotFoundError, ValueError):
        return None


def closed_marker(directory: str, who: str, counts: dict) -> None:
    """Leave word that this process has closed its streams, with what each wrote
    and dropped: the session's closing step waits for these, then reads them into
    the manifest's final counts."""
    path = os.path.join(directory, ".closed-%s.json" % who)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(counts, f)


class Follower:
    """Opens a process's streams when a session is announced and closes them when
    it ends, polling the announcement twice a second on a thread of its own.

    `opened(session)` returns whatever the process uses to record (its streams);
    `closing(handle)` closes them and returns their counts by stream, which go
    into the closed marker as `who`.
    """

    def __init__(self, who: str, opened, closing, poll_s: float = 0.5):
        self.who = who
        self._opened, self._closing = opened, closing
        self.handle = None
        self.session: dict | None = None
        self._stop = threading.Event()
        self._poll_s = poll_s
        self._thread = threading.Thread(target=self._follow, name="record-follow", daemon=True)
        self._thread.start()

    def _follow(self) -> None:
        while not self._stop.is_set():
            self.poll()
            self._stop.wait(self._poll_s)
        self._end()

    def poll(self) -> None:
        now = announced()
        if self.session is not None and (now is None or now.get("dir") != self.session.get("dir")):
            self._end()
        if now is not None and self.session is None:
            try:
                self.handle = self._opened(now)
                self.session = now
            except Exception as exc:
                print("[record] %s could not open its streams: %s" % (self.who, exc), flush=True)
        if self.session is not None:
            self._report()

    def _report(self) -> None:
        """Leave this process's queue depths and drops where the daemon's health
        stream reads them, so that health covers every stream recorded."""
        handle = self.handle
        if not isinstance(handle, dict):
            return
        status = {name: {"queue": s.depth(), "dropped": s.dropped, "queued": s.queued}
                  for name, s in handle.items() if isinstance(s, Stream)}
        path = os.path.join(self.session["dir"], ".status-%s.json" % self.who)
        try:
            with open(path + ".tmp", "w", encoding="utf-8") as f:
                json.dump(status, f)
            os.replace(path + ".tmp", path)
        except OSError:
            pass

    def _end(self) -> None:
        if self.session is None:
            return
        directory = self.session.get("dir")
        handle, self.handle, self.session = self.handle, None, None
        try:
            counts = self._closing(handle)
        except Exception as exc:
            counts = {"error": "%s: %s" % (type(exc).__name__, exc)}
        try:
            closed_marker(directory, self.who, counts)
        except Exception:
            pass

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(5.0)

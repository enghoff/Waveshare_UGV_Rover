"""One script at a time, in a child process the daemon can kill.

The daemon hands out tools; this hands out the machine those tools run on. A
client sends source, this writes it to a file, starts a `python3` on it with
[rover_api.py](rover_api.py) importable, and watches it -- capturing what it
prints, bounding how long it may run and how much memory it may take, and
killing the whole process group when either runs out. See
[docs/scripting.md](../docs/scripting.md) for why this is a process rather than
an interpreter inside the daemon.

The short version of that argument is in the three limits below. Stopping a
runaway is `SIGKILL` rather than an interrupt hook the language has to provide;
a memory ceiling is somebody else's arithmetic; and a script that crashes is a
child that exits, which the daemon does not even notice. None of that is
available to code running inside a process that is also holding the UART.

**One slot, deliberately.** Two scripts is two things aiming one gimbal, which
is the same reason `look_at` stops face tracking. A second `start` is refused
rather than queued, because the caller is a model that will otherwise be told
its behaviour started when what actually happened is that it is second in line.

**A run that ends stops the wheels.** A script killed in the middle of `drive`
leaves the daemon finishing a move on behalf of a connection that has gone, so
every ending -- clean, failed or killed -- is followed by a `stop_driving` that
is a no-op when nothing is moving. The gimbal is left where it is: stopping the
wheels is safety, and a behaviour that ends with the camera pointed at something
deliberately should not have it swung away.
"""

from __future__ import annotations

import json
import os
import signal
import subprocess
import sys
import tempfile
import threading
import time
from typing import Any, Callable

HERE = os.path.dirname(os.path.abspath(__file__))

# **Starting an interpreter here costs about as long as most scripts run.**
# Measured on this Pi with the daemon running: 1.8 s for a bare `python3 -c pass`
# and 4.2 s by the time `rover_api` is imported and the first line of the script
# executes, on a core the daemon is already more than half using. That is not
# overhead to be shaved away -- it is the shape of the thing, and both numbers
# below are derived from it.
STARTUP_S = 6.0
# What a script gets if it does not ask for something shorter. Sized so that a
# one-shot script which opens the camera fits, because that is the commonest
# thing a short script does and a cold `count_faces` is five seconds of it on its
# own -- measured here at 6.97 s for lights, a gimbal move and one look, of which
# the look was nearly all. This is the
# script's own budget and the wall cost is startup on top of it, so a blocking
# run that uses all of it holds the caller's connection for a quarter of a minute
# in the worst case -- which is well past the 12 s patience the conversation clients
# have (TIMEOUT_S in voice_chat/rover_tools.py) and is one more reason this call
# is not one of them. Anything that needs longer is a behaviour and belongs in
# `start_script`, where nobody is holding a connection open waiting for it.
RUN_LIMIT_S = 15.0
# A behaviour, on the other hand, is meant to outlive the question that started
# it. Five minutes by default and half an hour at the outside: past that it is
# not a behaviour, it is something that should have been deployed.
START_LIMIT_S = 300.0
MAX_LIMIT_S = 1800.0

# How much of the child's own memory is its business. This box has 474 MB and the
# daemon, the scan matcher and the OAK's buffers are all in it, so a script that
# is quietly accumulating frames has to be stopped well before the kernel starts
# choosing victims. Enforced by watching, not by `RLIMIT_AS`: an address-space
# limit low enough to be useful here is one CPython may fail to *start* under on
# armv6, and a limit that refuses to run correct scripts is worse than none.
MEMORY_MB = 96
POLL_S = 0.25
# Between the polite stop and the impolite one. `rover_api` turns SIGTERM into an
# exception at the next `every` or `wait`, which lets a script unwind through its
# own cleanup -- but a script blocked in a `drive` call notices nothing until the
# move ends, and one in a `while True` notices nothing ever.
GRACE_S = 2.0

# What comes back from a print. Bounded because it goes into a JSON reply that a
# model may end up reading out loud, and because a loop printing every tick for
# five minutes is a reply nobody can use anyway.
MAX_OUTPUT = 8192


class Runner:
    """The one script slot: start it, watch it, stop it, say what happened."""

    def __init__(self, address: str,
                 on_start: Callable[[], Any] | None = None,
                 on_finish: Callable[[], Any] | None = None) -> None:
        self.address = address
        self._on_start = on_start
        self._on_finish = on_finish
        self._lock = threading.Lock()
        self._run: _Run | None = None
        self._last: dict[str, Any] | None = None
        self._counter = 0

    # --- what the daemon calls ------------------------------------------

    def start(self, source: str, limit_s: float | None = None) -> dict[str, Any]:
        """Begin a script and return its handle. Refused if one is running."""
        if not isinstance(source, str) or not source.strip():
            return {"ok": False, "error": "there is no source to run"}
        limit = _limit(limit_s, START_LIMIT_S)
        with self._lock:
            if self._run is not None and self._run.running:
                running = self._run
                return {"ok": False,
                        "error": f"{running.id} is already running, {running.seconds:.0f}s "
                                 f"in -- stop it first, this rover runs one at a time"}
            self._counter += 1
            run = _Run(f"script-{self._counter}", source, limit, self.address,
                       self._finished)
            self._run = run
        if self._on_start is not None:
            try:
                self._on_start()
            except Exception:
                pass  # a script must not fail to start because tracking would not stop
        error = run.begin()
        if error:
            with self._lock:
                self._run = None
            return {"ok": False, "error": error}
        return {"ok": True, "id": run.id, "running": True, "limit_s": limit}

    def run(self, source: str, limit_s: float | None = None) -> dict[str, Any]:
        """Begin a script and wait for it. What `run_script` answers with.

        The only one of the three where the protocol's `ok` is the *script's*
        outcome rather than the call's, because that is what the caller of a
        blocking run is asking: did the thing I sent do what I asked. `outcome`
        says it in words either way, and everywhere else `ok` means the daemon
        answered -- a status query about a script that failed has succeeded.
        """
        limit = _limit(limit_s, RUN_LIMIT_S, ceiling=RUN_LIMIT_S)
        started = self.start(source, limit)
        if not started.get("ok"):
            return started
        with self._lock:
            run = self._run
        if run is not None:
            # Long enough to outlast the runner's own killing of it, or a
            # blocking run answers "still running" about a script that is at
            # that moment being shot. `wall_limit` is the one definition of when
            # that starts; the two graces are the SIGTERM wait and the SIGKILL.
            run.wait(run.wall_limit + 2 * GRACE_S + 2.0)
        state = self.status(started["id"])
        state["ok"] = state.get("outcome") == "finished"
        return state

    def status(self, run_id: str | None = None) -> dict[str, Any]:
        """The current run, or the last one to finish, or nothing yet."""
        with self._lock:
            run, last = self._run, self._last
        if run is not None and (run_id is None or run.id == run_id) and run.running:
            return {"ok": True, **run.snapshot()}
        if last is not None and (run_id is None or last.get("id") == run_id):
            return {"ok": True, **last}
        if run is not None and (run_id is None or run.id == run_id):
            return {"ok": True, **run.snapshot()}
        return {"ok": True, "running": False,
                "note": "no script has been run since the daemon started"
                        if run_id is None else f"there is no run called {run_id}"}

    def stop(self) -> dict[str, Any]:
        """Stop whatever is running. Never refused, and a no-op if nothing is."""
        with self._lock:
            run = self._run
        if run is None or not run.running:
            return {"ok": True, "running": False, "stopped": False,
                    "note": "nothing was running"}
        run.kill("stopped")
        run.wait(GRACE_S + 2.0)
        return {"ok": True, "stopped": True, **run.snapshot()}

    def close(self) -> None:
        with self._lock:
            run = self._run
        if run is not None and run.running:
            run.kill("the daemon is shutting down")
            run.wait(GRACE_S + 1.0)

    # --- housekeeping ----------------------------------------------------

    def _finished(self, snapshot: dict[str, Any]) -> None:
        with self._lock:
            self._last = snapshot
        if self._on_finish is not None:
            try:
                self._on_finish()
            except Exception:
                pass


def _limit(asked: float | None, default: float, ceiling: float = MAX_LIMIT_S) -> float:
    if asked is None:
        return default
    try:
        asked = float(asked)
    except (TypeError, ValueError):
        return default
    return max(1.0, min(asked, ceiling))


class _Run:
    """One child process, from source to whatever it turned out to mean."""

    def __init__(self, run_id: str, source: str, limit_s: float, address: str,
                 on_finish: Callable[[dict], None]) -> None:
        self.id = run_id
        self.source = source
        self.limit_s = limit_s
        self.address = address
        self._on_finish = on_finish
        self.running = False
        self.started_at = time.time()
        self._started = time.monotonic()
        self._proc: subprocess.Popen | None = None
        self._dir: str | None = None
        self._output = bytearray()
        self._truncated = False
        self._why: str | None = None
        self._result: dict[str, Any] = {}
        self._done = threading.Event()
        self._lock = threading.Lock()
        self._peak_mb = 0.0
        self._ended: float | None = None

    @property
    def wall_limit(self) -> float:
        """When this run gets killed, measured from the spawn.

        The startup allowance is added here, and it is not slack: the script's
        own deadline runs from its first line, so a cap measured from the spawn
        would take the interpreter's four seconds out of the script's budget and
        kill a correct behaviour a fifth of the way through what it was promised.
        `rover_api` raises Deadline first and cleanly; this is the backstop for a
        script that is in no position to notice.
        """
        return self.limit_s + STARTUP_S

    @property
    def seconds(self) -> float:
        """Wall clock since the child was spawned, frozen once it has exited.

        Deliberately not the figure the script measures for itself, which starts
        at its first line and so leaves out the seconds of interpreter that
        somebody waiting has very much been paying. Both are reported; this is
        the one that answers "how long did that take".
        """
        return (self._ended or time.monotonic()) - self._started

    def begin(self) -> str | None:
        """Start the child. Returns a sentence if it could not be started."""
        try:
            self._dir = tempfile.mkdtemp(prefix="rover-script-")
            source_path = os.path.join(self._dir, "script.py")
            self._result_path = os.path.join(self._dir, "result.json")
            with open(source_path, "w", encoding="utf-8") as handle:
                handle.write(self.source)

            env = dict(os.environ)
            env["ROVER_SCRIPT"] = source_path
            env["ROVER_RESULT"] = self._result_path
            env["ROVER_ADDRESS"] = self.address
            env["ROVER_LIMIT_S"] = str(self.limit_s)
            # So a print arrives while the script is still running rather than at
            # the end -- and so it is not lost altogether when one is killed.
            env["PYTHONUNBUFFERED"] = "1"
            env["PYTHONPATH"] = HERE + os.pathsep + env.get("PYTHONPATH", "")

            # Loaded once, under its own name: the script's own `import rover_api`
            # then shares this module rather than getting a second copy with a
            # second stop flag. See rover_api.main().
            command = [sys.executable, "-c", "import rover_api; rover_api.main()"]
            extra = {"start_new_session": True} if os.name == "posix" else {}
            self._proc = subprocess.Popen(
                command, cwd=self._dir, env=env,
                stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, **extra)
        except Exception as error:
            self._cleanup()
            return f"the script could not be started: {type(error).__name__}: {error}"

        self.running = True
        threading.Thread(target=self._read, daemon=True).start()
        threading.Thread(target=self._watch, daemon=True).start()
        return None

    def wait(self, timeout: float) -> bool:
        return self._done.wait(timeout)

    def kill(self, why: str) -> None:
        """Ask, then insist. The asking is what lets a script clean up after itself."""
        with self._lock:
            if self._why is None:
                self._why = why
        proc = self._proc
        if proc is None or proc.poll() is not None:
            return
        self._signal(proc, signal.SIGTERM)
        if not _wait_for_exit(proc, GRACE_S):
            self._signal(proc, signal.SIGKILL if hasattr(signal, "SIGKILL")
                         else signal.SIGTERM)

    @staticmethod
    def _signal(proc: subprocess.Popen, sig) -> None:
        """The whole process group, so that anything the script started goes too."""
        try:
            if os.name == "posix":
                os.killpg(os.getpgid(proc.pid), sig)
            elif sig == signal.SIGTERM:
                proc.terminate()
            else:
                proc.kill()
        except (ProcessLookupError, PermissionError, OSError):
            pass

    # --- the two threads --------------------------------------------------

    def _read(self) -> None:
        """Drain the child's output, or a 64 kB pipe fills and it stops dead."""
        proc = self._proc
        if proc is None or proc.stdout is None:
            return
        try:
            for line in proc.stdout:
                with self._lock:
                    room = MAX_OUTPUT - len(self._output)
                    if room > 0:
                        self._output += line[:room]
                    if len(line) > room:
                        self._truncated = True
        except (OSError, ValueError):
            pass

    def _watch(self) -> None:
        """Time, memory, and the end -- whichever arrives first."""
        proc = self._proc
        assert proc is not None
        try:
            while True:
                if proc.poll() is not None:
                    break
                if self.seconds > self.wall_limit:
                    self.kill(f"it ran past the {self.limit_s:.0f}s it was given")
                    break
                used = _resident_mb(proc.pid)
                if used:
                    self._peak_mb = max(self._peak_mb, used)
                    if used > MEMORY_MB:
                        self.kill(f"it took more than {MEMORY_MB} MB of memory")
                        break
                time.sleep(POLL_S)
            _wait_for_exit(proc, GRACE_S + 1.0)
        finally:
            self._finish()

    def _finish(self) -> None:
        self._ended = time.monotonic()
        proc = self._proc
        if proc is not None and proc.poll() is None:
            self._signal(proc, signal.SIGKILL if hasattr(signal, "SIGKILL")
                         else signal.SIGTERM)
            _wait_for_exit(proc, 1.0)
        # Written by the harness in the child. Absent means it never got that far,
        # which is what a kill and a segfault both look like from here.
        try:
            with open(self._result_path, "r", encoding="utf-8") as handle:
                self._result = json.load(handle)
        except (OSError, ValueError):
            self._result = {}
        self.running = False
        snapshot = self.snapshot()
        self._cleanup()
        self._done.set()
        try:
            self._on_finish(snapshot)
        except Exception:
            pass

    def _cleanup(self) -> None:
        proc = self._proc
        if proc is not None and proc.stdout is not None:
            try:
                proc.stdout.close()
            except OSError:
                pass
        for name in ("script.py", "result.json"):
            try:
                if self._dir:
                    os.unlink(os.path.join(self._dir, name))
            except OSError:
                pass
        try:
            if self._dir:
                os.rmdir(self._dir)
        except OSError:
            pass
        self._dir = None

    def snapshot(self) -> dict[str, Any]:
        """What happened, in the shape a caller reads out loud.

        Deliberately without an `ok` of its own. That word means "the daemon
        answered" everywhere else in this protocol, and a run that failed is
        still a question that was answered; conflating the two makes a status
        call about a broken script look like a broken status call. How the run
        went is `outcome` -- finished, failed or stopped -- with `error` saying
        why in the two cases where that is a sentence worth repeating.
        """
        with self._lock:
            output = bytes(self._output).decode("utf-8", "replace")
            truncated, why = self._truncated, self._why
        state: dict[str, Any] = {
            "id": self.id,
            "running": self.running,
            "seconds": round(self.seconds, 2),
            "output": output,
        }
        # How much of that was the interpreter rather than the script. Reported
        # rather than hidden, because a two-line script that takes five seconds
        # is otherwise a mystery, and on this host it is the usual case.
        if not self.running and "seconds" in self._result:
            state["script_seconds"] = self._result["seconds"]
            state["startup_s"] = round(self.seconds - self._result["seconds"], 2)
        if truncated:
            state["output_truncated"] = True
        if self._peak_mb:
            state["peak_mb"] = round(self._peak_mb, 1)
        if self.running:
            state["outcome"] = "running"
            return state
        result = self._result
        if result:
            # The child left an answer, so it reached its own ending -- even if a
            # SIGTERM is what sent it there. Measured on the rover: a script cut
            # off at its limit ran two more lines and turned the headlights off
            # before the SIGKILL was due, and reporting that as "stopped, nothing
            # known" would have hidden the tidying up that actually happened.
            state["outcome"] = ("finished" if result.get("ok")
                                else "stopped" if result.get("stopped") else "failed")
            if not result.get("ok"):
                state["error"] = result.get("error", "it failed without saying why")
            if why is not None:
                state["note"] = why
            if result.get("traceback"):
                state["traceback"] = result["traceback"]
        elif why is not None:
            state["outcome"] = "stopped"
            state["error"] = why
        else:
            proc = self._proc
            code = proc.poll() if proc is not None else None
            state["outcome"] = "failed"
            state["error"] = ("the script died without leaving an answer "
                              f"(exit {code})")
        return state


def _wait_for_exit(proc: subprocess.Popen, timeout: float) -> bool:
    try:
        proc.wait(timeout=timeout)
        return True
    except subprocess.TimeoutExpired:
        return False


def _resident_mb(pid: int) -> float:
    """How much memory the child is actually holding, or 0 where that is unknowable.

    `/proc/<pid>/statm` rather than a dependency: this is Linux-only and returns
    0 elsewhere, which means the workstation runs these scripts without a memory
    ceiling. That is the right way round -- the limit exists for the 474 MB box
    with a rover attached, and the machine without one is where scripts are
    written rather than where they are trusted.
    """
    try:
        with open(f"/proc/{pid}/statm", "r") as handle:
            resident = int(handle.read().split()[1])
        return resident * os.sysconf("SC_PAGE_SIZE") / (1024 * 1024)
    except (OSError, ValueError, IndexError, AttributeError):
        return 0.0

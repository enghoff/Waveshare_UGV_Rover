"""Scripting checks: one script at a time, and what it says when it ends.

A script is a job on the rover rather than a message to it, so what matters is
what happens around it: that a second one cannot start on top of the first, that
a behaviour keeps running until it is stopped, and that these tools are offered
to the rover's own loopback and not to the LAN.
"""
from __future__ import annotations

import json
import time

from test_fakes import FakeLink
from test_harness import check

def test_scripts_run_and_say_what_happened():
    """A script runs, prints, and comes back as an outcome rather than an exit code.

    These spawn a real `python3`, which is the point -- the isolation being tested
    is a process boundary, and a fake one would be testing nothing. They need no
    rover: none of these scripts calls a primitive.
    """
    import scripting

    runner = scripting.Runner("127.0.0.1:1")  # nothing there; nothing calls it
    try:
        done = runner.run("print('hello'); print(6 * 7)")
        check("a script's output comes back", done["output"].split(), ["hello", "42"])
        check("...and it says it finished", done["outcome"], "finished")

        # The line number is the whole point of compiling the source as
        # `<script>`: a traceback pointing into a temp file names something the
        # person who asked for the behaviour cannot look at.
        broken = runner.run("a = 1\nb = facse\n")
        check("a broken script fails", broken["outcome"], "failed")
        check("...and names the line", broken["error"].startswith("line 2: NameError"),
              True)
        check("a syntax error names its line too",
              runner.run("def (:")["error"].startswith("line 1: SyntaxError"), True)

        # A program starts with the primitives already defined, because the import
        # line was the step a model kept leaving a name out of. Proved against an
        # address with nothing on it: reaching the daemon and failing to is a
        # different error from never having heard of `lights`, and it is the one
        # that says the name was there.
        bare = runner.run("lights.set(255)")
        check("a script needs no import to reach a primitive",
              bare["error"].startswith("line 1: RoverError"), True)
        check("...and the exceptions are there to be caught by name",
              runner.run("try:\n"
                         "    lights.set(255)\n"
                         "except RoverError:\n"
                         "    print('refused')\n")["output"].strip(),
              "refused")
        # And the import that a program written by hand would use still works.
        check("importing them anyway changes nothing",
              runner.run("from rover_api import lights\n"
                         "lights.set(255)\n")["error"].startswith(
                             "line 2: RoverError"), True)
        # `ok` on a blocking run is the script's own fate, because that is what
        # the caller asked. Everywhere else it means the daemon answered.
        check("a failed script reports ok false", broken["ok"], False)
        check("a status call about a failed script still reports ok true",
              runner.status()["ok"], True)

        # The one that matters, and the one an interpreter inside the daemon
        # could not do without help from the language: a script with no exit in
        # it, spinning, is still stopped.
        started = time.monotonic()
        runaway = runner.run("while True:\n    pass\n", limit_s=1.0)
        took = time.monotonic() - started
        check("a runaway script is stopped", runaway["outcome"], "stopped")
        # Bounded in terms of the module's own allowance rather than a number
        # written here. Starting an interpreter is four seconds on the rover and
        # a fifth of one on a workstation, so a fixed bound would be measuring
        # which machine the test is running on.
        check(f"...at about the time it was given ({took:.1f}s)",
              0.9 < took < scripting.STARTUP_S + 1.0 + scripting.GRACE_S + 4.0, True)
    finally:
        runner.close()


class _StandInDaemon:
    """A daemon on loopback that takes its time over a turn, and remembers when.

    The offline checks could not ask about two calls at once before, because that
    needs something at the far end still holding the first when the second
    arrives. `turn_in_place` sleeps here; everything else answers at once; and
    what was called and when is kept both ways round, since the question these
    tests ask is about order in time rather than about answers.
    """

    TURN_S = 1.5

    def __init__(self) -> None:
        import socketserver
        import threading

        heard = self.heard = []  # (seconds in, name, "->" arriving or "<-" answered)
        began = time.monotonic()
        turn_s = self.TURN_S

        class Handler(socketserver.StreamRequestHandler):
            def handle(self) -> None:
                for line in self.rfile:
                    if not line.strip():
                        continue
                    name = json.loads(line).get("call")
                    heard.append((time.monotonic() - began, name, "->"))
                    if name == "turn_in_place":
                        time.sleep(turn_s)
                    heard.append((time.monotonic() - began, name, "<-"))
                    self.wfile.write(json.dumps({"ok": True}).encode() + b"\n")
                    self.wfile.flush()

        class Server(socketserver.ThreadingTCPServer):
            allow_reuse_address = True
            daemon_threads = True

        self._server = Server(("127.0.0.1", 0), Handler)
        self.address = "127.0.0.1:%d" % self._server.server_address[1]
        threading.Thread(target=self._server.serve_forever, daemon=True).start()

    def when(self, name: str, arrow: str):
        """The times a call arrived, or was answered, in the order it happened."""
        return [at for at, called, direction in self.heard
                if called == name and direction == arrow]

    def close(self) -> None:
        self._server.shutdown()
        self._server.server_close()


def test_two_things_at_once():
    """A job runs while the rover turns, and is over when the block that started it is.

    Everything that moves this rover blocks until the move is over, so a program
    written as one list of calls can only do one thing at a time -- and a model
    asked to turn and flash the headlights together produced a turn and then some
    flashing. What is checked here is the fix from the far end's point of view:
    light changes arriving while the turn the daemon is still working on has not
    been answered yet.

    Then the two failures that made threads unusable on their own. A job that
    raised used to leave a run reported as finished, with the traceback printed
    into the output where a model reads it as something the program meant to say;
    and a thread the program walked away from used to hold the rover's one script
    slot open for as long as it ran, behind a script that thought it had finished.
    """
    import scripting

    fake = _StandInDaemon()
    runner = scripting.Runner(fake.address)
    try:
        done = runner.run(
            "from rover_api import lights, drive, every, alongside\n"
            "def flashing():\n"
            "    for tick in every(0.3):\n"
            "        lights.set(255 if tick % 2 == 0 else 0)\n"
            "with alongside(flashing):\n"
            "    drive.turn(90)\n"
            "print('turned')\n", limit_s=15)
        check("a script with a job alongside it finishes", done["outcome"], "finished")
        began = (fake.when("turn_in_place", "->") or [0.0])[0]
        ended = (fake.when("turn_in_place", "<-") or [0.0])[0]
        during = [at for at in fake.when("set_lights", "->") if began < at < ended]
        check(f"the lights change while the rover is still turning ({len(during)}x)",
              len(during) >= 2, True)
        # One more may already be on its way when the block ends -- the job is
        # stopped at its next `every`, not mid-call -- and none after that.
        check("...and stop when the block that started them ends",
              [at for at in fake.when("set_lights", "->") if at > ended + 0.5], [])

        # The other way round, which is how the model actually wrote it: the slow
        # move is the job and the quick repeated thing is the block. Leaving the
        # block has to wait for a job that is one long call rather than cut it
        # off, or the rover stops half way through a drive it then reports as
        # done. The flashing here is over in a third of a second and the turn
        # takes the stand-in a second and a half.
        before = len(fake.when("turn_in_place", "<-"))
        inverted = runner.run(
            "def turning():\n"
            "    drive.turn(180)\n"
            "with alongside(turning):\n"
            "    for tick in every(0.1, ticks=3):\n"
            "        lights.set(255 if tick % 2 == 0 else 0)\n"
            "print('both done')\n", limit_s=15)
        check("a block whose job is the slow half finishes",
              inverted["outcome"], "finished")
        check("...having waited for the move rather than cutting it short",
              len(fake.when("turn_in_place", "<-")) > before, True)
        check(f"...which is why it took the turn's own time "
              f"({inverted['script_seconds']:.1f}s)",
              inverted["script_seconds"] >= _StandInDaemon.TURN_S, True)

        failed = runner.run(
            "from rover_api import drive, alongside\n"
            "def bad():\n"
            "    raise RuntimeError('the job fell over')\n"
            "with alongside(bad):\n"
            "    drive.turn(90)\n", limit_s=15)
        check("a job that fails fails the script", failed["outcome"], "failed")
        check("...and names the line inside the job",
              failed["error"], "line 3: RuntimeError: the job fell over")

        started = time.monotonic()
        left = runner.run(
            "import threading, time\n"
            "threading.Thread(target=time.sleep, args=(30,)).start()\n"
            "print('done')\n", limit_s=15)
        took = time.monotonic() - started
        check("a script is over when its last line has run", left["outcome"],
              "finished")
        # In terms of the module's own startup allowance rather than a number
        # written here, for the reason the runaway test gives: a fixed bound
        # measures which machine the test is running on. The thread sleeps thirty
        # seconds, so there is no reading of this that passes by accident.
        check(f"...even with a thread of its own still going ({took:.1f}s)",
              took < scripting.STARTUP_S + 4.0, True)
    finally:
        runner.close()
        fake.close()


def test_the_script_tools_are_offered_to_the_rover_and_not_to_the_lan():
    """Who is shown the three scripting tools, and whether they are usable.

    The gate is the interesting half. All three are refused on anything but
    loopback (`LOCAL_ONLY`), so a client across the LAN that was shown one of the
    schemas would be holding a tool whose every call comes back "reach it through
    an ssh tunnel" -- and a model with a tool like that reports doing things it
    has not done, which is the failure `Rover.tools` exists to prevent.

    Then that `run_script`'s description arrives finished. It is a literal with
    `{api}` in it until something fills it in, and a schema handed to a model with
    a formatting placeholder still in it is a schema that teaches it nothing.

    And last, that starting is not offered without stopping. A behaviour has no
    deadline any more, so those two are one facility: a model that can take the
    rover's single script slot and cannot give it back is worse off than one that
    was never able to take it.
    """
    import rover_daemon
    import scripting

    rover = rover_daemon.Rover(FakeLink(), "unused", device=None)
    def names(**kw):
        return [t["function"]["name"] for t in rover.tools(**kw)]

    check("a daemon not running scripts offers none, even on loopback",
          [n for n in names(local=True) if "script" in n], [])

    rover.scripts = scripting.Runner("127.0.0.1:1")  # nothing there; nothing calls it
    try:
        check("a client on the LAN is shown none of them",
              [n for n in names() if "script" in n], [])
        check("...and the default is the LAN, so a caller has to say otherwise",
              names(), names(local=False))
        check("a client on the rover is shown all three",
              names(local=True)[-3:], ["run_script", "start_script", "script_stop"])
        # Which is also the ordering check: they come after the tools whose
        # order was measured, and a start is never offered without its stop.
        check("no scripting tool comes before the measured ones",
              [n for n in names(local=True)[:-3] if "script" in n], [])

        described = rover.script_tools()[0]["function"]["description"]
        check("run_script's description is filled in, not the literal",
              "{api}" in described or "{limit_s}" in described, False)
        # Two primitives and the limit, read from the modules that own them
        # rather than written here: the point of generating this is that a
        # renamed primitive cannot go on being advertised.
        check("...with the primitives a program is written against",
              all(word in described
                  for word in ("gimbal.look_at", "drive.forward", "every(")), True)
        check("...and the runner's own limit in it",
              f"{scripting.RUN_LIMIT_S:.0f} seconds" in described, True)
        # The other two point at that list rather than carrying a second copy,
        # which is what keeps a realtime session from paying for it twice.
        starting = rover.script_tools()[1]["function"]["description"]
        check("start_script sends the model to run_script for the primitives",
              "run_script" in starting and "gimbal.look_at" not in starting, True)
        check("...and says what ends it", "script_stop" in starting, True)
        check("...and that only one runs at a time",
              "one program runs at a time" in starting, True)
        check("the list is the same object next time, built once",
              rover.script_tools() is rover.script_tools(), True)
    finally:
        rover.scripts.close()


def test_one_script_at_a_time():
    """The single slot, which is the whole of what keeps behaviours from piling up.

    It carries more weight than it did: a behaviour has no deadline, so nothing
    frees the slot on its own any more and the refusal has to name what is
    holding it. Started here with a limit, because this one is meant to be over
    quickly whichever way the test ends.
    """
    import scripting

    runner = scripting.Runner("127.0.0.1:1")
    try:
        first = runner.start("import time\ntime.sleep(30)\n", limit_s=30)
        check("the first script starts", first["ok"], True)
        second = runner.start("print(1)")
        check("a second is refused rather than queued", second["ok"], False)
        check("...and says which one is running", first["id"] in second["error"], True)
        stopped = runner.stop()
        check("stopping succeeds even though the script did not", stopped["ok"], True)
        check("...and the run is recorded as stopped", stopped["outcome"], "stopped")
        check("a slot freed by a stop takes the next script",
              runner.run("print('after')")["outcome"], "finished")
    finally:
        runner.close()


def test_a_behaviour_runs_until_it_is_stopped():
    """No deadline on a `start`, so a stop is the only thing that ends this one.

    The script here is the same runaway `while True` that the blocking test
    shoots on time, and the point is that nothing shoots it: the run carries no
    wall limit at all, the child is told so, and it is still going a second
    later. What it is checked against is the runner's own state rather than a
    number written here, because the assertion is "there is no deadline" and not
    "the deadline is long".
    """
    import scripting

    runner = scripting.Runner("127.0.0.1:1")
    try:
        started = runner.start("while True:\n    pass\n")
        check("a behaviour starts", started["ok"], True)
        check("...with no limit reported, because it has none",
              started["limit_s"], None)
        check("...and none recorded either", runner._run.wall_limit, None)
        # A second is nothing next to the five minutes this used to get, so it
        # is evidence about the watcher rather than about the clock: what it
        # proves is that the run is still alive with nobody having stopped it.
        time.sleep(1.0)
        check("it is still running with nothing to end it",
              runner.status()["outcome"], "running")
        stopped = runner.stop()
        check("a stop is what ends it", stopped["outcome"], "stopped")
        check("...and it says who did", stopped.get("error"), "stopped")
        check("the slot is free afterwards",
              runner.run("print('after')")["outcome"], "finished")

        # Asking for a deadline still works, and is still bounded below at a
        # second: this is the caller who does want its behaviour to end.
        asked = runner.start("import time\ntime.sleep(30)\n", limit_s=45)
        check("a behaviour may ask for a limit and gets it", asked["limit_s"], 45.0)
        runner.stop()
        # And nought is how a caller says "no limit" out loud rather than by
        # leaving the argument out, which the console has to be able to do.
        explicit = runner.start("import time\ntime.sleep(30)\n", limit_s=0)
        check("nought asks for no limit at all", explicit["limit_s"], None)
        runner.stop()
    finally:
        runner.close()


TESTS = (
    test_scripts_run_and_say_what_happened,
    test_two_things_at_once,
    test_the_script_tools_are_offered_to_the_rover_and_not_to_the_lan,
    test_one_script_at_a_time,
    test_a_behaviour_runs_until_it_is_stopped,
)

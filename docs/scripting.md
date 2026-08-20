# Programs instead of a longer tool list

A design for letting a model send **code** to the rover — a few lines for
something simple, a whole behaviour for something that is not — written against
primitives the daemon already owns, instead of answering every new idea by adding
another tool. Written 2026-08-20.

**The MVP of this is built and running on the rover** — see [what is
built](#what-is-built-and-what-waits) for what that covers and what it measured.
Everything past it is still design. None of it replaces the tools in
[rover_daemon.py](../rover_daemon/rover_daemon.py): scripting is a second surface
beside them, reached from the rover itself rather than from a conversation.

The case for it is expressiveness. A tool list is a fixed vocabulary: everything
it does not contain is something nobody can ask for, and the only way to widen it
is to write Python on a workstation, deploy it and restart the daemon. A program
composes what is already there — point the camera, take a frame, ask the
detector, drive a metre, look at the scan — into behaviours that were never
anticipated, and it does so at conversational speed rather than at deployment
speed. "Watch the door and tell me when somebody comes in" is not a tool and
never will be; it is four lines.

## The single core is the constraint, not the tool list

This is the correction that shapes everything else, because the obvious version
of this idea — hand the model the servos and let it write the control loop — is
wrong on this rover for reasons that have nothing to do with the design.

The host is a Pi 1: one 700 MHz core, 474 MB, no SIMD. See [hosts.md](hosts.md).
What is already on it, measured:

| | costs |
|---|---|
| scan-matched 2D SLAM, in C | 33.5% of the core |
| forwarding 30 fps of 640×480 MJPEG | ~30% of the core |
| the face-tracking loop | 2.3–2.4 frames a second |
| of which, decoding one JPEG | 275–308 ms wall, 115–135 ms busy |
| of which, the detection on the OAK | 123–127 ms |
| age of the frame that loop steers by | median 1.33 s |

The numbers are from [moving-to-new-hardware.md](moving-to-new-hardware.md),
which also explains why the frame age is arithmetic rather than a bug: a
five-frame queue drained four times a second is 1.3 seconds of delay, and no
software on this host changes it.

So a script that tried to close the tracking loop itself would be strictly worse
than the loop already running, and it would be worse in the way that is hardest
to see — not slower, but steering by a picture from a second ago. **Scripts
orchestrate; they never servo.** The primitives a script gets must therefore be
behaviour-sized rather than sample-sized: follow a face until something happens,
drive a metre, wait for a person, take one picture. Never a servo write at 30 Hz,
and never a per-frame callback.

That rule also gives a script's tick rate a meaning it would not otherwise have.
One "how many faces can you see" is a JPEG decode plus a round trip to the OAK,
so it is about 0.4 s of the core. A script polling that every two seconds costs a
fifth of the machine; the same script polling five times a second *is* the
machine, and the SLAM underneath it stops keeping up. The pacing primitive is
where that shows, which is why it is a primitive and not a `time.sleep`.

## The script is a process, not a sandbox

Run the snippet as its own process on the Pi, launched and owned by the daemon,
reaching the hardware only through the newline-delimited JSON the daemon already
speaks on 8769.

The alternative — a sandboxed interpreter inside the daemon — buys the same
properties at a much higher price, and the price is worth listing because it is
the whole of the decision:

| what a runaway script needs | in-process VM | a child process |
|---|---|---|
| being stopped mid-loop | an interrupt hook the VM has to provide | `SIGKILL` |
| a memory ceiling | a custom allocator | `/proc/<pid>/statm`, four times a second |
| not holding the UART or the camera | discipline, enforced by review | it never had them |
| a crash not taking the rover down | exception handling that has to be right | the daemon does not notice |

The last row is the important one. The daemon is the one process that may own the
serial port and the camera, and that invariant is not tidiness — it is the only
arrangement that works, since two writers on one UART is interleaved JSON on a
wire. A child process that can only ask, never touch, cannot break it however
badly it is written.

The cost is a round trip per primitive call. On loopback that is milliseconds
against primitives that are tenths of seconds, so it is invisible at the tick
rates the previous section allows — and it disappears entirely for anything that
was going to block anyway.

Two smaller consequences worth having deliberately. The script's custody belongs
to the daemon rather than to whoever submitted it, so a client that drops its
connection mid-behaviour does not leave a program running with nobody watching.
And nothing about this is specific to the machine: it is a socket and a standard
library, so unlike `libslam2d.so` and `liboak.so` it moves to new hardware as a
file copy.

## Python, and specifically not Luau

Python, because it is already on the box, because models write it better than
anything else by a wide margin, and because in a child process the operating
system supplies the preemption and the memory ceiling that a VM would otherwise
have to.

Lua earns its keep in exactly one situation: scripting *inside* the daemon, where
its instruction-count hook is a real interrupt point that CPython has no
equivalent of — `settrace` is too expensive to leave on, and an asynchronous
exception cannot break out of a C call. That situation does not arise here,
because per-frame scripting is what the previous section rules out. Luau
specifically adds a C++17 build on an armv6 host with 474 MB, and embedding
either language into a Python daemon needs a bridge — `lupa`, or a `ctypes` shim
carrying callbacks — which would be the hardest part of the whole job and would
exist to solve a problem this design does not have.

If per-frame scripting ever becomes worth having, it becomes worth having on the
new board, and the language question can be re-opened there against a host that
can actually run the loop.

## Five calls, and none of them is a tool

The daemon gained:

* `run_script` — source in, result out, for anything short enough that the caller
  can wait for it.
* `start_script` — the same, for a behaviour that runs for minutes; returns a
  handle instead of a result.
* `script_status` and `script_stop` — read the handle, or kill it.
* `list_api` — the reference a script is written against.

**None of them is offered to a model, and four of the five are refused on
anything but loopback.** That is a change from what this document first said,
and the security section below is the reason: submission is the code-execution
path, and served on the LAN it would hand a stranger a shell on the Pi rather
than the ability to flash the headlights. Bound to loopback it grants what an
ssh session on this Pi already grants, and it is reached the same way — a tunnel,
or an agent working on the rover itself. `script_status` is the exception and
stays on the LAN, because watching a behaviour run changes nothing and is what a
console on a desk wants. What lets a model use a behaviour is `run_behaviour`,
still to be built, which runs something already written and reviewed.

`list_api` follows the discipline `list_tools` already established: the daemon is
the only thing that knows what this rover can do, so it is the only thing that
describes it, and no client carries a copy that can drift out of step. It is
generated by looking at the module rather than written down beside it.

One script at a time, and starting one stops face tracking — exactly as `look_at`
does, and for the same reason, since two things aiming one gimbal is two robots.
Stopping works while the script is mid-call, because the daemon is threaded per
connection and `stop_driving` takes no hardware lock, so a second connection can
interrupt a first.

**Every run ends with the wheels stopped, and the gimbal left where it is.** A
script killed inside a `drive` leaves the daemon finishing a move on behalf of a
connection that has gone, so stopping is not optional. Centring the camera would
be, and this document originally said to do it: it is not safety, and a behaviour
that ends with the camera deliberately pointed at something should not have it
swung away. Nothing else is undone — a script killed with the headlights on
leaves them on, because there is no general way to reverse an arbitrary program
and pretending otherwise would be worse than saying so.

A failure comes back as an explanation with a line number attached rather than as
a traceback — `line 4: NameError: name 'facse' is not defined` — because the
source is compiled under its own name rather than run from the temporary file it
arrives in. How a run ended is `outcome`: finished, failed or stopped. It is
deliberately not the protocol's `ok`, which everywhere else means the daemon
answered, and a status call about a script that failed has succeeded.

## Starting an interpreter costs more than most scripts do

Measured on the rover, with the daemon running: **1.8 s for a bare `python3 -c
pass`, and 4.2 to 4.8 s before the first line of a script executes.** The core is
one 700 MHz ARM1176 that the daemon is already more than half using, so
everything on it runs at about half speed, and each stdlib module the API imports
is paid again on the way in. Trimming what could be trimmed — `base64` and
`traceback` moved to the two paths that actually need them — bought a few hundred
milliseconds of it. The rest is CPython starting.

That is not overhead to be shaved away later; it is the shape of the thing, and
two decisions come out of it. A blocking `run_script` is for acts rather than for
anything interactive, since the reply cannot arrive in less than five seconds
however trivial the program. And anything that matters belongs in `start_script`,
where nobody is holding a connection open. It is also the strongest argument for
a warm interpreter on the new board — which is on the list below and is not worth
building here, because 10 MB held permanently on a 474 MB machine to save four
seconds is the wrong trade on this one.

The two clocks this creates have to be kept apart, and getting it wrong is a real
fault rather than an untidiness. The script's own deadline runs from its first
line; the runner's kill runs from the spawn and has the startup allowance added
to it. Measured from the spawn instead, the interpreter's four seconds come out
of the script's budget, and a correct behaviour is killed a fifth of the way
through what it was promised.

## What a program looks like

Concrete, because primitives are only as good as the programs they make readable.
"Tell me when somebody comes into the room":

```python
from rover import faces, gimbal, say, every

gimbal.look_at(pan=0, tilt=0)
empty = True
for _ in every(2.0, for_s=600):      # tick, with a deadline
    seen = faces.count() > 0
    if seen and empty:
        say("somebody just came in")
    empty = not seen
```

`every` is doing more than `sleep`. It is where the run's deadline is enforced,
where the stop flag is checked, and where a program that would otherwise busy-wait
is made to yield the core — so the three ways a script can eat this rover are all
one primitive, and a script that does not use it does not loop.

`say` is the other half of long-running behaviour, and it is the piece with no
counterpart today: a program that runs for ten minutes and speaks once is the
whole point, and it needs a road into the conversation. That road exists in
outline already — `look` posts its picture straight to the model's host rather
than handing it back through the client — and this is the same shape with words
instead of a frame.

## Do not give a script the motors

Bounded navigator calls only: drive a distance, turn an angle, stop. No raw PWM.

The reason is that the only real failsafe on this rover is the firmware's own
heartbeat, and it is not automatic. The board's default is three seconds; the
navigator tightens it to 500 ms when it starts a move and then keeps feeding it,
and the board stops the base if it hears nothing for that long. Gimbal commands
deliberately do not feed it, so aiming the camera is never mistaken for driving.
A script issuing motor commands directly would have to reimplement all of that
correctly, and getting it wrong is a rover still driving after the program
steering it has died. Handing it `drive(1.0)` instead costs nothing in
expressiveness and leaves the deadman under the navigator, where it works —
including the watchdog that stops a move the moment the lidar goes quiet.

## The author is not the speaker

The realtime speech model is the wrong thing to be writing programs. Code
generation mid-conversation is slow, audio-first models write code badly, and a
turn spent emitting thirty lines is a turn in which the rover says nothing.

So separate authoring from speaking, in time. A text-capable agent — Claude Code
on a desk, most plausibly — writes a behaviour, runs it against
[mock_rover.py](../voice_chat/mock_rover.py), watches it work on the real rover,
and **saves it under a name**. The voice model never writes code; it invokes what
has been written. Code generation then grows what the rover can be asked to do
without growing what the model has to hold in its head while talking.

This also puts the development loop somewhere it can be honest. A behaviour
becomes a file that can be re-run, diffed and fixed, rather than a one-shot
utterance that worked once and cannot be recovered.

## How the author reaches the rover

Three shapes, in the order they should arrive.

**Today it is the deploy path, and that is the whole integration.** The agent is
Claude Code on a desk with the repository checked out. It writes the behaviour as
a file, runs it against the mock rover, copies it to the Pi and watches it work
over 8769 — the same road every other file here travels, already described in
[CLAUDE.md](../CLAUDE.md). Nothing new is built, and every behaviour that reaches
the rover has been read by a person on the way. What it costs is that teaching
the rover something needs a workstation and a few minutes, so it cannot be done
while standing in the room talking to it.

**Then the agent drives the daemon directly**, which the MVP has now made
possible: `run_script` over an ssh session to the rover is a shell one-liner, and
the loop it closes is write a program, run it on the real hardware, read back the
error with its line number, fix it. What is still missing is the end of that
sentence — there is nothing to save it as, so a behaviour that works exists only
in the transcript of the session that made it. `save_behaviour` and a catalogue
are the next piece. An MCP server over any of this would be sugar rather than new
machinery.

**Last, and deliberately deferred: the rover teaches itself.** The voice model
calls something like `learn_to("watch the door")`, a text model writes the
program, and it appears in the catalogue. This is where the product in the idea
actually is, and three things put it at the end. Writing and testing a behaviour
takes tens of seconds against a twelve-second tool patience, so it has to be
asynchronous, which lands it squarely on the cue problem in the next section. The
conversation has to hold a thread across that gap without going quiet. And it
means code that drives a robot around a house runs having been read by nobody.

Three decisions do not depend on which of the three is in force, and they are
worth settling before the first one is built.

**Where behaviours live conflicts with this repository's own rule** that the repo
is source of truth and nothing is edited in place on a host. The resolution is
that an agent-written behaviour is *data* rather than source: it belongs in a
store on the Pi with its provenance attached — who asked for it, in what words,
when, and what it was tested against — and one that has proved itself gets
promoted into the repo by hand, like any other code. What must not happen is the
two drifting apart silently, which is exactly what a store that a deploy
overwrites would do.

**Saving is a promotion, not a write.** A behaviour does not enter the catalogue
until it has run on the actual rover at least once, and what happened on that run
is stored beside it: what it printed, whether it finished, whether somebody had
to stop it. That is this repository's "prove it on the hardware" rule turned into
an invariant code can enforce, and it is the only thing standing between a
catalogue and a heap of programs nobody has ever watched run.

**The agent writes the catalogue entry as well as the program, and the entry
matters more.** The voice model chooses a behaviour by its one-line description
and nothing else. After the crowding measurements in the next section, that one
sentence is doing more work than the code underneath it.

## A saved behaviour must not become a tool

This is the part that looks obvious and is backwards. The tempting design is that
saving a behaviour adds it to `list_tools`, so the voice model simply sees a
longer list. Two measurements say not to.

**Crowding costs accuracy.** Against the nine base tools, `flash` calls the right
one for "Start tracking people." three times out of three. Against the daemon's
current fifteen, the same wording is one out of three and "Follow me." is zero out
of three — both failing by announcing in the past tense that it has done
something it never called. It is crowding rather than prompting, and two
rewordings did not fix it. Every saved behaviour makes it worse.

**And the schemas are paid every turn.** They are sent once per session, which
saves the upload and not the bill: prompt and schemas measured 1,450 of a
request's roughly 1,470 input tokens, and the service reports `cached_tokens` at
0 or 128 against 1,900–2,400 input tokens a turn.

So keep the tool list short and fixed, and let the catalogue travel as **data in
results** rather than as schemas in session state: one `run_behaviour(name, ...)`
whose schema never changes, with `list_behaviours()` beside it. A behaviour saved
a minute ago is then callable immediately.

That last property is not a nicety, because the alternative is genuinely awkward.
Tools are session state on the realtime path — [talk.py](../voice_chat/talk.py)
sends them once, in the single `session.update` it opens with — so a behaviour
saved mid-conversation is invisible until the next connection. They can be
refreshed without reconnecting, and the machinery is already there: the client
re-sends the whole session mid-conversation to switch turn detection off while a
picture uploads, and waits to be told it took. But two constraints come with it.
That API's `session.update` replaces rather than merges, so the entire session
goes back each time; and the update has to land between turns, because a turn
committed while it is still being applied is answered by a model that has the
prompt and not the tools — which presents as the model reading a tool call aloud
instead of making one. A catalogue that is data sidesteps all of it.

Making `list_tools` itself model-callable does not help either, for a reason
worth writing down: reading a name out of a tool result is not the same as having
its schema. The service validates calls against the list it was given, so a name
the model has only read is not reliably callable, and the likely outcome is that
same narrated-tool-call failure. Discovery through a result does not remove the
need for the schema to be in session state; it only moves where the confusion
happens.

The one thing a data catalogue still needs is a cue. The voice model has no
reason to ask what is in it unless something prompts it, so either the person
says they have taught the rover something, or whatever saves a behaviour drops a
line into the conversation.

## A code endpoint is a shell on the Pi

Port 8769 authenticates nothing, deliberately, on a home LAN — the same trade
`face-detect` makes. The worst a stranger on that LAN can do through the tools is
flash the headlights and drive the rover into a wall. Through a code endpoint
they would have arbitrary execution on the machine bolted to it.

That is a change in kind rather than in degree, and it is why the four calls that
submit or stop code are refused on anything but loopback, and why none of them is
in `list_tools` — the same treatment `set_vision` gets, for a stronger reason.
Reaching them means an ssh session to the rover or a tunnel over one, which is a
door that already existed. It is not real security and is not meant to be: the
point is that the rover's tools and a code endpoint should not inherit the same
exposure merely by arriving on the same port.

The obvious consequence is deliberate. No model can run a script on this rover,
because the clients that hold a conversation are on a desk across the LAN. What
a model gets later is `run_behaviour` — something already written, tested on the
hardware and given a name — which is a different proposition from code composed
in the middle of a conversation and never read by anybody.

## What is built, and what waits

The MVP is on the rover as of 2026-08-20. Two files beside the daemon and about
sixty lines inside it:

* [scripting.py](../rover_daemon/scripting.py) — the daemon's end. One slot, a
  child process per run, output drained into a bounded buffer, a wall-clock cap
  and a memory ceiling, and a kill that takes the whole process group.
* [rover_api.py](../rover_daemon/rover_api.py) — the script's end. The existing
  tools as named primitives, plus a frame, the detector run on a frame, absolute
  gimbal angles, the live scan, `every`, and `call()` underneath them for a tool
  that arrived after the module did.
* The five calls above, loopback-gated in the daemon's connection handler.

What it does on the rover, measured there rather than inferred:

| | |
|---|---|
| a one-shot script: headlights, a gimbal move, one look at the room | 9.96 s wall — 4.75 s of it starting the interpreter |
| what a script's own process holds | 10.3 MB peak, against a 96 MB ceiling |
| a script spinning in `while True: pass` | stopped, on time, without the daemon noticing |
| a script allocating without limit | stopped at 96.7 MB |
| a behaviour stopped mid-tick | its `finally` block ran, and its output survived |
| the same calls from the LAN | refused, with the tunnel named |

That last row and the one above it are the two that matter. A stop is polite
first — `SIGTERM` becomes an exception at the next `every`, so a behaviour
unwinds through its own cleanup — and impolite two seconds later, which is how a
program that is in no position to notice still stops. On the rover a script cut
off at its limit got two more lines out and turned the headlights back off before
the `SIGKILL` was due.

Three limits are worth knowing: a blocking run gets 15 seconds by default,
sized so that a script which opens the camera fits, since a cold `count_faces`
is five seconds on its own; a behaviour gets five minutes and may ask for
thirty; and either is killed at 96 MB.

[mock_rover.py](../voice_chat/mock_rover.py) speaks the same wire protocol and
holds the state a real rover would, so programs can be written and run against a
rover that is not there — which is where the real question gets answered, because
it is where you find out whether a model writes correct programs against these
primitives.

Later, roughly in the order they stop being premature: **a vision query that
answers**, since a script can post a picture to the conversation and cannot ask
about one, which is what the harder behaviours need; saved behaviours with
`run_behaviour` and a catalogue; `say`, so a running program can reach the
conversation; a warm interpreter, to spend 10 MB on the four seconds above;
a scheduler, so a behaviour can outlive the session that started it; persistent
state, so a program can remember where the kitchen is; per-script capability
limits, so a behaviour can be allowed to look and not to drive; and an in-process
VM, if per-frame scripting ever earns one.

## The Pi 1 can start this

It has. A script's own process peaks at 10.3 MB of the 474 — measured, against
the 15 MB this document guessed — and takes close to no core while it sits
blocked on a daemon call, which is affordable beside SLAM at a third of the
machine and the camera at another third. What is not affordable is anything
per-frame inside the script, which is exactly what the orchestration rule
protects, and the four-second interpreter start is the price of the process
boundary that makes a runaway killable.

The move to the Banana Pi Zero changes the options rather than the design. There
is nothing architecture-specific to port, unlike the two `.so` files that have to
be rebuilt per host, so the layer itself is a file copy. What the new board adds
is headroom: faster ticks, cheaper frames, and eventually the choice of an
in-process VM and per-frame scripting — a choice this host does not have and
would not benefit from being given.

## What is measured and what is assumed

The figures in [what is built](#what-is-built-and-what-waits) and in [starting an
interpreter](#starting-an-interpreter-costs-more-than-most-scripts-do) were taken
on the rover on 2026-08-20, against the running daemon, the real driver board and
the OAK. The CPU and timing figures for the tracking loop are quoted from
[hosts.md](hosts.md) and [moving-to-new-hardware.md](moving-to-new-hardware.md),
and the tool-selection and token figures from [talk.py](../voice_chat/talk.py)
and the voice service's README; those were measured when those documents say, not
for this one.

Two assumptions remain, and both can invalidate the design rather than merely
tune it:

* **That a model writes correct programs against these primitives at all.** Still
  untried: everything above was written by hand. It is cheap to answer — a mock
  rover, a handful of behaviours asked for in English, and a count of how many
  run — and it is the next thing worth doing, ahead of any more machinery.
* **That a behaviour is the right unit.** If most of what a program turns out to
  want is a single primitive the tool list was simply missing, the honest
  conclusion is to add the tool and not the interpreter.

# Programs instead of a longer tool list

A design for letting a model send **code** to the rover — a few lines for
something simple, a whole behaviour for something that is not — written against
primitives the daemon already owns, instead of answering every new idea by adding
another tool. Written 2026-08-20.

**The MVP of this is built and running on the rover** — see [what is
built](#what-is-built-and-what-waits) for what that covers and what it measured.
Everything past it is still design. None of it replaces the tools in
[rover_daemon.py](../rover_daemon/rover_daemon.py): scripting is a second surface
beside them, reached from the rover itself.

**And as of 2026-08-24 the conversation reaches it too**, which this document
spent its security section arguing against and its authorship section calling the
wrong idea. Both arguments turned on where the client was, and the client moved:
the rover holds its own session with Alibaba's model now, so `run_script` is a
model tool offered to a caller on loopback. What changed and what did not is in
[a code endpoint is a shell on the rover](#a-code-endpoint-is-a-shell-on-the-rover)
and [the author is not the speaker](#the-author-is-not-the-speaker), which have
been left standing rather than quietly reworded, because the reasoning in them is
still the reasoning — it is one of its premises that expired.

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

The host is a Jetson Orin Nano since 2026-08-31: six Cortex-A78AE cores and
7.3 GB, where the Banana Pi M4 Zero before it had four Cortex-A53 and 3.9 GB. See
[hosts.md](hosts.md). What is already on it, measured — the left column on the
Pi 1 this was written for, the right on the Banana Pi. **Neither column has been
re-measured on the Orin**, which is faster than both, so read the numbers as the
worst case rather than as this rover. The argument below survived one move and
survives this one; the margins are what changed:

| | Pi 1, one core | M4 Zero, four cores |
|---|---|---|
| scan-matched 2D SLAM, in C | 33.5% of the core | a core, near enough |
| the face-tracking loop | 2.3–2.4 fps | 6.6 fps |
| of which, decoding one JPEG | 275–308 ms wall, 115–135 busy | 7 ms |
| of which, the detection | 123–127 ms on the OAK's VPU | 146 ms on three cores |
| age of the frame that loop steers by | median 1.33 s | ~190 ms |
| the OAK, now a depth camera | — | 13% of one core |

The numbers come from a hardware-comparison document removed on 2026-08-25, which
also explained why the frame age is arithmetic rather than a bug: a
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
One "how many faces can you see" opens the camera, decodes a frame and runs YuNet
over it — about 0.3 s, of which 150 ms has three of the four cores. A script
polling that every two seconds costs a fifteenth of the machine; the same script
polling five times a second *is* the machine, and the SLAM underneath it stops
keeping up. On the Pi 1 that round trip went to the OAK and cost 0.4 s of the one
core, so the ratio is better now and the rule is not different. The pacing primitive is
where that shows, which is why it is a primitive and not a `time.sleep`.

## The script is a process, not a sandbox

Run the snippet as its own process on the rover, launched and owned by the daemon,
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

## Five calls, and three of them are tools

The daemon gained:

* `run_script` — source in, result out, for anything short enough that the caller
  can wait for it.
* `start_script` — the same, for a behaviour that runs until it is stopped;
  returns a handle instead of a result, and carries no deadline.
* `script_status` and `script_stop` — read the handle, or kill it.
* `list_api` — the reference a script is written against.

**Four of the five are refused on anything but loopback, and three of those four
— `run_script`, `start_script` and `script_stop` — are offered to the model as
tools.** The loopback rule is from the
security section below and has not changed: submission is the code-execution
path, and served on the LAN it would hand a stranger a shell on the rover rather
than the ability to flash the headlights. Bound to loopback it grants what an
ssh session on this board already grants, and it is reached the same way — a
tunnel, an agent working on the rover itself, or, since 2026-08-24, the
conversation, because the conversation is on the rover now.
`script_status` is the exception to the gate and stays on the LAN, because
watching a behaviour run changes nothing and is what a console on a desk wants.

`start_script` and `script_stop` were both control calls until a behaviour
stopped having a time limit, and that is what changed the argument. While a
behaviour was shot after five minutes, starting one was a thing to do from a
console with somebody watching, and the model had nothing it needed to stop
because a blocking run was over before the next turn. A behaviour that runs
until it is stopped is a different object: it is the only shape that answers
"follow me until I tell you to stop", and something has to be able to tell it.
So the two arrived in `list_tools` together, and deliberately together —
starting without stopping would hand the model the rover's single script slot
with no way to give it back.

The two that stay out stay out for their own reasons rather than for the
security one. `script_status` is watching, which is what a console wants and not
what a conversation does; and `list_api` is a catalogue whose contents are now
written into `run_script`'s own description — a model that has to ask what the
primitives are before it can write anything is a model that will write first and
ask afterwards.

The three are offered only to a client on loopback, which is `Rover.tools`
taking the same care it already takes over `look`: a tool that cannot do what it
says is worse than a missing one, and a schema handed across the LAN would be
one whose every call comes back "reach it through an ssh tunnel". So anything
holding a conversation from a desk -- there is no such client in this repository
any more, but the daemon still serves one -- sees the seventeen tools it always
saw, and the session on the rover sees twenty.

`list_api` follows the discipline `list_tools` already established: the daemon is
the only thing that knows what this rover can do, so it is the only thing that
describes it, and no client carries a copy that can drift out of step. It is
generated by looking at the module rather than written down beside it.

One script at a time, and starting one stops face tracking — exactly as `look_at`
does, and for the same reason, since two things aiming one gimbal is two robots.
Stopping works while the script is mid-call, because the daemon is threaded per
connection and `stop_driving` takes no hardware lock, so a second connection can
interrupt a first.

**That one slot is now the only thing bounding a behaviour, so the refusal has to
say what is holding it.** Nothing frees the slot on its own any more: a `start`
without a `limit_s` has no deadline, and what ends it is the script finishing, the
script failing, or somebody calling `script_stop`. A second `start` is refused
rather than queued — a caller told its behaviour started when it is really second
in line will say so out loud — and the refusal carries the id of the run in the
slot and how long it has been there, so the next thing to do is obvious.

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

**Those numbers are the Pi 1's, and the board underneath this changed.** On the
Banana Pi M4 Zero the same measurement is **0.51 s** from spawn to the script's
first line, reported by the runner as `startup_s` in every reply. That is the
difference between four A53s at 1.8 GHz and one ARM1176 at 700 MHz, and it is
what makes a blocking `run_script` usable in a conversation at all: the reply to
a three-second program arrives in three and a half seconds rather than in eight.
`STARTUP_S` is left at six seconds all the same, because it is an allowance
against being killed early rather than an estimate, and a rover mid-scan can be
slower than a rover at rest.

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

`every` is doing more than `sleep`. It is where a deadline is enforced if the run
was given one, where the stop flag is checked, and where a program that would
otherwise busy-wait is made to yield the core — so the three ways a script can eat
this rover are all one primitive, and a script that does not use it does not loop.
A behaviour has no deadline unless it asked for one, which makes the stop flag the
important half: `every` is how being stopped reaches a running program at all.

`say` is the other half of long-running behaviour, and it is the piece with no
counterpart today: a program that runs for ten minutes and speaks once is the
whole point, and it needs a road into the conversation. That road exists in
outline already — `look` posts its picture straight to the model's host rather
than handing it back through the client — and this is the same shape with words
instead of a frame.

## Do not give a script the motors

Bounded driving calls only: drive a distance, turn an angle, stop. No raw PWM.

The reason is that the only real failsafe on this rover is the firmware's own
heartbeat, and it is not automatic. The board's default is three seconds; whatever
is driving tightens it to 500 ms and then keeps feeding it — that is
`ros_nav/base_node.py` now, on the ROS side of the bridge — and the board stops
the base if it hears nothing for that long. Gimbal commands deliberately do not
feed it, so aiming the camera is never mistaken for driving. A script issuing
motor commands directly would have to reimplement all of that correctly, and
getting it wrong is a rover still driving after the program steering it has died.
Handing it `drive(1.0)` instead costs nothing in expressiveness and leaves the
deadman where it works — including the watchdog that stops a move the moment the
lidar goes quiet.

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

**2026-08-24: the speaker writes as well, and the paragraph above is still the
reason that is a compromise rather than a plan.** The rover now holds its own
conversation, which took away the accident that was enforcing this — a desk
across the LAN — and `run_script` was asked for as a model tool. So the voice
model may write a program in the middle of a sentence, and everything said
above about it being the wrong thing for the job is still true. What the design
does about it is name the cost in the tool's own description: the rover cannot
speak while a program runs, so the model is told to say what it is about to do
first, to keep the program to a few seconds, and that anything longer is
something to say it cannot do yet.

Two things make it survivable rather than merely allowed. The primitives are
written into the schema, generated from
[rover_api.py](../rover_daemon/rover_api.py) by introspection, so the model is
not guessing at names in the one turn it has to get them right — that is what
`Rover.script_tools` is for, and why that description is assembled at runtime
rather than typed out. And a failure comes back naming the line that failed, so
a wrong program is a correction rather than a dead end.

What is still not measured is whether it writes *correct* programs from speech,
and what three more tools — one of them carrying a page of primitives — do to the
seventeen around them: a tool is read against its neighbours, and every number in
[voice_chat/README.md](../voice_chat/README.md) was taken with ten of them. Those
runs are at least not invalidated: they are made on a desk against
[mock_rover.py](../voice_chat/mock_rover.py), and a client that is not on the
rover is not shown this tool at all.

None of which retires the authoring path. A behaviour worth keeping is still a
file somebody can read, and `run_behaviour` is still the shape for that; what has
changed is that the throwaway three-liner no longer has to become one.

## How the author reaches the rover

Three shapes, in the order they should arrive.

**Today it is the deploy path, and that is the whole integration.** The agent is
Claude Code on a desk with the repository checked out. It writes the behaviour as
a file, runs it against the mock rover, copies it to the rover and watches it work
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
store on the rover with its provenance attached — who asked for it, in what words,
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
Tools are session state on the realtime path —
[session.py](../voice_chat/session.py) sends them once, in the single
`session.update` it opens with — so a behaviour
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

## A code endpoint is a shell on the rover

Port 8769 authenticates nothing, deliberately, on a home LAN — the same trade
`face-detect` makes. The worst a stranger on that LAN can do through the tools is
flash the headlights and drive the rover into a wall. Through a code endpoint
they would have arbitrary execution on the machine bolted to it.

That is a change in kind rather than in degree, and it is why the four calls that
submit or stop code are refused on anything but loopback. Reaching them means an
ssh session to the rover or a tunnel over one, which is a door that already
existed. It is not real security and is not meant to be: the point is that the
rover's tools and a code endpoint should not inherit the same exposure merely by
arriving on the same port.

**What this section originally concluded was that no model could run a script
here, because the clients that hold a conversation were on a desk across the
LAN.** That sentence is about a deployment rather than about a rule, and the
deployment changed earlier the same day this was rewritten, when the rover took
over holding its own session — see
[drive_web/omni_bridge.py](../drive_web/omni_bridge.py). The gate did not move;
the conversation moved inside it. Worth being plain about what that means: the
sentence was read here as a security property while what it actually described
was where a microphone happened to be plugged in, which is a poor thing to have
been relying on either way.

So the exposure to weigh now is not the LAN, it is the microphone. Whoever can
talk to the rover can have it run a program on itself, and what stands between a
stranger and that is the console's own token — `~/.ugv/console.token`, which
gates the microphone button and nothing else — plus the fact that the driving
controls beside it were never gated at all. Somebody who can reach the console
page can already drive the rover into a wall. What they can now also do is spend
fifteen seconds of its CPU and read a file as the rover's own account, which is a
real widening,
and it is the reason the four remaining calls stay where they are: a fifteen-
second blocking run that reports what it printed is a much smaller thing to hand
out than a behaviour that outlives the conversation.

## What is built, and what waits

The MVP is on the rover as of 2026-08-20. Two files beside the daemon and about
sixty lines inside it:

* [scripting.py](../rover_daemon/scripting.py) — the daemon's end. One slot, a
  child process per run, output drained into a bounded buffer, a memory ceiling,
  a wall-clock cap on the blocking kind of run and none on a behaviour, and a
  kill that takes the whole process group.
* [rover_api.py](../rover_daemon/rover_api.py) — the script's end. The existing
  tools as named primitives, plus a frame, the detector run on a frame, absolute
  gimbal angles, the live scan, `every`, `alongside` for the one thing a list of
  calls cannot express, and `call()` underneath them for a tool that arrived
  after the module did.
* The five calls above, loopback-gated in the daemon's connection handler, three
  of them — `run_script`, `start_script` and `script_stop` — also offered as
  tools to whichever client is on loopback, with the primitives above written
  into the first one's description and pointed at from the second.

**Two things at once, added 2026-08-27.** Everything that moves this rover blocks
until the move is over, so a program written as one list of calls can only ever
do one thing at a time — and asked to turn and flash the headlights together, the
model did the only thing the surface allowed: a turn, then some flashing, or the
two chopped into alternating bursts. Threads were not the missing piece; they ran
perfectly well and achieved nothing, because every script shared one connection to
the daemon behind a lock and a `drive` holds that line for the length of the move.
Against a stand-in daemon whose turn takes three seconds, the shared line let a
single light change through and then nothing; a connection per thread flashed all
the way through the move. The daemon has always been threaded per connection, and
setting the lights holds the board only for the length of one JSON line, so the
concurrency was there to be had and it was the script's end declining it.

So the connection is per thread now, and the idiom on top of it is `alongside`,
which a model reaches for as a `with` block:

```python
def flashing():
    for tick in every(0.5):
        lights.set(255 if tick % 2 == 0 else 0)

with alongside(flashing):
    drive.turn(90)
lights.set(0)
```

The job is given the same kind of ending the script has, so `every` and `wait`
inside it raise `Stopped` when the block finishes: a loop with no end written into
it is exactly right there, and flashes for as long as the turn takes. Leaving the
block then waits for the job however long it takes, which matters because which
half goes where is not this design's to decide — asked out loud, the model wrote
the turn as the job and the flashing as the block, the same behaviour read the
way the English sentence runs. A job that is one long move has to be waited for
rather than cut off, since a drive stopped half way through and then described as
done is exactly the kind of lie this rover must not tell. Two things
that made bare threads worse than useless are fixed by going through the block
rather than around it. A job that raised used to leave the run reported as
`finished, ok: true` with the traceback printed into the output, where a model
reads it as something the program meant to say — it is now re-raised as the block
ends, at the line inside the job. And a thread the program walked away from used
to hold the rover's one script slot open for as long as it ran: a six-second
thread kept the slot shut for six seconds behind a script that reported nought
seconds of its own. `alongside` uses a daemon thread, and the harness no longer
waits for threads a script left behind, so the slot is free when the last line has
run. Tidying up therefore belongs after the block, not in the job — a daemon
thread is cut where it stands and its `finally` never runs, which is why the
headlights go out on the line after the block above.

On the rover, a job flashing the headlights through `drive.turn(90)` and
`drive.turn(-90)` put eight light changes inside 2.8 seconds of moving and none
after them, in a run that cost 12.4 MB and ended with the rover facing the way it
started. And asked out loud — "flash your lights while you turn around", through
the rover's own session — the model wrote a program for it, which is the part that
was in doubt. It took two attempts both times it was asked. The first ask went
wrong in a way worth keeping: it wrote `with alongside(drive.turn(180)):`, which
reads exactly right in English and is backwards in Python, since the argument is
evaluated before the block and the rover therefore turned before anything could
run beside it. The callable check named the fix, the model took it, and the
description now says the job is a function written with `def` and passed by name;
asked again, it got the shape right first time. What went wrong after that was a forgotten
import: `alongside` left out of the `from rover_api import ...` line that the
tool description spelled out in full, costing a wasted half-second run each time
before the model corrected itself from the NameError. So a program is handed the
primitives ready-made now — the six namespaces, the loose functions and the three
exceptions are in the namespace it starts with, and the tool description says it
imports nothing. Importing them still works and changes nothing; what is gone is
the step there was to get wrong.

What it does on the rover, measured there rather than inferred:

| | |
|---|---|
| a one-shot script: headlights, a gimbal move, one look at the room | 9.96 s wall — 4.75 s of it starting the interpreter |
| what a script's own process holds | 10.3 MB peak, against a 96 MB ceiling |
| a script spinning in `while True: pass` | stopped, on time, without the daemon noticing |
| a script allocating without limit | stopped at 96.7 MB |
| a behaviour stopped mid-tick | its `finally` block ran, and its output survived |
| the same calls from the LAN | refused, with the tunnel named |

And what the model does with it, measured on 2026-08-24 by speaking to the rover's
own session — a synthesised phrase pushed in where the browser's microphone goes,
against the deployed daemon on loopback:

| | |
|---|---|
| "Flash the headlights three times." | wrote a six-line program against `lights` and `wait`, correct first time, and said it had done it |
| what that run cost | 3.56 s wall, 0.51 s of it the interpreter, 12.1 MB peak |
| "Turn the lights on." | `set_lights`, not a program — the description's own instruction holding |
| "Sweep your camera slowly from left to right and tell me how many people you saw." | three `look_at` calls and one `count_faces`, no program |
| `list_tools` from the LAN, meanwhile | seventeen tools, `run_script` not among them |

The first row is the one that was in doubt: this document's own position was that
an audio-first model writes code badly. Asked for something plainly repetitive it
wrote `for _ in range(3)` around `lights.set` and `wait`, correctly, first time.

The third row is the more interesting one, and it is not a success. A sweep is
exactly what a program is for — point, look, count, move on — and what came back
instead was three gimbal moves with a single `count_faces` at the end of them, so
the answer it gave out loud was about the last position rather than about the
sweep. That is the discouraging sentence in the description working too well: told
not to write a program for anything a single tool already does, it decomposed the
job into tools that each did part of it. Nothing here yet distinguishes "several
acts in an order" from "several acts I can just perform".

Two phrases and a counter-example are not a measurement — the six-sample cells in
[voice_chat/README.md](../voice_chat/README.md) are what one looks like here.

That last row and the one above it are the two that matter. A stop is polite
first — `SIGTERM` becomes an exception at the next `every`, so a behaviour
unwinds through its own cleanup — and impolite two seconds later, which is how a
program that is in no position to notice still stops. On the rover a script cut
off at its limit got two more lines out and turned the headlights back off before
the `SIGKILL` was due.

Two limits are worth knowing, and the third is worth knowing about because it is
gone. A blocking run gets 15 seconds by default, sized so that a script which
opens the camera fits, since a cold `count_faces` is five seconds on its own; and
either kind is killed at 96 MB. A behaviour used to get five minutes and to be
allowed to ask for thirty, and now gets no deadline at all — it may still ask for
one with `limit_s`, but by default it runs until it ends or `script_stop` ends it,
which is why that call became a tool the model is offered.

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

Two changes of board since — to the Banana Pi M4 Zero and then to the Jetson Orin
Nano — have changed the options rather than the design. There is nothing
architecture-specific to port, unlike the two `.so` files that have to be rebuilt
per host, so the layer itself is a file copy. What each new board added is
headroom: faster ticks, cheaper frames, and eventually the choice of an in-process
VM and per-frame scripting — a choice the Pi 1 this was written for did not have
and would not have benefited from being given.

## What is measured and what is assumed

The figures in [what is built](#what-is-built-and-what-waits) and in [starting an
interpreter](#starting-an-interpreter-costs-more-than-most-scripts-do) were taken
on the rover on 2026-08-20, against the running daemon, the real driver board and
the OAK as it was then — a face detector rather than the depth camera it is now. The CPU and timing figures for the tracking loop are quoted from
[hosts.md](hosts.md) and from the hardware-comparison document removed on
2026-08-25, and the tool-selection and token figures from the realtime client -- then
`talk.py`, now [session.py](../voice_chat/session.py) -- and the voice service's
README; those were measured when those documents say, not
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

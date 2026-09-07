<!-- requirement-area: CTL -->

# The control surface

How the rover is reached, what each caller is allowed to see, and what the camera
and console must and must not do. The conventions for these records are in
[README.md](README.md).

<a id="r-ctl-1"></a>
### R-CTL-1 — The rover is reached through one protocol on one port

- **State:** settled
- **Evidence:** [rover_daemon/README.md](../../rover_daemon/README.md) — a
  JSON-lines protocol on TCP 8769

The console, the voice session, diagnostics and deployment verification all
arrive the same way. The practical consequence is that a change can be proved by
calling the affected function and reading the answer, which is what the working
rules in [../../AGENTS.md](../../AGENTS.md) mean by proof.

<a id="r-ctl-2"></a>
### R-CTL-2 — The tool schemas are held in one place and clients do not copy them

- **State:** settled
- **Evidence:** [rover_daemon/README.md](../../rover_daemon/README.md) —
  `tool_schemas.py` is the source of truth and `list_tools` returns the current set

A client keeping its own copy of a tool's shape is a client that will eventually
be calling a tool that no longer exists in that form.

<a id="r-ctl-3"></a>
### R-CTL-3 — The model is not shown controls it has no business choosing

- **State:** settled
- **Evidence:** [rover_daemon/README.md](../../rover_daemon/README.md) — the
  operational calls are deliberately absent from `list_tools`

Some calls exist for the console or for infrastructure and are kept off the
model's tool list: map rendering, raw camera frames, diagnostics, the
world-state inspection calls, and `refit_pose`.

`refit_pose` is the instructive one. It is safe — the search will not move the
rover more than a metre and the mapper does not fold the matched scan into the
graph — but whether the rover has been moved while switched off is a thing a
person knows and the rover does not. A model refused a route would reach for
anything labelled "fix the position", and the position is usually not what is
wrong.

<a id="r-ctl-4"></a>
### R-CTL-4 — The conversational model's access to the visual memory is read-only

- **State:** settled
- **Evidence:** [world_state/README.md](../../world_state/README.md) —
  `find_thing`, `go_to_thing` and `distance_between_things` are the whole surface

It cannot clear the store or write to it. The store is still a proof of concept
asking whether the rover builds a description of the room that stays coherent
across views, and until that has an answer nothing should be able to write to
that description or throw it away. The inspection calls also answer in the
console's vocabulary — identifiers, cosines, a placement as the store writes it —
which a model can neither say out loud nor reason about correctly.

See [R-SAFE-4](safety.md#r-safe-4) for the movement half of this.

<a id="r-ctl-5"></a>
### R-CTL-5 — Face tracking locates faces and never identifies people

- **State:** settled
- **Evidence:** [face_tracking/README.md](../../face_tracking/README.md);
  [rover_daemon/README.md](../../rover_daemon/README.md)

YuNet is a detector. The rover can find face boxes and hold a target by
geometric proximity; it cannot know who somebody is, whether a person who left
and came back is the same person, or classify glasses, age or expression.
`track_next` therefore means "suppress this detection briefly and acquire
another", not "identify a different person" — and the name is worth stating as a
requirement because it invites the opposite reading.

<a id="r-ctl-6"></a>
### R-CTL-6 — The depth camera follows what the rover is doing, not a switch

- **State:** settled
- **Evidence:** [rover_daemon/README.md](../../rover_daemon/README.md) —
  `rover_depth.py` reads the navigator's move mutex twice a second

The OAK comes on the moment the rover drives and goes off thirty seconds after it
stops. The move mutex is the signal because it is the fact rather than a flag
beside the fact: every drive, turn, tap on the map and exploration run holds it,
so this cannot drift out of step with what the rover is actually doing.

A daemon started without a navigator has no honest answer to "is it moving?" and
therefore starts no rule at all, because anything else would switch a sensor off
half a minute after boot and never switch it on again. The cost is four to six
seconds of firmware upload at the start of every drive, during which a look
records its regions with no distances.

<a id="r-ctl-7"></a>
### R-CTL-7 — The console is served over HTTPS to a trusted local network, with no login

- **State:** settled
- **Evidence:** [drive_web/README.md](../../drive_web/README.md);
  [wifi_roam/README.md](../../wifi_roam/README.md) — the certificate names the
  rover's fixed address, so the browser gets a clean padlock

The absence of authentication is a deliberate accepted limitation and not an
oversight: the console assumes anyone who can reach it on the house network is
entitled to drive the rover. It is recorded as a requirement so that exposing the
console beyond that network is recognisably a change to this record rather than a
configuration tweak.

<a id="r-ctl-8"></a>
### R-CTL-8 — The console shows state, never explanations

- **State:** settled
- **Evidence:** [drive_web/AGENTS.md](../../drive_web/AGENTS.md)

A reading, a count, what is happening now, what a button did, what failed and
why. If a line could have been written before the rover was switched on, it is
documentation and belongs in the README. The person at the console owns the rover
and has used it before.

<a id="r-ctl-9"></a>
### R-CTL-9 — A missing subsystem is reported missing, never silently replaced

- **State:** settled
- **Evidence:** [rover_daemon/README.md](../../rover_daemon/README.md) — an
  absent ROS stack is reported unavailable rather than falling back to the
  obsolete local planner

The failure this prevents is the worst kind: a rover that still drives, using
something nobody meant to be driving it, while every status reads healthy.

<a id="r-ctl-10"></a>
### R-CTL-10 — Rover-side scripts compose tools and never implement control loops

- **State:** settled
- **Evidence:** [runbooks/scripting.md](../runbooks/scripting.md)

A script runs as an ordinary child process under the same account as the daemon
and reaches hardware only through the tool protocol on loopback. That is process
isolation and not a sandbox, which is why scripts are a convenience for
sequences a person wants repeated and explicitly not the representation for
anything the rover might one day write for itself — see
[R-SAFE-8](safety.md#r-safe-8).

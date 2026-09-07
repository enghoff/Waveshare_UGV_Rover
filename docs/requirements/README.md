# Requirements: what has to be true of the rover

A requirement is a statement about the rover's behaviour that we intend to hold,
written so that it is possible to say whether it holds today. Each one carries a
stable identifier, so a plan can name what it is trying to achieve and a
measurement can name what it moved.

Requirements are grouped by the part of the rover they constrain, one file per
area:

| Area | File | Covers |
|---|---|---|
| `SAFE` | [safety.md](safety.md) | who may move the rover, what stops it, and what must never be bypassed |
| `NAV` | [navigation.md](navigation.md) | mapping, localization, routes and exploration |
| `WS` | [world-state.md](world-state.md) | observations, placement, identity and visual search |
| `CTL` | [control.md](control.md) | the tool protocol, the console, voice and the camera surfaces |
| `PLAT` | [platform.md](platform.md) | the host, runtime state, deployment and the network |

## What a requirement looks like

    ### R-WS-2 — A single observation never places a thing or fixes its identity

    - **State:** settled
    - **Evidence:** [world_state/README.md](../../world_state/README.md); `world_state/selftest.py`

    One viewpoint gives a bearing and no distance, so it cannot say where
    something is. ...

The heading is the identifier and a one-line statement of the requirement in the
present tense. Underneath it come the state and its supporting field, then as
much prose as the requirement needs to be understood — usually why it exists,
and what would make it false.

## The five states, and what each one owes

| State | Means | Must carry |
|---|---|---|
| `settled` | Required, implemented, and shown to hold on the rover | **Evidence:** what shows it |
| `open` | Required and agreed, but not shown to hold — or known not to | **Blocked by:** the plan, milestone or measurement that would settle it |
| `proposed` | Not yet agreed; may be dropped or rewritten | **Proposed in:** the plan, design or progress entry that proposes it |
| `failing` | Was settled, and a measurement has since shown it false | **Broken by:** the progress entry that found it |
| `retired` | No longer required | **Superseded by:** the decision record that dropped it |

`settled` is the strong claim and the bar for it is deliberately high: something
observed on the rover, or an offline suite that stands in for it where the
requirement is about logic rather than hardware. A passing unit test is evidence
for a rule about how the store behaves and is not evidence for a rule about where
the camera points.

A measurement is a legitimate origin for a requirement, which is why a progress
entry may propose one. Something the rover was found doing wrong is the most
common way a new requirement arrives here, and parking it as `proposed` keeps it
in the spine without pretending it has been agreed.

`failing` exists because this is a rover. A requirement that a measurement has
disproved should not quietly become `open`, as though it had never been believed
— the fact that it was thought to hold is part of what the next person needs to
know.

## Rules for writing them

**A requirement describes the rover, not how we work.** How to reproduce a
fault, when to deploy, who talks to whom: those are working rules and they live
in [../../AGENTS.md](../../AGENTS.md). If a statement would still make sense with
no people in the room, it is a requirement.

**The source wins, so do not restate it.** A requirement says that the chassis
refuses to drive without its calibration; it does not repeat the gyro scale.
Numbers that can change independently belong to the code and the runtime files,
and a requirement that copies one has simply created a second place for it to go
stale. Where a tolerance is genuinely part of the requirement, say where the
authoritative value lives.

**Say what would make it false.** A requirement nobody could disprove is a
slogan. "Identity is reliable" is a slogan; "no known incorrect
movement-eligible association in a reviewed sample of at least fifty decisions"
is a requirement.

**One requirement per statement.** If the state of half of it would differ from
the state of the other half, it is two requirements.

**Identifiers are permanent.** Numbers are assigned once and never reused, not
even after a requirement retires — a retired requirement stays in its file with
its state changed, because progress entries and commits refer to it by number.
Add new requirements at the end of their area.

## Changing one

Tightening the wording of a requirement, or moving it between states because
something was measured, is ordinary work: change it and record the measurement
in [../progress/](../progress/README.md).

Dropping a requirement, or changing what it demands, is a decision. Write it up
in [../decisions/](../decisions/README.md), set the requirement to `retired` and
point at that record. The point of the identifier is that somebody reading an
old progress entry can find out what happened to the thing it was measuring.

## Checking

    python docs/check_docs.py

This reads every requirement file, so a malformed record or a missing evidence
field is caught rather than discovered later. It also resolves every requirement
identifier mentioned anywhere in the repository, which is what stops a plan from
claiming a requirement that does not exist. See
[../check_docs.py](../check_docs.py).

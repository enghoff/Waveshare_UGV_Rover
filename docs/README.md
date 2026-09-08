# Documentation

Six kinds of document, one directory each, plus the component READMEs beside the
code they describe. If you are looking for how the rover works today, that is the
component README. Everything here is one of the other things: what has to be
true, what is planned, what was measured, why it is this way, what the hardware
is, and what to type.

| Directory | Answers | Lifecycle |
|---|---|---|
| [requirements/](requirements/README.md) | what has to be true of the rover | durable; changes state when something is measured |
| [plans/](plans/README.md) | how unfinished work is meant to get there | retired when the work lands |
| [progress/](progress/README.md) | what was actually measured, and when | append-only; entries are never revised |
| [decisions/](decisions/README.md) | why it is this way, and what was ruled out | frozen once closed |
| [reference/](reference/README.md) | what the hardware is and does, measured | updated when re-measured |
| [runbooks/](runbooks/README.md) | what to do, and how to tell it worked | kept current |
| [rover-architecture/](rover-architecture/README.md) | one drawing of the whole rover, for somebody who does not know it | redrawn when the shape changes |

## Where the rover stands

The spine of all this is the requirement: a numbered, testable statement about
the rover's behaviour that a plan can aim at and a measurement can move.
`settled` means shown to hold on the rover, not merely implemented.

<!-- begin: requirement summary (python docs/check_docs.py --write) -->

| Area | `settled` | `failing` | `open` | `proposed` | Total |
|---|---|---|---|---|---|
| [Safety and authority](requirements/safety.md) | 6 | -- | 5 | 4 | 15 |
| [Mapping and movement](requirements/navigation.md) | 10 | 1 | 2 | 1 | 14 |
| [Visual memory](requirements/world-state.md) | 12 | 1 | 3 | -- | 16 |
| [Control surface](requirements/control.md) | 10 | -- | -- | -- | 10 |
| [Host and deployment](requirements/platform.md) | 11 | -- | -- | -- | 11 |
| [Autonomy and its record](requirements/autonomy.md) | 10 | -- | 1 | -- | 11 |
| **All** | **59** | **2** | **11** | **5** | **77** |

Currently failing: [R-NAV-6](requirements/navigation.md#r-nav-6), [R-WS-10](requirements/world-state.md#r-ws-10).

<!-- end: requirement summary -->

The states are defined in [requirements/README.md](requirements/README.md).
`failing` is the one that should draw the eye: it means a requirement that was
believed to hold, and that a measurement has since disproved.

## Where to put a new document

Ask what question it answers, not what subject it is about.

- *How does this work now?* — the component's own README, beside the code.
- *What must be true?* — a requirement, in the area file it constrains.
- *What are we going to do?* — a plan.
- *What did we find out?* — a dated progress entry.
- *Why is it like this?* — a decision record.
- *What is this part, physically?* — reference.
- *What do I type?* — a runbook.

If it answers two of those, it is two documents. The most common mistake here
has been a plan that slowly absorbed the description of the thing it built,
leaving two accounts of the running system with only one of them maintained.

## The rules that outrank all of this

The working rules — reproduce a fault before fixing it, deploy to the host that
runs the change, talk to the other agent, never edit a tracked file on the rover
— are in [../AGENTS.md](../AGENTS.md), not here. Those are about how work is
done; these documents are about the rover.

And where any document disagrees with executable source or config, **the source
wins.** Correct the document. Never revive a setting because a document
remembers it.

## Checking

```bash
python check_docs.py
```

Run from this directory or the repository root. It validates every requirement
record, resolves every requirement identifier mentioned anywhere in the
repository, and checks that every relative link in the documentation and the
component READMEs points at something that exists. `--write` regenerates the
summary table above.

It is a standalone check rather than a commit hook, because documentation that
is briefly inconsistent mid-change is normal and being unable to commit it is
not.

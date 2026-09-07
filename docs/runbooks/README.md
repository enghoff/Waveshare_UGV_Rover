# Runbooks: what to do

Procedures. What to type, in what order, and how to tell whether it worked.
These are the documents somebody opens with the rover in front of them, so they
are kept current rather than dated — where a runbook and the running system
disagree, the runbook is a bug.

| Runbook | For |
|---|---|
| [deploy.md](deploy.md) | getting committed work onto the rover, restarting what needs it, and proving it runs |
| [hosts.md](hosts.md) | which machine is the rover, how to reach it, and what lives where on it |
| [scripting.md](scripting.md) | running short programs on the rover that compose its existing tools |
| [p0-gimbal-calibration.md](p0-gimbal-calibration.md) | printing and mounting the measured reference for the bounded P0 camera test |
| [rover-unresponsive.md](rover-unresponsive.md) | a rover that has dropped off the network |

## What a runbook owes

**The command, ready to run.** Not a description of the command. Somebody
reading a runbook is at a terminal and the value of the document is that they do
not have to reconstruct anything.

**How to tell it worked.** Every procedure ends with something to check, and
"the file was copied" is not it. On this rover the check is usually a call to the
affected function over the tool protocol, because that is the only thing that
distinguishes a running change from a copied one.

**What to do when it fails.** The failure path is the reason the document
exists. A procedure that only covers the happy case sends the reader to the
source at the worst moment.

## What a runbook does not own

**Why the procedure is what it is.** A runbook may say in a clause that the
deployer refuses a dirty tree; it should not argue for it. The reasoning is a
[decision](../decisions/README.md) or a [requirement](../requirements/README.md),
and mixing the two makes the procedure harder to follow at speed.

**Anything that would be a credential.** These documents are read and pasted
from. The mechanics of passing a password to a privileged step are in
[deploy.md](deploy.md); the password is not.

## One of these is not a closed case

[rover-unresponsive.md](rover-unresponsive.md) is a runbook by use and an
investigation by nature: it records a fault that was diagnosed on a host the
rover no longer has, and its recovery steps are what to try rather than a known
cure. It says so at the top. It is kept in this directory because it is what
somebody reaches for when the rover has gone quiet, which is the definition of a
runbook even when the underlying fault is still open.

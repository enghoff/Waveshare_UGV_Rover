# Working in this repository

Rules for changing the rover. Deploy/restart commands, directory ownership and manual
recovery are in [docs/runbooks/deploy.md](docs/runbooks/deploy.md); host and network facts are in
[docs/runbooks/hosts.md](docs/runbooks/hosts.md); a rover that has dropped off the network is
[docs/runbooks/rover-unresponsive.md](docs/runbooks/rover-unresponsive.md). A component's README
describes the component, and a rule that applies to one component lives in
that component's own AGENTS.md -- the web console's are in
[drive_web/AGENTS.md](drive_web/AGENTS.md).

## Report in plain English

Write for the person who owns the rover but does not carry its code in their head.
They want the conclusion and the decision, not the workings. Lead with what is the
case now: what works, what does not, and what it means for the rover. Anything they
cannot act on — the investigation, the options weighed, the names of files and
settings — is a supporting clause at most, and usually is not needed at all. Say
plainly when something is unproven, failed or was skipped. Two sentences beat two
paragraphs. Finish with the next step if there is one.

## Reproduce faults before fixing them

**A fix for a fault nobody reproduced is a guess.** Replay the reproduction first,
leave the running system alone until the model fails the way the rover did, then show
the fix succeeding there before deploying. Simulation does not replace hardware:
validate the model against a real recording, still observe the fix on the rover, and
where they disagree the hardware is right. Navigation has `ros_nav/nav_record.py`
with replay and controller simulations ([ros_nav/README.md](ros_nav/README.md));
`wifi_roam/selftest.sh` drives the network scripts against a fake board.

## Where things run

Rover services run on the Jetson Orin Nano (`orin`), which replaced the Banana Pi on
2026-08-31. The realtime voice model is Alibaba's hosted Qwen Omni; world-state
perception uses the Orin's GPU through TensorRT. What gets deployed is decided by
`deploy/manifest.json`, not by a list here — bench scripts under
`oak_camera/`, `lidar/` and `usb_cameras/` are not deployed, while files from
`face_tracking/`, `voice_chat/` and `driver_board/` are.

## A change is not done until it runs on the host that uses it

**Deploy to whichever host uses the changed files, restart what needs restarting, and
verify the running service there as part of the same work; say what was deployed and
what proved it.** A commit does not sync itself:
[`deploy/deploy.py`](deploy/README.md) copies the registered components that changed
and runs their own restart and verification checks, advancing state only once those
pass. Proof is taken on that machine — call the affected function over TCP 8769 and
read the answer, because "the file was copied" and "a local unit test passes" prove
nothing. A change touching no registered component needs no restart; a changed
manifest or source set wants `deploy.py --plan`.

## Another agent may be working here at the same time

Expect to be one of several. Work in your own files and your own scratchpad and it
will not come up. When it does — a tracked file changed underneath you, a deploy
refused because somebody else's edit is dirty, a service restarted mid-verification
— **talk to the other agent.** Silence is what turns a two-minute overlap into two
half-finished changes.

**Two things are never the answer.** Reverting, stashing or overwriting another
agent's work to get past it, and retrying in a loop until it clears. Neither is
made acceptable by being in a hurry.

**Say what you are holding and ask for what you need.** If your harness can address
another agent, do it directly; if it cannot, put it in your report so the person
can. Name the files, say whether your change overlaps theirs and how, and say when
you expect to be done. Answer the same question the same way when it is put to you:
commit or revert what you hold, then say the file is free. Handing back a file you
have already committed costs you nothing and unblocks somebody. If what you are
about to start needs files somebody else has dirty, ask before you start rather
than after you collide.

**Undeployed work is unfinished work, and "another agent had the tree" does not
change that.** A deploy sends a whole registered component at the current commit,
so it carries whatever anybody has committed — which makes it a shared act, not a
private one. Two agents must not deploy or restart the same service at the same
time, so agree who does it: usually whoever finishes last deploys both changes and
says so, and if you deploy somebody else's commit alongside yours, tell them what
went out and what proved it. When you are blocked only by a dirty tree and the
other agent cannot commit yet, deploy your own commit from a clean detached
worktree — [docs/runbooks/deploy.md](docs/runbooks/deploy.md) has it — and tell them you did.

Only when none of that is available do you stop: finish everything that does not
touch the contested ground, then say exactly what is committed, what is not on the
rover, and who or what is holding it. That is a handover to be picked up, and it is
worth saying as one — not a finished piece of work.

## Where a document goes

`docs/` has one directory per kind of document and
[`docs/README.md`](docs/README.md) is the index. Ask what question the document
answers, not what subject it covers: how a thing works now belongs in the
component's own README; what must be true is a
[requirement](docs/requirements/README.md); what we intend to do is a
[plan](docs/plans/README.md); what was measured is a dated
[progress entry](docs/progress/README.md); why it is this way is a
[decision](docs/decisions/README.md); what the hardware is belongs in
[reference](docs/reference/README.md); and what to type is a
[runbook](docs/runbooks/README.md).

Requirements are the spine, because they are what a plan can aim at and a
measurement can move. Each carries a permanent identifier — `R-WS-10`,
`R-SAFE-3` — and a state saying whether it is settled, open, proposed, failing or
retired. **Cite them by identifier** in plans, progress entries, decisions and
commit messages: that is what keeps a finding attached to the thing it was about
after both documents have been rewritten.

When work lands, move what it built into the component README and the
requirements, and leave the plan describing only what is still ahead. The failure
this avoids is the common one here: a plan that slowly absorbs a description of
the thing it built, leaving two accounts of the running system with only one of
them maintained.

`python docs/check_docs.py` validates every requirement record, resolves every
identifier mentioned anywhere in the repository, and checks that every document
path and link points at something real. It is worth running after moving or
renaming anything under `docs/`.

## The repository is the source of truth

Edit here and push; never edit a tracked file in place on the rover. The deployer
refuses dirty tracked files, because the recorded commit must describe the bytes that
were sent. Where prose and executable source or config disagree, **the source wins**:
correct the document, and never revive an old setting because a README remembers it.

`deploy/guards/` checks two of these rules mechanically: a shell command that
would edit the deploy tree in place or carry a credential is refused, and a
session that changed a deployed component is reminded before it finishes.
`git config core.hooksPath .githooks` adds the credential check to commits.

## Credentials

Deployment credentials are one-line gitignored files under `secrets/`; `jetson-orin.key`
is the `jetson` account's login password on the rover and therefore also its sudo
password. The old Banana Pi and Raspberry Pi keys are still there and all three are
different. The secrets the rover itself holds live in `~/.ugv/`, outside the deploy
tree: the DashScope key, the TLS material and deploy state. Never
put a credential in a commit, transcript, command line, or a path deployment can copy
back; [docs/runbooks/deploy.md](docs/runbooks/deploy.md) has the `sudo -S` mechanics.

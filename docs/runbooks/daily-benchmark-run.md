# The multi-day benchmark's daily run

What to do when the owner says "do the daily run". The rules it is judged by
are in [the decision](../decisions/the-multi-day-benchmark-is-judged-so.md);
the tools are in `experiments/multiday/`, staged on the rover in `~/multiday`
(copied, not deployed: `scp experiments/multiday/*.py experiments/multiday/*.json experiments/multiday/*.sh orin:multiday/`
after any change). Three days in all.

**The first round is over.** It was stopped after day 2 on 2026-10-20, once
its verdict was fixed ([the result](../progress/2026-10-20-the-multi-day-benchmark-ends-after-two-days.md)).
Until a second round is planned there is no daily run: say so if asked, and
point to the result.

## 1. Which day

The next day is one more than the `dayN-answers.json` files in
`experiments/multiday/days/`. After day 3 there is no run: say so and give the
verdict. Name the day `md-dayN-MMDD`.

Check nobody else is driving (`ListAgents`; `git log` for a fresh session's
work), and that the rover answers on its charger:
`ssh orin 'cd ~/m5 && python3 session_preflight.py'` reads READY (pose
trusted and settled, the GPU up, the shadow runner running -- it starts it).

## 2. Day 2 only: the restart, still plugged in

    ssh orin 'cd ~/multiday && python3 day.py restart md-day2-MMDD'

It asks every question, restarts the daemon and the world model's job, waits for
both, asks again and prints whether the answers are the same. Then check
navigation is settled (`nav_status`, `map_settled`). Answers that differ are
criterion 1 failing: say so, and run the day anyway.

## 3. The brief

    ssh orin 'cd ~/multiday && python3 day.py brief N'

Pass it to the owner in plain words, and ask them to: put the three things
where it says (moving nothing else); keep the living room, corridor and
bedroom clear while it drives; tell you if anything went somewhere else, and
where; then unplug the rover and say go. About 20 minutes of driving.

## 4. The run, once the owner says go

    ssh orin 'cd ~/m5 && python3 session_preflight.py'     # READY, unplugged
    ssh orin 'bash ~/multiday/launch.sh md-dayN-MMDD N'

Put a `now` hold on the console's "keep it on" card first (`power_hold`, as in
[AGENTS.md](../../AGENTS.md), with `hours` 2) and release it once the answers
are scored. It runs detached: the morning copy of the world model, the two arms in the
day's order, home, the lidar kept. Watch `~/.ugv/sessions/md-dayN-MMDD.out`
with a Monitor of short, retrying ssh calls -- ssh drops mid-drive and comes
back. STOP is the console's STOP button, or
`ssh orin 'touch ~/.ugv/sessions/md-dayN-MMDD.stop'`, which stops after the
current leg and keeps it from driving again. A loop that loses its position
stops itself where it stands. When it is home, ask the owner to plug it in.

## 5. Truth, then answers, then the score

Write `experiments/multiday/days/dayN-truth.json` from the owner's words first:
`{"as_asked": true}`, or `{"as_asked": false, "put": {"stool": "P4"}, "note":
"..."}` with a spot or `[x, y]`. Then:

    ssh orin 'cd ~/multiday && python3 answer.py md-dayN-MMDD'

It refuses until the live world model has read every picture of the day (a
couple of minutes after the rover parks). Copy
`~/.ugv/sessions/md-dayN-MMDD-answers.json` to
`experiments/multiday/days/dayN-answers.json`, and on day 2
`md-day2-MMDD-restart.json` to `days/day2-restart.json`. Then

    python experiments/multiday/score.py

## 6. Record it and report

A row in the [progress ledger](../progress/README.md) for each day, and after
day 3 a progress entry with the verdict by the decision's rules and the plan
updated. Commit and push. Tell the owner in two or three sentences how the day
went and what the running score says, and, after day 3, the verdict.

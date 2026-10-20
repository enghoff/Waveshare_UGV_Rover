# The multi-day usefulness benchmark

Autonomy against the fixed survey loop, one session of each a day for three
days, each judged on the questions from the morning's world model plus its own
pictures alone. The rules are
[the decision](../../docs/decisions/the-multi-day-benchmark-is-judged-so.md);
what to do each day is [the runbook](../../docs/runbooks/daily-benchmark-run.md).

| File | What it is |
|---|---|
| `benchmark.json` | the three things and their phrases, the five places with how to describe them, each day's placements, order and restart, the six controls and where they stand, the 1.0 m bars, the arms' nine minutes |
| `day.py` | on the rover: `brief N`, `restart NAME` (day 2), `run NAME N` -- the morning copy of the world model, the arms in the day's order through `experiments/m5_changes/session.py`, home, the day's lidar kept |
| `launch.sh` | `run`, detached, in navigation's environment |
| `answer.py` | on the rover: each arm's pictures filed alone onto the morning copy by the rover's own engine, and every question put as `find_thing` puts it; `--check` asks the live world model both ways |
| `score.py` | here: right, wrong and unresolved per arm per day against the owner's truth, moved-or-not from day 2, cost, the restart and stale-goal checks, and after day 3 the verdict |
| `days/` | each day's truth (first) and answers |
| `insight.py`, `insight.sh` | before a second round, one short session for the fixes of 2026-10-20 on the floor: two bean bags moved while the parked rover watches, a drive to living-3, an autonomy run, home; `score` judges it on criteria fixed in the script before it runs |

## The fixes' session

From day 2's arrangement (red bean bag P1, black P5, stool P3), in place
before the rover is switched on, the owner moves the red bean bag to P2 and
the black one to P4 while the rover watches from its charger, unplugs it and
leaves. Staged on the rover like the rest (`scp experiments/multiday/*.py
experiments/multiday/*.sh orin:multiday/`), then:

    ssh orin 'cd ~/m5 && python3 session_preflight.py'      # READY, still plugged in
    ssh orin 'bash ~/multiday/insight.sh insight-MMDD'      # parked, waiting for the go
    # the owner moves the two, unplugs, leaves and says done:
    ssh orin 'touch ~/.ugv/sessions/insight-MMDD.go'
    ssh orin 'cd ~/multiday && python3 insight.py score insight-MMDD'   # once it is home

`python3 insight.py brief` prints the owner's part; the staging page drawn
from the rover's map says the same with the places on it. Put a `now` hold on
the rover's power for the session and release it once it is scored.

Checked on 2026-10-18 with a mock day on the charger (four camera looks as arm
A, an empty arm B): the two windows filed together reproduce the live world
model exactly (638 records), and `--check` answers all nine questions as
`find_thing` does.

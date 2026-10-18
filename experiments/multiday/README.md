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

Checked on 2026-10-18 with a mock day on the charger (four camera looks as arm
A, an empty arm B): the two windows filed together reproduce the live world
model exactly (638 records), and `--check` answers all nine questions as
`find_thing` does.

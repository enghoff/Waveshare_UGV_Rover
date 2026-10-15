# Replaying a session's navigation from the graph it started with

A session's pose, run again in a second mapper and scored against a map known to
be right, so that a pose that went wrong on the rover can be studied — and a
setting tried — without the rover. Built for M5 session 3 (2026-10-15), after the
start-graph replay of 2026-10-12 had not been kept
([the pose among moved furniture](../../docs/progress/2026-10-12-the-pose-among-moved-furniture.md)).

| File | Runs on | What it does |
|---|---|---|
| `graph_replay.sh` | the rover | a second slam_toolbox on ROS domain 43, the session's starting graph loaded at the starting pose, the recording played into it; settings after `--` |
| `graph_replay_tool.py` | the rover | its helper: relays the recording's `/tf` without the live mapper's `map -> odom`, asks for the graph, and records the replay mapper's `map -> odom` |
| `pose_timeline.py` | here | every Nth scan scored against a grid at the believed pose, with the best pose within 0.5 m and 25 degrees; `--map-tf` scores a replay's corrections instead of the live ones |
| `compare_poses.py` | here | two timelines side by side |
| `turns_check.py` | here | each turn on the spot as odometry measured it against the lidar's own rotation (the range profile shifts round by the angle turned) |

The session script keeps what this needs: the starting graph
(`~/.ugv/sessions/NAME-map/current`), the starting grid (`NAME-grid.json`) and the
recording (`recordings/bags/session-NAME`, extracted on the rover with
[`../m5_changes/bag_extract.py`](../m5_changes/bag_extract.py) for the scoring here).

    # on the rover: baseline, then a setting
    graph_replay.sh BAG ~/.ugv/sessions/NAME-map/current X Y HEADING SECONDS OUT
    graph_replay.sh BAG ... OUT2 -- -p angle_variance_penalty:=0.1
    # here
    python pose_timeline.py REC.npz NAME-grid.json --every 20 > live.txt
    python pose_timeline.py REC.npz NAME-grid.json --every 20 --map-tf OUT > replay.txt
    python compare_poses.py live.txt replay.txt 08:03:00 08:11:30

Like `ros_nav/replay_bag.sh` it kills only what it started, and a deploy or a
navigation restart while it runs kills it. Loading a 190 MB graph took 38 s.

## Session 3

The baseline replay followed the live mapper within a few centimetres for the
whole session, and went wrong in the same two places: 0.5 m in the rearranged
living room at 08:07, and on the drive home from 08:10:45, where a 0.48 s gap
in the board reader dropped 30 degrees of a recovery spin
([progress](../../docs/progress/2026-10-15-m5-session-3.md)).

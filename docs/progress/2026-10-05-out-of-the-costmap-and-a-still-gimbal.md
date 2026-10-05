# A drive refused beside a wall backs off and goes; a near aim leaves the gimbal be

**Both faults from [M3 session 1](2026-10-05-m3-session-1.md) are fixed and on the
rover, and one of the two has been seen working there.** An ordinary aimed look no
longer swings the gimbal when its place is within 15 degrees of straight ahead. On
the rover, looks aimed 8 and 10 degrees off left the camera still and took 0.5 to
3 s; one aimed 25 degrees off still panned, to the 20 degree limit, and took 6.1 s.
A `drive_to` refused because the rover stands inside the costmap's margin now
backs off once and asks again. That is proven on the costmap recorded in that state
on 2026-09-01, on this workstation and in the rover's own suite. An attempt to put
the rover in that state by hand did not get close enough to a wall, so it is not
yet seen on hardware. [R-AUT-13](../requirements/autonomy.md#r-aut-13) stays open.
Deployed at fbf57b1 (rover_daemon and ros_nav), on branch
`autonomy-free-from-costmap`.

## The drive

Nav2 refuses every route with START_OCCUPIED while the rover's own cell is inside
the inscribed band, and a run can only `drive_to`, look and stop, so it had no
way out. `goto` in the navigation bridge now does what exploring has done since
2026-09-01: turn towards the nearest spot `goal_fit` says the body fits, at most
half a metre away, creep there, and plan again. It does this once; a second
refusal is handed back. An autonomous drive's own guard covers the back-off, so a
stop since the goal was issued, or a back-off spot outside the run's safe area,
moves nothing. Exploring keeps its own shuffling and its own limit. The back-off's
driving is reported as part of the drive, so it stays attributed to the run's goal.

The test drives the real `goto` and `back_off` against the recorded costmap with
Nav2 replaced: it fails on the previous code (the refusal comes back after one
try) and passes now. ros_nav's suite passes 566 here and 598 on the rover, where it
runs with the real ROS message types.

On the rover, the rover was driven to the session's stuck spot and crept towards
the wall until its drive stopped itself, 6 cm on, with 0.31 m clear. A `drive_to`
back to the charger from there planned without a refusal, so the rover had not
entered the band. A drive's collision check stops it before it gets that close,
so the state is reached only the way the run reached it, by arriving and turning.
The next session that reaches it will show the back-off.

## The gimbal

Each gimbal move is a 30 degree overshoot and return to take up the pan servo's
slack, and an aimed look made two of them, out and back to rest. The fix of
2026-10-03 skipped them only for an aim that rounded to zero; the session's looks
asked for 2 to 15 degrees. The OAK sees 65 degrees of the room, so a place 15
degrees round has 17 degrees to spare inside the depth picture, and rest is the
pan the calibration measured. Hypothesis checks still aim exactly; R-AUT-12 was
accepted that way. The daemon's suite passes 1105; the new checks fail on the
previous code.

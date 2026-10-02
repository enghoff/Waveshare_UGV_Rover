# M0a's runs reach 27 attempts: three answers, all right, and no wrong ones

**Over eight supervised runs, 27 hypothesis inspections ended inside their limits. Three
said something stands at the place, and all three were right. None was wrong. Eighteen
answered "can't tell" and six were refused before the rover moved.** That meets M0a's
sample:

- at least 20 attempts over at least three runs (six runs made attempts);
- more than ten distinct places;
- three false or absent places attempted: two dining chairs the owner removed, and a wrong
  crossing in front of the cabinet;
- many insufficient-view cases.

Since [2026-10-02's decision](../decisions/p0-measures-the-hardware.md) M0a has no
usefulness floor: the rates are what is reported. The records are in
`captures/m0a-2026-10-02/RUNS.txt`, with each run's executive log. This continues
[the first runs](2026-10-02-first-m0a-runs.md).

| | Count |
|---|---:|
| attempts | 27 |
| supported, and right (toolbox, tissue box, a red bottle on a side table) | 3 |
| contradicted | 0 |
| unresolved | 18 |
| refused at dispatch (fence margin, nowhere to stand) | 6 |
| wrong answers | 0 |
| attempts past their limits, identity-dependent follow-ons | 0 |

## Two faults found on the way, both fixed and deployed

- **The tilt for a place neither tilt fits** (autonomy `6ce6b70`). The generator fell back
  to the first tilt in its list, 20 degrees up, so a chair seat 0.19 m above the camera sat
  14 degrees below the middle of the depth picture, with the area being checked off the
  bottom edge. It now takes the tilt nearer the place.
- **Spent places crowding out open ones** (autonomy `d67ba49`). The generator took the
  twelve claims nearest the rover before the veto. Around the charger all twelve were
  answered or out of attempts, so two runs found "nothing to do" while an untried place
  stood 1.6 m away. The limit now applies to open places.

A third change retakes a check look once when the depth service was down (rover_daemon
`b73bcb3`). In its one chance it did not save the look: the camera had not come back in
time.

## What the hardware allows, measured here

- **Most "can't tell" answers are geometry.** From 1.0 to 1.6 m away, the area a check
  reads is 25 to 30 degrees tall, against the depth camera's 40. With only two calibrated
  tilts, level and 20 degrees up, only places within about 6 degrees of one of the two
  can be seen whole. Turning and panning put places in the middle of the picture side to
  side every time; the misses were all at the top or bottom edge.
- **Small objects are never found.** Two bottles, a water bottle and a small figure, put
  out by the owner, were never proposed as regions from 1.5 to 3 m. Things the size of a
  tissue box or larger are.
- **The depth camera drops off USB** while parked and while driving. The supply rail
  stayed flat while it did ([the soak](2026-10-02-oak-usb-drops.md)), which points at
  the cable, port or camera. That fault cost five checks.
- **The battery gives about 20 to 25 minutes of this driving** before it sags under load
  below the runs' 11.2 V reserve.

## Requirements

None moved; whether these runs close M0a and P0 is the owner's call.
[R-AUT-12](../requirements/autonomy.md#r-aut-12) has its hardware sample, with no wrong
answer and every limit held.

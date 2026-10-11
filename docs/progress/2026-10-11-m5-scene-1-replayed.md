# M5 scene 1 replayed: the bedroom gate was too loose, the armchairs' old places were found but small, and walls seen edge-on

**Replaying scene 1 with the hour the rover sat parked rebuilt, three things in
the [scene's entry](2026-10-11-m5-scene-1.md) turned out wrong, and three fixes
followed, all deployed at 16:30 and running.** The map gate deployed at 14:48
would not have stopped the bedroom's false reports; it is now strict enough to
(71d8414). The armchairs' old places were not missed by a blind spot: the watch
logged them gone, but under furniture size, and the count left them out. A
change that grows to furniture size is now logged again (71d8414), and walls
seen edge-on from across a room no longer read as seen through (28774db). On the
replay, scene 1 now gives seven true changes at furniture size and no false
ones, and every control stays clean.

## Corrections to the scene's entry

- **The armchairs moved after 13:17, not about 13:10.** The parked rover's scans
  at 13:07 and 13:16 are the same, both armchairs at the places the morning's map
  has them; by 14:13 both are gone. No recording holds the move: the rover sat
  unrecorded from 13:17 to 14:12. The replay rebuilds that hour from the 13:12 to
  13:17 scans until a move at 13:45, then the first still 16 s of scene 1.
- **Where the armchairs first stood was reported.** The live log has both at
  14:25:08, gone at (-16.97, -16.42) and (-18.59, -16.61), but 0.35 and 0.40 m
  across: the lidar sees only an armchair's front, and on the replay the larger
  of the two grew to 0.50 m and no further. Under 0.6 m is not furniture, so the
  count did not take them. The rule tried for the "blind spot" was answering the
  wrong question here; a move made while the rover watches is still blurred in
  the synthetic test (open, below).
- **The map gate at 0.6 did not stop the bedroom's reports.** They were logged at
  14:18:38 from the half minute before the mapper's correction at 14:18:45, when
  the scan's hits beyond 1.5 m fitted the map at 0.63 to 0.67, not the 0.41 that
  entry gives. The replay that showed the gate working had
  refused those scans by the watch's other check, against what it had seen
  itself, which depends on its history; with the parked hour rebuilt unchanged,
  the replay logs the bedroom as the rover did.

## What changed

| Fix | Why | On the replay |
|---|---|---|
| Map gate at 0.7 (71d8414) | the bad bedroom pose fitted at 0.63-0.67; good poses on both days at a median of 0.94-1.0, 1.5% of scans under 0.7 | the four bedroom reports gone; every true change kept |
| A change logged again when it grows to furniture size, the count taking the larger (71d8414) | on 2026-10-10 the armchair's old place was logged at 0.55 m and grew to 0.95 m unlogged | logged at 0.95 m at 14:57:43 |
| Only the scan within 4.5 m counted (28774db) | a corridor wall reported gone at 14:31 from 4.5 m away, beams running along it edge-on | wall gone; an armchair's old place and the other's first new place gained at furniture size; at 3.5 m control 1 reported a wall gone, at 5 m the corridor wall came back |

Scene 1 on the replay, furniture-sized, with the hour rebuilt and the move at
13:45: an armchair new at its first place (14:14), the other new at its first
place (14:15), one of their old places gone (14:16), both second places new
and both first places gone (14:25). Not found at that size: the other old
place, and the bucket. Nothing false. With the hour rebuilt
unchanged instead (no move), the same controls are clean and one change at the
corner by the balcony, new at 14:32, is logged at 0.80 m; the rover logged one
there live at that minute, 0.3 m. What it is, is not known: the corner has
come and gone in the log seven times since the evening of 2026-10-10, mostly
small, and a photograph from the charger at about 15:50 shows bare wall.

Deployed at 16:30: the watch's 48 checks pass on the rover, it is counting again
with the gate at 0.7 and the range at 4.5 m, and parked on the charger it refuses
nothing and reports nothing. Navigation restored where the rover stands, the
scan 99% on the map within 3 cm.

## Still open

- **A move made while the rover watches.** One visit spans the whole stay, so it
  sees both arrangements and claims neither. Rules that fix it in the synthetic
  test (a visit closing after 10-20 minutes, a change seen from one spot held
  back) add reports of the wall by the balcony gone on the replay. The sessions stage changes while the rover waits
  in another room, so it is not in their way; it is in the way of a rover that
  lives on the charger in the living room.
- **Armchairs seen from the front come out small.** Criterion 2's count at
  furniture size will miss some real moves: here one of the four places an
  armchair left.
- **The corner by the balcony** wants a look from close to, or the owner's word.

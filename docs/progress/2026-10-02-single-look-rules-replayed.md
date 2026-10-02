# Gating single ranged looks by the map does not help, and refusing them costs real things

**No rule for single ranged looks that the map can judge improved placement without losing
real things, so nothing was changed.** The idea was that a thing placed from one look's
depth reading should not stand on its own when that reading puts it behind a wall. Today's
mirror front and a far corridor patch were placed that way. On the taped drives, the
lidar map cannot tell good single readings from bad ones. Refusing single looks outright
fixed the worst placement on record, but it also discarded the bed, the wardrobe and the
hallway painting, all three correctly placed. The fault that remains is a reading taken from
whatever else stood in the box, and that needs a different remedy.
[R-WS-10](../requirements/world-state.md#r-ws-10) stays `failing`.

Scripts, logs and the copy of today's store are in
`captures/2026-10-02-single-look-rules/`. The taped drives are those of
[2026-10-01](2026-10-01-the-room-as-it-stands.md) and
[2026-10-02](2026-10-02-drive-carry-and-stops.md), replayed with the 10-02 lidar map.
Both days were mapped in one frame, to 0.4 degrees and 4 cm. The deployed build replays
the redo drive to exactly the scored positions (0.10, 0.10, 0.25, 0.88 and 0.12 m), and
today's run to 46 things against the rover's 45.

## The map cannot separate good single readings from bad ones

Each ranged look at a taped target was compared with the tape. It was also compared with
how far along its own bearing the lidar map reaches:

| Ranged looks at taped targets | n | Past the first lidar obstacle, median | Past the edge of the mapped room, median |
|---|---|---|---|
| Right (within 0.5 m of the tape) | 52 | +0.29 m (90%: +0.84) | +0.15 m (90%: +0.73) |
| Wrong (0.5 m or more off) | 20 | +1.03 m (90%: +2.02) | -0.02 m (90%: +1.21) |

The first obstacle is often furniture in front of the thing. The painting above the cabinet
reads 0.46 to 0.91 m past the cabinet and is right. Against the edge of the room, most wrong
readings land inside it, because they read the chair in front rather than the wall behind.

## Four rules replayed

Things placed, and the error of the taped targets:

| Rule | 10-01 | 10-02 first | 10-02 redo | Today |
|---|---|---|---|---|
| Deployed | 73 | 19 | 27; landscape painting 0.88 m | 46 |
| No placement from one look | 57; targets as before | 12; the cabinet lost | 17; landscape painting 0.37 m, painting above cabinet 0.17 | 40 |
| One look may not land past the mapped room | 69 | 16 | 24; landscape painting 0.88 m | 46 |
| One look waits 30 s for a crossing | 72 | 19 | 26; 0.88 m | 45 |
| One look waits 120 s for a crossing | 67 | 19 | 25; 0.88 m | 46 |

Only refusing single looks moved a target. The landscape painting's one look read the chair
in front of it, at 1.22 m against 2.54. Once that look cannot place the painting first, three
other viewpoints cross on it instead.

## What refusing single looks would throw away

The deployed build's single-look things on the redo drive and today were each looked at
(`singles-sheet.jpg`). There are 18 of them, and 13 are real objects:

- the bed, the wardrobe and the hallway painting, each within 14 cm of the wall it stands
  against;
- two dining chairs, the basket on the table, the table top, a light switch, a small fitting
  on the wall, a picture above the cabinet, a small box under the glass table and something
  on the table beside the owner;
- the landscape painting at its wrong range.

The other five are the owner bending over, a small patch by the owner's feet, two door
edges and a cable on the floor. Losing 13 real things to remove 5 pieces of junk is not the
trade that was asked for.

## What is left

The worst single-look error was not a reading behind a wall. It was the box's nearest
surface, which `oak_depth` is built to return, belonging to something standing in front of
the region. Taking the reading from the region's own outline rather than its box would answer
that. It can be replayed: depth maps are stored beside the frames for every ranged look of
the redo drive and of today, and the outlines can be regenerated from the stored pictures.

[R-WS-2](../requirements/world-state.md#r-ws-2) says that a single observation never places a
thing, and it is `settled`. Since the depth camera was wired in, `resolve._place_from_range`
has placed things from one ranged look, and 7 to 19 of the things on these drives were made
that way. The requirement and the code disagree. Which one should change is the owner's call,
and it is left open here.

## Requirements

None moved. [R-WS-10](../requirements/world-state.md#r-ws-10) stays `failing`. The
disagreement with [R-WS-2](../requirements/world-state.md#r-ws-2) is recorded above.

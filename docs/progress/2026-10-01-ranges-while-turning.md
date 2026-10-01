# The tissue box's wrong range came from a turn, and ranges taken while turning are now dropped

**The range that put the tissue box 1.77 m out was read off a depth frame the rover
had turned away from.** On [the morning's drive](2026-10-01-the-room-as-it-stands.md)
a tissue box 0.95 m away read 2.61 m. Its look was taken while the rover turned
54.5 degrees across the shutter, at about 136 degrees a second. The depth frame is
fetched after the encoders, a third of a second or more after the picture, so the
box was drawn some 17 degrees from where the tissue box stood in the depth frame.
It landed on the furniture behind. That single ranged look then placed the box, with
0.17 m claimed.

**Every wrong range at the targets was the same.** Of the morning's looks at the six
targets that the depth camera should have covered, six ranges missed the tape by
more than 0.5 m. All six came from looks taken while the rover was turning. Among
the seven taken still, none was wrong; three had no range at all.

So a range is now dropped, recorded as "the rover was turning", when the heading
moved more than a degree between the shutter and the range read, or when the turn
rate across the shutter times the two frames' separation does. Deployed as
`b1843c9` (world_state 904, rover_daemon 985 on the Orin). On the rover, a look
taken mid-turn dropped its four ranges and said why, and still looks kept theirs.

## What this changes for the next drive

Ranges now come only from looks the rover took standing still, or nearly so. The
morning's range tolerance counted every look at a target, and most of those were
taken while turning. Scored on its still looks alone, the morning had 4 good of 7,
the rest missing rather than wrong. That is too few to judge, and is not a pass.
The next drive's manifest declares, before it starts, that the range tolerance is
measured over still looks. The drive itself should give the rover still looks:
stop, then look.

## Requirements

None moved. The range half of M0's shared prerequisite 3 is untested under the new
rule until the next drive.

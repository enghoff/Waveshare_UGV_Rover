# The recovered rover completes the recording; range abstention leaves fragmentation unchanged

Correction: the original six-view landscape label combined two distinct paintings.
The [later subject diagnosis](2026-10-05-subject-fragmentation.md) separates them
using a frame showing both. The original labels and scores below remain unchanged
as the record of that mistake; corrected scores still retain 13 links and gain
none under range abstention. The later entry also strengthens and repeats the
map-invariance check against the resolver's map-error catch.

The rover completed the prepared short route and returned for charging. Withholding
four flagged distances preserved all 13 existing connections among 26 reviewed
views, but added none. Both front dining chairs stayed consistently identified;
paintings and armchairs still split across records or remained unassigned. This
recording does not justify blanket range abstention. The earlier isolated painting
correction remains a real positive result, and the older cabinet regression remains
a reason against blanket deployment. R-WS-13 remains open; R-WS-17 and R-WS-18 remain
proposed. R-WS-16 remains settled.

## Motion and recovery actually observed

After the owner's power cycle, raw board feedback changed again: six full messages
had six distinct IMU tuples, rotation integral advanced from 336 to 1390, and gyro Z
reached 1292 during an operator turn. Operator 15-degree and 90-degree turns completed
normally. These movements happened during the read-only recovery check; it was not
a purely stationary capture. This supersedes the unresolved recovery in the
[frozen-feedback entry](2026-10-05-frozen-imu.md), without identifying the internal
sensor fault or proving that it cannot recur. No motor/control change or frozen
feedback guard was deployed.

The owner then authorized control and confirmed the charger was disconnected.
All three outbound moves, two commanded turns and three return moves arrived.
Navigation tick inputs/outputs and the raw board stream were recorded throughout.
Reported battery fell from 45% / 11.42 V to 10% / 10.95 V in about two minutes;
the trial returned immediately when this was observed. The final map pose was
about 12 cm from the saved start, with raw heading about 32 degrees different.
The navigation recording's odometry displacement was 0.42 m; these are different
estimates, not tape-measured docking proof. The final camera showed the starting
armchair/window scene and STOP reported zero speed, turn and motor PWM. All files
were copied before telling the owner the rover could stay charging or be switched
off. No further movement was requested.

## Exact control and conditional diagnostic

The complete `depth-drive-20261005-2` recording contains before/after databases,
actual matching calls and saved grids. Its unchanged replay reproduces all 21
resolver passes, 81 identity checkpoint checks and 21,552 ordered reach queries.
Archived grids reproduce every logged answer. This includes entity assignments,
placement values and plain/masked exemplar bytes; generated wall-clock update
timestamps are not identity checkpoint truth.

There are 142 new observations in 18 frames, all with stored bearings, and 48
supplied ranges. Ten raw depth frames reproduce all 48 ranges and all 86 available
sampler answers exactly. Six projection frames lack raw depth, affecting 40
unranged observations; two other frames have no projection event, affecting 16
unranged observations. Complete sampler proof is therefore false. No omitted
observation supplied a range, and no missing measurement was reconstructed.

The fixed minority-band rule flags 68944, 68970, 68992 and 68998. Pass 5368 used two
archived grids differing in one cell across the occupied threshold. Before computing
the candidate, commit `3bb4b8c` froze a separate conditional diagnostic: every
candidate reach query must give exactly the same answer under every archived grid
used in that pass, otherwise abort. All 21,525 candidate queries passed this test.
It establishes invariance across those recorded contexts, not an unrecorded map
or inferred wall-clock refresh schedule. The default strict replay still refuses
multiple grids within a pass; the earlier protocol was not silently relaxed.

Only the four ranges and their uncertainty/absence fields were changed as inputs.
All observations, outlines, vectors and other measurements were retained. Two
previously unassigned views acquired owners: 68970 to object:349 and 68998 to
object:245. Older observation 68587 lost object:464. Individual correspondence of
the two new attachments and correctness of the old release were not established;
these are changes, not three claimed repairs. Entity IDs stayed identical; placement
values changed for object:245, object:313, object:349 and object:464. Resolver notes,
exemplars and generated timestamps may change as outputs.

## Reviewed coverage and decision

Photographs and outlined pixels supplied 26 frozen analyst development labels for
eight physical subjects before candidate results were read. Execution summaries
had already been seen, so this is not independent acceptance evidence. There are
36 labelled same-subject pairs and 289 different-subject pairs. Control and candidate
both connect 13 same-subject pairs and zero different-subject pairs: no lost links,
no gained links. Of 18 same-subject pairs satisfying the reported 0.4 m baseline
and 12-degree bearing-parallax conditions, nine connect in both arms. Reported
poses and analyst correspondence do not supply independent geometric truth.

The left and right front dining chairs each retain all four views in one record.
The snowy-forest painting connects two views but its third has another record.
The brown landscape painting has four different owners across six views, with
two unassigned. Green and purple armchairs each have two different owners; the
green chair has changing person occlusion and the initial purple view is cropped.
The painting behind the chairs has one assigned and two unassigned views. These
failures are unchanged by the four range abstentions.

The desired unoccluded side view of that background painting was not detected as
a separate region. No pure-table pair or taped glass distance was obtained. A
post-score review of flagged regions includes doorway/window structure and an
angled side-chair view; a minority depth band alone does not establish a wrong
distance. Do not count unresolved subjects as errors removed or claim the missing
controls passed. The older 98% useful-link retention criterion remains in force;
this small subset's 100% retention cannot override the older cabinet failure.

The firm decision is to keep the production resolver unchanged and retain range
abstention as a bounded candidate. The next diagnosis is why these photographed
same-object views fragment despite useful isolated pixels: distinguish appearance,
geometry and existing contaminated records before proposing another gate. This
can start offline with the recording. Independent clear-painting and measured
transparent-object controls remain necessary before a general policy decision.

The [measurement](2026-10-05-recovered-depth-drive.json) records hashes, scores,
coverage and limitations. Raw evidence is archived under
`captures/2026-10-05-depth-drive-2/`; it is not tracked in Git. The world/daemon
instrumentation remains deployed at `1fa5071`; this work changes only offline
experiments and documents, so it requires no rover restart.

```powershell
python experiments/entity_association/replay_call_recording.py --directory captures/2026-10-05-depth-drive-2/depth-drive-20261005-2 --output .cache/new-drive2-control
python experiments/entity_association/diagnose_recorded_depth.py --recording captures/2026-10-05-depth-drive-2/depth-drive-20261005-2 --frames captures/2026-10-05-depth-drive-2/depth-drive-20261005-2/frames --output .cache/new-drive2-depth.json --allow-missing-depth
python experiments/entity_association/replay_call_recording.py --directory captures/2026-10-05-depth-drive-2/depth-drive-20261005-2 --output .cache/new-drive2-conditional --withhold 68944,68970,68992,68998 --map-invariant
```

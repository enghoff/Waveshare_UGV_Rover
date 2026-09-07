# P0 calibration handover

Status at handover: the gimbal placement measurement has passed within a bounded
operating envelope. The fixed OAK-to-gimbal mount measurement has not passed, so
no OAK mount transform has been adopted and M0 remains open. The latest held-out
set is useful evidence, but its 0.863-degree yaw range exceeds the 0.75-degree
repeatability limit declared before capture.

No further physical action is needed from the owner yet. Keep the printed target
flat and mounted if convenient. The next investigation uses the saved images to
identify whether the yaw spread follows one gimbal frame, one OAK frame, or the
pairing calculation before deciding whether another capture is justified.

## What has passed

The Canon MG2577S printed the A4 ChArUco target at the correct scale. The owner
confirmed all five checks: the 100 mm scale bar, 240 mm board width, 168 mm board
height, 120 mm horizontal five-square span and 120 mm vertical five-square span.
The target is a 10 x 7 square board with 24 mm squares, 17 mm markers and
DICT_4X4_50.

The independent gimbal campaign passed at 1280 x 960. Within commanded pan -20 to
+20 degrees at tilt zero, keep gain unchanged and finish each placement from the
ascending direction. The held-out gain error was -0.535%, absolute residual p95
was 0.457 degrees, and stationary duplicate differences were 0.117 degrees median
and 0.423 degrees p95. Opposite approaches still differ by 1.19-2.23 degrees, so a
capture reached from another direction is outside the demonstrated state. Normal
tracking remains at 640 x 480; the camera's 2592 x 1944 maximum is only a
contingency because 1280 x 960 already passed the measurement gate.

The OAK's stored colour-camera distortion is exposed through its deployed health
diagnostic. Across the emitted 640 x 360 frame, the runtime pinhole model differs
from the stored lens model by 0.467 degrees at p95 and 0.548 degrees maximum. This
is inside the predeclared 0.75-degree bench allowance.

After the target was lowered by about 120 mm, the development mount set passed all
of its gates. Its frozen candidate transform, from the gimbal camera to the fixed
OAK optical centre, is:

| Component | Development median | Development range |
| --- | ---: | ---: |
| Yaw | +0.115525 degrees | 0.501943 degrees |
| Pitch | +7.488340 degrees | 0.301035 degrees |
| Roll | -0.976055 degrees | 0.078911 degrees |
| Forward | +0.089174 m | 0.000756 m |
| Left | -0.014052 m | 0.004111 m |
| Up | -0.103349 m | 0.002681 m |

This is a candidate only. It differs materially from the earlier ruler estimate,
especially in the forward component, which is why the independent distance check
must pass before it is used by the rover.

## Latest held-out result

The first held-out attempt at 0.880 m was invalid because the normal 640 x 360 OAK
stream could not resolve any markers. This established a physical resolution limit;
the corner threshold was not lowered. At the replacement position, 1280 x 720 found
42 corners, still below the unchanged 45-corner gate, while the calibration-only
1920 x 1080 path found all 54. The resident OAK service remained configured for its
normal 640 x 360 colour/depth stream.

The formal `oak-mount-held-out-02` set was captured at a median gimbal-to-target
distance of 0.685685 m, 0.129695 m beyond the 0.555990 m development set. All five
OAK frames found 54 corners, and the five gimbal frames found 50-54. Coverage,
reprojection, offset repeatability, runtime lens-model error, distance separation
and comparison with the frozen candidate all passed.

| Component | Held-out median | Held-out range | Change from development |
| --- | ---: | ---: | ---: |
| Yaw | +0.825402 degrees | **0.863364 degrees** | +0.709877 degrees |
| Pitch | +7.939693 degrees | 0.474097 degrees | +0.451353 degrees |
| Roll | -1.046309 degrees | 0.128249 degrees | -0.070254 degrees |
| Forward | +0.102819 m | 0.001252 m | +0.013645 m |
| Left | -0.010252 m | 0.008590 m | +0.003800 m |
| Up | -0.111009 m | 0.004679 m | -0.007660 m |

The development-to-held-out changes are within the declared limits of 0.75 degrees
per angle and 15 mm per offset component. However, the held-out yaw range is
0.863364 degrees, so angular repeatability fails. The overall result is
**inconclusive**. Do not average the two transforms, adopt the development
candidate, or relax the gate based on this result.

## Preserved evidence

The raw captures and generated analyses are intentionally ignored by Git and are
currently on this workstation under:

- `captures/p0-gimbal-2026-09-07/held-out-01/analysis.json`
- `captures/p0-gimbal-2026-09-07/oak-mount-dev/`
- `captures/p0-gimbal-2026-09-07/oak-mount-held-out/`: invalid 0.880 m attempt
- `captures/p0-gimbal-2026-09-07/oak-mount-held-out-02/`

Re-run the latest fit without touching hardware or recapturing images with:

```powershell
python usb_cameras/calibrate_oak_mount.py `
  captures/p0-gimbal-2026-09-07/oak-mount-held-out-02 `
  --gimbal-analysis captures/p0-gimbal-2026-09-07/held-out-01/analysis.json `
  --oak-size 1920x1080 `
  --compare captures/p0-gimbal-2026-09-07/oak-mount-dev/mount-analysis.json `
  --fit-only
```

The last capture cleanup returned the gimbal to pan zero and tilt zero with tracking
off, restored the resident OAK service, and found it healthy on the normal stream.
This is the last observed state, not a continuous guarantee after handover.

## Next work

1. Factor the 25 held-out pair estimates by their five gimbal and five OAK source
   frames. Identify which input accounts for the 0.863-degree yaw span and inspect
   that frame for detection or pose-fit instability. This uses the existing evidence
   and does not alter an acceptance rule.
2. If the source is understood, declare the corrective capture and its acceptance
   rule before collecting new data. Preserve `held-out-02` as an inconclusive set;
   do not reuse it as a fresh acceptance trial.
3. Only after a held-out mount result passes, update the runtime OAK mount constant,
   test it, deploy the affected registered component to the Orin and prove the live
   value through TCP 8769.
4. Check foreground and background depth-to-object alignment at the intended working
   distances. Then enforce the demonstrated ascending gimbal approach in the
   semantic capture path and reject captures outside the supported state.
5. Complete the remaining M0 work: the R-WS-16 confirmed-pose capture gate,
   background/floor eligibility, movement-eligible identity and reviewed
   associations. Semantic movement remains disabled while these gates are open;
   read-only M1 recording and M2 shadow decisions may continue.

## Repository state at handover

`origin/main` was at P0 commit `1281456` when this handover was written. The local
branch also contains the P0 bench
commit `70ee031`, which adds calibration-only 1920 x 1080 OAK capture and leaves the
resident stream unchanged. It has no deployed component and needs no rover restart.
The branch also contains commits from the concurrent navigation work; coordinate
before pushing or deploying so those changes are not published or restarted as an
unreviewed side effect.

The deployed P0 runtime changes are the 1280 x 960 gimbal diagnostic snapshot and
the OAK distortion health diagnostic. The former passed the full daemon suite on
the Orin (841 passed, no failures or skips) and returned a live 1280 x 960 image;
the latter returned the live stored distortion through the OAK health response.
No gimbal gain, gimbal backlash correction, OAK mount transform or semantic-movement
setting has been deployed.

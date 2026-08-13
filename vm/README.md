# ugv — D500 lidar + OAK-D-Lite on the rover, running in this VM

Everything here assumes the two sensors reach the guest over VMware USB passthrough
(`usb.autoConnect.device*` in `ugv-rover.vmx`, `autoclean:0` — see the note there
before changing it). There is no Raspberry Pi and no motor control: the rover is
moved by hand.

## Run a mapping session

    bash ~/ugv/bin/start_slam.sh rviz        # lidar + camera + SLAM + RViz
    bash ~/ugv/checks/slam.sh                # confirm it is actually mapping
    bash ~/ugv/bin/record.sh 300 hallway     # optional: record 5 minutes
    bash ~/ugv/bin/stop.sh                   # release the sensors

**Keep the rover still for the first 15 seconds of a run.** That window is where
the gyro's bias is measured, and moving through it poisons the correction for the
whole session. The node checks and complains rather than trusting it blindly.

Add `crop` whenever a person is pushing the rover — at a fixed bearing half a
metre behind, they are a large consistent return, and a scan matcher will treat
them as a landmark that never moves. Add `nocam` for a long recording, where the
cores are better spent on the writer than on a camera nobody is watching; note
that this also removes the gyro, since the IMU is inside the camera, so odometry
falls back to plain rf2o. `rf2o` and `static` select the other odometry sources.

Everything launches detached and returns immediately. That is deliberate: an
earlier launcher held the SSH channel open, a retry stacked a second copy on the
first, and four copies of the camera driver saturated the guest until sshd could
no longer answer.

## Layout

    bin/       operate the rig — start, stop, record, look at the screen
    checks/    measure and verify; run when hardware moves or something looks off
    setup/     one-shot provisioning, for rebuilding this VM from scratch
    launch/    ROS 2 launch files
    config/    node parameters and RViz layouts
    nodes/     the one node we had to write ourselves

`launch/` and `config/` are the parts worth reading. The measured geometry lives
in `launch/bringup.launch.py` with the provenance of every number, and
`launch/slam.launch.py` inherits it.

## What is established

Both sensors run at their native rates: depth 15.1 Hz, rectified right 15.0 Hz,
IMU ~206 Hz, scan 10.000 Hz with 504 ranges and no CRC failures. The two agree
about where objects are to **−14.9 mm median, 17.7 mm RMS over 0.3–1.0 m**,
measured by `checks/overlay_scan_depth.py`. Lidar yaw and handedness are
confirmed against the camera, so the map is not mirrored.

Odometry is an EKF (`config/ekf.yaml`) taking translation from rf2o and rotation
from the OAK's gyro, because rf2o is adequate at the first and poor at the second:
measured stationary over four minutes, side by side in one run,

    source           drift          yaw
    rf2o           50.3 mm    +20.73 deg   (+5.183 deg/min)
    EKF            31.0 mm    − 0.08 deg   (−0.020 deg/min)

The gyro cannot simply be calibrated once. Its offset came out −0.044, −0.150 and
−0.154 °/s on three consecutive startups and keeps moving as the device warms, and
holding a single value gave a filter that drifted *worse* than the rf2o it
replaced — 3.5 °/min, and smoothly rather than noisily, which is harder to spot.
`nodes/fusion_prep.py` therefore re-learns the bias continuously, but only while
the rover can be shown to be standing still.

Establishing that is subtler than it looks, and getting it wrong produced the
worst bug in this thing so far: **phantom spin**, where the map kept rotating in
RViz long after the rover was set down, sometimes apparently forever. The first
gate asked whether the gyro's *spread* was small and whether rf2o's *linear* speed
was low. A steady turn by hand passes both — a constant rate has almost no spread,
and turning on the spot barely translates. So the bias absorbed the turn, and the
moment the rover stopped, the corrected gyro read minus that rate.

What catches it is the distance between the current reading and the bias already
believed. The two live on different scales: genuine offset drift is around
0.05 °/s over minutes, hand rotation is tens of °/s. A hard clamp backs that up —
the tracked bias may never move more than 0.6 °/s from the startup measurement, so
even a leak is bounded, and hitting the clamp logs a warning because it means the
gate let something through.

Still open: the stationary numbers above are solid, but the moving case has only
been confirmed by eye. `checks/spin_watch.py` puts a number on it — turn the rover
by hand, set it down, and it reports how long the estimate took to settle and how
much heading leaked away afterwards.

## Gotchas that cost real time

The SVGA output comes up at 800×600 every boot. At that size RViz cannot dock the
image panel, disables the display to cope, and stores that in Qt settings
(`~/.config/ros.org/persistent_settings`) — so the camera view then stays missing
at *any* resolution, looking exactly like a camera fault. `bin/start_slam.sh`
clears it; `setup/persist_display.sh` stops it recurring. Do not fix it by ticking
the display back on: creating the image render panel against this GL driver after
startup takes RViz down with no crash logged anywhere.

`ros2 topic hz` badly under-reports large image topics — it is a Python subscriber
deserialising 0.6 MB frames, and reported 3.8 Hz for a stream that `ros2 bag
record` measured at 15.1. Trust the recorder.

The lidar is CDC-ACM (`/dev/ttyACM0`), not the `/dev/ttyUSB0` Waveshare documents,
and the Ubuntu cloud image ships no USB serial drivers at all until
`linux-modules-extra` is installed. `setup/install_lidar_tty.sh` handles both and
gives it the stable name `/dev/rover-lidar`.

Crop angles are in the **sensor's** frame, where zero points out of the rover's
left side — so the rover's rear is at 270°, not 180°.

Two traps share a shape worth naming: both produce a result that looks like
success. robot_localization reads `imu0_config` **in the sensor's frame** and
rotates the mask into base_link, and the OAK's IMU sits with its +y axis vertical
— so the slot meaning "base_link yaw" is `vpitch`, not `vyaw`. Getting that wrong
made the filter report exactly zero rotation, which on a stationary rover is
indistinguishable from perfect drift correction. Separately, any node missing from
`bin/stop.sh` survives a restart and runs a second copy of itself; five EKFs
accumulated once, all publishing to `/odometry/filtered`, and the interleaved
output read as one erratic filter rather than five stable ones.

The lesson both times: never accept a stationary null result as proof. Every check
here is built to fail loudly instead — `checks/ekf_response.py` compares the
filter's output noise against its input, `checks/odom_drift.py` reports the swept
range alongside the net and uses the gyro as an independent witness that the rover
really was still, and `checks/slam.sh` flags any node running more than once.
`checks/spin_watch.py` covers the case none of those could reach, being the only
one that requires the rover to actually move.

RViz's view controllers divide on a detail that matters here. `TopDownOrtho`
derives from RViz's frame-*position*-tracking controller: it centres its target
frame and ignores the frame's heading entirely, so pointing it at `base_link`
keeps the rover in the middle of a world that never turns. Only
`ThirdPersonFollower` applies the target's yaw, which is what `config/slam.rviz`
now uses. Rover-locked is the better view for driving; swap back to `TopDownOrtho`
on `map` when judging the estimate, because rover-locked turns a heading error
into an apparently rotating world — which is precisely what made the phantom spin
so hard to read.

# Console runs 3 and 4 stopped looping, but mostly looked the wrong way

**The next two runs from the console, run/e3efe1d1/3 and /4, no longer looped:
each fruitless goal was put aside after one try, as the fix of the morning
intended. But almost none of the goals could have helped, and run 4 sat still
for thirty seconds at a time.** Three faults, each fixed in `64cb020` and deployed
to the rover. A run has not yet shown the fixes.

Run 3 tried six things in 43 s and was stopped by hand. Run 4 tried nine in
160 s, travelled 0.6 m and improved one placement, by 0.19 m, then had nothing
left that was not put aside. It ended when it was driven by hand.

## What went wrong

- **The camera was not pointed at the thing.** A geometry goal's drive carried no
  heading and its look no aim, so the rover looked wherever the drive left it
  facing. Measured from the record, the thing was more than 30 degrees off the
  camera's axis in 16 of the 19 attempts, and behind the rover in seven. The one
  goal that improved anything happened to face its thing, at 8 degrees. Most
  viewpoints were within 0.3 m of where the rover stood, so the rover mostly
  turned on the spot and took a picture of whatever was in front of it.
- **Its look was refused while the rover's own look held the camera.** The rover
  looks once a second by itself, and a run's look arriving at that moment was
  refused, not queued. That happened to six goals of nineteen, and each counted as
  a failed goal. Twice it was two in a row, one short of the three that end a run.
- **A look in progress read as a broken camera.** A look is written down as
  "running" before it is taken. The run read that as a failed look, declined to
  act, and waited 30 s before deciding again.

The other long pause is by design. Once every thing within reach has been put
aside, there is nothing worth doing, and the run waits until a fifteen-minute
cooldown lapses. With the camera pointed the wrong way, every thing nearby was put
aside quickly, so that wait came early.

## The fixes

A geometry goal now drives to its viewpoint facing the thing, and the look is
aimed at it from the heading the rover measures against the map, within the 20
degrees either way the bearing calibration covers. A hypothesis check's look
already worked that way. A run's look waits up to five seconds for a look
already being taken. A look in progress no longer counts as a failure; one that
never finishes still goes stale. Each fix has a test that fails on the previous
code: autonomy 715 and rover_daemon 1078 passed, on the Orin too. Asked on the
rover after the deploy, the chooser picks a viewpoint 2.5 m from object:56,
facing 166 degrees, with the look aimed at the thing.

## What it leaves

Viewpoints the rover cannot stand on are still offered. Now that the look faces
the thing, a viewpoint close to the rover can be worth taking, but whether the
predictions hold once the thing is actually in the picture is not yet measured.

## Requirements

None moved. [R-SAFE-16](../requirements/safety.md#r-safe-16) held. Neither run
crossed a limit, and no limit was needed to stop a loop.

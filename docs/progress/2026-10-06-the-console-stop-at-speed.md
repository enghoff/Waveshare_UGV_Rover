# The console's stop halts the rover at speed within 0.17 m; a reboot left it unable to feel itself turn

**Pressed by the owner while the rover drove at 0.41 m/s, the console's stop
brought it to rest 0.36-0.49 s and 0.15-0.17 m after the daemon logged the
press.** The run ended 0.10 s after the press and latched off. The executive's
next renewal was refused with "somebody stopped the rover". That is the
measurement M3's criterion 10 was still owed. Getting it took two
detours worth knowing about. A reboot of the rover's computer alone left the
rotation sensor reading nonsense, and only a full power cycle cleared it. A
console page left open across a reboot was not connected to anything.
[R-SAFE-11](../requirements/safety.md#r-safe-11) held.

## The stop

The [trial script](2026-10-06-repeat-hang-and-drop-on-the-rover.md) in its
`console` mode, after one change: it waits the whole leg for the press, and
times the stop from the daemon's own record of the latch. Run
`run/573f7022/2`, on a 4.4 m leg back towards the charger, with the owner at the
console. The press came 2.6 s into the leg, at 0.41 m/s measured from the pose.

| | |
|---|---|
| run ended | 0.10 s after the press |
| speed reading at rest | 0.36 s and 0.17 m after the press |
| last movement of the pose | 0.49 s and 0.145 m after the press |
| Nav2 | up throughout |

The two distances differ by a map correction landing in the same moment. The
2026-10-02 trials measured `stop_driving` from an agent at 0.31 m/s: 0.18 s
and 0.11 m. The console sends the same call, and at today's speed it stops
inside the 0.17 m the safe-area margin already allows for
([0.6 m](2026-10-06-repeat-hang-and-drop-on-the-rover.md)).

## Three attempts that did not count

- **The first two** were made before the rover was switched off. The legs
  finished before a press, and nothing reached the daemon.
- **After a reboot of the computer, the rover could not measure its own turning.**
  A leg moved 0.3 m and then stood for 27 s with navigation still driving,
  wanting a 66-degree turn the heading never made. The gyro's bias read
  -2,063 deg/s against 0.41 that morning, and navigation still called its
  position trusted. My fallback stopped it. This is the fault of
  [2026-10-05](2026-10-05-frozen-imu.md) in another form, and the owner's full
  power cycle cleared it again: 0.395 deg/s. Nothing in a run's gates looks at
  this number.
- **After that power cycle a leg ran out without a stop.** The console server's
  log showed no browser had connected since boot. It had also made itself a new
  certificate. A console page left open across a reboot is not a stop button
  until it is reloaded. Reloaded, the next press worked.

## What it means

M3's stop criteria, 9 and 10, are both met on the rover. Two things follow for
anyone running a session. Check `gyro_bias_dps` in `nav_status` before the
rover moves: a healthy rover reads about 0.4. And reload the console after
any restart before relying on its stop. A gate that refuses to open a run on
an absurd gyro bias would make the first check automatic. It is not built, and
it should be reproduced against this recording and 2026-10-05's first.

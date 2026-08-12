# Why depthai is pinned to `<3`

`requirements.txt` pins `depthai>=2.32,<3`. Do not relax it without re-running
`preview_depth.py` and a CAM_B-only capture.

## depthai 3.8.0 crashes this camera's left mono sensor

Measured 2026-08-11: any pipeline streaming **CAM_B** kills the device about a
second after start, before the first frame, with every XLink stream erroring at
once and a firmware crash dump on both LEON cores. It reproduces at 640×480 and
640×400, through both the v3 `Camera` node and the legacy `MonoCamera` node, and
the device log's last line is `Patching intrinsics for socket 1`.

**CAM_C is fine** under the same code, and CAM_A streams 150 frames at a steady
15 fps and 1080p at 10 fps, so it is one sensor path — not mono in general, and not
power or bandwidth. The stored calibration is intact and matches the factory copy,
so that is not the cause either. Under 2.32.0.0 the same CAM_B streams cleanly
along with stereo depth. The hardware is healthy; this is a 3.x regression.

Retested on 3.8.0 in a throwaway venv with `dai.Device(dai.UsbSpeed.HIGH)` on
every run, which rules out [USB speed](oak-usb-link.md) as a confound — three runs
per case, 60 frames each:

| Case | Result |
|---|---|
| CAM_A colour 960×540 | 3/3 ok, 15.2 fps |
| CAM_C mono 640×480 | 3/3 ok, 15.3 fps |
| CAM_B mono 640×480 | **0/3** |
| stereo depth | **0/3** (needs CAM_B) |
| colour + aligned depth | **0/3** (needs CAM_B) |

The failure takes two forms: usually the host dies at `close()` with an access
violation (exit `3221225477` = `0xC0000005`), which is the crash-dump bug below;
once it delivered zero frames for a full 30 s and exited cleanly. Either way no
CAM_B frame ever arrives. Test each case in its own subprocess, or the segfault
takes your test runner with it. Device-side the dumps report `errorId 9001` ending
in a watchdog timeout.

Consequence: stereo depth is unusable on 3.x here, because it needs both mono
cameras.

## There is no newer firmware to move to

Firmware is not separately flashable on this board — the crash log says `Invalid
Flash JEDEC ID... No NOR available` and `Is booted from flash by bootloader: 0`,
`getBootloaderVersion()` returns None, and the host uploads firmware from the
depthai wheel on every boot. The firmware version *is* the depthai version, so the
only choice is which wheel:

* **2.32.0.0 (2026-01-27)** is the newest 2.x — what is pinned here.
* **3.8.0 (2026-07-10)** is the newest release overall — the one measured above.

Two open upstream issues describe the same class of fault, neither fixed as of
2026-08-11:

* [depthai-core#1900](https://github.com/luxonis/depthai-core/issues/1900) —
  OAK-D-Lite AF, same DM9095 board, v3 pipeline kills the device ~1.6 s after start
  with a USB re-enumeration, works under 2.32.0. **But there it is CAM_C that
  crashes and CAM_B that is fine — the mirror of this unit.** Same bug class
  landing on a different socket per device, so this is a v3 regression in the mono
  init path, not a bad sensor on this camera. Its device-side signature is
  `RTEMS_FATAL_SOURCE_INVALID_HEAP_FREE` rather than the watchdog timeout here, so
  treat them as related rather than identical.
* [depthai-core#1920](https://github.com/luxonis/depthai-core/issues/1920) —
  OAK-D-Lite failing the v3 health check on camera and power after 3.6.1 → 3.8.0.

Conclusion: stay on 2.32.0.0. 3.8.0 offers this camera nothing — CAM_A works no
better than on 2.x — and costs both mono-dependent features.

## The crash-dump trap on 3.x

Worth knowing if you ever do run 3.x. The RVC2 stores its last crash dump until
something reads it out, and depthai reads and archives it during `close()`. On
Windows that archiving path segfaults the host process — an access violation inside
`CrashDumpRVC2::toTar`. The result is a run that captures every frame correctly and
*still* dies with a long stack trace the moment the pipeline shuts down, which
reads as a camera fault but is not one. It repeats on every close until the dump is
cleared, and `DEPTHAI_DISABLE_CRASHDUMP_COLLECTION=1` does not prevent it.
`read_crash_dump.py` clears it.

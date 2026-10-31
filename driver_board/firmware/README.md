# The driver board's firmware

The General Driver board's ESP32 (an ESP32-D0WD-V3 rev 3.1 with 16 MB of flash)
runs the motors, the gimbal servos, the OLED and the ICM-20948 IMU, and talks to
the Orin over `/dev/ttyTHS1` at 115200. This directory is its source:
Waveshare's [ugv_base_ros](https://github.com/waveshareteam/ugv_base_ros)
`ROS_Driver`, version 0.96 at `2e7df97`, imported unchanged in one commit, with
the rover project's changes on top. GPL-3.0, as [LICENSE](LICENSE) says.

Until 2026-10-31 the board ran the vendor's own 0.96 image. It was read back
whole that day (16 MB; the app image, 1,208,400 bytes, sha256 `4093e3f1…`), and
the copies are kept off git, since the board's settings partition holds its Wi-Fi
credentials: `~/.ugv/board_firmware/` on the rover and `tmp/board_firmware/` here.
What the board runs is recorded on the rover in `~/.ugv/board_firmware.json`,
which every recording's manifest names
([rover_daemon](../../rover_daemon/README.md#recording-sessions-for-perception)).

## What the rover project changed

- **The board's own timing on every feedback line** (`board_timing.h`), for
  Perception's recording schema version 2 (`D:\Perception\docs\rover-schema-v2.json`
  and `rover-drive-spec.md`, "Schema, version 2"): `ts` (micros() as the line is
  written), `bi` (a boot counter kept in NVS), and one gyro snapshot -- `ti` (micros()
  when its DMP packet was read), `n` (gyro packets accepted since boot), `gc` (the
  packet's DMP footer, raw) and `ln` (loss events since boot) -- changed together
  with `gx..gz` and only when a packet is accepted. A packet is accepted only with
  the FIFO overflow status clear before and after its read and the read complete;
  otherwise it is discarded and the FIFO reset. The vendor's loop ignored a packet
  read that stopped part way, which left the FIFO misaligned. Before the first
  accepted packet, `ti`, `n`, `gc`, `ln` and `gx..gz` are left out.
- **Firmware updates over Wi-Fi** (ArduinoOTA, host `ugv-driver-board`, port
  3232), with a password from `secrets/board-ota.key` compiled in through the
  untracked `ota_secret.h`. The board's partition table already has two app slots.
  The motors are stopped when an update starts.

The IMU is read as before: one DMP packet each pass of the loop, at the vendor's
DMP rate (55 Hz divided by 4 for each output).

## Building

`build.sh` installs the pinned core and libraries with arduino-cli and compiles
with the vendor image's flash settings, so flashing replaces only the app slot.
The output is `build/ROS_Driver.ino.bin` (untracked).

## Flashing

The board cannot be put into download mode by anything the Orin controls: its
USB port can reset the ESP32 but not hold IO0 low, and over USB the ROM never
answered the host even in download mode -- most likely because the Orin's UART
drives the same RX line (2026-10-31, not proven). So the
first flash needs the rover opened and BOOT pressed by hand:

1. Copy the image and `flash.sh` to `~/.ugv/board_firmware/` on the rover.
2. `bash ~/.ugv/board_firmware/flash.sh ROS_Driver.ino.bin` -- it stops the daemon
   and waits.
3. Hold BOOT, tap EN, hold BOOT two seconds more. The ROM says "waiting for
   download" on `/dev/ttyTHS1`, and the script writes app0, verifies it, and clears
   otadata so that app0 boots.
4. Tap EN. The script prints the first feedback lines, then restarts the daemon.

Back to the vendor's firmware: the same, with `vendor-app0.bin`. Once a build with
the Wi-Fi updater is on the board, later updates need neither the rover opened nor
the daemon stopped.

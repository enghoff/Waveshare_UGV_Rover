# Reference: measured facts about the hardware

What the physical parts are, how they are wired, what they draw and what they
were measured to do. These documents describe things that are true of the
hardware rather than choices anybody made, so they change when something is
re-measured or re-wired, not when the software changes.

| Document | Covers |
|---|---|
| [d500-lidar.md](d500-lidar.md) | the scanning lidar: its power, its wiring to the driver board, its data |
| [oak-d-lite.md](oak-d-lite.md) | the stereo depth camera as a board — what it reports itself as, and what it will and will not do |
| [driver-board.md](driver-board.md) | the ESP32 board that owns the motors, lights, servos and telemetry, and driving it from a game pad |
| [i2c.md](i2c.md) | why the host header's I2C is not a spare bus, measured |
| [usb-cameras.md](usb-cameras.md) | the workstation's and rover's UVC cameras, and measuring a camera's field of view |
| [datasheets/](datasheets/) | vendor datasheets and a CAD step file, unmodified |

Each of the first five is the companion to a bench directory that exercises the
hardware from a workstation: [`lidar/`](../../lidar/),
[`oak_camera/`](../../oak_camera/), [`driver_board/`](../../driver_board/) and
[`usb_cameras/`](../../usb_cameras/). Those scripts are not deployed to the
rover.

## What belongs here

A document belongs here if it answers "what is this thing and what does it
actually do" and the answer was arrived at by measuring rather than deciding.
Power draw, connector pinouts, what a bus is really connected to, how wide a
lens sees, what a device calls itself.

**A choice is not a fact.** Why the depth camera's driver is pinned is a
[decision](../decisions/depthai-version-pin.md), even though it rests on
measurements, because somebody could reasonably have chosen otherwise. What the
board reports itself as is a fact.

**What a component does with the hardware is not a fact about the hardware.** How
the rover uses the depth camera is [oak_depth/](../../oak_depth/README.md); how
it uses the lidar is [ros_nav/](../../ros_nav/README.md). This directory stops at
the edge of the device.

## Keeping them honest

Say when a measurement was taken and on what. Several of these were measured
against hosts the rover no longer has — the Raspberry Pi, then the Banana Pi —
and a number measured on a different machine is still useful as long as it says
so. [i2c.md](i2c.md) is the model: it names the board, the date and the
conditions, so a reader can tell what still transfers to the Jetson and what
does not.

Where a number here disagrees with the running system, the running system is
right and this document is stale.

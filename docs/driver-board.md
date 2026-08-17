# The driver board, and driving from a game pad

The rover's *General Driver for Robots* board carries an ESP32 that owns the
motors, the headlights and the pan/tilt servos, and speaks JSON over either WiFi
or its USB serial port. Exercised by [`driver_board/`](../driver_board), whose
one script `drive_gamepad.py` is the only thing in this repository that makes the
rover move; everything else is sensing, with the rover pushed by hand.

It needs no dependencies at all over WiFi, and pyserial only for its `--serial`
path. A game controller needs no driver install either: any pad Windows presents
as XInput will do, and XInput is a DLL Windows already has.

## Running it

```powershell
python driver_board\drive_gamepad.py                       # over WiFi: the rover's AP, else this LAN
python driver_board\drive_gamepad.py --host 192.168.4.1    # straight to a known address
python driver_board\drive_gamepad.py --serial              # over USB, port auto-detected
python driver_board\drive_gamepad.py --no-gimbal           # a rover with no camera fitted
```

There is no window, so it reads the pad through XInput and does not need focus —
you can watch the rover rather than the screen.

## The controls

The triggers drive, right forward and left back, and the right stick steers. The
left stick aims the pan/tilt camera and it stays where you leave it, with a click
of the stick to recentre. `Y` switches the white LEDs and the D-pad sides dim
them, a tap to a notch and a hold to fade — they are PWM, not a switch. RB is
full speed, LB a crawl, D-pad up and down move the speed cap, and Back quits.

Triggers and sticks are all proportional, through a deadzone and an expo curve,
so a small push is a slow crawl rather than a lurch.

## Finding the board

Over WiFi it finds the board itself: the ESP32's own access point first, at
192.168.4.1, and failing that every address on the local network, asking each for
base feedback until one answers. That search exists because the firmware
publishes no mDNS name and sets no DHCP hostname, so a rover that has joined a
home network is an anonymous lease with nothing to look up. `--host` skips the
search once you know the address.

## Stopping

It stops the motors on the way out, and sets the firmware's heartbeat to 500 ms
first, so the rover also stops itself if the script dies or the link drops.

**That failsafe is not the power switch.** It stops the base if the board hears
nothing for the heartbeat interval, which covers a crashed client or a dropped
connection and covers nothing else.

Gimbal commands deliberately do not feed that timer, which is why aiming the
camera is not mistaken for driving — see
[face tracking](face-tracking.md), which commands the servos and never the wheels.

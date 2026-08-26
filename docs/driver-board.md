# The driver board, and driving from a game pad

The rover's *General Driver for Robots* board carries an ESP32 that owns the
motors, the headlights and the pan/tilt servos, and speaks JSON over either WiFi
or its USB serial port. Exercised by [`driver_board/`](../driver_board), whose
one script `drive_gamepad.py` is the only thing in this repository that makes the
rover move; everything else is sensing, with the rover pushed by hand.

The IMU, the OLED and the pack monitor are the ESP32's too, on I2C. The 40-pin
host header brings that same bus out on pins 3/5; it is not a vacant host bus.
See [i2c.md](i2c.md).

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

## When the IMU freezes

The board's IMU can hang while the board itself stays perfectly healthy, and the
failure is quiet: the ESP32 keeps answering over both the serial port and WiFi,
and keeps sending `T:1001` telemetry at the usual rate. Only the numbers stop
moving. Seen on 2026-08-26, where `ax`, `ay`, `az`, `gx`, `gy`, `gz`, the three
magnetometer axes *and* the battery voltage were all byte-identical for over an
hour, with `az` sitting on exactly 16384.

**How to be sure.** Ask twice, a second apart, and compare:

```bash
python3 - <<'EOF'
import http.client, json
from urllib.parse import quote
c = http.client.HTTPConnection("192.168.1.22", timeout=6)
c.request("GET", "/js?json=" + quote(json.dumps({"T": 130}), safe=""))
print(c.getresponse().read().decode())
EOF
```

Live readings jitter in the last digit or two even on a stationary rover, and
`az` sits near 8400 rather than on a round power of two. `{"T":126}` asks the
board's own fusion for roll/pitch/yaw; all-zero there alongside frozen `T:1001`
values is the same fault seen from the other side.

**Why it matters more than it looks.** `ros_nav/base_node.py` takes the rover's
whole heading from the gyro -- `d_yaw` comes from consecutive `gz` readings and
nothing else -- so a frozen gyro is a rover whose odometry can never turn. Nav2
still accepts goals and still drives; it simply never believes the rover has
rotated. Worse, `base_node` reads the stuck value as an enormous zero-offset and
says so, which is the tell in the log:

    gyro bias -769.134 deg/s over 35954 still samples

Anything outside roughly a degree per second is not a bias, it is a dead sensor.

**The fix is to reboot the board**, which the firmware documents as `CMD_REBOOT`
in `json_cmd.h` -- the same header that defines the `CMD_BASE_FEEDBACK 130` and
`CMD_LED_CTRL 132` this repository already uses, so the numbering is known
rather than guessed:

```bash
python3 - <<'EOF'
import http.client, json
from urllib.parse import quote
c = http.client.HTTPConnection("192.168.1.22", timeout=4)
try:
    c.request("GET", "/js?json=" + quote(json.dumps({"T": 600}), safe=""))
    c.getresponse().read()
except Exception:
    pass          # the board drops the link as it goes down; that is the reply
EOF
```

Send it over WiFi rather than the serial port: the daemon owns the UART, and
interleaving with its traffic is a needless risk. The board is back in about
three seconds. Do it with the motors stopped.

**Then restart `ros_nav`.** `base_node` keeps its bias estimate across the
board coming back, so it will spend a while averaging dead readings together
with live ones -- ours reported 346 deg/s of rotation on a stationary rover
until it was restarted. `~/ugv/ros_nav/restart.sh` and a half-minute of standing
still gets it back to the tenth-of-a-degree figure it should have.

## Stopping

It stops the motors on the way out, and sets the firmware's heartbeat to 500 ms
first, so the rover also stops itself if the script dies or the link drops.

**That failsafe is not the power switch.** It stops the base if the board hears
nothing for the heartbeat interval, which covers a crashed client or a dropped
connection and covers nothing else.

Gimbal commands deliberately do not feed that timer, which is why aiming the
camera is not mistaken for driving — see
[face tracking](face-tracking.md), which commands the servos and never the wheels.

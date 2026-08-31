# Header I2C on the rover

The 40-pin host header's I2C — pins 3 (SDA) and 5 (SCL) — is not a vacant bus
for whatever the host wants to hang on it. It is the ESP32's I2C, brought out
through the stack. Waveshare's own note is that the host occupies the GPIO UART
and nothing else; the IMU, the OLED and the pack monitor belong to the
sub-controller. That is also what the wiring does.

Measured 2026-08-23 on the Banana Pi M4 Zero sitting where the Pi sat, with the
daemon running and `T:1001` flowing.

## The buses on the M4 Zero

| adapter | hardware | pins | what is on it |
|---|---|---|---|
| `/dev/i2c-0` | TWI0, `5002000.i2c` | header 3/5 = PI6 / PI5 | the driver board's I2C |
| `/dev/i2c-5` | DesignWare HDMI DDC | — | ignore |
| `/dev/i2c-6` | SoC R_I2C, `7081400.i2c` | not on the header | AXP313a PMIC at `0x36`, claimed (`UU`) |

`i2c-6` being rock-stable is a control, not a comparison: different controller,
board-level pull-ups, one kernel driver. It only proves that `i2c-tools` and
`mv64xxx` work. Header TWI0 is the odd one out.

On a Pi the same header pins are I2C1 (`/dev/i2c-1`) with 1.8 kΩ pull-ups to
3.3 V. The M4 Zero's live pinctrl for those pins is `function = "i2c0"` and
nothing else: **bias disabled**, 20 mA drive, no `clock-frequency` in the DT
(100 kHz default). There is no `pinconf-set` on this kernel, and the pins are
claimed by the controller, so pull-ups cannot be turned on from userspace.

## What answers on TWI0

Five `i2cdetect -y -r 0` passes in a row, telemetry running, were identical.

| address | chip | how it was identified | already on UART |
|---|---|---|---|
| `0x68` | ICM-20948 | `WHO_AM_I` at `0x00` is `0xea`; `PWR_MGMT_1` at `0x06` is `0x01` (out of sleep) | `ax/ay/az`, `gx/gy/gz`, `mx/my/mz` in `T:1001` |
| `0x3c` | SSD1306 OLED | register `0x00` stably `0x02`; a current-address read fails, which is normal — it wants a control byte | `{"T":3,...}` / `{"T":-3}` |
| `0x42` | INA219-class pack monitor | register `0x02` high byte `0x5b` → ~11.65 V against the daemon's 11.68 V. `0xFE` is not TI's `0x5449`, so a clone or an unimplemented manufacturer id | `"v"` in `T:1001` |

`0x10`, `0x14`, `0x66` and the rest that move between passes are ghosts.
`i2cdetect -q` (quick write) invents more of them than `-r`.

The ESP32 is the master and polls this bus at the telemetry rate (~20 Hz). A
host scan is a second master on the same wire. Address ACKs still look real;
data phases lose the arbitration. That is why a dump fails two times out of
three and a one-byte `WHO_AM_I` sometimes works. Reads taken just after a
`T:1001` line is the quietest window.

Quieting the ESP32 with `{"T":131,"cmd":0}` stops the UART stream and makes the
IMU *harder* to see, not easier. The firmware is what keeps the ICM-20948
awake. Put feedback back with `{"T":131,"cmd":1"}` — and do it over Wi‑Fi
(`http://192.168.1.22/js?json=...`) if the daemon owns the UART.

## What the host already has, and what it uses

Nothing in this repository opens `/dev/i2c-*`. The daemon's `SerialLink` drains
the GPIO UART (`/dev/ttyTHS1` on the Orin, `ttyS4` on the M4 Zero, `ttyAMA0`
on the Pi).
[`ros_nav/base_node.py`](../ros_nav/base_node.py) takes only `gz` from that
stream: as a rotation witness once rest is known, and as a motion prior once
confirmed turns have measured the scale. Accel, the other gyro axes and the
magnetometer stay in the line and are never parsed. Pack volts go to the
`battery` tool the same way.

`HttpLink` can `GET /js?json=...` if the daemon is started with `--host`. The
live supervisor does not pass that flag. SLAM and navigation never talk HTTP
to the board; a Wi‑Fi link has no stream to integrate, so odometry would
report unknown.

So adding a host I2C client for the IMU, the OLED or the pack monitor is
duplicating a bus the ESP32 already owns, and fighting it.

## Adding a device the host should own

Use a net the ESP32 does not sit on.

The M4 Zero also brings TWI1 out on header pins 27/28 (PI8 / PI7). That is
the Pi's ID-EEPROM pair; on this stack it is the first place to try for a
host sensor (ToF, extra IMU, display that is not the rover's OLED). Confirm
with `i2cdetect` that the three addresses above do **not** appear there —
if they do, that header is tied to the same bus and you are back where you
started.

Then:

1. Enable the controller in the device tree. On TWI0 the pin node is only
   `function` and `pins`; add `bias-pull-up` (and a `clock-frequency` of
   50–100 kHz if the wiring is long). The M4 Zero does not have the Pi's
   1.8 kΩ resistors. A reboot applies an overlay; there is no runtime knob.
2. Identify the chip with a `WHO_AM_I` / manufacturer id, not with
   `i2cdetect`. A scan that moves between passes is not a chip list.
3. Do not pull those lines to 5 V.

If the only connector in reach is pins 3/5, the work is electrical first:
continuity from those pins to the ESP32's SDA/SCL, and idle voltage vs 3.3 V.
Software cannot split a shared net.

## Seeing the map again

```bash
i2cdetect -l
i2cdetect -y -r 0          # header; want 3c, 42, 68 and nothing that moves
i2cdetect -y -r 6          # SoC; want UU at 0x36 only
i2cget -y -f 0 0x68 0x00   # 0xea when the ICM answers
```

A failed `i2cdump` is not "no device". A dump is a long transaction; the
ESP32 will kill it even when a one-byte read works.

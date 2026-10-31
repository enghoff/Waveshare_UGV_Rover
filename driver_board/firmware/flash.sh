#!/bin/bash
# Flash an app image into the driver board's app0 slot, on the Orin, over the
# Orin's own UART. Not deployed: copy it and the image to ~/.ugv/board_firmware/.
#
#     bash flash.sh <image.bin> [seconds to wait for BOOT + EN]
#
# The board cannot be put into download mode from the Orin: someone holds BOOT,
# taps EN and holds BOOT two seconds more (the rover opened). The daemon shares
# the ESP32's UART0, so it is stopped first and restarted whatever happens. After
# writing, EN is tapped by hand to start the new firmware; this script then reads
# a few feedback lines before restarting the daemon. To go back, flash
# vendor-app0.bin, the image read from the board on 2026-10-31.
set -u
IMAGE="${1:?image}"
WAIT="${2:-240}"
U=/dev/ttyTHS1
ESPTOOL="env PYTHONPATH=$HOME/.ugv/esptool /usr/bin/python3 -m esptool"
cd "$(dirname "$0")" || exit 1
[ -s "$IMAGE" ] || { echo "no image $IMAGE"; exit 1; }
size=$(stat -c %s "$IMAGE")
[ "$size" -le $((0x140000)) ] || { echo "$IMAGE does not fit app0"; exit 1; }
sha256sum "$IMAGE"

restart() { echo "--- restarting the daemon"; ~/ugv/restart.sh 2>&1 | tail -1; }
trap restart EXIT
echo "--- stopping the daemon"
pkill -f 'ugv/run_daemo[n].sh'
pkill -f 'ugv/rover_daemo[n].py'
sleep 3
pgrep -fa 'rover_daemo[n].py' && { echo "daemon still running"; exit 1; }

echo "--- waiting up to $WAIT s for BOOT + EN (watching $U)"
/usr/bin/python3 - "$WAIT" <<'EOF' || exit 1
import serial, sys, time
s = serial.Serial("/dev/ttyTHS1", 115200, timeout=0.3)
t = time.time()
while time.time() - t < float(sys.argv[1]):
    for line in s.read(4096).decode("utf-8", "replace").splitlines():
        if "rst:" in line or "waiting" in line:
            print(round(time.time() - t, 1), line.strip(), flush=True)
        if "waiting for download" in line:
            s.close(); sys.exit(0)
s.close(); print("no download mode seen"); sys.exit(1)
EOF

echo "--- writing $IMAGE to app0 (0x10000)"
$ESPTOOL --port $U --before no-reset --after no-reset -b 460800 \
  write-flash --flash-mode keep --flash-freq keep --flash-size keep 0x10000 "$IMAGE" || exit 1
$ESPTOOL --port $U --before no-reset --after no-reset -b 460800 \
  verify-flash 0x10000 "$IMAGE" || exit 1
# app0 must be the slot that boots: an otadata pointing at app1 would bypass it.
$ESPTOOL --port $U --before no-reset --after no-reset -b 460800 \
  erase-region 0xe000 0x2000 || exit 1

echo "--- tap EN now to start the new firmware; reading its first lines for 60 s"
/usr/bin/python3 - <<'EOF'
import serial, time
s = serial.Serial("/dev/ttyTHS1", 115200, timeout=0.3)
t, seen = time.time(), 0
while time.time() - t < 60 and seen < 5:
    for line in s.read(4096).decode("utf-8", "replace").splitlines():
        if '"T":1001' in line:
            seen += 1
            print(line.strip(), flush=True)
        elif "rst:" in line or "DMP" in line:
            print(line.strip(), flush=True)
s.close()
print("feedback lines seen:", seen)
EOF

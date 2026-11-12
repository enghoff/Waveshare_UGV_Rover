#!/bin/sh
# Bluetooth audio for the rover's voice. Run once, as root:
#
#     ssh orin 'sudo -S -p "" sh ~/ugv/drive_web/install_bt_audio.sh' < secrets/jetson-orin.key
#
# The console sends the conversation to a paired speaker and takes the microphone
# from a paired headset (rover_sound.py), through the PipeWire that Ubuntu runs
# for the `jetson` user. Out of the box three things stop that, and this undoes
# them and nothing else:
#
# 1. NVIDIA's drop-in for bluetoothd (nv-bluetooth-service.conf) starts it with
#    --noplugin=audio,a2dp,avrcp,sap. Without the a2dp plugin there is no
#    music-quality link at all, so a speaker -- which offers nothing else --
#    cannot be used, and a headset is held to its 16 kHz call profile. Ours sorts
#    after NVIDIA's and replaces its command line, keeping its -d and leaving
#    only the SIM-access plugin off.
# 2. PipeWire is a user service, and with nobody logged in there is no user
#    session for it to run in. Linger starts it at boot.
# 3. WirePlumber only lets a user with a session on a seat use Bluetooth audio,
#    and a rover has no seat: nobody logs in at a screen. That check is switched
#    off for this user.
#
# Restarting bluetoothd drops whatever is connected; a trusted controller or
# headset reconnects by itself when it is next switched on.
set -e

NAME=${SUDO_USER:-jetson}
NUMBER=$(id -u "$NAME")
HOME_DIR=$(getent passwd "$NAME" | cut -d: -f6)

mkdir -p /etc/systemd/system/bluetooth.service.d
cat > /etc/systemd/system/bluetooth.service.d/zz-rover-audio.conf <<'EOF'
# Written by ~/ugv/drive_web/install_bt_audio.sh: bluetoothd with its audio
# plugins, which NVIDIA's nv-bluetooth-service.conf turns off.
[Service]
ExecStart=
ExecStart=/usr/libexec/bluetooth/bluetoothd -d --noplugin=sap
EOF
systemctl daemon-reload

loginctl enable-linger "$NAME"
systemctl start "user@$NUMBER.service"

CONF="$HOME_DIR/.config/wireplumber/bluetooth.lua.d"
runuser -u "$NAME" -- mkdir -p "$CONF"
runuser -u "$NAME" -- sh -c "cat > '$CONF/51-rover.lua'" <<'EOF'
-- Written by ~/ugv/drive_web/install_bt_audio.sh. The rover has no seat --
-- nobody logs in at a screen -- so the logind check would keep Bluetooth audio
-- off for good. mSBC is the 16 kHz call codec, which is the rate the voice
-- model listens at.
bluez_monitor.properties["with-logind"] = false
bluez_monitor.properties["bluez5.enable-msbc"] = true
EOF

systemctl restart bluetooth
sleep 2
# The sound server registers its audio endpoints with bluetoothd when it
# starts, so it has to start after the bluetoothd that will take them.
systemctl --user -M "$NAME@" restart pipewire.service pipewire-pulse.service wireplumber.service
sleep 3

echo "bluetoothd: $(systemctl show -p ExecStart --value bluetooth | tr -s ' ' | cut -c1-120)"
echo "linger: $(loginctl show-user "$NAME" -p Linger --value)"
if bluetoothctl show | grep -q "Audio Source"; then
    echo "ok: the rover offers A2DP and hands-free audio"
else
    echo "FAIL: bluetoothctl show lists no Audio Source" >&2
    bluetoothctl show | grep UUID >&2
    exit 1
fi

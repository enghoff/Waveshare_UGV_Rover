#!/usr/bin/env bash
# Keep the JMTek USB audio dongle away from the guest's USB stack.
#
# 0c76:1229 shares VMware's virtual USB 1.1 hub with the lidar -- the dongle on
# port 2-2.2, the lidar on 2-2.1 -- and it fails there. Every 60 to 80 seconds
# the kernel resets it, the reset ends in "device descriptor read/64, error -71",
# and the lidar is disconnected in the same instant:
#
#   usb 2-2.2: reset full-speed USB device number 18 using uhci_hcd
#   usb 2-2.2: device descriptor read/64, error -71
#   usb 2-2.1: USB disconnect, device number 26        <- the lidar
#   usb 2-2.1: new full-speed USB device number 27
#
# 19 lidar disconnects in the first hour of a session. The lidar itself is
# healthy; it is collateral damage from a neighbour on the same hub. Nothing on
# this rover wants a sound card, so the cheapest fix is to keep the guest from
# driving it at all.
#
# Deauthorizing is not enough, and it is worth saying why, because authorized=0
# looks like the right tool and reads as sufficient. Two rounds of measurement:
#
#   * Deauthorizing a device whose drivers are already bound leaves those
#     bindings in place, and snd-usb-audio goes on re-probing after each reset.
#     7 lidar disconnects in the 4 minutes after, against ~3 before.
#
#   * Refusing authorization at add time, before any interface is configured,
#     is cleaner -- ATTR{authorized}=="0" with no 2-2.2:* interfaces at all --
#     and it still does not help. 39 kernel events on the port and 3 more lidar
#     disconnects in the following 4.5 minutes.
#
# So the resets are not driven by driver I/O at all. USB core resets the device
# on its own, the descriptor read fails at -71, and the hub takes its neighbour
# down regardless of whether anything in the guest is talking to it. The device
# has to be gone, not merely idle:
#
#   for i in 1.0 1.1 1.2 1.3; do
#       echo "2-2.2:$i" | sudo tee /sys/bus/usb/devices/2-2.2:$i/driver/unbind
#   done
#   echo 1 | sudo tee /sys/bus/usb/devices/2-2.2/remove
#
# which is exactly what the helper below does, on every add. It has to run on
# every add rather than once, because the host hands the device back: with
# usb.generic.autoconnect="TRUE" in the vmx, VMware re-attaches it within a few
# minutes of any removal -- observed at 18:56 -> 19:01 and again at 19:04 ->
# 19:06. The rule turns that into a loop the guest always wins.
#
# The removal runs through systemd-run --no-block rather than directly from
# RUN+=. A udev worker that writes to its own device's "remove" is waiting on
# the event it is itself processing, and that is how udev workers hang.
#
# This is the guest's half. The host's half is usb.generic.autoconnect in
# ugv-rover.vmx -- see docs/vm-usb.md. The vmx is the better fix and this is the
# more reliable one, because VMware rewrites that file on every power-off and a
# hand edit there can be quietly lost.
set -u

sudo tee /usr/local/sbin/usb-drop-device > /dev/null <<'EOF'
#!/bin/sh
# Detach one USB device from the guest, given its /sys path. Interfaces are
# unbound first: "remove" on a device with drivers still attached leaves the
# driver tearing down underneath the removal, which is a slower and noisier path
# to the same place.
set -u
d="${1:-}"
[ -n "$d" ] && [ -d "$d" ] || exit 0

for drv in "$d":*/driver; do
    [ -e "$drv" ] || continue
    iface=$(basename "$(dirname "$drv")")
    echo "$iface" > "$drv/unbind" 2>/dev/null || true
done

echo 1 > "$d/remove" 2>/dev/null || true
logger -t usb-drop-device "detached $(basename "$d")"
EOF
sudo chmod +x /usr/local/sbin/usb-drop-device

sudo tee /etc/udev/rules.d/80-no-usb-audio.rules > /dev/null <<'EOF'
# JMTek USB PnP Audio Device: unstable on the virtual UHCI hub, and it takes the
# lidar down with it every time it faults. Deauthorizing is not enough -- USB
# core resets it regardless of whether any driver is bound -- so drop it from
# the guest entirely, on every add, because VMware keeps handing it back.
ACTION=="add", SUBSYSTEM=="usb", ENV{DEVTYPE}=="usb_device", ATTR{idVendor}=="0c76", ATTR{idProduct}=="1229", RUN+="/usr/bin/systemd-run --no-block /usr/local/sbin/usb-drop-device /sys%p"
EOF

sudo udevadm control --reload-rules

echo "== rule installed =="
cat /etc/udev/rules.d/80-no-usb-audio.rules

echo "== is it attached right now? =="
# A new rule does not re-run against devices that are already present, so deal
# with this one by hand rather than waiting for the next attach.
dev=$(grep -ls '^PRODUCT=c76/1229' /sys/bus/usb/devices/*/uevent 2>/dev/null | head -1)
if [ -n "$dev" ]; then
    d=$(dirname "$dev")
    echo "  attached at $(basename "$d") -- dropping it now"
    sudo /usr/local/sbin/usb-drop-device "$d"
    sleep 2
fi
lsusb | grep '0c76:1229' && echo "  STILL ATTACHED" || echo "  gone"

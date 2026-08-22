#!/bin/sh
# Grant the daemon permission to reset its own USB devices. Needs root, once.
#
#   cat secrets/rpi-sudo.key | ssh rpi 'sudo -S -p "" ~/ugv/lidar_slam/install-udev.sh'
#
# Idempotent: run it again after editing the rule. Without this the recovery in
# usbreset.py still runs and still says what it would have done -- it just comes
# back "not allowed to reset /dev/bus/usb/001/005" every time, which is a rover
# that stays blind until somebody replugs it by hand.
set -e
cd "$(dirname "$0")"

RULE=99-rover-usb-reset.rules
install -m 0644 "$RULE" "/etc/udev/rules.d/$RULE"
udevadm control --reload-rules

# Applied to what is already plugged in, not only to the next thing plugged in: a
# rule that takes effect at the next reboot is no use to a rover that has lost its
# lidar now. The action has to be `add` and not `change`: udev sets a node's owner
# and mode when the node is created, and re-running the rules under `change` matched
# this rule, said "GROUP 46, MODE 0660" in `udevadm test`, and left every node
# root:root 0664 -- which looks exactly like a rule that did not match.
udevadm trigger --subsystem-match=usb --action=add

echo "installed /etc/udev/rules.d/$RULE"
# Shown as owner and mode rather than tested with -w, because this script is running
# as root and everything is writable to root -- which would report success whatever
# the rule did. What matters is the group, and that the daemon's user is in it.
echo "the USB nodes now read:"
ls -l /dev/bus/usb/*/* | sed 's/^/  /'

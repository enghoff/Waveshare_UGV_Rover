#!/usr/bin/env bash
# Give the D500's CH343 a driver and a stable device name in the guest.
#
# The descriptor is plain CDC-ACM (interface classes 2 + 10), not vendor-specific,
# so cdc_acm binds it and it appears as /dev/ttyACM0 -- not /dev/ttyUSB0 as
# Waveshare's Pi-based documentation says. Nothing from ch341 or WCH is needed.
#
# A symlink is worth the extra rule: the host has five other CH340 adapters that
# come and go, so anything matching on enumeration order is fragile. Match on the
# CH343's own vid:pid instead and always open /dev/rover-lidar.
set -u

sudo modprobe cdc_acm
echo cdc_acm | sudo tee /etc/modules-load.d/cdc-acm.conf > /dev/null

sudo tee /etc/udev/rules.d/81-rover-lidar.rules > /dev/null <<'EOF'
# Waveshare General Driver for Robots, LIDAR Type-C port (CH343 behind an FE1.1S hub)
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d3", SYMLINK+="rover-lidar", MODE="0666"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty
sudo udevadm settle

echo "== tty devices =="
ls -l /dev/ttyACM* /dev/rover-lidar 2>&1 | head -5
echo "== dmesg =="
sudo dmesg | grep -i -E 'cdc_acm|ttyACM' | tail -5

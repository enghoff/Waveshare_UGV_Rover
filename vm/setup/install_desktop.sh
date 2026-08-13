#!/usr/bin/env bash
# Minimal XFCE so RViz has somewhere to draw.
#
# XFCE rather than GNOME: on 4 vCPUs of a Lunar Lake laptop, the desktop should
# cost as little as possible because the 3D point cloud rendering is what will
# actually be slow. Autologin so the VMware console comes up ready to screenshot
# with vmrun captureScreen -- there is no one sitting at this VM to type a password.
set -eo pipefail

export DEBIAN_FRONTEND=noninteractive
export NEEDRESTART_MODE=a

sudo apt-get install -y -qq xfce4 xfce4-terminal lightdm dbus-x11 mesa-utils

sudo mkdir -p /etc/lightdm/lightdm.conf.d
sudo tee /etc/lightdm/lightdm.conf.d/50-autologin.conf > /dev/null <<'EOF'
[Seat:*]
autologin-user=rover
autologin-user-timeout=0
user-session=xfce
EOF

sudo systemctl set-default graphical.target
sudo systemctl enable lightdm
sudo systemctl start lightdm

sleep 10
echo "=== display manager ==="
systemctl is-active lightdm
echo "=== X session ==="
ls /tmp/.X11-unix/ 2>/dev/null || echo "(no X socket yet)"
who

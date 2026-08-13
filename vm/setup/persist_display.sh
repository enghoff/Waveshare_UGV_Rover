#!/usr/bin/env bash
# Make 1280x800 survive a reboot.
#
# The VMware SVGA output comes up at 800x600 every boot, and at that size RViz
# has no room to dock the Image panel beside Displays -- the camera view simply
# is not drawn, which reads as the camera having failed. fix_display.sh corrects
# it for the session; this makes it stick.
set -eo pipefail

mkdir -p "$HOME/.config/autostart"
cat > "$HOME/.config/autostart/rover-resolution.desktop" <<'EOF'
[Desktop Entry]
Type=Application
Name=Rover display mode
Comment=VMware SVGA comes up at 800x600; RViz needs more to dock the image panel
Exec=sh -c 'sleep 3; xrandr --output Virtual1 --mode 1280x800'
X-GNOME-Autostart-enabled=true
EOF

echo "installed:"
cat "$HOME/.config/autostart/rover-resolution.desktop"

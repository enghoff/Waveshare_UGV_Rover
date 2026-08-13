#!/usr/bin/env bash
# Put a "Start SLAM (RViz)" icon on the XFCE desktop.
#
# Safe to re-run; run it again after changing the entry below. Separate from
# install_desktop.sh because that one builds the desktop before ~/ugv is
# deployed, and this needs the scripts to already be there.
set -eo pipefail

DESK="$(xdg-user-dir DESKTOP 2>/dev/null || echo "$HOME/Desktop")"
ENTRY="$DESK/slam-rviz.desktop"
mkdir -p "$DESK"

# Exec must be an absolute path -- a .desktop file gets no shell, so ~ and $HOME
# are left as literal text. Terminal=true hands the command to XFCE's terminal;
# start_slam_gui.sh is what decides how long that window stays.
cat > "$ENTRY" <<EOF
[Desktop Entry]
Type=Application
Version=1.0
Name=Start SLAM (RViz)
Comment=Lidar, camera, EKF odometry and slam_toolbox, with RViz on this display
Exec=$HOME/ugv/bin/start_slam_gui.sh
Path=$HOME/ugv
Icon=applications-science
Terminal=true
StartupNotify=false
Categories=Science;
EOF

chmod +x "$ENTRY"

# xfdesktop refuses to run a launcher it has not seen before, and asks the user
# to confirm once. It records the answer as a sha256 of the file, so writing that
# ourselves skips the prompt -- and, because it is a checksum, editing the entry
# later correctly brings the prompt back.
if command -v gio > /dev/null && command -v sha256sum > /dev/null; then
    gio set -t string "$ENTRY" metadata::xfce-exe-checksum \
        "$(sha256sum "$ENTRY" | cut -d' ' -f1)" 2>/dev/null || true
fi

chmod +x "$HOME/ugv/bin/start_slam_gui.sh"

echo "installed: $ENTRY"
ls -l "$ENTRY"
desktop-file-validate "$ENTRY" 2>/dev/null && echo "entry validates" || true

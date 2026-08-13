#!/usr/bin/env bash
# xfce4-screensaver maps a full-screen window over an idle desktop, which is why
# screen grabs came back uniformly black while lightdm, XFCE and RViz were all
# healthy. Kill it, stop it coming back, and give RViz more than 800x600.
set -eo pipefail
export DISPLAY=:0

pkill -9 -f xfce4-screensaver 2>/dev/null || true
sudo rm -f /etc/xdg/autostart/xfce4-screensaver.desktop
xfconf-query -c xfce4-screensaver -p /saver/enabled -s false --create -t bool 2>/dev/null || true

xset s off
xset s noblank
xset -dpms

OUT=$(xrandr | awk '/ connected/{print $1; exit}')
echo "output: ${OUT:-none}"
if [ -n "$OUT" ]; then
    xrandr --output "$OUT" --mode 1280x800 2>/dev/null \
        || xrandr --output "$OUT" --auto 2>/dev/null \
        || true
fi
sleep 3
xwininfo -root | grep -E 'Width:|Height:'

scrot -o /tmp/screen.png
ls -l /tmp/screen.png

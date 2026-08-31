#!/bin/bash
# Build the escape behaviours and install them beside this file.
#
#     ~/ugv/ros_nav/behaviors/build.sh
#
# Run by the manifest on every deploy of ros_nav, which is not belt and braces:
# `lidar_slam/` has the same arrangement and the reason is written there, that a
# stale shared object is a rover running last week's code with this week's source
# next to it on disk and nothing anywhere saying so. A rebuild is seconds; being
# wrong about which code is loaded has cost this repository days.
#
# It installs into ./install rather than into the conda environment. The
# environment is an installed dependency that ros_nav/install.sh builds and
# `mamba install` may rewrite; a deploy must not be editing it. nav.launch.py
# puts ./install on the behaviour server's own environment and on nothing else's.

set -eu

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD="$DIR/build"
PREFIX="$DIR/install"

# The conda ROS, for the headers and the libraries to link against. Sourced with
# nounset off for the reason env.sh spells out: RoboStack's activation hook reads
# $CONDA_BUILD without a default and dies under `set -u`, naming a variable
# nobody here has heard of.
case $- in *u*) _had_nounset=1 ;; *) _had_nounset=0 ;; esac
set +u
# shellcheck disable=SC1091
. "$DIR/../env.sh"
[ "$_had_nounset" = 1 ] && set -u
unset _had_nounset

echo "== building against $ROS_ENV_PREFIX"

# A clean configure whenever the environment moved, because a CMake cache
# remembers absolute paths into a conda prefix that a reinstall can replace.
if [ -f "$BUILD/CMakeCache.txt" ] &&
   ! grep -q "$ROS_ENV_PREFIX" "$BUILD/CMakeCache.txt" 2>/dev/null; then
    echo "== the ROS environment moved; starting the build tree again"
    rm -rf "$BUILD"
fi

cmake -S "$DIR" -B "$BUILD" \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_PREFIX_PATH="$ROS_ENV_PREFIX" \
      -DCMAKE_INSTALL_PREFIX="$PREFIX" > "$BUILD.log" 2>&1 || {
    echo "== cmake configure failed:"; tail -25 "$BUILD.log"; exit 1; }

cmake --build "$BUILD" --parallel "$(nproc)" >> "$BUILD.log" 2>&1 || {
    echo "== compile failed:"; tail -40 "$BUILD.log"; exit 1; }

cmake --install "$BUILD" >> "$BUILD.log" 2>&1 || {
    echo "== install failed:"; tail -25 "$BUILD.log"; exit 1; }

# What was actually produced, printed rather than assumed. The failure this
# catches is a build that succeeded against headers the running behaviour server
# will not load -- an undefined symbol at plugin load time, which appears in the
# log as "Failed to create behavior" and reads as a typo in the class name.
so="$PREFIX/lib/libugv_behaviors.so"
if [ ! -f "$so" ]; then
    echo "== no library at $so"; exit 1
fi
echo "== built:   $so ($(stat -c %s "$so") bytes, $(date -r "$so" -Is))"
missing=$(LD_LIBRARY_PATH="$ROS_ENV_PREFIX/lib:${LD_LIBRARY_PATH:-}" \
          ldd -r "$so" 2>&1 | grep -c "undefined symbol" || true)
echo "== symbols: ${missing:-0} undefined (0 = the server will be able to load it)"
if [ "${missing:-0}" != 0 ]; then
    LD_LIBRARY_PATH="$ROS_ENV_PREFIX/lib:${LD_LIBRARY_PATH:-}" \
        ldd -r "$so" 2>&1 | grep "undefined symbol" | head -5
    exit 1
fi
echo "== plugins: $(grep -c 'class type' "$DIR/ugv_behaviors.xml") declared"

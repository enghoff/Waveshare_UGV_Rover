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

# **The build is keyed on what the sources say, never on when they were written,
# and that is not a refinement -- without it this never rebuilds at all.**
#
# `deploy.py` packs every file with `mtime = 0` on purpose, so that an unchanged
# file has an unchanged tar and rsync's quick check can skip it. The consequence
# here is that every source on the rover is dated 1970 and is therefore older
# than any object file `make` has already produced. Incremental builds see
# nothing to do, for ever. This was not theoretical: the escape behaviours were
# rebuilt by three deploys running and the running behaviour server kept the
# library from the first one, while the source beside it plainly said otherwise
# -- exactly the fault lidar_slam/build.sh avoids by having no build cache at
# all, and the one CLAUDE.md warns about.
#
# So the stamp is a hash of the sources. Same hash, nothing to do; different
# hash, the build tree goes and is made again from scratch. A CMake cache also
# remembers absolute paths into a conda prefix that a reinstall can replace, so
# the environment goes into the hash too.
STAMP="$PREFIX/.built-from"
current=$(cat "$DIR/CMakeLists.txt" "$DIR/ugv_behaviors.xml" \
              "$DIR/src/"*.cpp "$DIR/include/ugv_behaviors/"*.hpp 2>/dev/null |
          sha256sum | cut -d" " -f1)
current="$current $ROS_ENV_PREFIX"

if [ -f "$STAMP" ] && [ "$(cat "$STAMP")" = "$current" ] &&
   [ -f "$PREFIX/lib/libugv_behaviors.so" ]; then
    echo "== sources unchanged since the last build; nothing to do"
    echo "== built:   $PREFIX/lib/libugv_behaviors.so"
    exit 0
fi

echo "== sources differ from the last build; building from scratch"
rm -rf "$BUILD"

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

# Written last, so that a build which failed anywhere above is not recorded as
# done and the next deploy tries again rather than skipping.
printf '%s' "$current" > "$STAMP"

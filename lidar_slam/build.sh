#!/bin/sh
# Build the SLAM core and its selftest. Run this on the machine that will run it --
# the rover's Pi is armv6 and nothing else here is, so there is no cross-compiler
# and no build cache to get stale.
#
#   ssh rpi 'cd ~/ugv/lidar_slam && ./build.sh && ./selftest'
set -e
cd "$(dirname "$0")"

CC=${CC:-gcc}
# -O2 is worth about 6x over -O0 here and -O3 measured no better: the hot loop is
# waiting on cache misses into the likelihood field, not on instruction count.
# No -ffast-math: the pose search compares sums for equality of ordering, and the
# saturating arithmetic in the map update wants predictable rounding.
CFLAGS=${CFLAGS:--O2 -Wall -Wextra -std=c99 -fPIC}

echo "building with $($CC --version | head -1)"
$CC $CFLAGS -shared -o libslam2d.so slam2d.c -lm
$CC $CFLAGS -o selftest selftest.c slam2d.c -lm
echo "built libslam2d.so and selftest"

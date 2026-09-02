#!/bin/sh
# Build the lidar parser and its selftest. Run this on the machine that will run it --
# libslam2d.so is per-host and is not committed, so there is no cross-compiler
# and no build cache to get stale.
#
#   ssh orin 'cd ~/ugv/lidar_slam && ./build.sh && ./selftest'
set -e
cd "$(dirname "$0")"

CC=${CC:-gcc}
# -O2 is worth several times -O0 on the CRC and the segmentation, and -O3 measured
# no better. No -ffast-math: the corner splitting compares deviations against a
# threshold and wants the rounding it was tuned against.
CFLAGS=${CFLAGS:--O2 -Wall -Wextra -std=c99 -fPIC}

echo "building with $($CC --version | head -1)"
$CC $CFLAGS -shared -o libslam2d.so slam2d.c -lm
$CC $CFLAGS -o selftest selftest.c slam2d.c -lm
echo "built libslam2d.so and selftest"

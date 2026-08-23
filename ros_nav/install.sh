#!/bin/sh
# Put ROS 2 Jazzy on the rover, from RoboStack. Idempotent -- run it again after
# changing the package list and it adds what is missing and nothing else.
#
#     scp -r ros_nav bpi-m4zero:~/ugv/
#     ssh bpi-m4zero 'sh ~/ugv/ros_nav/install.sh'
#
# No sudo anywhere in here, which is the whole reason it is conda and not apt.
# This board runs Debian trixie and ROS 2 Jazzy's own packages are built for
# Ubuntu noble, so there is nothing to `apt install`; building from source on
# four Cortex-A53 cores is most of a day. RoboStack publishes the same releases
# as conda packages with linux-aarch64 builds, which is a download and an unpack.
#
# It lands outside ~/ugv on purpose. ~/ugv is a copy of the repository and gets
# overwritten by every deploy; a three-gigabyte environment is an installed
# dependency, like the unpacked wheels in vendor/, and must not be in the path of
# an scp.

set -eu

HERE=$(cd "$(dirname "$0")" && pwd)
PREFIX=${PREFIX:-$HOME/miniforge3}
ENVNAME=${ENVNAME:-ros}
ENVDIR="$PREFIX/envs/$ENVNAME"
MFURL=${MFURL:-https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-aarch64.sh}

# What the rover needs and no more. `ros-base` rather than `desktop`: rviz and the
# demo packages are several hundred megabytes of things that would never be run on
# a headless board, and the map comes back as a file rather than as a window.
PKGS="
ros-jazzy-ros-base
ros-jazzy-slam-toolbox
ros-jazzy-navigation2
ros-jazzy-nav2-bringup
ros-jazzy-robot-localization
ros-jazzy-pointcloud-to-laserscan
ros-jazzy-rmw-cyclonedds-cpp
ros-jazzy-tf-transformations
ros-jazzy-teleop-twist-keyboard
ros-jazzy-nav2-map-server
colcon-common-extensions
pyserial
numpy
"

say() { echo "== $*"; }

# Prove the arithmetic before installing the thing that runs it. The self-test
# needs neither ROS nor a radio -- it is the sign of the steering, the binning of
# the scan and the handling of a counter that reset -- so it costs a second here
# and saves finding out on a rover that is already driving.
if [ -r "$HERE/selftest.py" ]; then
    if out=$(python3 "$HERE/selftest.py" 2>&1); then
        echo "== selftest: $(echo "$out" | tail -1)"
    else
        echo "$out" | grep -E 'FAIL|Error' || echo "$out" | tail -5
        echo "not installing"
        exit 1
    fi
fi

# --- miniforge ------------------------------------------------------------
if [ ! -x "$PREFIX/bin/conda" ]; then
    say "installing miniforge to $PREFIX"
    tmp=$(mktemp -d)
    trap 'rm -rf "$tmp"' EXIT
    curl -fL --retry 3 -o "$tmp/mf.sh" "$MFURL"
    sh "$tmp/mf.sh" -b -p "$PREFIX"
else
    say "miniforge already at $PREFIX"
fi

CONDA="$PREFIX/bin/conda"
# mamba if this miniforge shipped it; the solve is minutes rather than tens of
# minutes on this class of core, and every recent miniforge has it.
if [ -x "$PREFIX/bin/mamba" ]; then SOLVER="$PREFIX/bin/mamba"; else SOLVER="$CONDA"; fi

# RoboStack will not solve against defaults, and says so in a way that reads as a
# package that does not exist rather than as a channel problem.
"$CONDA" config --file "$PREFIX/.condarc" --set channel_priority flexible
for ch in robostack-jazzy conda-forge; do
    "$CONDA" config --file "$PREFIX/.condarc" --add channels "$ch" 2>/dev/null || true
done
"$CONDA" config --file "$PREFIX/.condarc" --remove channels defaults 2>/dev/null || true

# --- the environment ------------------------------------------------------
if [ ! -d "$ENVDIR" ]; then
    say "creating env '$ENVNAME' -- this is the slow part, ~2 GB"
    # shellcheck disable=SC2086
    "$SOLVER" create -y -n "$ENVNAME" -c robostack-jazzy -c conda-forge $PKGS
else
    say "env '$ENVNAME' exists; ensuring the package list is complete"
    # shellcheck disable=SC2086
    "$SOLVER" install -y -n "$ENVNAME" -c robostack-jazzy -c conda-forge $PKGS
fi

# --- how everything else finds it ----------------------------------------
# One file that every launcher, every supervisor and every interactive ssh
# session sources, so the environment lives in one place rather than being
# reconstructed slightly differently in each of them.
cat > "$HERE/env.sh" <<EOF
# Written by install.sh -- source this from **bash** to get ROS 2 on PATH.
#
# Bash and not /bin/sh, which on this board is dash. RoboStack's activation hooks
# are bash scripts and use \`source\`, which dash does not have, so sourcing this
# from a POSIX shell fails with "source: not found" -- a message that names
# neither this file nor the shell and reads as a missing package. Every launcher
# beside this one therefore starts \`#!/bin/bash\`.
export ROS_ENV_PREFIX="$ENVDIR"
# \`conda activate\`, not \`. setup.sh\`. RoboStack does not ship ROS's setup script
# as the way in: it puts the same work in the environment's own activation hooks
# under etc/conda/activate.d, so that AMENT_PREFIX_PATH and the library path are
# built from wherever the environment actually is. Sourcing setup.sh by itself
# leaves the environment's bin/ off PATH entirely, which fails as "python: command
# not found" and reads as a broken install rather than as the wrong door.
#
# The nounset dance around it is not superstition. RoboStack's own activation
# hook reads \$CONDA_BUILD without a default, so under \`set -u\` -- which every
# careful script in this repository uses -- activation dies with "CONDA_BUILD:
# parameter not set" and takes the whole launcher with it. Turned off across the
# activation only, and put back exactly as it was found.
case \$- in *u*) _had_nounset=1 ;; *) _had_nounset=0 ;; esac
set +u
. "$PREFIX/etc/profile.d/conda.sh"
conda activate "$ENVNAME"
[ "\$_had_nounset" = 1 ] && set -u
unset _had_nounset
# The DDS implementation. Cyclone rather than the default Fast-DDS because this
# is a wifi-only board on a home LAN: Fast-DDS's shared-memory transport writes
# files under /dev/shm for every participant and its discovery is chattier, and
# neither is what you want on a link that is the thing being debugged.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# One domain, so that a laptop on the same LAN running rviz does not have to be
# told anything, and a second rover would have to be.
export ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-42}
# There is deliberately no workspace to source. This repository's own nodes are
# plain scripts that the launch files name by absolute path, so there is nothing
# to \`colcon build\` and therefore nothing to forget to rebuild -- which is the
# trap lidar_slam/ already has one of, where a stale libslam2d.so is the rover
# running last week's code with this week's file sitting next to it on disk.
EOF
chmod 644 "$HERE/env.sh"

# The package cache is a second copy of everything that was just unpacked, and
# nothing reads it again unless a second environment is built. On a 29 GB card
# carrying a rover that is two gigabytes worth keeping.
say "dropping the package cache"
"$CONDA" clean -y --tarballs --index-cache >/dev/null 2>&1 || true

say "checking it imports"
# Under bash, for the reason env.sh gives: this script is /bin/sh and the
# activation hooks are not.
bash -c '. "$1/env.sh"
         python -c "import rclpy, sys; print(\"rclpy\", sys.version.split()[0])"
         printf "== packages: %s\n" "$(ros2 pkg list 2>/dev/null | grep -c .)"' _ "$HERE"
du -sh "$ENVDIR" 2>/dev/null | sed 's/^/== env size: /'
df -h / | tail -1 | sed 's/^/== disk: /'
say "done -- source ~/ugv/ros_nav/env.sh to use it"

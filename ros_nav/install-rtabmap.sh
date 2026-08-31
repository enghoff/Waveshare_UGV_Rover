#!/bin/sh
# Put RTAB-Map on the rover, from Ubuntu's own ROS 2 Jazzy packages.
#
#     ssh orin 'sudo -S -p "" sh ~/ugv/ros_nav/install-rtabmap.sh' < secrets/jetson-orin.key
#
# Idempotent: run it again and it adds what is missing and nothing else.
#
# ## Why this is apt and not install.sh's conda
#
# Every other ROS package on this rover comes from RoboStack, unpacked into
# ~/miniforge3/envs/ros with no sudo at all, and install.sh explains at length
# why that is worth having. RTAB-Map cannot come from there: **RoboStack
# publishes no rtabmap package**, for any platform, under any name. Checked on
# 2026-08-31 against robostack-jazzy and conda-forge together --
# `mamba repoquery search "rtabmap*"` returns nothing.
#
# The alternatives were building RTAB-Map from source into the conda environment
# -- PCL, g2o, Ceres and OpenCV all resolved a second time, hours of compiling on
# six cores, and a build step somebody has to remember -- or taking the prebuilt
# arm64 package that already exists. Ubuntu 24.04 is exactly what packages.ros.org
# builds Jazzy for, so the second is a download.
#
# ## Two ROS installations, talking over DDS
#
# What this leaves is /opt/ros/jazzy beside ~/miniforge3/envs/ros. They are never
# mixed inside one process -- that would put two builds of rclcpp and two of
# libstdc++ on one library path -- and nothing here changes the conda
# environment. RTAB-Map runs from /opt/ros/jazzy with its own environment (see
# run_rtabmap.sh) and reaches the rest of the stack the way any other machine's
# ROS node would: over DDS. Both sides are Jazzy, so the message definitions are
# the same ones, and both are pointed at CycloneDDS on loopback.
#
# ## GPU
#
# There is none here, and no configuration switch adds one. These packages are
# built against Ubuntu's stock libopencv-*406t64, which is compiled without CUDA,
# and against no libtorch at all -- so RTAB-Map's GPU code paths (`ORB/Gpu`,
# `FAST/Gpu`, `SURF/GpuVersion`, SuperPoint) are not merely switched off, they are
# not in the binary. The board would not help them anyway: it has the driver's
# libcuda.so and no CUDA toolkit, so nothing on it can compile CUDA today.
#
# That costs this rover nothing, because those switches only ever accelerate
# *visual* feature extraction, and the configuration in config/rtabmap.yaml is
# lidar-only -- no image reaches RTAB-Map for a GPU to work on. Making the GPU
# relevant is a different project: the OAK-D's colour and depth onto ROS topics,
# a CUDA toolkit, OpenCV rebuilt with CUDA, and RTAB-Map rebuilt against that.
# config/rtabmap.yaml says which parameters would come alive if it were done.

set -eu

[ "$(id -u)" = 0 ] || { echo "run this with sudo"; exit 1; }

KEYRING=/usr/share/keyrings/ros-archive-keyring.gpg
LIST=/etc/apt/sources.list.d/ros2.list
KEYURL=https://raw.githubusercontent.com/ros/rosdistro/master/ros.key

PKGS="ros-jazzy-ros-base ros-jazzy-rtabmap-slam ros-jazzy-rtabmap-util ros-jazzy-rmw-cyclonedds-cpp"

say() { echo "== $*"; }

# --- the repository -------------------------------------------------------
if [ ! -s "$KEYRING" ]; then
    say "fetching the ROS archive key"
    # mktemp, and never a fixed name under /tmp. This kernel runs
    # `fs.protected_regular = 2`, which stops even root from opening an existing
    # file for writing in a sticky world-writable directory when somebody else
    # owns it. A hardcoded /tmp/ros.key left behind by a hand-run of this as
    # `jetson` therefore makes the sudo run fail -- and it fails as
    # `curl: (23) Failure writing output to destination`, which reads as a full
    # disk rather than as a permission the root account does not have.
    tmpkey=$(mktemp)
    trap 'rm -f "$tmpkey"' EXIT INT TERM
    # -4 for the same reason as below: this board resolves several of these
    # hosts to IPv6 addresses it has no route to.
    curl -4 -fsSL --retry 3 -o "$tmpkey" "$KEYURL"
    gpg --dearmor < "$tmpkey" > "$KEYRING"
    chmod 644 "$KEYRING"
else
    say "ROS archive key already at $KEYRING"
fi

# **http, not https, and that is deliberate.** packages.ros.org currently serves
# a certificate for *.osuosl.org, which is its mirror's name and not its own, so
# every TLS client on this board rejects it:
#
#     curl: (60) SSL: no alternative certificate subject name matches
#           target host name 'packages.ros.org'
#
# That is upstream's misconfiguration and not this rover's, and apt over http
# loses nothing to it: `signed-by` above means every package is still verified
# against the ROS release key before it is unpacked, which is the guarantee that
# actually matters. Change this line back to https when the certificate is fixed.
want="deb [arch=arm64 signed-by=$KEYRING] http://packages.ros.org/ros2/ubuntu noble main"
if [ ! -f "$LIST" ] || [ "$(cat "$LIST")" != "$want" ]; then
    say "writing $LIST"
    echo "$want" > "$LIST"
    chmod 644 "$LIST"
    NEEDUPDATE=yes
else
    say "$LIST already correct"
    NEEDUPDATE=no
fi

# --- the packages ---------------------------------------------------------
missing=
for p in $PKGS; do
    dpkg-query -W -f='${Status}' "$p" 2>/dev/null | grep -q "^install ok installed$" || missing="$missing $p"
done

if [ -n "$missing" ]; then
    say "installing:$missing"
    apt-get update -o Dir::Etc::sourcelist="$LIST" -o Dir::Etc::sourceparts="-" \
                   -o APT::Get::List-Cleanup="0" >/dev/null 2>&1 || apt-get update >/dev/null
    # shellcheck disable=SC2086
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends $missing
else
    if [ "$NEEDUPDATE" = yes ]; then apt-get update >/dev/null 2>&1 || true; fi
    say "all packages already installed"
fi

# --- what was actually installed -----------------------------------------
# Printed rather than assumed, because the interesting facts about this install
# are the ones a version number does not carry.
say "version: $(dpkg-query -W -f='${Version}' ros-jazzy-rtabmap 2>/dev/null || echo MISSING)"
say "node:    $([ -x /opt/ros/jazzy/lib/rtabmap_slam/rtabmap ] && echo present || echo MISSING)"
# Deliberately not `rtabmap --version`. That name is the *standalone GUI*, which
# is a different program in a different directory, needs a display, and fails
# here with a Qt xcb error that reads as a broken install. The ROS node is the
# binary above and takes no --version.
say "cyclone: $([ -e /opt/ros/jazzy/lib/librmw_cyclonedds_cpp.so ] && echo present || echo MISSING)"
# The GPU claim in the header, checked rather than repeated. If a future build
# does link CUDA this line is where it will show up.
cuda=$(ldd /opt/ros/jazzy/lib/librtabmap_core.so* 2>/dev/null | grep -ciE 'libcud|libnpp|libtorch' || true)
say "cuda in librtabmap_core: ${cuda:-0} linked libraries (0 = CPU-only, as expected)"
df -h / | tail -1 | sed 's/^/== disk: /'
say "done -- run_rtabmap.sh is what starts it"

#!/bin/sh
# Build liboak.so on the machine that will run it. The rover's Pi is armv6 and
# nothing else here is, so there is no cross-compiler to hand and this is a
# native build -- about three minutes on that host, most of it XLink.
#
#     ssh rpi '~/ugv/oak_detect/build.sh && ~/ugv/oak_detect/selftest.py'
#
# What goes in is Intel's own XLink and mvnc from OpenVINO 2021.4.2, vendored
# under vendor/movidius/, plus one file of ours to decode JPEG. Together they
# are the whole stack: boot the camera's VPU over USB, push a compiled graph
# into it, write frames to one FIFO and read detections back from another.
#
# Neither -dev package is installed on the Pi and sudo there wants a password,
# so this links the versioned runtime libraries by path and carries its own
# copy of libusb.h. That is also why the TurboJPEG prototypes are written out
# in oakjpeg.c rather than included from a header.
set -e

DIR=$(cd "$(dirname "$0")" && pwd)
MV="$DIR/vendor/movidius"
OBJ="$DIR/build"

find_lib() {
    for path in /usr/lib/*/"$1" /usr/lib/"$1" /usr/lib64/"$1"; do
        [ -e "$path" ] && { echo "$path"; return 0; }
    done
    echo "$0: cannot find $1 -- is the runtime package installed?" >&2
    return 1
}

USB=$(find_lib libusb-1.0.so.0)
JPEG=$(find_lib libturbojpeg.so.0)

# __PC__ picks the host side of code shared with the device firmware, and
# USE_USB_VSC the USB transport rather than PCIe; both come straight from
# OpenVINO's own CMakeLists, as does XLINK_USE_BUS.
DEFS="-D__PC__ -DHAVE_STRUCT_TIMESPEC -D_CRT_SECURE_NO_WARNINGS -DUSE_USB_VSC -DXLINK_USE_BUS"
INCS="-I$MV/XLink/shared/include -I$MV/XLink/pc/protocols -I$MV/XLink/pc/MacOS"
INCS="$INCS -I$MV/mvnc/include -I$MV/mvnc/include/watchdog -I$DIR/vendor/libusb"
WARN="-Wall -Wno-unused-variable -Wno-unused-but-set-variable -Wno-sign-compare"
OPT="-O2 -fPIC -pthread"

# Objects are kept and only rebuilt when their source is newer. A first build is
# about thirteen minutes on the Pi, and nearly all of it is Intel's code, which
# never changes -- so touching oakjpeg.c and rebuilding should cost seconds, not
# another thirteen minutes. `build.sh --clean` starts over.
[ "$1" = "--clean" ] && rm -rf "$OBJ"
mkdir -p "$OBJ"

sources=$(ls "$MV"/XLink/pc/*.c "$MV"/XLink/pc/protocols/*.c \
             "$MV"/XLink/shared/src/*.c "$MV"/XLink/pc/MacOS/pthread_semaphore.c \
             "$MV"/mvnc/src/*.c "$DIR"/oakjpeg.c)
count=$(echo "$sources" | wc -l)
n=0
built=0
for src in $sources; do
    n=$((n + 1))
    obj="$OBJ/$(basename "$src" .c).o"
    [ -e "$obj" ] && [ "$obj" -nt "$src" ] && continue
    printf '\r  compiling %d/%d  %-28s' "$n" "$count" "$(basename "$src")"
    gcc $OPT $WARN $DEFS $INCS -c "$src" -o "$obj"
    built=$((built + 1))
done
for src in "$MV"/mvnc/src/watchdog/*.cpp; do
    obj="$OBJ/$(basename "$src" .cpp).o"
    [ -e "$obj" ] && [ "$obj" -nt "$src" ] && continue
    printf '\r  compiling      %-28s' "$(basename "$src")"
    g++ $OPT -std=c++11 $DEFS $INCS -c "$src" -o "$obj"
    built=$((built + 1))
done
printf '\r  %d of %d objects rebuilt, linking%-20s\n' "$built" "$((count + 2))" ''

# g++ links, because the watchdog is C++. libatomic is not always a separate
# library on this toolchain, so it is added only when it exists.
ATOMIC=""
find_lib libatomic.so.1 >/dev/null 2>&1 && ATOMIC="-latomic"
g++ -shared -o "$DIR/liboak.so" "$OBJ"/*.o "$USB" "$JPEG" -pthread -ldl $ATOMIC

ls -l "$DIR/liboak.so"

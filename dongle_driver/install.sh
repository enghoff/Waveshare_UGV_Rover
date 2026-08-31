#!/bin/sh
# Build and install the driver for the rover's USB radio, patched so that the
# radio survives the USB errors it produces.
#
#     python deploy/deploy.py --only dongle_driver --system
#
# **Why this exists at all.** NVIDIA's L4T kernel is built with CONFIG_RTL8XXXU
# unset and ships no drivers/net/wireless/realtek directory, so the dongle sits
# on the USB bus with no driver bound and no interface. The driver it needs is
# in-tree and has supported this exact device (0bda:f179, RTL8188FTV) for
# several releases; it simply was not compiled. So it is built out of tree from
# the kernel.org sources for the version this kernel is based on, and registered
# with DKMS so a JetPack kernel update rebuilds it instead of silently dropping
# it. The kernel does not enforce module signatures, which is why an unsigned
# out-of-tree module loads at all.
#
# **Why it is patched.** See rx-urb-recovery.patch. In short: the stock driver
# retires a receive buffer for good every time one completes with a USB error,
# and it only has thirty-two, so a device that produces the occasional transient
# error eventually receives nothing at all while still reporting itself
# associated and up. Only reloading the module brings it back.
#
# Idempotent. Run it again after a kernel update, or after changing the patch.

set -eu

HERE=$(cd "$(dirname "$0")" && pwd)
# The kernel this module has to load into, and the kernel.org release it is
# based on -- 6.8.12-1021-tegra is 6.8.12 with NVIDIA's patches on top, and the
# driver sources are identical.
KVER=$(uname -r)
BASE=$(echo "$KVER" | sed 's/-.*//')
NAME=rtl8xxxu
# Not the bare kernel version: this is the kernel's driver plus the patch beside
# this script, and `dkms status` should say so rather than implying stock.
VERSION="$BASE-ugv1"
SRC=/usr/src/$NAME-$VERSION
MIRROR=${MIRROR:-https://git.kernel.org/pub/scm/linux/kernel/git/stable/linux.git/plain/drivers/net/wireless/realtek/rtl8xxxu}
FILES="rtl8xxxu_core.c rtl8xxxu_8188e.c rtl8xxxu_8188f.c rtl8xxxu_8192c.c
       rtl8xxxu_8192e.c rtl8xxxu_8192f.c rtl8xxxu_8710b.c rtl8xxxu_8723a.c
       rtl8xxxu_8723b.c rtl8xxxu.h rtl8xxxu_regs.h"

[ "$(id -u)" = 0 ] || { echo "run this with sudo"; exit 1; }

# The firmware trap, which costs an afternoon if it is not known. This kernel is
# built with CONFIG_FW_LOADER_COMPRESS unset while Ubuntu 24.04 ships every blob
# in linux-firmware as .zst, so the driver finds the device, identifies it
# correctly, and then fails with "Direct firmware load for
# rtlwifi/rtl8188fufw.bin failed with error -2" beside a .zst that plainly
# exists. Read "no such file" as "the file is there, compressed".
FW=/lib/firmware/rtlwifi/rtl8188fufw.bin
if [ ! -r "$FW" ] && [ -r "$FW.zst" ]; then
    zstd -q -d "$FW.zst" -o "$FW"
    echo "firmware: decompressed $(basename "$FW"), which this kernel cannot read packed"
fi
[ -r "$FW" ] || echo "firmware: $FW is missing and there is no .zst to unpack" >&2

# --- the source tree ------------------------------------------------------
#
# Fetched rather than kept in this repository: it is 400 kB of somebody else's
# GPL driver, it must match whatever kernel the rover is running after the next
# JetPack update, and the only part of it that is ours is the patch.
if [ ! -d "$SRC" ] || [ -n "${REFETCH:-}" ]; then
    echo "fetching the $BASE driver sources"
    tmp=$(mktemp -d)
    for f in $FILES; do
        curl -sf --max-time 120 "$MIRROR/$f?h=v$BASE" -o "$tmp/$f" ||
            { echo "could not fetch $f for v$BASE" >&2; rm -rf "$tmp"; exit 1; }
    done
    rm -rf "$SRC"
    mkdir -p "$SRC"
    cp "$tmp"/* "$SRC/"
    rm -rf "$tmp"
fi

cat > "$SRC/Makefile" <<'MAKEFILE'
# SPDX-License-Identifier: GPL-2.0-only
obj-$(CONFIG_RTL8XXXU)	+= rtl8xxxu.o

rtl8xxxu-y	:= rtl8xxxu_core.o rtl8xxxu_8192e.o rtl8xxxu_8723b.o \
		   rtl8xxxu_8723a.o rtl8xxxu_8192c.o rtl8xxxu_8188f.o \
		   rtl8xxxu_8188e.o rtl8xxxu_8710b.o rtl8xxxu_8192f.o
MAKEFILE

cat > "$SRC/dkms.conf" <<CONF
PACKAGE_NAME="$NAME"
PACKAGE_VERSION="$VERSION"
BUILT_MODULE_NAME[0]="$NAME"
DEST_MODULE_LOCATION[0]="/kernel/drivers/net/wireless/realtek/rtl8xxxu"
MAKE[0]="make -C \${kernel_source_dir} M=\${dkms_tree}/\${PACKAGE_NAME}/\${PACKAGE_VERSION}/build CONFIG_RTL8XXXU=m modules"
AUTOINSTALL="yes"
CONF

# --- our one change -------------------------------------------------------
PATCH=$HERE/rx-urb-recovery.patch
if patch -d "$SRC" -p1 --dry-run --reverse --force < "$PATCH" > /dev/null 2>&1; then
    echo "patch: already applied"
else
    patch -d "$SRC" -p1 < "$PATCH"
    echo "patch: applied"
fi

# --- build ----------------------------------------------------------------
#
# The stock DKMS entry is removed rather than left beside this one. Two modules
# of the same name in the tree is a coin toss over which one modprobe finds, and
# the wrong side of that coin is the fault this patch exists to fix, come back
# looking like a regression.
dkms status "$NAME" | cut -d, -f1 | tr -d ' ' | while IFS=/ read -r pkg ver; do
    [ "$pkg" = "$NAME" ] || continue
    [ "$ver" = "$VERSION" ] && continue
    echo "removing the earlier $pkg/$ver"
    dkms remove "$pkg/$ver" --all > /dev/null 2>&1 || true
    rm -rf "/usr/src/$pkg-$ver"
done

dkms status "$NAME/$VERSION" | grep -q . && dkms remove "$NAME/$VERSION" --all > /dev/null 2>&1 || true
dkms add "$SRC" > /dev/null
dkms build "$NAME/$VERSION" -k "$KVER" > /tmp/dkms-$NAME.log 2>&1 || {
    echo "build failed; the tail of /tmp/dkms-$NAME.log:" >&2
    tail -20 /tmp/dkms-$NAME.log >&2
    exit 1
}
dkms install "$NAME/$VERSION" -k "$KVER" > /dev/null
echo "dkms: $(dkms status "$NAME")"

# --- load it --------------------------------------------------------------
#
# Reloading takes this radio down for a second or two, and since `wifi_dual` was
# armed here that is no longer certainly the spare: the manager gives the traffic
# to whichever radio scores better, and the dongle wins whenever it is the one
# sitting next to a router. So this can take down the link the command arrives
# over, and what makes it survivable rather than safe is the manager -- it notices
# the radio is gone and moves the traffic and the service address to the other
# one, measured at about twenty seconds. Prefer to run it when the onboard radio
# is active; check with `wifi_ctl.sh status` first if it matters.
modprobe -r "$NAME" 2>/dev/null || true
modprobe "$NAME"
sleep 5
echo "loaded: $(modinfo -F filename "$NAME")"
echo "recovery: rx_urb_recover=$(cat /sys/module/$NAME/parameters/rx_urb_recover)"

# --- the keeper -----------------------------------------------------------
#
# The backstop for whatever the patch does not cover. Proved on this machine
# first, because it needs no radio and takes a second: a copy that arrived with
# CRLF line endings, or half written, fails here rather than at three in the
# morning on a rover whose spare radio it was supposed to be watching.
if [ -r "$HERE/selftest.sh" ]; then
    if out=$(sh "$HERE/selftest.sh" 2>&1); then
        echo "keeper selftest: $(echo "$out" | tail -1)"
    else
        echo "$out" | grep -E 'FAIL|failed'
        echo "not installing the keeper"
        exit 1
    fi
fi

install -m 755 "$HERE/keeper.sh" /usr/local/sbin/dongle-keeper.sh
install -m 644 "$HERE/dongle-keeper.service" "$HERE/dongle-keeper.timer" \
    /etc/systemd/system/
systemctl daemon-reload

# `KEEPER=off` is for one situation and it is worth naming: a rover being worked
# on, where a driver reload arriving in the middle of a measurement would make
# the measurement a lie.
if [ "${KEEPER:-on}" = off ]; then
    echo "keeper: left disabled (KEEPER=off)"
    echo "  arm it with: systemctl enable --now dongle-keeper.timer"
else
    systemctl enable --now dongle-keeper.timer > /dev/null 2>&1
    echo "keeper: $(systemctl is-active dongle-keeper.timer) and $(systemctl is-enabled dongle-keeper.timer)"
    systemctl list-timers --no-pager dongle-keeper.timer | sed -n 2p
fi

echo "--- one dry run"
/usr/local/sbin/dongle-keeper.sh -n || echo "(dry run exited $?)"

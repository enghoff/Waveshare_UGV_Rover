#!/bin/sh
# A certificate for the drive console, so that a browser will hand it a microphone.
#
#     ssh bpi-m4zero ~/ugv/drive_web/make_cert.sh
#     ssh bpi-m4zero ~/ugv/drive_web/make_cert.sh --force     # start again
#
# **This exists because of one browser rule, not because anything here is
# secret.** `getUserMedia` -- the call that opens the microphone -- is refused
# outside a secure context, and a secure context is HTTPS or localhost and
# nothing else. A phone on the sofa is neither, so the console has to speak TLS
# before it can be talked to. Nothing else about this console is protected: there
# is still no password on it, and anyone on the LAN who can load the page can
# drive the rover.
#
# **What it writes lives outside ~/ugv on purpose.** A deploy copies over
# ~/ugv/drive_web/ and a private key that a deploy could overwrite -- or worse,
# that a deploy could carry *off* the rover into the repository -- is a key in
# the wrong place. It goes to ~/.ugv/tls/ with the conda environment's reasoning:
# things a deploy must not touch do not live where a deploy lands.
#
# **A tiny CA, and a certificate under it, rather than one self-signed leaf.**
# The leaf alone works -- click through the warning once per browser and the
# microphone is available, because a bypassed certificate error is still a secure
# context. But a self-signed leaf cannot be *installed* as trusted on Android at
# all: that dialogue takes CA certificates and refuses everything else. With a CA
# here, `console-ca.crt` can be copied to a phone or a laptop once and the console
# gets an ordinary padlock and no warning; skip that and nothing is lost but the
# clicking. The CA's key never leaves this board and signs nothing but this.
set -e

DIR="${HOME}/.ugv/tls"
DAYS_CA=3650
# Not 3650. Safari refuses a certificate valid for more than 825 days even from a
# root it trusts, and a leaf nobody can load is a worse failure than a renewal.
DAYS_LEAF=820
# How long to wait for the wifi to have an address before giving up on naming it.
# The board is not on the network at all for the first half-minute after a boot.
WAIT_ADDRESS=90
FORCE=""
[ "$1" = "--force" ] && FORCE=1

mkdir -p "$DIR"
chmod 700 "$DIR"

# Every name and address this board answers to. A certificate is checked against
# what was *typed*, so the address matters as much as the name: `bpi-m4zero.local`
# from a laptop and `192.168.1.47` from a phone are two different subjects and
# both have to be in here. The board is wifi-only and its address can move, which
# is the one thing that dates this file -- run it again when it does.
host="$(hostname)"

lan_addresses() {
    for address in $(hostname -I 2>/dev/null); do
        case "$address" in
            *:*) continue ;;                  # IPv6; the console is not on one
            127.*) continue ;;
        esac
        echo "$address"
    done
}

# **Wait for an address before deciding anything**, because at boot there is not
# one yet. run_drive_web.sh calls this from a @reboot crontab entry, which fires
# long before DHCP has finished on a wifi-only board, and `hostname -I` is empty
# for the first half-minute. A certificate built in that window names nothing but
# loopback -- and the freshness check below is computed from the same empty list,
# so every boot afterwards agrees that the loopback-only certificate is current
# and never replaces it. The console is then reachable by name and by no address
# at all, which quietly makes an mDNS lookup the only way in. That is not a
# theory: the certificate on this board had been in exactly that state.
waited=0
while [ -z "$(lan_addresses)" ] && [ "$waited" -lt "$WAIT_ADDRESS" ]; do
    sleep 3
    waited=$((waited + 3))
done
[ "$waited" -gt 0 ] && echo "waited ${waited}s for an address"

# Still nothing: keep whatever is there rather than replacing a certificate that
# names yesterday's address with one that names none.
if [ -z "$(lan_addresses)" ] && [ -f "$DIR/console.crt" ]; then
    echo "no address after ${WAIT_ADDRESS}s; leaving $DIR/console.crt as it is"
    exit 0
fi

names="DNS:${host},DNS:${host}.local,DNS:localhost"
ips="IP:127.0.0.1"
for address in $(lan_addresses); do
    ips="${ips},IP:${address}"
done

# The rover's service address, named whether or not it is held at this moment.
# `hostname -I` lists it only once NetworkManager has brought the network up
# with it, and that is exactly the problem: this script runs from
# `run_drive_web.sh` at boot, so a certificate written before the radio has
# associated would name every address the board has *except* the one that is
# meant to be the stable way in. It is a constant of this board rather than a
# lease -- see wifi_roam/install-profiles.sh, which puts it on every profile --
# so it goes in unconditionally. Override with SERVICE_IP= for a board that does
# not have one.
SERVICE_IP=${SERVICE_IP-192.168.1.80}
case ",${ips}," in
    *",IP:${SERVICE_IP},"*) ;;
    *) [ -n "$SERVICE_IP" ] && ips="${ips},IP:${SERVICE_IP}" ;;
esac
# Anything else worth covering -- a hostname the router hands out, say -- can be
# given on the command line as extra SANs.
for extra in "$@"; do
    case "$extra" in
        --force) continue ;;
        [0-9]*.[0-9]*.[0-9]*.[0-9]*) ips="${ips},IP:${extra}" ;;
        *) names="${names},DNS:${extra}" ;;
    esac
done
SAN="${names},${ips}"

fresh() {
    # Valid for another month, and still naming everything it should. Either
    # answer being no is a reason to make a new one.
    [ -f "$DIR/console.crt" ] && [ -f "$DIR/console.key" ] || return 1
    openssl x509 -in "$DIR/console.crt" -noout -checkend 2592000 >/dev/null 2>&1 || return 1
    have="$(openssl x509 -in "$DIR/console.crt" -noout -ext subjectAltName 2>/dev/null)"
    for want in $(echo "$SAN" | tr ',' ' '); do
        case "$have" in *"$want"*) ;; *) return 1 ;; esac
    done
    return 0
}

if [ -z "$FORCE" ] && fresh; then
    echo "certificate is current: $DIR/console.crt"
    openssl x509 -in "$DIR/console.crt" -noout -subject -enddate -ext subjectAltName
    exit 0
fi

umask 077
if [ -n "$FORCE" ] || [ ! -f "$DIR/console-ca.key" ]; then
    # P-256 rather than RSA: a handshake on four Cortex-A53 cores is cheaper, the
    # files are a tenth the size, and every browser made this decade takes it.
    openssl ecparam -name prime256v1 -genkey -noout -out "$DIR/console-ca.key"
    openssl req -x509 -new -key "$DIR/console-ca.key" -sha256 -days "$DAYS_CA" \
        -out "$DIR/console-ca.crt" -subj "/CN=UGV rover console CA ($host)" \
        -addext "basicConstraints=critical,CA:TRUE,pathlen:0" \
        -addext "keyUsage=critical,keyCertSign,cRLSign"
fi

openssl ecparam -name prime256v1 -genkey -noout -out "$DIR/console.key"
openssl req -new -key "$DIR/console.key" -out "$DIR/console.csr" \
    -subj "/CN=${host}"
# An extension file rather than bash's <(...): this runs under dash on the board,
# which has no process substitution, and the failure would be an empty SAN -- a
# certificate Chrome rejects outright with ERR_CERT_COMMON_NAME_INVALID, because
# a common name on its own has meant nothing to it since 2017.
cat > "$DIR/console.ext" <<EXT
basicConstraints=critical,CA:FALSE
keyUsage=critical,digitalSignature,keyEncipherment
extendedKeyUsage=serverAuth
subjectAltName=${SAN}
EXT
openssl x509 -req -in "$DIR/console.csr" -CA "$DIR/console-ca.crt" \
    -CAkey "$DIR/console-ca.key" -CAcreateserial -days "$DAYS_LEAF" -sha256 \
    -extfile "$DIR/console.ext" -out "$DIR/console.crt" 2>/dev/null
rm -f "$DIR/console.csr" "$DIR/console.ext"
chmod 600 "$DIR/console.key" "$DIR/console-ca.key"
chmod 644 "$DIR/console.crt" "$DIR/console-ca.crt"

echo "wrote $DIR/console.crt"
openssl x509 -in "$DIR/console.crt" -noout -subject -enddate -ext subjectAltName
echo
echo "the console serves this as soon as it is restarted:"
echo "    ~/ugv/drive_web/restart.sh"
echo "to lose the browser warning, install $DIR/console-ca.crt on the device"

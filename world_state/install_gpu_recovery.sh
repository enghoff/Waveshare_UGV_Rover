#!/bin/sh
# Give the rover the one privileged thing the perception sidecar needs: a way to
# reload the GPU driver on a boot that came up without a GPU. Idempotent -- run it
# again after changing gpu_ctl.sh.
#
#     ssh orin 'sudo ~/ugv/world_state/install_gpu_recovery.sh'
#
# There is nothing to arm and no daemon to start. `run_perception.sh` checks for a
# GPU before each start of the server and calls the helper when there is none, so
# the recovery happens inside the restart loop that was already there. Without
# this script the check still runs and says in the log that the helper is not
# installed, which is a rover that needs a person rather than one that is quietly
# broken.
#
# Why it is a separate script from `install_perception.sh`: that one fetches five
# gigabytes of models over the rover's wifi and is a thing you do once on a new
# board. This is a hundred lines of shell and a sudo rule, and wants running again
# whenever the helper changes.

set -eu

HERE=$(cd "$(dirname "$0")" && pwd)

[ "$(id -u)" = 0 ] || { echo "run this with sudo"; exit 1; }

[ -r "$HERE/gpu_ctl.sh" ] || {
    echo "no gpu_ctl.sh beside this script; nothing to install" >&2
    exit 1
}

# Prove the helper before making it the one that runs. `status` needs no
# privilege and no GPU -- it answers about whichever node it is pointed at -- so
# this catches a copy that arrived with CRLF line endings or half written, here
# rather than at three in the morning on a rover that has stopped seeing.
if NODE=/nonexistent sh "$HERE/gpu_ctl.sh" status > /dev/null 2>&1; then
    echo "gpu_ctl.sh claims a GPU at a path that cannot exist; not installing" >&2
    exit 1
fi
if ! sh "$HERE/gpu_ctl.sh" status > /dev/null 2>&1; then
    echo "note: this board has no GPU up right now -- installing anyway"
fi

# --- the sudo rule ---------------------------------------------------------
#
# The rule goes down before the script it names, and not after. The other way
# round leaves a few seconds in which the helper exists and may not be run, and a
# sidecar that restarts in that window is told a password is required -- which is
# true, and says nothing about what is actually wrong. A rule naming a script that
# is not there yet fails instead as "not installed", which is the message that
# sends somebody to the right place.
#
# Written through a temporary file and checked before it is put in place. A
# malformed file in /etc/sudoers.d makes *every* sudo on the box fail, including
# the one that would be used to repair it.
rule=/etc/sudoers.d/world-state-gpu
tmp=$(mktemp)
printf '%s ALL=(root) NOPASSWD: /usr/local/sbin/gpu_ctl.sh\n' "${SUDO_USER:-jetson}" \
    > "$tmp"
if visudo -c -q -f "$tmp"; then
    install -m 440 -o root -g root "$tmp" "$rule"
    echo "sudo rule: $(cat "$rule")"
else
    echo "refusing to install a sudoers rule that visudo will not accept" >&2
    rm -f "$tmp"
    exit 1
fi
rm -f "$tmp"

install -m 755 "$HERE/gpu_ctl.sh" /usr/local/sbin/gpu_ctl.sh
echo "helper: /usr/local/sbin/gpu_ctl.sh"

# Prove the whole path the sidecar will take, as the account it runs as, because
# the rule and the helper being in place separately is not the same as sudo
# granting this user that helper without a password.
if sudo -n -u "${SUDO_USER:-jetson}" sudo -n /usr/local/sbin/gpu_ctl.sh status \
        > /dev/null 2>&1; then
    echo "checked: ${SUDO_USER:-jetson} may run it without a password, and the GPU is up"
else
    echo "checked: ${SUDO_USER:-jetson} may run it without a password" \
         "(it reports no GPU, which is what recover is for)"
fi

echo "--- installed; run_perception.sh uses it at its next restart"

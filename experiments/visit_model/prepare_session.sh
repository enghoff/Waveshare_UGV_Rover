#!/bin/bash
# After a staged session: fetch what session_collect.sh left on the rover and
# rebuild the benchmark's looks with it, ready for labelling and the frozen
# versions. Run here, from this directory.
#
#     bash prepare_session.sh NAME
#
# Leaves tmp/NAME/ (recording arrays, store, frames, session record) and
# tmp/visit-model/looks-NAME.npz with wall places, colour and the learned
# likeness frozen in metric-1015.npz. Written after M5 session 5, whose
# pipeline was typed step by step (2026-10-16).
set -e
NAME="$1"
T="D:/UGV Rover/tmp"
D="$T/$NAME"
mkdir -p "$D/frames"
scp -q "orin:/tmp/$NAME/*" "$D/"
tar --force-local -xf "$D/frames.tar" -C "$D/frames"
RECS=("$T/ww-recordings/shakedown.npz" "$T"/session?-10??/session?-10??.npz "$T"/ww-recordings/m5-*.npz "$T"/ww-recordings/1010/*.npz)
LOOKS="$T/visit-model/looks-$NAME.npz"
python data.py "$D/world-after.db" "$LOOKS" "${RECS[@]}"
python wallplace.py "$LOOKS" "$T/visit-model/walls.npz"
python extend_looks.py "$LOOKS" "$D/colour.npz" metric-1015.npz
echo "recordings used: ${#RECS[@]}"

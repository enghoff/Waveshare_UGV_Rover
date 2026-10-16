#!/bin/bash
# Version 3 development: every look through session 5, scored on every session with wrong merges counted across sessions.
T="D:/UGV Rover/tmp"
OUT="$T/visit-model/${1:-v3}.json"
python model.py --test "$T/visit-model/looks5.npz" "$OUT" "$T/ww-recordings/shakedown.npz" "$T/session1-1014/session1-1014.npz" "$T/session2-1014/session2-1014.npz" "$T/session3-1015/session3-1015.npz" "$T/session4-1015/session4-1015.npz" "$T/session5-1016/session5-1016.npz" "$T"/ww-recordings/m5-*.npz "$T"/ww-recordings/1010/*.npz > /dev/null || exit 1
for role in development test "session 5"; do
  python score.py --role "$role" "$T/visit-model/looks5.npz" "$OUT" | grep -E "wrong merge|strict=|: \{'trials" | grep -v "armchair  *\(kept\|split\|unresolved\)" | sed -E "s/ before .*//; s/'stayed_put_keys'.*//" | cut -c1-200
done

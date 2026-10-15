#!/bin/bash
# Version 2 development: every look through session 4 (sessions 3 and 4 are seen now), scored on all five.
T="D:/UGV Rover/tmp"
OUT="$T/visit-model/${1:-v2}.json"
python model.py --test "$T/visit-model/looks.npz" "$OUT" "$T/ww-recordings/shakedown.npz" "$T/session1-1014/session1-1014.npz" "$T/session2-1014/session2-1014.npz" "$T/session3-1015/session3-1015.npz" "$T/session4-1015/session4-1015.npz" "$T"/ww-recordings/m5-*.npz "$T"/ww-recordings/1010/*.npz && python score.py --test "$T/visit-model/looks.npz" "$OUT" | tail -${2:-2}

#!/bin/bash
# Development run: the model over looks up to the end of session 2, scored on development only.
T="D:/UGV Rover/tmp"
OUT="$T/visit-model/${1:-dev}.json"
python model.py "$T/visit-model/looks.npz" "$OUT" "$T/ww-recordings/shakedown.npz" "$T/session1-1014/session1-1014.npz" "$T/session2-1014/session2-1014.npz" "$T"/ww-recordings/m5-*.npz "$T"/ww-recordings/1010/*.npz && python score.py "$T/visit-model/looks.npz" "$OUT" | tail -${2:-1}

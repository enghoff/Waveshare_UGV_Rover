#!/bin/bash
# Honesty over draws: a frozen version over a looks file and five draws of it,
# each without a random 1% of the pictures that hold no labelled region
# (draws.py), scored by compare.py. Version 4's test counts a wrong merge in
# any of the six runs.
#
#     bash run_draws.sh LOOKS.npz TAG COMMIT
#
# Writes tmp/visit-model/TAG-drawN.npz and TAG-full.json, TAG-drawN.json.
set -e
LOOKS="$1"
TAG="$2"
COMMIT="$3"
T="D:/UGV Rover/tmp"
V="$T/visit-model"
RECS=("$T/ww-recordings/shakedown.npz" "$T"/session?-10??/session?-10??.npz "$T"/drives?-10??/drives?-10??.npz "$T"/ww-recordings/m5-*.npz "$T"/ww-recordings/1010/*.npz)
python draws.py "$LOOKS" "$V/$TAG" 5
git show "$COMMIT:experiments/visit_model/model.py" > "model_frozen_$TAG.py"
PYTHONHASHSEED=0 python "model_frozen_$TAG.py" --test "$LOOKS" "$V/$TAG-full.json" "${RECS[@]}" > "$V/$TAG-full.log" 2>&1 &
for s in 0 1 2 3 4; do
  PYTHONHASHSEED=0 python "model_frozen_$TAG.py" --test "$V/$TAG-drop$s.npz" "$V/$TAG-draw$s.json" "${RECS[@]}" > "$V/$TAG-draw$s.log" 2>&1 &
done
wait
rm -f "model_frozen_$TAG.py"
echo "=== all pictures"
python compare.py "$LOOKS" deployed run="$V/$TAG-full.json"
for s in 0 1 2 3 4; do
  echo "=== draw $s"
  python compare.py "$V/$TAG-drop$s.npz" run="$V/$TAG-draw$s.json"
done

#!/bin/bash
# The frozen versions over a looks file, each from the commit it was frozen
# at, then set side by side with the deployed store by compare.py.
#
#     bash run_versions.sh LOOKS.npz TAG
#
# Writes tmp/visit-model/TAG-v1.json, -v2.json, -v3.json.
set -e
LOOKS="$1"
TAG="$2"
T="D:/UGV Rover/tmp"
RECS=("$T/ww-recordings/shakedown.npz" "$T"/session?-10??/session?-10??.npz "$T"/ww-recordings/m5-*.npz "$T"/ww-recordings/1010/*.npz)
declare -A FROZEN=([v1]=c36aee0 [v2]=3c2498e [v3]=e5b39fd)
for v in v1 v2 v3; do
  git show "${FROZEN[$v]}:experiments/visit_model/model.py" > "model_frozen_$v.py"
  PYTHONHASHSEED=0 python "model_frozen_$v.py" --test "$LOOKS" "$T/visit-model/$TAG-$v.json" "${RECS[@]}" > "$T/visit-model/$TAG-$v.log" 2>&1 &
done
wait
rm -f model_frozen_v1.py model_frozen_v2.py model_frozen_v3.py
tail -n 1 "$T/visit-model/$TAG"-v?.log
python compare.py "$LOOKS" deployed v1="$T/visit-model/$TAG-v1.json" v2="$T/visit-model/$TAG-v2.json" v3="$T/visit-model/$TAG-v3.json"

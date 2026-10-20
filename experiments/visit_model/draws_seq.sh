#!/bin/bash
# run_draws.sh one run at a time on one core (another agent trains on this
# desktop's GPU and needs its CPU): five draws without 1% of the pictures that
# hold no labelled region, version 4 and a variant, scored on every session.
#     bash draws_seq.sh TAG [ENV=VALUE ...]     (from experiments/visit_model)
TAG="$1"; shift
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTHONHASHSEED=0
T="D:/UGV Rover/tmp"; W="$T/ws17"; V="$T/visit-model"
RECS=("$T/ww-recordings/shakedown.npz" "$T"/session?-10??/session?-10??.npz "$T"/drives?-10??/drives?-10??.npz "$T"/ww-recordings/m5-*.npz "$T"/ww-recordings/1010/*.npz "$W/ws17-m10trial1-1018.npz" "$W/ws17-md-day1-1019.npz" "$W/ws17-md-day2-1020.npz")
SP="$(dirname "$0")"
[ -f "$V/mdd-drop4.npz" ] || python draws.py "$V/looks-md-1020.npz" "$V/mdd" 5
for s in 0 1 2 3 4; do
  python model.py --test "$V/mdd-drop$s.npz" "$V/mdd-v4-draw$s.json" "${RECS[@]}" > "$V/mdd-v4-draw$s.log" 2>&1
  env "$@" python model.py --test "$V/mdd-drop$s.npz" "$V/mdd-$TAG-draw$s.json" "${RECS[@]}" > "$V/mdd-$TAG-draw$s.log" 2>&1
  echo "=== draw $s"
  python "$SP/score_all.py" "$V/mdd-drop$s.npz" v4="$V/mdd-v4-draw$s.json" "$TAG=$V/mdd-$TAG-draw$s.json"
done

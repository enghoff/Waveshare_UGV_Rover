#!/bin/bash
# Version 4 over every picture to 2026-10-17 with three walls: the frozen
# walls.npz (four grids intersected), one grid's occupied cells (what a new
# map would have), and none. Run 2026-10-18: one grid filed as the four do
# (630 records against 624, every score the same); none filed 188 of the
# paintings' labelled pictures against 206. Main adopted one grid for a map
# other than 7da19bef3888 (world_state/visit_model/job.py, walls_for).
#
#     bash walls_variants.sh
set -e
cd "D:/UGV Rover worktrees/visit-model/experiments/visit_model"
T="D:/UGV Rover/tmp"
O="$T/visit-model/walls"
mkdir -p "$O"
python - "$O" "$T" <<'PY'
import sys, numpy as np
sys.path.insert(0, ".")
import walls
o, t = sys.argv[1:3]
w = walls.build([t + "/session3-1015/session3-1015-grid.json"])
np.savez_compressed(o + "/walls-one.npz", **w)
print("one grid: stable cells", int(w["stable"].sum()))
z = dict(w); z["stable"] = np.zeros_like(w["stable"])
np.savez_compressed(o + "/walls-none.npz", **z)
PY
cp "D:/UGV Rover/world_state/visit_model/walls.npz" "$O/walls-full.npz"
git show cabe003:experiments/visit_model/model.py > model_frozen_wv4.py
RECS=("$T/ww-recordings/shakedown.npz" "$T"/session?-10??/session?-10??.npz "$T"/drives?-10??/drives?-10??.npz "$T"/ww-recordings/m5-*.npz "$T"/ww-recordings/1010/*.npz)
for v in full one none; do
  cp "$T/visit-model/looks-drives1-1017.npz" "$O/looks-$v.npz"
  python wallplace.py "$O/looks-$v.npz" "$O/walls-$v.npz" > "$O/wallplace-$v.log" 2>&1
  (PYTHONHASHSEED=0 python model_frozen_wv4.py --test "$O/looks-$v.npz" "$O/w-$v.json" "${RECS[@]}" > "$O/w-$v.log" 2>&1) &
done
wait
rm -f model_frozen_wv4.py
tail -n 1 "$O"/wallplace-*.log "$O"/w-*.log
for v in full one none; do echo "== $v"; python score.py "$O/looks-$v.npz" "$O/w-$v.json" 2>&1 | tail -n 25; done

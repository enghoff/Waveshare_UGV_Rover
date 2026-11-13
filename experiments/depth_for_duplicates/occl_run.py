"""Frozen version 4 over a looks file, keeping the final records (place, height,
frames, state) as well as the filing, for the occlusion study.

    python occl_run.py LOOKS.npz OUT.pkl
"""
import glob
import json
import os
import pickle
import sys

HERE = os.environ["VISIT_MODEL"]
sys.path.insert(0, HERE)
import data  # noqa: E402
import model_v4 as model  # noqa: E402
import recorded  # noqa: E402

T = "D:/UGV Rover/tmp"
W = T + "/ws17"
RECS = ([T + "/ww-recordings/shakedown.npz"] + sorted(glob.glob(T + "/session?-10??/session?-10??.npz"))
        + sorted(glob.glob(T + "/drives?-10??/drives?-10??.npz")) + sorted(glob.glob(T + "/ww-recordings/m5-*.npz"))
        + sorted(glob.glob(T + "/ww-recordings/1010/*.npz"))
        + [W + "/ws17-m10trial1-1018.npz", W + "/ws17-md-day1-1019.npz", W + "/ws17-md-day2-1020.npz"])

if __name__ == "__main__":
    a = data.load(sys.argv[1])
    recs = [recorded.Recording(p) for p in RECS if os.path.exists(p)]
    print("recordings", len(recs), "of", len(RECS), file=sys.stderr)
    filing, mem, decisions = model.run(a, recs)
    objs = []
    for o in mem.objects:
        objs.append({k: o[k] for k in ("id", "x", "y", "z", "var", "extent", "frames", "state", "since", "seen",
                                       "visits", "gone_at")})
    pickle.dump({"filing": filing, "objects": objs, "alias": dict(mem.alias), "decisions": decisions},
                open(sys.argv[2], "wb"))
    print("%d regions filed of %d, %d objects" % (len(filing), len(a["id"]), len(mem.objects)))

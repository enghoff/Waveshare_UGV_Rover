"""Draw the merge proposals as contact sheets, for a person to accept or refuse.

    python3 ~/ugv/world_state/merge_sheet.py                   ask the daemon, draw them
    python3 ~/ugv/world_state/merge_sheet.py proposals.json    draw a saved answer instead

Run on the rover, which holds the pictures. Each row is one proposal: its number and
score, then crops of the thing that would keep its name, a red bar, and crops of the
thing that would be joined to it. The sheets go to `~/.ugv/world/review/` and their
paths are printed. Nothing is changed; joining is `world_state_merge {"apply": ...}`
with the pairs the person accepted. See docs/runbooks/world-state-merge.md.
"""
from __future__ import annotations

import json
import os
import socket
import sqlite3
import sys
import time

ROW_PX = 96
PER_PAGE = 12
REVIEW = os.path.expanduser("~/.ugv/world/review")
DATABASE = os.path.expanduser("~/.ugv/world/world.db")


def proposals_from_daemon() -> dict:
    with socket.create_connection(("127.0.0.1", 8769), 60) as connection:
        stream = connection.makefile("rwb")
        stream.write(json.dumps({"call": "world_state_merge", "arguments": {}}).encode()
                     + b"\n")
        stream.flush()
        return json.loads(stream.readline())


def crops(cv2, np, db, ids):
    out = []
    for observation_id in ids:
        row = db.execute("SELECT frame_path, bbox_json FROM observations WHERE id = ?",
                         (observation_id,)).fetchone()
        picture = cv2.imread(row[0]) if row and row[0] else None
        if picture is None or not row[1]:
            continue
        height, width = picture.shape[:2]
        x0, y0, x1, y1 = json.loads(row[1])
        crop = picture[int(y0 * height):max(int(y1 * height), int(y0 * height) + 2),
                       int(x0 * width):max(int(x1 * width), int(x0 * width) + 2)]
        wide = max(24, min(150, int(ROW_PX * crop.shape[1] / max(crop.shape[0], 1))))
        out += [cv2.resize(crop, (wide, ROW_PX)), np.full((ROW_PX, 3, 3), 255, np.uint8)]
    return out


def label(cv2, np, lines, width=170):
    tile = np.full((ROW_PX, width, 3), 255, np.uint8)
    for index, text in enumerate(lines):
        cv2.putText(tile, text, (4, 22 + 24 * index), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5 if index else 0.6, (0, 0, 0), 2 if index == 0 else 1)
    return tile


def main() -> int:
    import cv2
    import numpy as np

    answer = (json.load(open(sys.argv[1])) if len(sys.argv) > 1
              else proposals_from_daemon())
    if not answer.get("ok"):
        print(answer.get("error") or answer)
        return 1
    proposals = answer.get("proposals") or []
    if not proposals:
        print("nothing to propose")
        return 0
    db = sqlite3.connect(f"file:{DATABASE}?mode=ro", uri=True)
    rows = []
    for one in proposals:
        keep, gone = one["keep"], one["gone"]
        row = ([label(cv2, np, [f"{one['n']}", f"score {one['score']:+.2f}",
                                f"{one['apart_m']:.2f} m apart"])]
               + [label(cv2, np, [keep, f"{one['looks'][0]} looks"], 150)]
               + crops(cv2, np, db, one["shown"][keep])
               + [np.full((ROW_PX, 14, 3), (0, 0, 200), np.uint8)]
               + [label(cv2, np, [gone, f"{one['looks'][1]} looks"], 150)]
               + crops(cv2, np, db, one["shown"][gone]))
        rows.append(np.hstack(row))
    width = max(row.shape[1] for row in rows)
    os.makedirs(REVIEW, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    for page, start in enumerate(range(0, len(rows), PER_PAGE), start=1):
        chunk = []
        for row in rows[start:start + PER_PAGE]:
            chunk += [np.hstack([row, np.full((ROW_PX, width - row.shape[1], 3), 255,
                                              np.uint8)]),
                      np.full((6, width, 3), 160, np.uint8)]
        path = os.path.join(REVIEW, f"merge-{stamp}-{page}.jpg")
        cv2.imwrite(path, np.vstack(chunk), [cv2.IMWRITE_JPEG_QUALITY, 85])
        print(path)
    return 0


if __name__ == "__main__":
    sys.exit(main())

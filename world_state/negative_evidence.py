"""Conservative, depth-backed contradictions of a proposed entity location.

This is not a generic detector-miss counter. Unknown depth, occlusion, image
edges, uncertain pose, and stale depth abstain. A challenge needs three distinct
viewpoints showing measured surfaces beyond the entire uncertainty patch.
"""
from collections import Counter
import gzip
import json
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from . import oak


class DepthEvidence:
    def __init__(self,frames,calibration=None):
        self.frames=Path(frames) if frames else None
        self.counts=Counter()
        self.cache_key=None
        self.cache=None
        self.misses={}
        # Archived device intrinsics, not invented from nominal field of view.
        source=Path(calibration) if calibration else Path(__file__).resolve().parent.parent/"captures/p0-gimbal-2026-09-07/oak-highres-preflight/calibration-health.json"
        self.calibration=str(source)
        self.lens=None
        if source.exists():
            data=json.loads(source.read_text(encoding="utf-8"))["colour"]["intrinsics"]
            self.lens=SimpleNamespace(**data)

    def _load(self,frame):
        if self.cache_key==frame:
            return self.cache
        self.cache_key,self.cache=frame,None
        if self.frames is None or self.lens is None:
            self.counts["no_calibration_or_frames"]+=1
            return None
        try:
            with gzip.open(self.frames/(frame+".depth.gz"),"rb") as f:
                raw=f.read()
            length=int.from_bytes(raw[:4],"little")
            meta=json.loads(raw[4:4+length])
            if meta.get("unit")!="mm" or meta.get("dtype")!="uint16":
                raise ValueError("unsupported depth units")
            if meta.get("age_s",100)>.15 or abs(meta.get("apart_s",100))>.05:
                self.counts["stale_depth"]+=1
                return None
            depth=np.frombuffer(raw[4+length:],dtype="<u2").reshape(meta["height"],meta["width"])/1000.0
        except (OSError,ValueError,KeyError):
            self.counts["no_usable_depth"]+=1
            return None
        self.counts["depth_frames"]+=1
        self.cache=depth
        return depth

    def evidence(self,point,row):
        """Return clear/support/unknown for this *particular* position."""
        depth=self._load(row.get("frame_id") or "")
        pose=row.get("pose")
        if depth is None or not pose or point.get("height_m") is None:
            return "unknown"
        if row.get("bearing_deg") is None or float(row.get("origin_sigma_m") or 0)>.15 or float(row.get("bearing_sigma_deg") or 1.5)>3:
            self.counts["uncertain_capture"]+=1
            return "unknown"
        sigma=float(point.get("uncertainty_m") or 0)
        if sigma>.6:
            self.counts["uncertain_position"]+=1
            return "unknown"
        dx,dy=point["x_m"]-pose["x_m"],point["y_m"]-pose["y_m"]
        heading=math.radians(pose["heading_deg"])
        forward=dx*math.cos(heading)+dy*math.sin(heading)
        left=-dx*math.sin(heading)+dy*math.cos(heading)
        height=float(point["height_m"])
        distance=math.sqrt(forward*forward+left*left+height*height)
        if distance<.5 or distance>8:
            return "unknown"
        xyz=oak._in_oak((forward/distance,left/distance,height/distance),distance)
        if xyz is None:
            return "unknown"
        u,v=oak._project(xyz,self.lens)
        h,w=depth.shape
        # Include placement, pointing and height uncertainty. Requiring the
        # whole patch behind it deliberately favours abstention over deletion.
        margin=sigma+.15+distance*math.tan(math.radians(3))
        rx=max(2,math.ceil(self.lens.fx/self.lens.width*w*margin/xyz[2]))
        ry=max(2,math.ceil(self.lens.fy/self.lens.height*h*(margin+float(point.get("height_sigma_m") or 0))/xyz[2]))
        x,y=round(u*w),round(v*h)
        if x-rx<2 or y-ry<2 or x+rx>=w-2 or y+ry>=h-2:
            self.counts["outside_or_edge"]+=1
            return "unknown"
        patch=depth[y-ry:y+ry+1,x-rx:x+rx+1]
        valid=patch[patch>.15]
        if valid.size!=patch.size:
            self.counts["incomplete_depth"]+=1
            return "unknown"
        if float(np.min(valid))>xyz[2]+margin+.25:
            self.counts["clear_contradictions"]+=1
            return "clear"
        self.counts["occluded_or_supported"]+=1
        return "unknown"

    def observe(self,entity,rows):
        if not rows:
            return False
        row=rows[0]
        point=entity["placement"]
        answer=self.evidence(point,row)
        eid=entity["id"]
        previous=self.misses.get(eid)
        anchor=(point["x_m"],point["y_m"],point.get("height_m"))
        # Misses belong to a hypothesis, never a free-floating entity counter.
        if previous is None or previous[0]!=anchor:
            previous=(anchor,[])
            self.misses[eid]=previous
        if answer!="clear":
            return False
        pose=row["pose"]
        places=previous[1]
        if any(math.hypot(pose["x_m"]-x,pose["y_m"]-y)<.3 for x,y in places):
            self.counts["correlated_misses_ignored"]+=1
            return False
        places.append((pose["x_m"],pose["y_m"]))
        return len(places)==3

    def stats(self):
        return {**dict(self.counts),"calibration":self.calibration,
                "note":"Device intrinsics from Sept 7; current mount transform. No calibrated detector-miss probability is assumed."}

"""Replay selected recorded depth answers; supplies no independent range truth."""
import argparse
import json
from pathlib import Path
import sqlite3
import sys

sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
import numpy as np
from PIL import Image
from world_state import outline
from experiments.entity_association.audit_depth_abstention import load_depth, trace, digest


def run(directory, ids):
    c=sqlite3.connect((directory/'world.db').resolve().as_uri()+'?mode=ro',uri=True)
    c.row_factory=sqlite3.Row
    rows=[]
    for oid in ids:
        o=dict(c.execute('SELECT * FROM observations WHERE id=?',(oid,)).fetchone())
        f=directory/'images'/o['frame_id']
        image,raw,header,inferred=load_depth(f.with_suffix('.depth.gz'))
        with Image.open(f.with_suffix('.jpg')) as im:size=im.size
        box=json.loads(o['bbox_json']);boxed=outline.box_range(image,box,size)
        answer=outline.read(np,raw,image.width,image.height,image.lens,
                            [(box,o['outline_blob'])],size)[0]
        rows.append({'id':oid,'stored_range_m':o['range_m'],'stored_method':o['range_from'],
                     'pan':o['observer_pan_deg'],'replay':answer,
                     'matches_stored':answer.get('range_m')==o['range_m'] and answer.get('method')==o['range_from'],
                     'trace':trace(image,o['outline_blob'],size,boxed.get('range_m')),
                     'header':header,'lens_inferred':inferred,
                     'depth_sha256':digest(f.with_suffix('.depth.gz'))})
    c.close()
    return rows


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--directory',type=Path,required=True)
    p.add_argument('--ids',type=int,nargs='+',required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();result=run(a.directory,a.ids)
    a.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps([{'id':r['id'],'matches_stored':r['matches_stored']} for r in result]))

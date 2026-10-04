"""Audit saved appearance provenance without running perception or changing labels.

The saved outline is clipped and downsampled. Its reconstructed area is only an
approximation and cannot recover the appearance mask outside the original box.
"""
from __future__ import annotations

import argparse
from contextlib import closing
import hashlib
import json
from pathlib import Path
import sqlite3
import sys
from types import SimpleNamespace

import numpy as np
from PIL import Image

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from world_state import outline
from world_state.perceive import Perception, LETTERBOX_GREY
from experiments.entity_association.repair_geometry import unit


def padding_demonstration():
    """Exercise the real preprocessing with a mask extending beyond its box."""
    image=np.full((100,100,3),220,dtype=np.uint8)
    box=[.3,.3,.7,.7]
    mask=np.zeros((100,100),bool);mask[29:71,29:71]=True
    model=object.__new__(Perception);model._np=np
    captured=[]
    def appearance(patches):
        captured.extend(patches)
        return np.zeros((len(patches),1)),0.
    model._appearance=appearance
    patch=model._crop(image,box)
    model._appearance_alone(image,[(box,1.,patch,0)],SimpleNamespace(of=lambda _:mask))
    x0,y0,x1,y1=model._window(image,box)
    kept=np.any(captured[0]!=LETTERBOX_GREY,axis=2)
    xx,yy=np.meshgrid(np.arange(x0,x1),np.arange(y0,y1))
    beyond=(xx<30)|(xx>=70)|(yy<30)|(yy>=70)
    saved=outline.decode(np,outline.encode(np,mask,box))
    return {'appearance_selected_pixels_outside_box':int((kept&beyond).sum()),
            'saved_outline_window':[saved[0],saved[1],saved[0]+saved[3].shape[1]*saved[2],
                                    saved[1]+saved[3].shape[0]*saved[2]],
            'scope':'Synthetic preprocessing demonstration, not proof of the cause of a real attachment.'}


def inspect(database,frames,ids):
    digest=hashlib.sha256(database.read_bytes()).hexdigest()
    result=[]
    with closing(sqlite3.connect(database.resolve().as_uri()+'?mode=ro',uri=True)) as db:
        db.row_factory=sqlite3.Row
        for i in ids:
            row=db.execute('SELECT * FROM observations WHERE id=?',(i,)).fetchone()
            if row is None:raise ValueError(f'missing observation {i}')
            row=dict(row)
            with Image.open(frames/(row['frame_id']+'.jpg')) as im:size=im.size
            box=json.loads(row['bbox_json']);tight=outline.window(box,size)
            padded=Perception._window(np.empty((size[1],size[0],3),np.uint8),box)
            crop_area=(padded[2]-padded[0])*(padded[3]-padded[1])
            decoded=outline.decode(np,row['outline_blob'])
            selected=Image.new('L',size)
            if decoded:
                x,y,stride,bits=decoded
                selected.paste(Image.fromarray(bits.astype('uint8')).resize(
                    (bits.shape[1]*stride,bits.shape[0]*stride),Image.Resampling.NEAREST),(x,y))
            approximate_pixels=int(np.asarray(selected.crop(tight)).sum())
            v,a=unit(row['dino_blob']),unit(row['dino_alone_blob'])
            result.append({'id':i,'frame_id':row['frame_id'],'frame_size':list(size),
                           'vectors_from':row['vectors_from'],'bbox':box,
                           'appearance_window':list(padded),'saved_outline_window':list(tight),
                           'padded_area':crop_area,'stored_mask_share':row['mask_share'],
                           'approximate_saved_outline_selected_pixels':approximate_pixels,
                           'approximate_saved_pixels_over_padded_area':approximate_pixels/crop_area,
                           'outline_is_exact_appearance_mask':False,
                           'plain_masked_self_cosine':float(v@a) if v is not None and a is not None else None,
                           'vector_bytes':{k:len(row[k] or b'') for k in ['dino_blob','dino_alone_blob','siglip_blob']},
                           'range_m':row['range_m'],'range_from':row['range_from']})
    assert hashlib.sha256(database.read_bytes()).hexdigest()==digest
    return {'database_sha256':digest,'source_unchanged':True,'observations':result,
            'preprocessing_demonstration':padding_demonstration(),
            'limitations':['Rounded boxes and half-resolution outlines prevent exact appearance-mask recovery.',
                           'No model inference rerun: stored embeddings and source implementation were inspected.',
                           'The tensorrt backend name does not identify model artifact hashes.']}


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--database',type=Path,required=True);p.add_argument('--frames',type=Path,required=True)
    p.add_argument('--observations',type=int,nargs='+',required=True);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    if args.output.exists():raise ValueError('choose a new result path')
    answer=inspect(args.database,args.frames,args.observations)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(answer,indent=2)+'\n')

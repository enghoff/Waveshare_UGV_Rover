"""Re-encode saved frames with box-clipped outlines, after checking plain crops.

Requires the rover's DINOv2-small or SigLIP2 vision fp16 ONNX export and onnxruntime.
This is a CPU diagnostic, not a production backend or an exact full-mask replay.
"""
import argparse
from contextlib import closing
import hashlib
import json
from pathlib import Path
import sqlite3
import sys
from types import SimpleNamespace

import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from world_state import outline
from world_state.perceive import Perception, LETTERBOX_GREY
from experiments.entity_association.repair_geometry import unit


def run(database,frames,model_path,runtime,ids,output,kind='dino'):
    if output.exists():raise ValueError('choose a new output directory')
    sys.path.insert(0,str(runtime.resolve()))
    import onnxruntime as ort
    options=ort.SessionOptions();options.intra_op_num_threads=2;options.inter_op_num_threads=1
    session=ort.InferenceSession(str(model_path),sess_options=options,providers=['CPUExecutionProvider'])
    encoder=object.__new__(Perception);encoder._np=np;encoder._cv2=cv2
    if kind=='dino':
        encoder._models=SimpleNamespace(appearance=lambda batch:session.run(None,{'pixel_values':batch})[0][:,0])
        encode=encoder._appearance;key='dino_blob';comparison_key='dino_alone_blob'
    elif kind=='semantic':
        encoder._models=SimpleNamespace(image_vectors=lambda batch:session.run(['pooler_output'],{'pixel_values':batch})[0])
        encode=encoder._semantic;key=comparison_key='siglip_blob'
    else:raise ValueError('choose dino or semantic')
    digest=hashlib.sha256(database.read_bytes()).hexdigest()
    patches=[];observations=[];stored=[];stored_masked=[]
    with closing(sqlite3.connect(database.resolve().as_uri()+'?mode=ro',uri=True)) as db:
        db.row_factory=sqlite3.Row
        for i in ids:
            row=db.execute('SELECT * FROM observations WHERE id=?',(i,)).fetchone()
            if row is None:raise ValueError(f'missing observation {i}')
            image=cv2.imread(str(frames/(row['frame_id']+'.jpg')))
            if image is None:raise ValueError(f'missing frame for {i}')
            box=json.loads(row['bbox_json']);window=encoder._window(image,box)
            plain=encoder._crop(image,box)
            decoded=outline.decode(np,row['outline_blob'])
            if decoded is None:raise ValueError(f'missing outline for {i}')
            x,y,stride,piece=decoded
            mask=np.zeros(image.shape[:2],np.uint8)
            expanded=cv2.resize(piece.astype(np.uint8),(piece.shape[1]*stride,piece.shape[0]*stride),
                                interpolation=cv2.INTER_NEAREST)
            h=min(expanded.shape[0],mask.shape[0]-y);w=min(expanded.shape[1],mask.shape[1]-x)
            mask[y:y+h,x:x+w]=expanded[:h,:w]
            left,top,right,bottom=outline.window(box,(image.shape[1],image.shape[0]))
            mask[:top]=0;mask[bottom:]=0;mask[:,:left]=0;mask[:,right:]=0
            clipped=plain.copy();clipped[mask[window[1]:window[3],window[0]:window[2]]==0]=LETTERBOX_GREY
            patches.extend([plain,clipped]);stored.append(unit(row[key]))
            stored_masked.append(unit(row[comparison_key]))
            observations.append({'id':i,'frame_id':row['frame_id']})
    values=[]
    for start in range(0,len(patches),16):
        vectors,_=encode(patches[start:start+16]);values.extend(vectors)
        print(f'encoded {min(start+16,len(patches))}/{len(patches)} crops',flush=True)
    values=np.asarray(values);plain=values[::2];clipped=values[1::2]
    for n,row in enumerate(observations):
        row['plain_vs_recorded_cosine']=float(plain[n]@stored[n])
        row['clipped_vs_recorded_comparison_cosine']=float(clipped[n]@stored_masked[n])
    assert hashlib.sha256(database.read_bytes()).hexdigest()==digest
    output.mkdir(parents=True)
    comparison_name='stored_masked' if kind=='dino' else 'stored_semantic'
    np.savez_compressed(output/'vectors.npz',ids=ids,plain=plain,clipped=clipped,
                        stored_plain=np.stack(stored),**{comparison_name:np.stack(stored_masked)})
    summary={'database_sha256':digest,'source_unchanged':True,'model_sha256':hashlib.sha256(model_path.read_bytes()).hexdigest(),
             'runtime':ort.__version__,'provider':'CPUExecutionProvider','observations':observations,
             'kind':kind,'comparison_column':comparison_key,
             'limitations':['CPU rather than TensorRT execution; use plain-crop agreement to assess fidelity.',
                            'Saved outlines have half resolution and boxes are rounded.',
                            'The old appearance mask outside the box was not saved; clipping is confounded with mask reconstruction.',
                            'This diagnostic neither fits an association score nor accepts identity.']}
    (output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    return summary


if __name__=='__main__':
    p=argparse.ArgumentParser()
    for name in ['database','frames','model','runtime','output']:p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--observations',type=int,nargs='+',required=True)
    p.add_argument('--kind',choices=['dino','semantic'],default='dino')
    a=p.parse_args();run(a.database,a.frames,a.model,a.runtime,a.observations,a.output,a.kind)

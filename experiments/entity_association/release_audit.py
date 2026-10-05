"""Draw the release-only proposals for review by eye, and tally a frozen review.

`sheets` draws one block per original record: a strip of its retained members
(largest kept cluster first, spread over time, outline green), then each released
observation as its full frame (box red, outline yellow), its selected pixels and its
plain box. `summarize` checks a frozen review against the proposal and counts it.
Reads the snapshot read-only; never changes a store.
"""
import argparse
from collections import Counter
from contextlib import closing
import hashlib
import json
from pathlib import Path
import sqlite3
import sys

import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from world_state import outline

#: Release was right (not the record's object), wrong (the record's own object, or one of
#: several identical chairs whose individual identity no picture settles), neutral (bare
#: floor, which is not an object) or could not be told.
VERDICTS={'OUT':'right','MIX':'right','JUNK':'right','SAME':'wrong','CLASS':'wrong',
          'SURFACE':'neutral','UNC':'unclear'}
WIDTH=1340


def mask_of(row,shape):
    if not row['outline_blob']:return None
    x,y,s,piece=outline.decode(np,row['outline_blob'])
    m=np.zeros(shape[:2],np.uint8)
    e=cv2.resize(piece.astype(np.uint8),(piece.shape[1]*s,piece.shape[0]*s),interpolation=cv2.INTER_NEAREST)
    h=min(e.shape[0],m.shape[0]-y);w=min(e.shape[1],m.shape[1]-x);m[y:y+h,x:x+w]=e[:h,:w]
    l,t,r,b=outline.window(json.loads(row['bbox_json']),(shape[1],shape[0]))
    m[:t]=0;m[b:]=0;m[:,:l]=0;m[:,r:]=0
    return m


def box_px(row,shape,pad=0.0):
    x0,y0,x1,y1=json.loads(row['bbox_json']);W,H=shape[1],shape[0];w,h=x1-x0,y1-y0
    return (max(0,int((x0-pad*w)*W)),max(0,int((y0-pad*h)*H)),min(W,int((x1+pad*w)*W)),min(H,int((y1+pad*h)*H)))


def fit(img,W,H):
    h,w=img.shape[:2];k=min(W/w,H/h)
    im=cv2.resize(img,(max(1,int(w*k)),max(1,int(h*k))),interpolation=cv2.INTER_AREA)
    canvas=np.full((H,W,3),30,np.uint8);y=(H-im.shape[0])//2;x=(W-im.shape[1])//2
    canvas[y:y+im.shape[0],x:x+im.shape[1]]=im
    return canvas


def text(img,s,x,y,scale=0.45,colour=(255,255,255)):
    cv2.putText(img,s,(x,y),cv2.FONT_HERSHEY_SIMPLEX,scale,(0,0,0),3,cv2.LINE_AA)
    cv2.putText(img,s,(x,y),cv2.FONT_HERSHEY_SIMPLEX,scale,colour,1,cv2.LINE_AA)


def outlined(image,mask,colour,width):
    if mask is not None:
        contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
        cv2.drawContours(image,contours,-1,colour,width)
    return image


def sheets(proposal,database,frame_dirs,output):
    if output.exists():raise ValueError('choose a new output directory')
    p=json.loads(proposal.read_text());released=p['released'];before=p['before_owners']
    if p['database_sha256']!=hashlib.sha256(database.read_bytes()).hexdigest():raise ValueError('proposal is for another store')
    with closing(sqlite3.connect(database.resolve().as_uri()+'?mode=ro',uri=True)) as db:
        db.row_factory=sqlite3.Row
        rows={r['id']:r for r in db.execute('SELECT id,frame_id,bbox_json,outline_blob,observed_at FROM observations')}
    clusters={}
    for tile in p['tiles']:
        for c in tile['clusters']:clusters.setdefault(before[str(c[0])],[]).append(c)
    for c in clusters.values():c.sort(key=len,reverse=True)
    cache={}
    def frame(fid):
        if fid not in cache:
            found=[d/(fid+'.jpg') for d in frame_dirs if (d/(fid+'.jpg')).exists()]
            if not found:raise ValueError(f'missing frame {fid}')
            cache[fid]=cv2.imread(str(found[0]))
        return cache[fid]
    def member(i):
        r=rows[i];im=outlined(frame(r['frame_id']).copy(),mask_of(r,frame(r['frame_id']).shape),(0,255,0),1)
        x0,y0,x1,y1=box_px(r,im.shape,0.25);return fit(im[y0:y1,x0:x1],165,165)
    def panel(i):
        r=rows[i];im=frame(r['frame_id']);m=mask_of(r,im.shape);x0,y0,x1,y1=box_px(r,im.shape)
        full=outlined(im.copy(),m,(0,255,255),2);cv2.rectangle(full,(x0,y0),(x1,y1),(0,0,255),2)
        l,t,rr,b=outline.window(json.loads(r['bbox_json']),(im.shape[1],im.shape[0]))
        chosen=im.copy()
        if m is not None:chosen[m==0]=(90,90,90)
        gap=np.zeros((210,4,3),np.uint8)
        out=np.hstack([fit(full,280,210),gap,fit(chosen[t:b,l:rr],190,210),gap,fit(im[y0:y1,x0:x1],150,210)])
        text(out,f'R {i}'+('' if m is None else f' sel {int(m[y0:y1,x0:x1].mean()*100)}%'),4,16,0.5,(0,255,255))
        return out
    parents=sorted({before[str(i)] for i in released},key=lambda par:min(i for i in released if before[str(i)]==par))
    blocks=[];index={};n=0
    for par in parents:
        rel=sorted(i for i in released if before[str(i)]==par);kept=clusters.get(par,[]);shown=[]
        if kept:
            main=sorted(kept[0],key=lambda i:rows[i]['observed_at']);k=min(5 if len(kept)>1 else 8,len(main))
            shown=[(main[int(j*(len(main)-1)/max(1,k-1))],0) for j in range(k)]
            for ci,c in enumerate(kept[1:],1):
                if len(shown)>=8:break
                shown.append((sorted(c,key=lambda i:rows[i]['observed_at'])[len(c)//2],ci))
        seen=set();shown=[s for s in shown if not (s[0] in seen or seen.add(s[0]))]
        strip=np.zeros((165,WIDTH,3),np.uint8)
        for j,(i,ci) in enumerate(shown):
            c=member(i);text(c,f'c{ci} {i}',3,160,0.4,(0,255,0));strip[:,j*167:j*167+165]=c
        header=np.full((26,WIDTH,3),60,np.uint8)
        text(header,f'{par}  kept {sum(map(len,kept))} in clusters {[len(c) for c in kept]}  releases {len(rel)}: '+
             ' '.join(f'#{n+1+k}={i}' for k,i in enumerate(rel)),6,18,0.5)
        for k,i in enumerate(rel):index[n+1+k]={'observation':i,'parent':par}
        n+=len(rel);panels=[panel(i) for i in rel];lines=[]
        for a in range(0,len(panels),2):
            line=np.zeros((214,WIDTH,3),np.uint8)
            for b,pnl in enumerate(panels[a:a+2]):line[2:212,b*670:b*670+pnl.shape[1]]=pnl
            lines.append(line)
        blocks.append((par,np.vstack([header,strip]+lines+[np.full((8,WIDTH,3),255,np.uint8)])))
    output.mkdir(parents=True);pages=[];page=[]
    def flush():
        if page:
            cv2.imwrite(str(output/f'sheet-{len(pages)+1:02d}.jpg'),np.vstack([b for _,b in page]),[cv2.IMWRITE_JPEG_QUALITY,88])
            pages.append([par for par,_ in page]);page.clear()
    for par,b in blocks:
        if page and sum(x.shape[0] for _,x in page)+b.shape[0]>1500:flush()
        page.append((par,b))
    flush()
    (output/'index.json').write_text(json.dumps({'items':index,'sheets':pages},indent=1)+'\n')
    return len(pages)


def summarize(review,proposal):
    r=json.loads(review.read_text());p=json.loads(proposal.read_text())
    if r['proposal_sha256']!=hashlib.sha256(proposal.read_bytes()).hexdigest():raise ValueError('review is of another proposal')
    items=r['items'];ids=[x['observation'] for x in items]
    if sorted(ids)!=sorted(p['released']) or len(set(ids))!=len(ids):raise ValueError('review must cover every release once')
    if any(x['verdict'] not in VERDICTS for x in items):raise ValueError('unknown verdict')
    if any(p['before_owners'][str(x['observation'])]!=x['parent'] for x in items):raise ValueError('parent mismatch')
    size=Counter(p['before_owners'].values());solo=[x for x in items if size[x['parent']]==1]
    changing=[x for x in items if size[x['parent']]>1]
    return {'releases':len(items),'verdicts':dict(Counter(x['verdict'] for x in items)),
            'outcome':dict(Counter(VERDICTS[x['verdict']] for x in items)),
            'single_member_records':{'releases':len(solo),'outcome':dict(Counter(VERDICTS[x['verdict']] for x in solo))},
            'pair_changing_outcome':dict(Counter(VERDICTS[x['verdict']] for x in changing)),
            'moderate_confidence':dict(Counter(x['verdict'] for x in items if x.get('moderate')))}


if __name__=='__main__':
    p=argparse.ArgumentParser();sub=p.add_subparsers(dest='command',required=True)
    s=sub.add_parser('sheets')
    for name in ['proposal','database','output']:s.add_argument('--'+name,type=Path,required=True)
    s.add_argument('--frames',type=Path,nargs='+',required=True)
    t=sub.add_parser('summarize')
    for name in ['review','proposal']:t.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args()
    if a.command=='sheets':print(sheets(a.proposal,a.database,a.frames,a.output),'sheets')
    else:print(json.dumps(summarize(a.review,a.proposal),indent=2))

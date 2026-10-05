"""Read-only, observation-verified semantic features for standalone experiments."""
from contextlib import contextmanager
import hashlib
import json

import numpy as np

from experiments.entity_association.probe_clipped_appearance import observation_fingerprint
from world_state import merging


class SemanticFeatures:
    def __init__(self,directory):
        self.directory=directory
        self.provenance=json.loads((directory/'summary.json').read_text())
        if self.provenance.get('kind')!='semantic':raise ValueError('need semantic probe features')
        with np.load(directory/'vectors.npz',allow_pickle=False) as data:
            ids=data['ids'].tolist();vectors=data['clipped'].astype(float)
        if len(set(ids))!=len(ids) or len(ids)!=len(vectors) or vectors.ndim!=2 or not np.isfinite(vectors).all():
            raise ValueError('invalid semantic feature rows')
        norms=np.linalg.norm(vectors,axis=1)
        if np.any(norms<1e-9):raise ValueError('empty semantic feature')
        self.vectors=dict(zip(ids,vectors/norms[:,None]))
        self.records={r['id']:r for r in self.provenance['observations']}
        if set(self.records)!=set(ids):raise ValueError('feature provenance IDs differ')
        self.sha256=hashlib.sha256((directory/'vectors.npz').read_bytes()).hexdigest()

    def validate(self,rows):
        missing=[]
        for row in rows:
            i=row['id']
            if i not in self.vectors:missing.append(i);continue
            if self.records[i].get('input_sha256')!=observation_fingerprint(row):
                raise ValueError(f'appearance inputs changed for observation {i}')
        return missing


@contextmanager
def reader_channel(bank,weight):
    """Temporarily add one channel to the existing reader, preserving its sample."""
    original_thing,original_appearance=merging._Thing,merging.appearance
    class AugmentedThing(original_thing):
        def __init__(self,row,looks):
            super().__init__(row,looks)
            usable=[look for look in looks if look.get('dino_blob') and look.get('siglip_blob')]
            if len(usable)>merging.SAMPLE:
                step=(len(usable)-1)/(merging.SAMPLE-1)
                usable=[usable[round(i*step)] for i in range(merging.SAMPLE)]
            kept=[look for look in usable if merging._unit(look['dino_blob']) is not None
                  and merging._unit(look['siglip_blob']) is not None]
            self.extra=None
            if kept and all(look['id'] in bank.vectors for look in kept):
                self.extra=np.stack([bank.vectors[look['id']] for look in kept])
                assert self.plain is not None and len(self.extra)==len(self.plain)

    def appearance(a,b):
        if a.extra is None or b.extra is None:return None
        base=original_appearance(a,b)
        if base is None:return None
        comparable=a.backend[:,None]==b.backend[None,:]
        return base+weight*float((a.extra@b.extra.T)[comparable].mean())
    merging._Thing,merging.appearance=AugmentedThing,appearance
    try:yield
    finally:merging._Thing,merging.appearance=original_thing,original_appearance

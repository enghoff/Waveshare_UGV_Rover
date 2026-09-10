"""Bounded memoization control: identical resolver decisions, less repeated geometry.

An offline context manager, not a process-global runtime switch. Measurement
and placement content form the keys, including map reach. Mutable result dicts
are copied on return. Caches are discarded when the context exits.
"""
from collections import OrderedDict, Counter
from contextlib import contextmanager, ExitStack
from unittest.mock import patch

from . import locate, resolve


def frozen(value):
    if isinstance(value,dict):
        return tuple((key,frozen(val)) for key,val in sorted(value.items()))
    if isinstance(value,(list,tuple)):
        return tuple(map(frozen,value))
    return value


@contextmanager
def cached_geometry(capacity=65536):
    counts=Counter()
    def cached(name,fn):
        entries=OrderedDict()
        def call(*args,**kwargs):
            key=(tuple(frozen(arg) for arg in args),frozen(kwargs))
            if key in entries:
                counts[name+"_hits"]+=1
                value=entries.pop(key)
                entries[key]=value
            else:
                counts[name+"_misses"]+=1
                value=fn(*args,**kwargs)
                entries[key]=dict(value) if isinstance(value,dict) else value
                if len(entries)>capacity:
                    entries.popitem(last=False)
                counts[name+"_peak_entries"]=max(counts[name+"_peak_entries"],len(entries))
            return dict(value) if isinstance(value,dict) else value
        return call
    with ExitStack() as stack:
        for module,name in ((locate,"fix"),(locate,"agrees"),(resolve,"_allowance_used")):
            stack.enter_context(patch.object(module,name,cached(name,getattr(module,name))))
        yield counts

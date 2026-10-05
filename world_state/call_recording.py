"""Opt-in inspection/resolver provenance; never changes measured evidence or policy."""
from __future__ import annotations
import base64
from contextlib import closing
import hashlib
import json
from pathlib import Path
import re
import sqlite3
import time
import zlib

REQUEST = 'record-calls.json'


def encode(value):
    if isinstance(value, bytes):
        return {'base64': base64.b64encode(value).decode('ascii')}
    if hasattr(value, 'item'):
        return value.item()
    raise TypeError(type(value).__name__)


def checkpoint(store):
    """Only deterministic identity state; timestamps are not decisions."""
    with store._lock:
        owners = [tuple(r) for r in store.db.execute('SELECT id,entity_id FROM observations ORDER BY id')]
        entities = [tuple(r) for r in store.db.execute(
            'SELECT id,placement_json,exemplars,exemplars_alone FROM entities ORDER BY id')]
        meta = [tuple(r) for r in store.db.execute(
            "SELECT key,value FROM meta WHERE key IN ('map_session','map_id','generation') ORDER BY key")]
    body = json.dumps([owners, entities, meta], default=encode, separators=(',', ':')).encode()
    return hashlib.sha256(body).hexdigest()


class CallRecording:
    def __init__(self, store, session):
        if not isinstance(session, str) or not re.fullmatch(r'[a-z0-9][a-z0-9-]{0,63}', session):
            raise ValueError('recording session must be a short lowercase name')
        self.directory = Path(store.dir)/'recordings'/session
        self.directory.mkdir(parents=True, exist_ok=False)
        self.session, self.sequence, self.error = session, 0, ''
        self._grids = []
        (self.directory/'maps').mkdir()
        source = Path(__file__).parent
        self.manifest = {'session': session, 'complete': False, 'started_at': time.time(),
            'source_sha256': {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in source.glob('*.py')},
            'meaning': 'Actual locked inspector operations and resolver calls; optional diagnostic, unchanged matching policy.'}
        self._manifest()
        self.backup(store, 'before.db')
        self.file = (self.directory/'events.jsonl').open('a', encoding='utf-8')
        self.emit('start', checkpoint=checkpoint(store))

    def _manifest(self):
        (self.directory/'manifest.json').write_text(json.dumps(self.manifest, indent=2)+'\n', encoding='utf-8')

    def backup(self, store, name):
        with store._lock, closing(sqlite3.connect(self.directory/name)) as destination:
            store.db.backup(destination)

    def emit(self, kind, **fields):
        if self.error:
            return
        try:
            self.sequence += 1
            row = {'sequence': self.sequence, 'kind': kind, 'wall_time': time.time(),
                   'monotonic': time.monotonic(), **fields}
            self.file.write(json.dumps(row, default=encode, separators=(',', ':'))+'\n')
            if kind != 'reach':
                self.file.flush()
        except Exception as error:
            self.error = f'{type(error).__name__}: {error}'

    def boundary(self, kind, store, **fields):
        try:
            with store._lock:
                maximum = store.db.execute('SELECT COALESCE(MAX(id),0) FROM observations').fetchone()[0]
                meta = dict(store.db.execute(
                    "SELECT key,value FROM meta WHERE key IN ('map_session','map_id','generation')"))
            self.emit(kind, max_observation_id=maximum, meta=meta, checkpoint=checkpoint(store), **fields)
        except Exception as error:
            self.error = f'{type(error).__name__}: {error}'

    def grid(self, context):
        if context is None:
            return None
        width, height, resolution, ox, oy, cells = context
        for previous, key in self._grids:
            if cells is previous:
                return key
        data = {'ok': True, 'width': int(width), 'height': int(height),
                'resolution_m': float(resolution), 'origin_x_m': float(ox), 'origin_y_m': float(oy),
                'data': base64.b64encode(zlib.compress(cells.tobytes())).decode('ascii')}
        body = json.dumps(data, sort_keys=True, separators=(',', ':')).encode()
        key = hashlib.sha256(body).hexdigest()
        (self.directory/'maps'/(key+'.json')).write_bytes(body)
        self._grids.append((cells, key))
        return key

    def reach(self, original, grid_context):
        if original is None:
            return None
        def measured(x, y, bearing):
            answer = original(x, y, bearing)
            try:
                key = self.grid(grid_context() if grid_context else None)
                self.emit('reach', arguments=[x, y, bearing], result=answer, map_sha256=key)
            except Exception as error:
                self.error = f'{type(error).__name__}: {error}'
            return answer
        return measured

    def close(self, store):
        try:
            self.boundary('stop', store)
            self.backup(store, 'after.db')
            self.manifest.update(complete=not bool(self.error), error=self.error,
                                 finished_at=time.time(), events=self.sequence)
            self._manifest()
        finally:
            self.file.close()

    def status(self):
        return {'session': self.session, 'events': self.sequence, 'error': self.error}

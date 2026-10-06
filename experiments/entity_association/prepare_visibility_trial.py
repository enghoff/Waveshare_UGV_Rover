"""Own the short trial's recorders; default checks preparation without any motion.

Run under the rover ROS environment, from a staged diagnostic directory. Only
--execute --motion-authorized enables the serial motion runner. Creating a
recording marker is exclusive; somebody else's session is never removed.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import signal
import socket
import subprocess
import sys
import threading
import time

from run_visibility_trial import Live, rpc, run


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--session', required=True)
    p.add_argument('--nav-source', type=Path, required=True)
    p.add_argument('--card', type=Path, required=True)
    p.add_argument('--execute', action='store_true')
    p.add_argument('--motion-authorized', action='store_true')
    p.add_argument('--interrupt-recorder', action='store_true',
                   help='Stationary verification only: close recorder with SIGINT instead of its stop file')
    a = p.parse_args()
    if not re.fullmatch(r'[a-z0-9-]+',a.session): p.error('Invalid session name')
    if a.execute and not a.motion_authorized: p.error('Fresh owner handover is required')
    if a.execute and a.interrupt_recorder: p.error('Interruption test is stationary only')
    # No STOP here: preparation must not interfere with somebody else's drive.
    status = rpc(8769, {'call':'nav_status','arguments':{}})
    if status.get('driving') or status.get('exploring') or status.get('autonomy') or status.get('pwm') != [0,0]:
        raise RuntimeError('Rover is in use; preparation refused')
    world = Path.home()/'.ugv/world'
    marker = world/'record-calls.json'
    root = Path.home()/'.ugv/diagnostics'/a.session
    root.mkdir(parents=True, exist_ok=False)
    support = root/'support'; support.mkdir()
    output = root/'run'; output.mkdir()
    marker_owned = False
    recorder = None
    stop_board = threading.Event()
    board_thread = None
    board_error = []
    result = {'motion_requested':a.execute}
    log = (support/'navigation.log').open('w')
    try:
        with marker.open('x') as handle:
            json.dump({'session':a.session},handle)
        marker_owned = True
        recorder = subprocess.Popen([sys.executable,str(Path(__file__).with_name('record_navigation_trial.py')),
            '--nav-source',str(a.nav_source),'--out',str(support/'navigation.json'),
            '--stop-file',str(support/'stop'),'--seconds','300'],stdout=log,stderr=subprocess.STDOUT)
        def board():
            try:
                with socket.create_connection(('127.0.0.1',8772),3) as sock, (support/'board.jsonl').open('w') as out:
                    sock.settimeout(3)
                    stream = sock.makefile()
                    while not stop_board.is_set():
                        row = json.loads(stream.readline())
                        out.write(json.dumps({'received_at':time.time(),'board':row})+'\n')
                        out.flush()
            except Exception as error:
                board_error.append(str(error))
        board_thread = threading.Thread(target=board,daemon=True); board_thread.start()
        # Wait only for concrete readiness, with a bound; this happens before motion.
        until = time.monotonic()+35
        while True:
            if recorder.poll() is not None or board_error:
                raise RuntimeError('Support recorder failed to start')
            try:
                episode = json.loads((support/'navigation.json').read_text())
                if all(episode.get(k) for k in ('poses','costmaps','params')) and (support/'board.jsonl').stat().st_size:
                    break
            except (FileNotFoundError,json.JSONDecodeError): pass
            if time.monotonic() >= until: raise RuntimeError('Support recorders did not become ready')
            time.sleep(.2)
        backend = Live(output,json.loads(a.card.read_text()),support,a.session)
        result['world_warmup'] = backend.call('world_inspect',
            {'fresh':True,'keep_depth':True,'settle':False,'wait':True}, timeout=12)
        manifest_path = world/'recordings'/a.session/'manifest.json'
        if not manifest_path.exists() or json.loads(manifest_path.read_text()).get('complete') is not False:
            raise RuntimeError('World call recorder did not become active')
        if a.execute:
            result.update(run(backend,backend.card['points'],
                              direct_return=backend.card.get('direct_return',False)))
        else:
            result.update({'home':backend.preflight(backend.card['points']),'movement':False,'preflight_pass':True})
    except Exception as error:
        result['error'] = str(error)
    finally:
        if marker_owned and marker.exists() and json.loads(marker.read_text()).get('session') == a.session:
            marker.unlink()
            try:
                result['world_close'] = rpc(8769,{'call':'world_inspect','arguments':{'settle':False,'wait':True}},timeout=12)
            except Exception as error: result['world_close_error'] = str(error)
        if recorder is not None:
            if a.interrupt_recorder:
                recorder.send_signal(signal.SIGINT)
            else:
                (support/'stop').touch()
            try: result['recorder_exit'] = recorder.wait(timeout=12)
            except subprocess.TimeoutExpired:
                recorder.kill(); recorder.wait(timeout=3)
                result['recorder_error'] = 'Recorder failed to close; last checkpoint is not final proof'
        stop_board.set()
        if board_thread: board_thread.join(timeout=4)
        log.close()
    if (support/'navigation.json').exists():
        episode = json.loads((support/'navigation.json').read_text())
        result['navigation_closed'] = episode.get('trial_recording',{}).get('closed')
        result['navigation_counts'] = {k:len(episode.get(k,[])) for k in ('poses','costmaps','plans','commands')}
    result['board_error'] = board_error
    record = world/'recordings'/a.session
    result['world_recording_expected'] = str(record)
    if (record/'manifest.json').exists():
        result['world_recording_complete'] = json.loads((record/'manifest.json').read_text()).get('complete')
    (root/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    manifest = {str(f.relative_to(root)):hashlib.sha256(f.read_bytes()).hexdigest()
                for f in root.rglob('*') if f.is_file()}
    (root/'sha256.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps({'directory':str(root),'result':result}),flush=True)
    return 0 if ('error' not in result and result.get('navigation_closed') and
                 result.get('recorder_exit')==0 and result.get('world_recording_complete')) else 1


if __name__ == '__main__':
    raise SystemExit(main())

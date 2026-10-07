"""Run on Orin after stationary setup. Default is a movement-free preflight.

An explicit --execute --motion-authorized is required after owner handover.
Support/world recorders must already be active and verified. All evidence lands
outside the deploy tree. This experiment never modifies navigation or perception.
"""
import argparse
import base64
import json
import math
from pathlib import Path
import socket
import subprocess
import sys
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parent))
from serial_visibility_trial import ReturnNow, bearing, run
from check_recorded_turn import check as turn_check
import goal_fit


def scan_decides(measurement):
    """The scan's pose, when it disagrees with navigation and is plainly right.

    After a large turn navigation's heading can be 10 degrees or more out, and
    it stays out until the rover drives again, because the mapper folds no scan
    in while the rover stands still. On 2026-10-07 the trial stopped at its
    first viewpoint on exactly that: navigation said 47 degrees, the scan fitted
    59 with 99% of it on a wall, against 49% at navigation's pose. A still look
    already takes its bearings from the same fit (world_state/headingcheck.py),
    so the fit is what the trial's view and its photographs are judged by.

    Only where nothing is ambiguous: the fit is trusted and scores at least 0.90,
    beats navigation's own pose by 0.30 or more, sits within 0.25 m of it, and
    beats the best other place the scan could fit by 0.15. Otherwise None, and
    the caller refuses as before.
    """
    try:
        score = float(measurement['score'])
        at_navigation = float(measurement['guess_score'])
        rival = float(measurement.get('rival') or 0.0)
        moved = abs(float(measurement['moved_m']))
        pose = {'x_m': float(measurement['x_m']), 'y_m': float(measurement['y_m']),
                'heading_deg': float(measurement['heading_deg'])}
    except (KeyError, TypeError, ValueError):
        return None
    if (measurement.get('trusted') and score >= .90 and score - at_navigation >= .30
            and score - rival >= .15 and moved <= .25):
        return pose
    return None


def wheels_unpowered(pwm):
    """Whether the base's last motor command leaves the wheels unpowered.

    None means navigation has sent the board nothing since it started, which is
    every fresh boot until the first move: the board's own heartbeat stops a base
    that hears nothing, so that is as still as an explicit zero. Refusing it made
    the stationary preflight call an idle rover "in use" on 2026-10-07.
    """
    return pwm is None or pwm == [0,0]


def rpc(port, request, timeout=8):
    with socket.create_connection(('127.0.0.1', port), timeout=3) as sock:
        sock.settimeout(timeout)
        sock.sendall((json.dumps(request)+'\n').encode())
        answer = json.loads(sock.makefile().readline())
    if not answer.get('ok'):
        raise RuntimeError(str(answer))
    return answer


def path_check(plan, snapshot, start, goal):
    direct = math.dist(start, goal)
    if not plan.get('ok') or not plan.get('path') or plan['length_m'] > direct+.5:
        raise RuntimeError('Missing or looping path')
    g = snapshot['grids']['global']
    if g['frame'] != 'map' or not 0 <= g['age_s'] <= 2:
        raise RuntimeError('Wrong or stale global grid')
    if g['origin_orientation'] != [0.0, 0.0, 0.0, 1.0]:
        raise RuntimeError('Rotated grid unsupported')
    data = base64.b64decode(g['data'], validate=True)
    if len(data) != g['width']*g['height']:
        raise RuntimeError('Incomplete grid')
    grid = goal_fit.CostGrid(g['width'], g['height'], g['resolution'], *g['origin'], list(data))
    footprint = goal_fit.polygon_from('', .20/math.cos(math.pi/32), sides=32)
    points = [p[:2] for p in plan['path']]
    if math.dist(points[0], start) > .25 or math.dist(points[-1], goal) > .1:
        raise RuntimeError('Path endpoints do not describe this leg')
    for a,b in zip(points, points[1:]+[points[-1]]):
        count = max(1, math.ceil(math.dist(a,b)/(g['resolution']/2)))
        for i in range(count+1):
            x,y = (a[j]+(b[j]-a[j])*i/count for j in range(2))
            if grid.cost(*grid.cell_of(x,y)) >= 253 or any(
                grid.cost(c,r) >= 254 for c,r in goal_fit.covered(grid, footprint, x,y,0)):
                raise RuntimeError('Path crosses occupied or unknown floor')


class Live:
    def __init__(self, directory, card, support, session):
        self.directory, self.card, self.support, self.session = directory, card, support, session
        self.aborted = threading.Event()
        self.deadline = None
        self.return_started = False
        self.return_motion = False
        self.guard = None
        self.stop_seq = None

    def log(self, kind, **values):
        with (self.directory/'events.jsonl').open('a') as handle:
            handle.write(json.dumps({'at': time.time(), 'kind': kind, **values})+'\n')

    def call(self, name, arguments=None, timeout=8):
        started = time.time()
        result = rpc(8769, {'call': name, 'arguments': arguments or {}}, timeout)
        self.log('rpc', call=name, arguments=arguments or {}, started_at=started, result=result)
        return result

    def reserve(self, seconds, returning=False):
        if self.aborted.is_set():
            raise RuntimeError('Watchdog stopped the trial')
        if not returning and self.deadline and time.monotonic()+seconds+15 > self.deadline:
            raise ReturnNow()

    def helper(self, filename, arguments=(), returning=False):
        self.reserve(8, returning)
        script = Path(__file__).with_name(filename)
        answer = subprocess.run([sys.executable, str(script), *map(str,arguments)],
                                capture_output=True, text=True, timeout=8, check=True)
        result = json.loads(answer.stdout)
        self.log('helper', name=filename, result=result)
        return result

    def health(self, returning=False):
        self.reserve(5, returning)
        status = self.wait_for_still()
        if self.stop_seq is not None and status.get('stop_seq') != self.stop_seq:
            raise RuntimeError('An external STOP changed control ownership')
        if (status.get('map_id') != self.card['map_id'] or status.get('driving') or
            status.get('exploring') or status.get('autonomy') or status.get('estop') or
            not wheels_unpowered(status.get('pwm')) or not all(status.get(k) for k in
                ('board_ok','lidar_live','nav2_ready','position_trusted')) or
            status.get('scan_age_s', 99) > 1 or status.get('transform_age_s',99) > 1):
            raise RuntimeError('Navigation/board/localization is not ready and stationary')
        measurement = rpc(8773, {'op':'measure'}, timeout=3)
        self.log('measure', result=measurement)
        if (not measurement.get('trusted') or measurement.get('score',0) < .90 or
            abs(measurement.get('moved_m',99)) > .25 or abs(measurement.get('turned_deg',99)) > 10):
            fitted = scan_decides(measurement)
            if fitted is None:
                raise RuntimeError('Fresh stationary localization check failed')
            # Navigation is lagging, not lost: judge the view from the scan.
            self.log('navigation_lags_scan', navigation=status.get('pose'), scan=fitted)
            status = dict(status, pose=fitted, pose_from='scan')
        charge = self.call('battery', timeout=3)
        if charge.get('reading_age_s',99) > 5:
            raise RuntimeError('Battery reading stale')
        if charge['percent'] <= 15:
            raise RuntimeError('Battery at recovery floor; STOP for manual recovery')
        if charge['percent'] <= 35 and not returning:
            raise ReturnNow()
        return status, charge

    def wait_for_still(self, timeout=3):
        """Command completion can precede physical rest; require repeated feedback.

        Exact zero uses the bridge's rounded measured odometry, not its command.
        The 0.4 s window and 3 s bound are trial policy, not stopping calibration.
        No movement or pose correction is issued here.
        """
        deadline = time.monotonic() + timeout
        quiet_since = None
        while time.monotonic() < deadline:
            status = self.call('nav_status', timeout=min(.5, deadline-time.monotonic()))
            now = time.monotonic()
            quiet = (status.get('driving') is False and status.get('board_ok') is True
                     and status.get('speed_ms') == 0 and status.get('turn_dps') == 0
                     and wheels_unpowered(status.get('pwm'))
                     and isinstance(status.get('transform_age_s'), (int,float))
                     and 0 <= status['transform_age_s'] <= .2)
            if quiet:
                if quiet_since is None:
                    quiet_since = now
                if now < deadline and now-quiet_since >= .4:
                    return status
            else:
                quiet_since = None
            time.sleep(min(.1, max(0, deadline-time.monotonic())))
        raise RuntimeError('Measured motion did not remain stopped within 3 seconds')

    def preflight(self, points):
        status, charge = self.health(returning=True)
        self.stop_seq = status['stop_seq']
        if charge['percent'] < 60:
            raise RuntimeError('Start charge below 60%')
        marker = json.loads((Path.home()/'.ugv/world/record-calls.json').read_text())
        if marker.get('session') != self.session:
            raise RuntimeError('World recorder belongs to a different session')
        nav = json.loads((self.support/'navigation.json').read_text())
        meta = nav.get('trial_recording', {})
        if meta.get('closed') is not False or not 0 <= time.time()-meta.get('saved_at',0) < 8:
            raise RuntimeError('Navigation checkpoint is absent, closed or stale')
        if not all(nav.get(k) for k in ('poses','costmaps','params')):
            raise RuntimeError('Navigation recorder is not capturing complete inputs')
        rows = []
        with socket.create_connection(('127.0.0.1',8772), 3) as sock:
            sock.settimeout(3)
            stream = sock.makefile()
            end = time.monotonic()+3
            while time.monotonic() < end and len(rows) < 2:
                row = json.loads(stream.readline())
                if row.get('telemetry') and row.get('telemetry_age',99) < 1:
                    rows.append(row)
        self.log('board_preflight', rows=rows)
        keys = ('ax','ay','az','gx','gy','gz')
        if len(rows) != 2 or tuple(rows[0]['telemetry'][k] for k in keys) == tuple(rows[1]['telemetry'][k] for k in keys):
            raise RuntimeError('No changing fresh IMU feedback')
        if time.time()-(self.support/'board.jsonl').stat().st_mtime > 3:
            raise RuntimeError('Passive board recorder is stale')
        home = [status['pose']['x_m'], status['pose']['y_m']]
        route = [home] + [p['xy'] for p in points]
        direct = self.card.get('direct_return', False)
        legs = list(zip(route,route[1:]))
        legs += [(p['xy'],home) for p in points] if direct else list(zip(reversed(route),list(reversed(route))[1:]))
        headings = {tuple(p['xy']):p['view_heading_deg'] for p in points}
        headings[tuple(home)] = status['pose']['heading_deg']
        for a,b in legs:
            heading = bearing(a,b)
            start_heading = headings[tuple(a)] if direct else heading
            goal_heading = headings[tuple(b)] if direct and b != home else heading
            plan = self.helper('plan_visibility_route.py', ['--start',*a,start_heading,'--goal',*b,goal_heading], True)
            grid = self.helper('capture_route_costmaps.py', returning=True)
            path_check(plan, grid, a,b)
        self.log('preflight_complete', home=home, status=status)
        latest, _ = self.health(returning=True)
        if math.dist(home,[latest['pose']['x_m'],latest['pose']['y_m']]) > .05:
            raise RuntimeError('Rover moved during preflight; obtain a new handover')
        return home

    def arm(self, deadline):
        self.deadline = deadline
        def expired():
            if not self.return_motion:
                self.aborted.set()
                self.stop()
        self.guard = threading.Timer(max(0,deadline-time.monotonic()), expired)
        self.guard.daemon = True
        self.guard.start()

    def motion(self, call, arguments, returning):
        limit = 8 if call == 'turn_in_place' else 16
        self.reserve(limit, returning)
        if returning:
            if time.monotonic() > self.deadline and not self.return_motion:
                raise RuntimeError('Return motor deadline missed; stop for recovery')
            self.return_motion = True
        def expired():
            self.aborted.set()
            self.stop()
        guard = threading.Timer(limit, expired)
        guard.daemon = True
        guard.start()
        try:
            answer = self.call(call, arguments, timeout=limit+4)
            if self.aborted.is_set() or answer.get('reason') not in (None,'arrived'):
                raise RuntimeError('Motion did not complete normally')
            status = self.wait_for_still()
            if self.aborted.is_set() or status.get('move',{}).get('reason') != 'arrived':
                raise RuntimeError('Motion completion/STOP not confirmed')
        finally:
            guard.cancel()

    def face(self, heading, returning=False):
        status, _ = self.health(returning)
        angle = (heading-status['pose']['heading_deg']+180)%360-180
        if abs(angle) <= 5:
            return
        if self.card.get('direct_return', False):
            raise RuntimeError('Direct arrival missed the observation heading; no extra trial turn')
        grid = self.helper('capture_route_costmaps.py', returning=returning)
        if not turn_check(grid, now=time.time())['ok']:
            raise RuntimeError('No fresh turn clearance')
        self.motion('turn_in_place', {'angle_deg':angle}, returning)

    def travel(self, xy, returning=False):
        status, _ = self.health(returning)
        start = [status['pose']['x_m'],status['pose']['y_m']]
        if math.dist(start,xy) <= .15:
            return
        heading = bearing(start, xy)
        if self.card.get('direct_return', False):
            if not returning:
                heading = next(p['view_heading_deg'] for p in self.card['points'] if p['xy']==xy)
        else:
            self.face(heading, returning)
            status, _ = self.health(returning)
            start = [status['pose']['x_m'],status['pose']['y_m']]
        plan = self.helper('plan_visibility_route.py',['--goal',*xy,heading], returning)
        grid = self.helper('capture_route_costmaps.py', returning=returning)
        path_check(plan, grid, start,xy)
        self.motion('drive_to', {'x_m':xy[0],'y_m':xy[1],'heading_deg':heading,'speed_ms':.34}, returning)
        status = self.call('nav_status', timeout=3)
        if math.dist([status['pose']['x_m'],status['pose']['y_m']],xy) > .3:
            raise RuntimeError('Arrival outside trial tolerance')
        if self.card.get('direct_return', False):
            self.health(returning=True)

    def inspect(self):
        self.reserve(12)
        status, _ = self.health()
        self.reserve(12)
        self.call('world_inspect', {'fresh':True,'keep_depth':True,'settle':False,'wait':True}, timeout=12)

    def begin_return(self):
        self.return_started = True
        self.log('return_started')

    def stop(self):
        self.call('stop_driving', timeout=3)

    def verify_stop(self):
        self.wait_for_still()
        return True

    def disarm(self):
        if self.guard:
            self.guard.cancel()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--card', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--support', type=Path, required=True)
    parser.add_argument('--session', required=True)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--motion-authorized', action='store_true')
    args = parser.parse_args()
    if args.execute and not args.motion_authorized:
        parser.error('Owner handover required before movement')
    args.output.mkdir(parents=True, exist_ok=False)
    card = json.loads(args.card.read_text())
    backend = Live(args.output, card, args.support, args.session)
    if args.execute:
        result = run(backend, card['points'], direct_return=card.get('direct_return',False))
    else:
        result = {'home': backend.preflight(card['points']), 'movement': False}
    (args.output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result))


if __name__ == '__main__':
    main()

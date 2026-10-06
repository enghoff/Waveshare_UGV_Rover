"""Recorded timing/geometry regressions for trial preparation, no hardware calls."""
import json
import math
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch
from experiments.entity_association.run_visibility_trial import Live, ReturnNow, run, path_check

ROOT = Path(__file__).resolve().parents[2]
CAPTURE = ROOT/'captures/visibility-morning-prepared-3'


class Clock:
    now = 0.0
    def __call__(self): return self.now


class RecordedBackend:
    def __init__(self, clock, rows, reserve=False, failed=False):
        self.clock, self.reserve, self.failed = clock, reserve, failed
        self.durations = {name: [r['finished_at']-r['started_at'] for r in rows if r['call']==name]
                          for name in ('drive_to','turn_in_place','world_inspect')}
        self.moves, self.return_at, self.stopped = [], None, False
    def preflight(self, points): return [0,0]
    def arm(self, deadline): self.deadline = deadline
    def travel(self, point, returning=False):
        self.moves.append((point,returning))
        self.clock.now += self.durations['drive_to'].pop(0)
        if self.failed: raise RuntimeError('partial drive; do not treat goal as reached')
    def face(self, heading, returning=False):
        self.clock.now += self.durations['turn_in_place'].pop(0)
    def inspect(self):
        if self.reserve:
            self.clock.now = 40
            raise ReturnNow()
        self.clock.now += self.durations['world_inspect'][0]
    def begin_return(self): self.return_at = self.clock.now
    def stop(self): self.stopped = True
    def verify_stop(self): return self.stopped
    def disarm(self): pass


class Trial(unittest.TestCase):
    def setUp(self):
        self.rows = [json.loads(line) for line in (CAPTURE/'calls.jsonl').read_text().splitlines()]
        self.points = [{'xy':[1,0], 'observe':False}, {'xy':[2,0], 'view_heading_deg':83}]

    def test_recorded_failure_and_serial_return(self):
        moves = [r for r in self.rows if r['call'] in ('drive_to','turn_in_place')]
        self.assertGreater(moves[3]['started_at']-moves[0]['started_at'],60)
        clock = Clock(); backend = RecordedBackend(clock,self.rows)
        result = run(backend,self.points,clock=clock)
        self.assertTrue(result['returned'] and result['stop_verified'])
        self.assertLess(backend.return_at,60)
        self.assertEqual([p for p,returning in backend.moves if returning], [[1,0],[0,0]])

    def test_budget_reserve_returns_without_another_question(self):
        clock = Clock(); backend = RecordedBackend(clock,self.rows,reserve=True)
        result = run(backend,self.points,clock=clock)
        self.assertEqual(backend.return_at,40)
        self.assertFalse(result['collection_complete'])
        self.assertTrue(result['returned'])

    def test_failed_motion_stops_without_guessing_a_return(self):
        clock = Clock(); backend = RecordedBackend(clock,self.rows,failed=True)
        result = run(backend,self.points,clock=clock)
        self.assertFalse(result['returned'])
        self.assertTrue(result['stop_verified'])
        self.assertFalse(any(returning for _,returning in backend.moves))

    def test_return_reserve_and_watchdog_refusal(self):
        backend = Live(Path('.'),{},Path('.'),'unused')
        backend.deadline = 60
        with patch('time.monotonic', return_value=38):
            with self.assertRaises(ReturnNow): backend.reserve(8)
            backend.reserve(8,returning=True)
            backend.aborted.set()
            with self.assertRaises(RuntimeError): backend.reserve(8,returning=True)

    def test_motion_watchdogs_and_expired_return(self):
        backend = Live(Path('.'),{},Path('.'),'unused')
        backend.deadline = 60
        backend.call = lambda name,*args,**kwargs: ({'move':{'reason':'arrived'},'pwm':[0,0]}
                                                   if name=='nav_status' else {'ok':True})
        with patch('time.monotonic',return_value=20), patch('threading.Timer') as timer:
            backend.motion('turn_in_place',{'angle_deg':60},False)
            self.assertEqual(timer.call_args.args[0],8)
            backend.motion('drive_to',{'x_m':1,'y_m':0},False)
            self.assertEqual(timer.call_args.args[0],16)
        with patch('time.monotonic',return_value=61):
            with self.assertRaises(RuntimeError): backend.motion('turn_in_place',{},True)

    def test_deadline_stop_does_not_allow_more_motion(self):
        backend = Live(Path('.'),{},Path('.'),'unused')
        stops = []
        backend.stop = lambda: stops.append(True)
        with patch('threading.Timer') as timer:
            backend.arm(60)
            timer.call_args.args[1]()
        self.assertEqual(stops,[True])
        with self.assertRaises(RuntimeError): backend.reserve(1,returning=True)

    def test_saved_grid_new_side_segment(self):
        grid = json.loads((CAPTURE/'costmaps-preflight.json').read_text(encoding='utf-8-sig'))
        card = json.loads((ROOT/'experiments/entity_association/visibility_trial_card.json').read_text())
        a,b = [p['xy'] for p in card['points']]
        plan = {'ok':True,'length_m':math.dist(a,b),'path':[*[a],b]}
        path_check(plan,grid,a,b)
        target = card['target_xy']
        angles = [math.degrees(math.atan2(target[1]-p[1],target[0]-p[0])) for p in (a,b)]
        self.assertGreater(abs(angles[0]-angles[1]),12)
        # The apparently obvious farther-west extension hits the real obstacle.
        bad = [-20.15,-14.85]
        with self.assertRaises(RuntimeError):
            path_check({'ok':True,'length_m':math.dist(a,bad),'path':[a,bad]},grid,a,bad)

    def test_wrong_goal_and_loop_rejected(self):
        grid = json.loads((CAPTURE/'costmaps-preflight.json').read_text(encoding='utf-8-sig'))
        a,b = [-19.05,-14.85],[-20,-14.2]
        for plan in [{'ok':True,'length_m':5,'path':[a,b]},
                     {'ok':True,'length_m':1.2,'path':[a,[-20,-14]]}]:
            with self.assertRaises(RuntimeError): path_check(plan,grid,a,b)


if __name__ == '__main__': unittest.main()

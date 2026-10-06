"""Direct routes preserve all admission limits and a return from either view."""
import json
from pathlib import Path
import unittest
from unittest.mock import patch
from experiments.entity_association.run_visibility_trial import Live, path_check, run
from experiments.entity_association.test_serial_visibility_trial import Clock, RecordedBackend, CAPTURE


class DirectTrial(unittest.TestCase):
    def test_direct_return_and_early_collection_end(self):
        rows=[json.loads(s) for s in (CAPTURE/'calls.jsonl').read_text().splitlines()]
        for points,reserve in [([{'xy':[1,0],'observe':False},{'xy':[2,0],'observe':False}],False),
                               ([{'xy':[1,0],'view_heading_deg':55}],True)]:
            clock=Clock(); b=RecordedBackend(clock,rows,reserve=reserve)
            result=run(b,points,clock=clock,direct_return=True)
            self.assertTrue(result['returned'] and result['stop_verified'])
            self.assertEqual([xy for xy,ret in b.moves if ret],[[0,0]])

    def test_recorded_actual_heading_paths_and_both_returns(self):
        root=Path(__file__).resolve().parents[2]/'captures/entity-target-audit-20261006'
        a=json.loads((root/'angled-view-check.json').read_text())
        self.assertEqual(len(a['routes']),4)
        for r in a['routes']:
            path_check(r['plan'],a['grid'],r['start'][:2],r['goal'][:2])
        original=json.loads((root/'direct-route-check.json').read_text())
        failed=original['routes'][2]
        with self.assertRaises(RuntimeError):
            path_check(failed['plan'],original['grid'],failed['start'][:2],failed['goal'][:2])

    def test_no_separate_turn_and_localization_checked_after_arrival(self):
        b=Live(Path('.'),{'direct_return':True,'points':[{'xy':[1,0],'view_heading_deg':55}]},Path('.'),'unused')
        states=iter([({'pose':{'x_m':0,'y_m':0,'heading_deg':142}},{}),
                     ({'pose':{'x_m':1,'y_m':0,'heading_deg':55}}, {})])
        checks=[]; moves=[]
        def health(*a,**k): checks.append(True); return next(states)
        b.health=health; b.helper=lambda *a,**k:{}
        b.motion=lambda name,args,ret:moves.append((name,args))
        b.call=lambda *a,**k:{'pose':{'x_m':1,'y_m':0}}
        b.face=lambda *a,**k:self.fail('preliminary turn must not run')
        with patch('experiments.entity_association.run_visibility_trial.path_check'):
            b.travel([1,0])
        self.assertEqual(len(checks),2)
        self.assertEqual(moves,[('drive_to',{'x_m':1,'y_m':0,'heading_deg':55,'speed_ms':.34})])

    def test_missed_heading_cannot_trigger_extra_turn(self):
        b=Live(Path('.'),{'direct_return':True},Path('.'),'unused')
        b.health=lambda *a,**k:({'pose':{'heading_deg':20}}, {})
        b.motion=lambda *a,**k:self.fail('must not turn')
        with self.assertRaisesRegex(RuntimeError,'missed'):
            b.face(55)


if __name__=='__main__':unittest.main()

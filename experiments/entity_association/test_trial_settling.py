"""Replay the real premature completion; the appended rest window is synthetic."""
import json
from pathlib import Path
import unittest
from unittest.mock import patch
from experiments.entity_association.run_visibility_trial import Live


class Settling(unittest.TestCase):
    def setUp(self):
        self.record = json.loads(Path(__file__).with_name('fixtures').joinpath(
            'premature_turn_completion.json').read_text())
        self.backend = Live(Path('.'), {}, Path('.'), 'unused')
        self.now = 0

    def wait(self, statuses):
        def call(*args, **kwargs):
            return statuses[min(int(round(self.now*10)), len(statuses)-1)]
        def sleep(seconds): self.now += seconds
        self.backend.call = call
        with patch('time.monotonic', side_effect=lambda:self.now), patch('time.sleep',side_effect=sleep):
            return self.backend.wait_for_still()

    def test_real_reply_passed_old_gate_despite_112_degrees_per_second(self):
        s = self.record['moving'][0]
        self.assertFalse(s['driving'])
        self.assertEqual(s['pwm'], [0,0])
        self.assertEqual(s['move']['reason'], 'arrived')
        self.assertEqual(s['turn_dps'], -112.6)
        with self.assertRaisesRegex(RuntimeError, 'remain stopped'):
            self.wait(self.record['moving'])

    def test_waits_for_continuous_rest_after_recorded_motion(self):
        final = self.record['later_stopped']
        self.assertEqual(self.wait(self.record['moving']+[final]*8), final)
        self.assertGreaterEqual(self.now, .7)

    def test_a_single_zero_does_not_establish_rest(self):
        with self.assertRaises(RuntimeError):
            self.wait([self.record['later_stopped']]+self.record['moving'])

    def test_stale_feedback_cannot_verify_stop(self):
        with self.assertRaises(RuntimeError):
            self.wait([{**self.record['later_stopped'], 'transform_age_s':1}])

    def test_quiet_feedback_does_not_waive_heading_check(self):
        s = self.record['later_stopped']
        self.backend.card = {'map_id':s['map_id']}
        self.backend.wait_for_still = lambda:s
        self.backend.log = lambda *a,**k:None
        with patch('experiments.entity_association.run_visibility_trial.rpc', return_value={
                'trusted':True,'score':.989,'moved_m':.102,'turned_deg':12.5}):
            with self.assertRaisesRegex(RuntimeError,'localization'):
                self.backend.health()


if __name__ == '__main__': unittest.main()

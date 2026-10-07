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

    def test_a_rover_not_driven_since_boot_is_at_rest(self):
        # 2026-10-07: after a reboot navigation had sent the board nothing, its
        # motor reading was empty, and the preflight called the rover in use.
        final = {**self.record['later_stopped'], 'pwm': None}
        self.assertEqual(self.wait([final]*8), final)

    def test_a_motor_command_still_held_is_not_rest(self):
        with self.assertRaises(RuntimeError):
            self.wait([{**self.record['later_stopped'], 'pwm': [40, 40]}]*40)

    def test_stale_feedback_cannot_verify_stop(self):
        with self.assertRaises(RuntimeError):
            self.wait([{**self.record['later_stopped'], 'transform_age_s':1}])

    def test_navigation_lagging_a_decisive_scan_is_judged_by_the_scan(self):
        # 2026-10-07, the trial's first viewpoint: the measurement as recorded.
        s = self.record['later_stopped']
        self.backend.card = {'map_id':s['map_id']}
        self.backend.wait_for_still = lambda:s
        self.backend.log = lambda *a,**k:None
        self.backend.call = lambda *a,**k:{'reading_age_s':0,'percent':70}
        real = {'trusted':True,'score':.993,'guess_score':.487,'rival':.712,
                'moved_m':.06,'turned_deg':12.0,'x_m':-19.036,'y_m':-14.986,
                'heading_deg':59.1}
        with patch('experiments.entity_association.run_visibility_trial.rpc', return_value=real):
            status, _ = self.backend.health()
        self.assertEqual(status['pose']['heading_deg'], 59.1)
        self.assertEqual(status['pose_from'], 'scan')

    def test_a_decisive_scan_inside_the_limit_still_sets_the_pose(self):
        # 2026-10-07, C: 10.0 deg apart, inside the old limit, and the view was
        # judged from navigation's 39.6 rather than the scan's 49.6.
        s = self.record['later_stopped']
        self.backend.card = {'map_id':s['map_id']}
        self.backend.wait_for_still = lambda:s
        self.backend.log = lambda *a,**k:None
        self.backend.call = lambda *a,**k:{'reading_age_s':0,'percent':70}
        real = {'trusted':True,'score':.978,'guess_score':.586,'rival':.694,
                'moved_m':.10,'turned_deg':10.0,'x_m':-20.0,'y_m':-14.2,'heading_deg':49.6}
        with patch('experiments.entity_association.run_visibility_trial.rpc', return_value=real):
            status, _ = self.backend.health()
        self.assertEqual((status['pose']['heading_deg'], status['pose_from']), (49.6, 'scan'))

    def test_an_ambiguous_scan_still_refuses(self):
        s = self.record['later_stopped']
        self.backend.card = {'map_id':s['map_id']}
        self.backend.wait_for_still = lambda:s
        self.backend.log = lambda *a,**k:None
        for close in ({'guess_score':.80}, {'rival':.90}, {'moved_m':.4}, {'trusted':False}):
            measured = {'trusted':True,'score':.993,'guess_score':.487,'rival':.712,
                        'moved_m':.06,'turned_deg':12.0,'x_m':-19.036,'y_m':-14.986,
                        'heading_deg':59.1, **close}
            with patch('experiments.entity_association.run_visibility_trial.rpc', return_value=measured):
                with self.assertRaisesRegex(RuntimeError,'localization'):
                    self.backend.health()

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

"""Reproduce interrupted nav_record loss, then retain that real episode on close."""
import ast
import json
from pathlib import Path
import tempfile
import types
import unittest
from unittest.mock import patch
from experiments.entity_association.record_navigation_trial import collect

ROOT = Path(__file__).resolve().parents[2]
RECORDING = ROOT/'captures/2026-10-05-visibility-drive/support/navigation.json'


class Recording(unittest.TestCase):
    def setUp(self):
        self.episode = json.loads(RECORDING.read_text())
        self.node = types.SimpleNamespace(episode=lambda: self.episode,
            fetch_params=lambda: None, fetch_global=lambda: None, params={}, started=0)
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.output = Path(self.folder.name)/'navigation.json'

    def interrupt(self, *_args, **_kwargs):
        raise KeyboardInterrupt

    def test_original_interrupt_loses_real_episode(self):
        tree = ast.parse((ROOT/'ros_nav/nav_record.py').read_text())
        main = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'main')
        import argparse, math, os, sys, time
        namespace = dict(__doc__='record', argparse=argparse, math=math, os=os,
            sys=sys, time=time, json=json, Recorder=lambda: self.node,
            rclpy=types.SimpleNamespace(init=lambda: None, spin_once=self.interrupt),
            threading=types.SimpleNamespace(Timer=lambda *a: types.SimpleNamespace(start=lambda: None)))
        exec(compile(ast.Module(body=[main], type_ignores=[]), 'original_nav_record_main', 'exec'), namespace)
        with patch.object(sys, 'argv', ['recorder', '--out', str(self.output)]):
            with self.assertRaises(KeyboardInterrupt):
                namespace['main']()
        self.assertFalse(self.output.exists())

    def test_interrupt_preserves_real_episode(self):
        collect(self.node, self.interrupt, self.output, 300, lambda: False)
        saved = json.loads(self.output.read_text())
        self.assertTrue(saved.pop('trial_recording')['closed'])
        self.assertEqual(saved, self.episode)

    def test_explicit_close_preserves_final_commands(self):
        collect(self.node, lambda n: self.fail('should already be stopped'),
                self.output, 300, lambda: True)
        saved = json.loads(self.output.read_text())
        self.assertEqual(saved['commands'], self.episode['commands'])
        self.assertEqual(saved['trial_recording']['reason'], 'requested')

    def test_fault_is_saved_but_not_claimed_closed(self):
        def fail(node):
            raise RuntimeError('DDS failure')
        with self.assertRaises(RuntimeError):
            collect(self.node, fail, self.output, 300, lambda: False)
        self.assertFalse(json.loads(self.output.read_text())['trial_recording']['closed'])

    def test_checkpoint_precedes_first_spin(self):
        def inspect(node):
            saved = json.loads(self.output.read_text())
            self.assertFalse(saved['trial_recording']['closed'])
            self.assertEqual(saved['poses'], self.episode['poses'])
            raise KeyboardInterrupt
        collect(self.node, inspect, self.output, 300, lambda: False)


if __name__ == '__main__':
    unittest.main()

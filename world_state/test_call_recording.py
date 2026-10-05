"""Recorder failures must not change a range answer or escape into inspection."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock
from types import SimpleNamespace
from .call_recording import CallRecording, REQUEST
from .inspector import Inspector
from .store import WorldStore


class RecordingFailureTests(unittest.TestCase):
    def test_stop_snapshot_failure_does_not_leave_a_closed_recorder_active(self):
        with tempfile.TemporaryDirectory() as root:
            store=WorldStore(root)
            try:
                inspector=Inspector(store,None,lambda:None,lambda:None)
                inspector._call_recording=Mock()
                inspector._call_recording.close.side_effect=OSError('snapshot failed')
                inspector._call_recording_request='old-session'
                self.assertTrue(inspector.settle()['ok'])
                self.assertIsNone(inspector._call_recording)
                self.assertIn('snapshot failed',inspector.recording_status()['error'])
            finally:store.close()

    def test_depth_projection_logging_preserves_measured_ranges(self):
        import numpy as np
        from .depth_client import DepthMap, Lens
        with tempfile.TemporaryDirectory() as root:
            store=WorldStore(root)
            try:
                lens=Lens(fx=500.3,fy=500.17,cx=321.23,cy=190.69,width=640,height=360)
                depth=DepthMap(millimetres=np.full((360,640),2500,dtype=np.uint16).tobytes(),
                               width=640,height=360,dtype='uint16')
                ranger=SimpleNamespace(depth_map=lambda **kwargs:depth)
                inspector=Inspector(store,None,lambda:None,lambda:None,ranger=ranger)
                capture={'frame_id':'fixture','frame_size':(640,480),'pan':0,'tilt':20,'turn_rate_dps':0}
                regions=[SimpleNamespace(bbox=[.4,.3,.6,.6],outline=b'')]
                original=inspector._ranges_read(capture,regions,lens,(640,480))
                recorder=CallRecording(store,'depth');inspector._call_recording=recorder
                recorded=inspector._ranges_read(capture,regions,lens,(640,480))
                self.assertEqual(original,recorded)
                recorder.close(store)
                rows=[json.loads(s) for s in Path(root,'recordings/depth/events.jsonl').read_text().splitlines()]
                projection=next(r for r in rows if r['kind']=='depth_projection')
                self.assertEqual(projection['frame_id'],'fixture')
                self.assertEqual(projection['answers'][0]['range_m'],original[0][0].range_m)
            finally:store.close()

    def test_invalid_name_cannot_create_outside_directory(self):
        with tempfile.TemporaryDirectory() as root:
            store=WorldStore(root)
            try:
                inspector=Inspector(store,None,lambda:None,lambda:None)
                Path(root,REQUEST).write_text(json.dumps({'session':'../escape'}))
                answer=inspector.settle()
                self.assertTrue(answer['ok'])
                self.assertIn('ValueError',inspector.recording_status()['error'])
                self.assertFalse(Path(root,'recordings').exists())
            finally:store.close()

    def test_disk_error_preserves_actual_reach_and_invalidates_recording(self):
        with tempfile.TemporaryDirectory() as root:
            store=WorldStore(root)
            try:
                recorder=CallRecording(store,'disk-error')
                original_file=recorder.file
                recorder.file=Mock()
                recorder.file.write.side_effect=OSError('disk full')
                actual=Mock(return_value=1.234)
                self.assertEqual(recorder.reach(actual,None)(1,2,3),1.234)
                actual.assert_called_once_with(1,2,3)
                self.assertIn('disk full',recorder.error)
                recorder.file=original_file
                recorder.close(store)
                manifest=json.loads(Path(root,'recordings/disk-error/manifest.json').read_text())
                self.assertFalse(manifest['complete'])
            finally:store.close()

    def test_reused_name_is_reported_and_existing_recording_not_overwritten(self):
        with tempfile.TemporaryDirectory() as root:
            store=WorldStore(root)
            try:
                recorder=CallRecording(store,'existing');recorder.close(store)
                path=Path(root,'recordings/existing/manifest.json');before=path.read_bytes()
                inspector=Inspector(store,None,lambda:None,lambda:None)
                Path(root,REQUEST).write_text(json.dumps({'session':'existing'}))
                self.assertTrue(inspector.settle()['ok'])
                self.assertIn('FileExistsError',inspector.recording_status()['error'])
                self.assertEqual(path.read_bytes(),before)
            finally:store.close()


if __name__=='__main__':unittest.main()

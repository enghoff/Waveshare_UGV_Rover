"""Local evidence-drive admission checks; never sends commands to a rover."""
from __future__ import annotations

MOTION = {'drive_to', 'turn_in_place'}


def check_request(call, arguments, history, now, *, motion_active=False,
                  returning=False, outbound_seconds=60.0):
    fresh_look = call == 'world_inspect' and arguments.get('fresh', False)
    if motion_active and (call in MOTION or fresh_look):
        raise RuntimeError('Wait for the active motion call to finish before moving or inspecting.')
    if returning or not (call in MOTION or fresh_look):
        return
    starts = [event['started_at'] for event in history if event['call'] in MOTION]
    if not starts:
        return
    allowance = 16.0 if call in MOTION else 8.0
    if now + allowance > min(starts) + outbound_seconds:
        raise RuntimeError('Outbound budget exhausted: begin the checked return; no further outbound action.')

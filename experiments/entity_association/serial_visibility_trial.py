"""Serial trial policy. Backend owns live checks; no conversational waits in a run.

The 60-second limit is a deadline for the first return motor command, not merely
for declining another observation. Slow/failed checks cause STOP, never unchecked
motion. A partial move requires recovery rather than treating its goal as reached.
"""
import math
import time


class ReturnNow(Exception):
    pass


def bearing(a, b):
    return math.degrees(math.atan2(b[1]-a[1], b[0]-a[0]))


def run(backend, points, *, clock=time.monotonic):
    home = backend.preflight(points)
    reached = [home]
    start = clock()
    backend.arm(start + 60)
    result = {'returned': False, 'reached': [], 'collection_complete': False}
    try:
        try:
            for point in points:
                backend.travel(point['xy'], returning=False)
                reached.append(point['xy'])
                result['reached'].append(point['xy'])
                if point.get('observe', True):
                    backend.face(point['view_heading_deg'], returning=False)
                    backend.inspect()
            result['collection_complete'] = True
        except ReturnNow:
            result['collection_end'] = 'return reserve reached'
        backend.begin_return()
        # Current point is the last successfully reached one. A failed or timed
        # out motion raises a different exception and goes directly to STOP.
        for target in reversed(reached[:-1]):
            backend.travel(target, returning=True)
        result['returned'] = True
    except Exception as error:
        result['error'] = str(error)
    finally:
        try:
            backend.stop()
            result['stop_verified'] = backend.verify_stop()
        except Exception as error:
            result['stop_verified'] = False
            result['stop_error'] = str(error)
        if result['stop_verified']:
            backend.disarm()
    result['elapsed_s'] = clock()-start
    return result

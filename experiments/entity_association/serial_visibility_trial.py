"""Serial trial policy. Backend owns live checks; no conversational waits in a run.

The RETURN_BY_S limit is a deadline for the first return motor command, not
merely for declining another observation. Slow/failed checks cause STOP, never unchecked
motion. A partial move requires recovery rather than treating its goal as reached.
"""
import math
import time


#: The first return motor command must start by this many seconds after the
#: outbound leg begins. It was 60 until 2026-10-07, when a run that reached both
#: viewpoints and centred the painting at each ran out at C before its look:
#: two drives, two centring turns and two looks with their checks do not fit
#: in 60 s on this chassis.
RETURN_BY_S = 90


class ReturnNow(Exception):
    pass


def bearing(a, b):
    return math.degrees(math.atan2(b[1]-a[1], b[0]-a[0]))


def run(backend, points, *, clock=time.monotonic, direct_return=False):
    home = backend.preflight(points)
    reached = [home]
    start = clock()
    backend.arm(start + RETURN_BY_S)
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
        targets = [home] if direct_return and len(reached)>1 else reversed(reached[:-1])
        for target in targets:
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

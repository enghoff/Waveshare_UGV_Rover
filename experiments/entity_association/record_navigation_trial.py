"""Read-only trial recorder with atomic checkpoints and explicit close acknowledgement.

Run on Orin with the ROS environment sourced and --nav-source ~/ugv/ros_nav.
Creating --stop-file requests a flush and exit. No publishers or motion commands.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import sys
import threading
import time


def save(node, output, *, closed, reason):
    episode = dict(node.episode())
    episode['trial_recording'] = {'closed': closed, 'reason': reason,
                                  'saved_at': time.time()}
    temporary = output.with_suffix(output.suffix + '.tmp')
    with temporary.open('w') as handle:
        json.dump(episode, handle)
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(temporary, output)


def collect(node, spin, output, seconds, stop, *, clock=time.monotonic,
            checkpoint_seconds=5):
    end, next_save = clock() + seconds, clock()
    reason = 'duration'
    try:
        while clock() < end:
            if stop():
                reason = 'requested'
                break
            if clock() >= next_save:
                save(node, output, closed=False, reason='recording')
                next_save = clock() + checkpoint_seconds
            spin(node)
    except KeyboardInterrupt:
        reason = 'interrupted'
    except BaseException:
        save(node, output, closed=False, reason='error')
        raise
    save(node, output, closed=True, reason=reason)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--nav-source', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--stop-file', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=300)
    args = parser.parse_args()
    if args.out.exists() or args.stop_file.exists() or not 0 < args.seconds <= 600:
        parser.error('Use fresh output/stop paths and a duration in (0,600].')
    args.out.parent.mkdir(parents=True, exist_ok=True)
    sys.path.insert(0, str(args.nav_source))
    import nav_record
    stopped = threading.Event()
    # A wedged DDS call cannot block indefinitely. Its last complete checkpoint
    # remains marked open, so a forced exit cannot masquerade as final-STOP proof.
    watchdog = threading.Timer(args.seconds + 30, lambda: os._exit(2))
    watchdog.daemon = True
    watchdog.start()
    nav_record.rclpy.init()
    for number in (signal.SIGINT, signal.SIGTERM):
        signal.signal(number, lambda *_: stopped.set())
    node = nav_record.Recorder()
    try:
        node.fetch_params()
        node.fetch_global()
        collect(node, lambda n: nav_record.rclpy.spin_once(n, timeout_sec=.05),
                args.out, args.seconds, lambda: stopped.is_set() or args.stop_file.exists())
    finally:
        node.destroy_node()
        nav_record.rclpy.shutdown()
        watchdog.cancel()
    print(json.dumps({'closed': True, 'path': str(args.out)}), flush=True)


if __name__ == '__main__':
    main()

#!/bin/sh
# Reload the drive console after deploying a new drive_web.py.
#
#     ssh bpi-m4zero ~/ugv/drive_web/restart.sh
#
# Separate from run_drive_web.sh so that its own command line cannot match the
# pattern it greps for. This is worth repeating in every directory that has a
# supervisor, because the repository has made the mistake twice: a
# `pkill -f drive_web/drive_web.py` typed into an ssh command *matches that ssh
# command*, since the pattern is part of its own text. The session dies
# mid-sentence, the output vanishes, and it reads as the service failing to come
# back when it is merely restarting. Here the pattern lives in a file, where
# nothing else can quote it.
#
# Only the server is killed, never the supervisor: the supervisor is what notices
# and starts the next one, and it holds the arguments this was started with.
DIR="$(cd "$(dirname "$0")" && pwd)"

if pgrep -f 'run_drive_web.sh' > /dev/null; then
    pkill -f "$DIR/drive_web.py" || {
        echo "nothing was running; the supervisor will start one within 15 s" >&2
    }
else
    # Nothing is supervising it -- first install, or after `pkill -f run_drive_web.sh`.
    setsid nohup "$DIR/run_drive_web.sh" > /dev/null 2>&1 < /dev/null &
fi

# urllib rather than curl, because python3 is the one interpreter this board is
# certain to have. /health is this process answering, not the daemon.
python3 - "$DIR" <<'PY'
import json, sys, time, urllib.error, urllib.request

sys.path.insert(0, sys.argv[1])
from drive_session import ROVER_HTTP_PORT

for _ in range(20):
    time.sleep(1)
    try:
        with urllib.request.urlopen(
                f"http://127.0.0.1:{ROVER_HTTP_PORT}/health", timeout=3) as reply:
            print(json.dumps(json.loads(reply.read()), indent=None))
            sys.exit(0)
    except (urllib.error.URLError, OSError, ValueError):
        continue
print(f"the drive console did not answer within 20 s -- see "
      f"{sys.argv[1]}/drive_web.log", file=sys.stderr)
sys.exit(1)
PY

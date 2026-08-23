#!/bin/sh
# Reload the depth service after deploying a new depth_server.py.
#
#     ssh rpi ~/ugv/oak_depth/restart.sh
#
# Separate from run_oak_depth.sh so that its own command line cannot match the
# pattern it greps for. This is worth repeating in every directory that has a
# supervisor, because the repository has made the mistake twice: a
# `pkill -f oak_depth/depth_server.py` typed into an ssh command *matches that ssh
# command*, since the pattern is part of its own text. The session dies
# mid-sentence, the output vanishes, and it reads as the service failing to come
# back when it is merely restarting. Here the pattern lives in a file, where
# nothing else can quote it.
#
# Only the server is killed, never the supervisor: the supervisor is what notices
# and starts the next one, and it holds the arguments this was started with.
DIR="$(cd "$(dirname "$0")" && pwd)"

pkill -f "$DIR/depth_server.py" || {
    echo "nothing was running; the supervisor will start one within 15 s" >&2
}

# Not a process restart time: the host uploads firmware to the VPU and the stereo
# pipeline starts, which is several seconds. urllib rather than curl, because
# python3 is the one interpreter this board is certain to have.
python3 - "$DIR" <<'PY'
import json, sys, time, urllib.error, urllib.request

for _ in range(40):
    time.sleep(2)
    try:
        with urllib.request.urlopen("http://127.0.0.1:8770/health", timeout=3) as reply:
            print(json.dumps(json.loads(reply.read()), indent=None))
            sys.exit(0)
    except (urllib.error.URLError, OSError, ValueError):
        continue
print(f"the depth service did not answer within 80 s -- see "
      f"{sys.argv[1]}/oak_depth.log", file=sys.stderr)
sys.exit(1)
PY

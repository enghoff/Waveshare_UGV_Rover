#!/bin/sh
# Reload the depth service after deploying a new depth_server.py.
#
#     ssh orin '~/ugv/oak_depth/restart.sh'
#     ssh orin '~/ugv/oak_depth/restart.sh --supervisor'   # after changing
#                                                          # run_oak_depth.sh
#
# Separate from run_oak_depth.sh so that its own command line cannot match the
# pattern it greps for. This is worth repeating in every directory that has a
# supervisor, because the repository has now made the mistake three times: a
# `pkill -f oak_depth/depth_server.py` typed into an ssh command *matches that ssh
# command*, since the pattern is part of its own text. The session dies
# mid-sentence, the output vanishes, and it reads as the service failing to come
# back when it is merely restarting. The third time was the deploy manifest,
# which carried the supervisor replacement inline and killed its own ssh with
# exit 255 on the first deploy of this component to the Jetson. Here the patterns
# live in a file, where nothing else can quote them.
#
# Only the server is killed, never the supervisor: the supervisor is what notices
# and starts the next one, and it holds the arguments this was started with. The
# exception is --supervisor, which replaces the supervisor as well, and is needed
# when run_oak_depth.sh itself has changed -- a shell holds a parsed copy of the
# script it is running, so a new file on disk is not a new supervisor.
DIR="$(cd "$(dirname "$0")" && pwd)"

if [ "${1:-}" = "--supervisor" ]; then
    echo "--- replacing the supervisor"
    pkill -f 'oak_depth/run_oak_depth[.]sh' || true
    sleep 2
fi

if pgrep -f 'oak_depth/run_oak_depth[.]sh' > /dev/null; then
    # Supervised: kill the server and let the supervisor bring the next one up
    # with the arguments it was started with, which is where --fps and
    # --decimation live.
    pkill -f "$DIR/depth_server.py" || {
        echo "nothing was running; the supervisor will start one within 15 s" >&2
    }
else
    # Nothing is supervising it -- a first install, or a supervisor that was
    # stopped. Start it the way boot does, reading the flags from the crontab
    # rather than inventing a second set here.
    echo "--- no supervisor; starting one as the crontab does"
    args="$(crontab -l 2>/dev/null | sed -n 's|^@reboot .*run_oak_depth\.sh *||p' | head -1)"
    # shellcheck disable=SC2086 -- word splitting is the point: these are flags.
    setsid nohup "$DIR/run_oak_depth.sh" $args > /dev/null 2>&1 < /dev/null &
fi

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

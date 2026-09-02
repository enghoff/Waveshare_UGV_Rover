#!/bin/sh
# Reload the perception sidecar, and wait until it can actually answer.
#
#     ssh orin '~/ugv/world_state/restart_perception.sh'
#     ssh orin '~/ugv/world_state/restart_perception.sh --supervisor'
#
# Separate from run_perception.sh for a reason this repository has learned three
# times: a `pkill -f` typed into an ssh command matches that ssh command, the
# session dies
# mid-sentence, and it reads as the service failing rather than as the shell
# killing itself. The patterns live in files, where nothing else quotes them.
#
# Only the server is killed, never the supervisor: the supervisor is what starts
# the next one and it holds the arguments. --supervisor replaces it too, which is
# needed when run_perception.sh itself has changed, because a shell holds a
# parsed copy of the script it is running.

DIR="$(cd "$(dirname "$0")" && pwd)"

if [ ! -f "$DIR/vendor/yoloe-11s-seg-objectness.onnx" ]; then
    echo "no perception models at $DIR/vendor -- run install_perception.sh first" >&2
    exit 1
fi

if [ "${1:-}" = "--supervisor" ]; then
    echo "--- replacing the supervisor"
    pkill -f 'world_state/run_perception[.]sh' || true
    sleep 2
fi

if pgrep -f 'world_state/run_perception[.]sh' > /dev/null; then
    pkill -f 'world_state/perception_server[.]py' || {
        echo "nothing was running; the supervisor will start one within 15 s" >&2
    }
else
    echo "--- no supervisor; starting one as the crontab does"
    setsid nohup "$DIR/run_perception.sh" > /dev/null 2>&1 < /dev/null &
fi

# Health is cheap and does not load the models, so this comes back in seconds
# rather than in the seven the first look costs. urllib rather than curl, because
# python3 is the one interpreter this rover is certain to have.
python3 - "$DIR" <<'PY'
import json, sys, time, urllib.error, urllib.request

for _ in range(30):
    time.sleep(1)
    try:
        with urllib.request.urlopen("http://127.0.0.1:8776/health",
                                    timeout=3) as reply:
            health = json.loads(reply.read())
    except (urllib.error.URLError, OSError, ValueError):
        continue
    if health.get("ready"):
        print(f"perception: ready on {health.get('backend')}")
        sys.exit(0)
    print(f"perception: answering, but {health.get('detail')}", file=sys.stderr)
    sys.exit(1)
print(f"the perception sidecar did not answer within 30 s -- see "
      f"{sys.argv[1]}/perception.log", file=sys.stderr)
sys.exit(1)
PY

#!/bin/sh
# Reload the Cosmos sidecar, and wait until it can actually answer.
#
#     ssh orin '~/ugv/world_state/restart.sh'
#     ssh orin '~/ugv/world_state/restart.sh --supervisor'   # after changing
#                                                            # run_cosmos.sh
#
# Separate from run_cosmos.sh so that its own command line cannot match the
# pattern it greps for. The repository has made that mistake three times: a
# `pkill -f` typed into an ssh command matches that ssh command, the session dies
# mid-sentence, and it reads as the service failing rather than as the shell
# killing itself. The patterns live in files, where nothing else quotes them.
#
# Only the server is killed, never the supervisor: the supervisor is what starts
# the next one and it holds the arguments. --supervisor replaces it too, which is
# needed when run_cosmos.sh itself has changed, because a shell holds a parsed
# copy of the script it is running.

DIR="$(cd "$(dirname "$0")" && pwd)"

if [ ! -f "$DIR/vendor/Cosmos-Reason2-2B-Q4_K_M.gguf" ]; then
    echo "no model installed at $DIR/vendor -- run install.sh first" >&2
    exit 1
fi

if [ "${1:-}" = "--supervisor" ]; then
    echo "--- replacing the supervisor"
    pkill -f 'world_state/run_cosmos[.]sh' || true
    sleep 2
fi

if pgrep -f 'world_state/run_cosmos[.]sh' > /dev/null; then
    pkill -f "$DIR/vendor/llama/llama-server" || {
        echo "nothing was running; the supervisor will start one within 15 s" >&2
    }
else
    echo "--- no supervisor; starting one as the crontab does"
    setsid nohup "$DIR/run_cosmos.sh" > /dev/null 2>&1 < /dev/null &
fi

# Not a process restart time. Two gigabytes of weights are read off NVMe and the
# vision projector is loaded with them, which is tens of seconds on this board --
# and the first inference after that is slower again. urllib rather than curl,
# because python3 is the one interpreter this rover is certain to have.
python3 - "$DIR" <<'PY'
import json, sys, time, urllib.error, urllib.request

for _ in range(60):
    time.sleep(2)
    try:
        with urllib.request.urlopen("http://127.0.0.1:8775/health",
                                    timeout=3) as reply:
            health = json.loads(reply.read())
    except (urllib.error.URLError, OSError, ValueError):
        continue
    if health.get("status") == "ok":
        try:
            with urllib.request.urlopen("http://127.0.0.1:8775/v1/models",
                                        timeout=5) as reply:
                models = json.loads(reply.read()).get("data") or []
            print("cosmos:", ", ".join(str(m.get("id")) for m in models) or "?")
        except (urllib.error.URLError, OSError, ValueError) as error:
            print(f"cosmos: healthy, but the model list failed: {error}")
        sys.exit(0)
print(f"the Cosmos sidecar did not become healthy within 120 s -- see "
      f"{sys.argv[1]}/cosmos.log", file=sys.stderr)
sys.exit(1)
PY

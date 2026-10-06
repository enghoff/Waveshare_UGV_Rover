#!/bin/bash
# Diagnostic launcher only; no motion without both explicit Python flags.
. "$HOME/ugv/ros_nav/env.sh"
. "$HOME/ugv/ros_nav/dds.sh"
export PYTHONPATH="$HOME/ugv/ros_nav${PYTHONPATH:+:$PYTHONPATH}"
trial_dir="$(cd "$(dirname "$0")" && pwd)"
exec timeout --kill-after=5 360 python3 "$trial_dir/prepare_visibility_trial.py" \
  --nav-source "$HOME/ugv/ros_nav" --card "$trial_dir/visibility_trial_card.json" "$@"

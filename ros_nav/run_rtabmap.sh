#!/bin/bash
# Start RTAB-Map beside the rest of the stack, out of /opt/ros/jazzy.
#
#     ~/ugv/ros_nav/run_rtabmap.sh              # compare: publishes no transform
#     ~/ugv/ros_nav/run_rtabmap.sh --primary    # owns map -> odom instead of
#                                               # slam_toolbox
#     ~/ugv/ros_nav/run_rtabmap.sh --keep-db    # carry the previous drive's graph
#
# Normally started by slam.launch.py rather than by hand -- `rtabmap:=compare`
# there is what runs this.
#
# The environment this needs is not the one the launch file will hand it: every
# other node in this stack runs from the RoboStack conda environment, RTAB-Map
# runs from Ubuntu's ROS 2 packages, and the two must never be on one process's
# library path. native.sh is what steps between them, and says why at length.

set -eu

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

MODE=compare
DELETE_DB=--delete_db_on_start
while [ $# -gt 0 ]; do
    case "$1" in
        --primary)  MODE=primary ;;
        --compare)  MODE=compare ;;
        --keep-db)  DELETE_DB= ;;
        *) echo "run_rtabmap.sh: unknown argument $1" >&2; exit 2 ;;
    esac
    shift
done

# The graph lives with the maps rather than in ~/.ros, because `maps/` is on the
# manifest's preserve list and a deploy therefore cannot delete a drive's data.
DB_DIR="$DIR/maps"
mkdir -p "$DB_DIR"
DB="$DB_DIR/rtabmap.db"

# **Whether RTAB-Map is allowed to publish `map -> odom` is the whole safety
# question here**, and it is why `compare` is the default.
#
# A frame in TF has exactly one parent. slam_toolbox is already publishing
# `map -> odom` and Nav2 is steering on it, so a second publisher does not give
# you two opinions to choose between -- it gives you one `odom` frame whose
# parent transform flickers between two answers at whatever rate the two happen
# to publish, and a controller reading the tree gets whichever landed last.
#
# In `compare` the transform is off and RTAB-Map is a passenger: it reads /scan
# and takes the odometry off TF, builds its own graph, and publishes it as topics
# under /rtabmap that slam_compare.py reads. Nothing it decides can move the
# rover. `--primary` is the cutover, and slam.launch.py will not start
# slam_toolbox alongside it.
#
# The transform is not the only thing that has to move at a cutover, and the
# other half is quieter about being missing. Everything downstream reads the
# occupancy grid off `/map`: Nav2's global costmap has a static layer subscribed
# to it, and nav_bridge answers `map_png` from it. Left in the namespace the grid
# is `/rtabmap/map`, which nothing subscribes to -- and the way that fails is not
# an error anywhere. The costmap comes up with every cell unknown, the global
# planner will not cross unknown space, and the rover refuses every goal with a
# planner failure while `ros2 node list` shows a stack that is entirely healthy.
# So `--primary` also puts the grid where the rest of the stack is already
# looking, and `compare` deliberately does not.
if [ "$MODE" = primary ]; then
    PUBLISH_TF=true
    MAP_REMAP=(-r map:=/map)
    GRID=/map
else
    PUBLISH_TF=false
    MAP_REMAP=()
    GRID=/rtabmap/map
fi

echo "run_rtabmap.sh: mode=$MODE publish_tf=$PUBLISH_TF grid=$GRID" \
     "db=$DB${DELETE_DB:+ (fresh)}"

# `--delete_db_on_start` comes before `--ros-args` because it is RTAB-Map's own
# argument rather than rcl's, and rcl takes everything after the separator.
#
# The namespace is what keeps this out of the way in compare mode: RTAB-Map
# publishes its occupancy grid as `map`, which unqualified would be the same
# `/map` slam_toolbox publishes and Nav2's global costmap subscribes to. Under
# /rtabmap it is a second opinion nobody is steering on. The namespace stays in
# primary mode as well -- the node keeps its own name for everything else it
# publishes -- and only the grid is remapped back out of it, by MAP_REMAP above.
#
# The remap is `map:=/map` rather than `/rtabmap/map:=/map` because a remap rule
# is matched after the namespace is applied: the plain name on the left is this
# node's `map`, and the absolute name on the right is where it is to appear.
# shellcheck disable=SC2086
exec "$DIR/native.sh" ros2 run rtabmap_slam rtabmap $DELETE_DB \
    --ros-args \
        -r __ns:=/rtabmap \
        -r __node:=rtabmap \
        "${MAP_REMAP[@]+"${MAP_REMAP[@]}"}" \
        --params-file "$DIR/config/rtabmap.yaml" \
        -p publish_tf:="$PUBLISH_TF" \
        -p database_path:="$DB" \
        -r scan:=/scan

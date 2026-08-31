# Jetson Orin SLAM and navigation strategy

_Last reviewed: 2026-08-31_

## Decision

Do **not** replace the rover's current SLAM Toolbox + Nav2 implementation with the Waveshare navigation stack wholesale.

Waveshare's Orin ROS 2 package is useful as a source of hardware integration, launch files, RTAB-Map support, localization modes, and examples, but its primary 2D navigation architecture is fundamentally the same as the one already used here:

```text
LiDAR / odometry / IMU
        |
        v
  SLAM Toolbox
        |
        v
      Nav2
```

The important difference is that the configuration in this repository has been measured and tuned for this physical rover. Several of the generic settings in the Waveshare package are settings that have already caused real failures on this chassis.

The preferred direction is therefore:

1. Keep the current SLAM Toolbox + Nav2 architecture and rover-specific tuning.
2. Move the navigation workload to the Jetson Orin when practical.
3. Reuse Waveshare's Orin-specific hardware and sensor integration where it is useful.
4. Benchmark Nav2 MPPI on the Orin instead of assuming DWB remains the best controller.
5. Add saved-map localization for routine navigation.
6. Evaluate RTAB-Map + OAK-D as an additional localization/perception layer, not as an immediate replacement for the 2D navigation stack.
7. Evaluate frontier exploration on top of Nav2.

## Current rover stack

The current navigation implementation lives under [`ros_nav/`](../ros_nav/README.md).

The data flow is approximately:

```text
Waveshare driver board / ESP32
        |
        | encoders + gyro, motor commands
        v
rover_daemon --board-bridge
        |
        v
base_node.py ---------------------> /odom, /imu/data_raw
lidar_node.py --------------------> /scan
        |
        v
slam_toolbox ---------------------> /map, map -> odom
        |
        v
Nav2 -----------------------------> /cmd_vel
        |
        v
nav_bridge.py
        |
        v
rover daemon tools / web UI / agent
```

This replaced the rover's earlier custom scan matcher and planner because the custom implementation had no loop closure. SLAM Toolbox provides a pose graph and loop closure, while Nav2 provides standard global planning, local control, collision checking, behaviors, and recovery actions.

The current implementation is therefore already based on the standard ROS 2 navigation stack rather than a proprietary rover-specific planner.

## What Waveshare provides on the Orin

Waveshare's current ROS 2 workspace for the UGV family provides a broader set of packaged navigation examples around the same ROS ecosystem.

Relevant components include:

- Nav2 bringup.
- SLAM Toolbox based mapping and navigation.
- Saved-map navigation/localization modes.
- Cartographer examples.
- RTAB-Map integration.
- RGB-D / depth-camera mapping.
- OAK-D integration in the Orin sensor stack.
- Automatic exploration examples based on frontier exploration / `explore_lite`.
- RViz and Vizanti visualization/control examples.
- Gazebo-oriented launch/configuration support.

The Waveshare ROS 2 workspace is:

- <https://github.com/waveshareteam/ugv_ws>

Relevant launch files in the Waveshare workspace include:

```text
src/ugv_main/ugv_nav/launch/slam_nav.launch.py
src/ugv_main/ugv_nav/launch/nav.launch.py
src/ugv_main/ugv_nav/launch/nav_rtabmap.launch.py
src/ugv_main/ugv_nav/launch/rtabmap_localization_launch.py
```

The important point is that `slam_nav.launch.py` ultimately starts the standard Nav2 SLAM and navigation bringup. It is not an alternative NVIDIA/Waveshare SLAM engine that is inherently more robust than the current stack.

## Why the Waveshare defaults are not preferable to the current configuration

The Waveshare `slam_nav.yaml` is much closer to a generic Nav2 configuration. Examples include:

- `SimpleProgressChecker`.
- generic DWB velocity sampling.
- a Rotation Shim Controller in front of DWB.
- nominal velocity and acceleration values rather than values measured on this rover.
- a stock/default-scale Nav2 server acknowledgement timeout.
- generic robot radius and costmap parameters.

Several of these conflict with things learned from this chassis through measurement and actual navigation failures.

### Minimum sustainable drive speed

The rover cannot continuously execute arbitrarily small forward velocity commands.

Measurement showed that the chassis has a minimum sustainable wheel speed of roughly **0.33 m/s**. Commands below this are effectively quantized into the same physical wheel speed once the motors begin to move.

A generic DWB configuration that samples many velocities between zero and the configured maximum therefore evaluates trajectories the rover cannot physically execute.

The current configuration deliberately restricts the controller's velocity space to commands that correspond to real chassis behavior.

See [`ros_nav/config/nav2.yaml`](../ros_nav/config/nav2.yaml).

### Pivoting is legitimate progress

A tracked/skid-steer rover frequently changes heading by pivoting in place.

A `SimpleProgressChecker` that mainly considers translation can interpret a legitimate pivot as "stuck" and trigger unnecessary recovery behavior.

The current configuration uses `nav2_controller::PoseProgressChecker`, allowing either translation or meaningful angular movement to count as progress.

This change was made after observing otherwise-correct navigation around obstacles being repeatedly aborted while the rover was turning to align with a new path segment.

### Rotation Shim Controller

The concept behind Nav2's Rotation Shim Controller is sensible for this rover: turn toward the path before following it.

In practice, it caused transform-timing failures on the current hardware. The transform lookup behavior interacted badly with the update rate of the rover's odometry/TF chain and reduced the effective controller loop rate.

The current DWB sample set already allows stationary pivot commands, so removing the shim did not remove the ability to turn in place.

### Nav2 action-server acknowledgement timeout

The stock-scale acknowledgement timeout was too aggressive on the current compute platform.

Measured planner acknowledgement latency was high enough that valid goals could fail before the planner had actually processed them. The current configuration raises the timeout based on measured behavior instead of treating scheduler latency as a navigation failure.

### Reversing

The rover's primary planar obstacle sensor looks forward.

The current controller therefore avoids choosing long reverse trajectories as ordinary path-following actions. Short bounded reverse motion remains available as a recovery behavior.

A generic controller does not inherently know that reverse driving is sensor-blind on this particular installation.

## Why moving Nav2 to the Orin is still attractive

The current stack uses DWB at a deliberately conservative update rate because the Banana Pi has a limited CPU budget while also running SLAM Toolbox and the ROS support nodes.

The current Nav2 configuration explicitly notes that MPPI would be a stronger controller but is too expensive for the available Banana Pi CPU budget.

That constraint largely disappears on a Jetson Orin Nano.

The Orin therefore changes the preferred architecture even though it does not change the preferred high-level software stack.

A target architecture is:

```text
                  DRIVER BOARD / ESP32
                         |
                         | encoders, gyro, motor commands
                         v
               hardware / ROS bridge
                         |
             +-----------+-----------+
             |                       |
             v                       v
           /odom                    /imu

D500 LiDAR -----------------------> /scan
OAK-D -----------------------> RGB / depth

                         JETSON ORIN
                             |
                    +--------+--------+
                    |                 |
                    v                 v
              SLAM Toolbox       RTAB-Map
                    |            experimental
                    v
                  /map
                    |
                    v
                   Nav2
          planner + costmaps + BT
                    |
           DWB initially / MPPI test
                    |
                    v
                 /cmd_vel
                    |
                    v
              hardware bridge

                    +
             existing nav_bridge
                    |
                    v
      rover daemon / UI / voice / VLM agent
```

The key migration principle is:

> Move the tuned stack to the faster computer; do not discard the tuning just because the computer changed.

## DWB versus MPPI on Orin

DWB should remain the known-good baseline during the migration.

Once the navigation nodes run reliably on the Orin, MPPI should be tested against the same repeatable routes.

MPPI is interesting because it evaluates many possible control sequences and can generally produce smoother, more anticipatory local control than a simple sampled dynamic-window controller. The Orin has enough compute that the controller no longer needs to be selected primarily around CPU scarcity.

However, MPPI must still respect the rover's real actuator envelope:

- minimum sustainable translational speed;
- skid-steer/pivot behavior;
- maximum useful angular speed before LiDAR scan smear becomes significant;
- limited rear obstacle sensing;
- measured acceleration/deceleration behavior;
- physical footprint and inflation margins.

The correct comparison is therefore **tuned DWB versus tuned MPPI**, not current DWB versus stock MPPI.

### Suggested MPPI acceptance test

Run both controllers over the same route set and record:

- successful goals / attempted goals;
- total route time;
- path length;
- number of replans;
- recovery invocations;
- oscillations / direction reversals;
- minimum obstacle clearance;
- final goal-position error;
- final heading error;
- CPU usage;
- controller-loop deadline misses;
- average and worst-case command latency.

The existing [`ros_nav/plan_bench.py`](../ros_nav/plan_bench.py) and navigation diagnostics provide a useful starting point for automating this comparison.

## Saved-map localization

The current stack primarily maps while navigating, with SLAM Toolbox producing `map -> odom`.

That is appropriate while exploring or changing a map, but it is not necessarily the best mode for routine operation in a known environment.

For repeatable navigation through a house or other stable environment, add a second operational mode:

```text
Exploration / map creation
    -> SLAM Toolbox mapping mode

Routine operation
    -> saved occupancy map
    -> dedicated localization
    -> Nav2
```

Candidate localizers include:

- AMCL;
- SLAM Toolbox localization mode;
- Cartographer localization;
- RTAB-Map localization when visual/depth information is desired.

AMCL is the simplest initial choice for a frozen 2D LiDAR map and is well understood by Nav2.

The important TF rule remains unchanged: exactly one system should own `map -> odom` at a time.

## RTAB-Map and OAK-D

Waveshare's RTAB-Map integration is the most interesting part of its Orin package that is not already represented in the current production navigation stack.

The rover's intended sensor set includes an OAK-D, making it possible to combine:

```text
LiDAR
+
RGB
+
depth
+
odometry
+
IMU
        |
        v
     RTAB-Map
```

Potential benefits include:

- visual loop closure;
- richer relocalization than planar LiDAR geometry alone;
- 3D environmental reconstruction;
- improved behavior in geometrically repetitive 2D spaces;
- a persistent visual/spatial database;
- richer perception data for future semantic navigation.

This should initially run as an experimental subsystem rather than replace SLAM Toolbox.

A practical progression is:

1. Keep SLAM Toolbox as the authoritative 2D mapper/localizer.
2. Run OAK-D and RTAB-Map in parallel without controlling Nav2.
3. Compare trajectories and loop closures.
4. Test RTAB-Map localization against a saved database.
5. Only then consider making RTAB-Map authoritative for `map -> odom` in a dedicated test mode.

Do not allow SLAM Toolbox, AMCL, and RTAB-Map to publish competing `map -> odom` transforms simultaneously.

## Automatic exploration

Waveshare demonstrates frontier exploration on top of its mapping/navigation stack.

This capability is useful and largely orthogonal to the SLAM implementation.

An exploration node can examine the occupancy grid for boundaries between known free space and unknown space, choose a reachable frontier, and repeatedly submit goals to Nav2.

Conceptually:

```text
SLAM map
   |
   v
frontier detector
   |
   v
frontier goal selector
   |
   v
Nav2 NavigateToPose
   |
   v
new map data
   |
   +---- repeat
```

This is a good addition because it leaves the existing navigation safety and recovery stack in charge of physical motion.

`explore_lite` is an obvious first baseline. A custom exploration policy can be substituted later if semantic/VLM-driven exploration becomes useful.

## Components worth reusing from Waveshare

The Waveshare Orin stack should be treated as a component/reference source rather than as a complete replacement.

Likely useful pieces are:

| Waveshare component | Recommendation |
|---|---|
| Jetson ROS 2 hardware bringup | Reuse where it simplifies device integration |
| D500 LiDAR integration | Compare with current parser/driver before replacing |
| OAK-D ROS integration | Strong candidate for reuse |
| SLAM Toolbox launch | Architecture already present; use ours as authoritative |
| Nav2 default parameters | Reference only; do not overwrite rover tuning |
| AMCL / saved-map examples | Reuse/adapt |
| Cartographer examples | Optional experiment |
| RTAB-Map launch/config | Reuse/adapt for experimental mode |
| `explore_lite` integration | Reuse/adapt |
| Vizanti | Optional; current web tooling already covers primary UI needs |
| Gazebo configuration | Useful for simulation/regression tests |

## Recommended migration sequence

### Phase 1 - preserve the baseline

Before changing compute hosts:

- retain the current Banana Pi navigation image/configuration;
- record a standard indoor navigation route set;
- capture current success rate and timing;
- retain the current `odometry.json` calibration;
- retain the current Nav2 YAML and SLAM Toolbox parameters.

The Banana Pi configuration becomes the regression baseline.

### Phase 2 - Orin hardware bridge

Bring the necessary ROS topics onto the Orin without changing navigation behavior:

```text
/scan
/odom
/imu/data_raw
/tf
/tf_static
/cmd_vel
```

Validate rates, timestamps, frame names, covariance, and command latency before moving SLAM or Nav2.

### Phase 3 - move SLAM Toolbox

Run SLAM Toolbox on the Orin with the same map resolution and comparable configuration.

Validate:

- map quality;
- loop closure;
- CPU load;
- TF timing;
- mapping latency;
- saved-map compatibility where practical.

### Phase 4 - move Nav2 with current controller

Run the current tuned Nav2 configuration on the Orin using DWB first.

This isolates host migration from controller migration.

Do not introduce MPPI, RTAB-Map, new costmaps, and a new hardware bridge in the same step.

### Phase 5 - MPPI experiment

Create an MPPI parameter set that respects the measured rover motion envelope and benchmark it against DWB.

Promote MPPI only if it materially improves navigation without reducing safety or reliability.

### Phase 6 - saved-map mode

Add a routine-navigation launch mode using a frozen map and a single dedicated localization source.

Start with AMCL or SLAM Toolbox localization mode.

### Phase 7 - RTAB-Map/OAK-D

Add RTAB-Map as a parallel experimental localization/3D mapping mode.

Do not make it part of the default boot path until repeatable tests show a concrete benefit.

### Phase 8 - frontier exploration

Add `explore_lite` or equivalent as a goal-generation layer above Nav2.

## What not to do

Avoid these migration shortcuts:

- replacing `ros_nav/config/nav2.yaml` with the Waveshare default YAML;
- running multiple nodes that publish `map -> odom` simultaneously;
- assuming the Orin makes physical chassis limits disappear;
- allowing a controller to plan long reverse motion without rear obstacle sensing;
- switching host, SLAM implementation, controller, and sensor fusion at the same time;
- treating a visually richer RTAB-Map model as automatically more robust for simple 2D navigation;
- removing the Banana Pi baseline until the Orin stack has passed repeatable route tests.

## Bottom line

Waveshare does **not** currently provide a fundamentally superior 2D SLAM/navigation stack that warrants replacing the rover's existing implementation.

The current architecture already uses the mature ROS components Waveshare itself builds around: SLAM Toolbox and Nav2. More importantly, the local Nav2 configuration incorporates measurements and failure analysis specific to this chassis that the vendor defaults do not.

The Jetson Orin is nevertheless a meaningful navigation upgrade because it removes the compute constraint that forced conservative controller choices and makes additional perception/localization pipelines practical.

The preferred target is therefore:

```text
Production navigation:
    Jetson Orin
      + SLAM Toolbox
      + Nav2
      + existing rover-specific tuning
      + DWB initially
      + MPPI if benchmarking justifies it

Routine localization:
    saved 2D map
      + AMCL or SLAM Toolbox localization

Experimental perception/localization:
    OAK-D
      + LiDAR
      + RTAB-Map

Autonomous map coverage:
    frontier exploration
      -> Nav2 goals
```

In short: **adopt selected Waveshare Orin integrations, but migrate and extend the tuned navigation stack rather than replacing it.**

## References

- Current rover ROS navigation: [`../ros_nav/README.md`](../ros_nav/README.md)
- Current tuned Nav2 parameters: [`../ros_nav/config/nav2.yaml`](../ros_nav/config/nav2.yaml)
- OAK-D notes: [`oak-d-lite.md`](oak-d-lite.md)
- Host architecture: [`hosts.md`](hosts.md)
- Waveshare UGV ROS 2 workspace: <https://github.com/waveshareteam/ugv_ws>
- Waveshare 2D LiDAR mapping documentation: <https://www.waveshare.com/wiki/UGV_Rover_Jetson_Orin_ROS2_4._2D_Mapping_Based_on_LiDAR>
- Waveshare depth-camera / 3D mapping documentation: <https://www.waveshare.com/wiki/UGV_Rover_Jetson_Orin_ROS2_5._3D_Mapping_Based_on_Depth_Camera>
- Waveshare navigation documentation: <https://www.waveshare.com/wiki/UGV_Rover_Jetson_Orin_ROS2_6._Auto_Navigation>

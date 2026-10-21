# Keeping the rover pinned to its map: a survey of 2D lidar localization

Survey of 2026-10-21, desk research only: no code was changed and the rover was not used.
It answers one question: what should keep the rover's position continuously fitted to the
stored map during everyday driving, given a skid-steer chassis that slips, a 10 Hz 2D lidar
with 8 m of useful range, a 5 cm map of a 60 m² flat whose furniture moves, and an Orin
Nano that is already busy.

## The answer in short

The rover should stop tracking itself against its own recent scans and track against a
frozen copy of the stored map instead. A small continuous correlative scan-to-map tracker
should own `map -> odom` (route c), and slam_toolbox should become a mapper that runs only in
deliberate mapping sessions. Nav2's AMCL against the same frozen grid (route b) is the
off-the-shelf alternative, and it should be run as the control in the same replay benchmark.
If it matches the tracker there, it is the cheaper thing to maintain. slam_toolbox's
localization mode (route a) would not fix either measured failure, because its code path
tracks each scan exactly as mapping mode does. Tuning mapping mode (route d) runs back into
the trade-off the rover has already hit, where a matcher that resists moved furniture also
resists correcting a real slip. None of this has been shown on the rover yet. The tracker's
result comes from one offline replay of two failures, and AMCL has not been tried here.

## What the two failures say about the problem

Both failures come from one design choice. In mapping mode slam_toolbox corrects each new scan mainly
against a buffer of its own recent scans, and those scans carry the same odometry error.
The code shows the mechanism. `Mapper::Process` matches the scan against the "running scans"
with the sequential matcher
([Mapper.cpp, `Process`](https://github.com/SteveMacenski/slam_toolbox/blob/jazzy/lib/karto_sdk/src/Mapper.cpp#L2716)).
`AddEdges` then also matches it against older chains of scans nearby (`LinkNearChains`) and
sets the pose to a covariance-weighted mean of all these matches
([Mapper.cpp, `AddEdges`](https://github.com/SteveMacenski/slam_toolbox/blob/jazzy/lib/karto_sdk/src/Mapper.cpp#L1434),
[`LinkNearChains`](https://github.com/SteveMacenski/slam_toolbox/blob/jazzy/lib/karto_sdk/src/Mapper.cpp#L1639)).
So the old map does pull on the pose. But every one of those matches goes through the same
penalised search window: ±0.25 m, since the rover's `correlation_search_space_dimension` is
0.5 in `ros_nav/config/slam_toolbox.yaml`, and `distance_variance_penalty` is 0.05 with a
floor of 0.1. The README describes that
penalty as charging a match for how far it lies from the odometric pose
([slam_toolbox README](https://github.com/SteveMacenski/slam_toolbox)). The pull from the old
map is also averaged against the confident match to the robot's own recent scans. A full
snap back to the map happens only when a loop closure is accepted and `CorrectPoses` runs
([`TryCloseLoop`](https://github.com/SteveMacenski/slam_toolbox/blob/jazzy/lib/karto_sdk/src/Mapper.cpp#L1500)).

That explains the trade-off recorded on 2026-10-12. The strong distance penalty stopped moved
armchairs dragging the pose 1.33 m, but it "resists a large correction whether that
correction is right or wrong" (progress entry
`docs/progress/2026-10-12-the-pose-among-moved-furniture.md`). A reference that drifts with
the robot needs a strong odometry penalty to stay put. A reference that is the stored map
does not drift with the robot, so it can afford a weaker penalty. That is most likely the
structural reason a scan-to-map tracker held both failures where the mapper did not.

## What the field has learned

### Academic work

**Particle-filter localization against a grid.**
- Monte Carlo localization represents the pose as weighted samples and can hold several
  hypotheses at once ([Dellaert, Fox, Burgard, Thrun, ICRA 1999](https://publications.ri.cmu.edu/monte-carlo-localization-for-mobile-robots)).
- KLD-sampling adapts the particle count: few particles when the belief is tight, many when
  it is not ([Fox, IJRR 2003](https://rse-lab.cs.washington.edu/postscripts/adaptive-ijrr-2003.pdf)).
- The likelihood-field sensor model scores each return by its distance to the nearest
  occupied cell. Augmented MCL injects random particles when the average likelihood falls
  ([Thrun, Burgard, Fox, *Probabilistic Robotics*, 2005](https://mitpress.mit.edu/9780262201629/probabilistic-robotics/)).
  These are the models nav2_amcl implements. Its differential motion model is the book's
  `sample_motion_odometry`, with noise σ = √(α₃)·Δd on translation
  ([source](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_amcl/src/motion_model/differential_motion_model.cpp)).
- Nav2's maintainers credit AMCL with "5 cm or better in many practical environments", say it
  "has a number of shortcomings", and call Cartographer "difficult to recommend" because
  out-of-the-box results "are typically poor"
  ([Macenski et al., ROS 2 algorithms survey, 2023](https://arxiv.org/abs/2307.15236)). Nav2's
  lead maintainer has also written that "AMCL is not awesome"
  ([ROS Discourse, 2019](https://discourse.openrobotics.org/t/next-generation-navigation-localizer-amcl-qmcl-non-mcls-oh-my/11658)).

**Correlative scan matching.**
- Olson argues for an exhaustive, probabilistically motivated search over a pose window
  instead of gradient heuristics, because heuristics "can produce poor results, particularly
  when the prior is weak". He makes the search real-time with multi-resolution CPU and GPU
  versions ([Olson, ICRA 2009](https://doi.org/10.1109/ROBOT.2009.5152375); a follow-up is
  [M3RSM, ICRA 2015](https://april.eecs.umich.edu/media/pdfs/olson2015scanmatch.pdf)).
- Cartographer uses scan-to-submap matching locally and branch-and-bound correlative search
  against submaps for loop closure, at 5 cm resolution
  ([Hess, Kohler, Rapp, Andor, ICRA 2016](https://research.google/pubs/pub45466)). Its own
  documentation says its online correlative matcher "is very expensive and will essentially
  override any signal from other sensors but the range finder"
  ([algorithm walkthrough](https://google-cartographer-ros.readthedocs.io/en/latest/algo_walkthrough.html)).
  That cost is for large maps. On a 60 m² grid the same idea is cheap, as the rover's own
  measurements below show.
- LaMa localizes by scan matching against a continuous likelihood field and claims a
  Raspberry Pi 3B+ is enough
  ([Pedrosa, Pereira, Lau, ICARSC 2017, via iris_lama](https://github.com/iris-ua/iris_lama)).

**NDT and ICP against a map.** NDT-MCL puts a normal-distributions map inside a particle
filter. Nav2 maintainers flagged it as a possible AMCL successor, but that work stalled
([navigation2 #1391](https://github.com/ros-navigation/navigation2/issues/1391)). PL-ICP is
fast and precise, but it is "less robust to very large initial displacement errors"
([Censi, ICRA 2008](https://resolver.caltech.edu/CaltechAUTHORS:20100512-152648883)). That is
the wrong failure profile for a rover whose problem is large slips.

**Changing environments.** This literature agrees on one point: separate the long-lived
structure from what moves, and do not let the second define the frame.
- Fox, Burgard and Thrun filtered out readings that the map cannot explain and that are
  shorter than expected. In crowded runs the failure rate fell from 26.8% to under 2%
  ([JAIR 1999](https://jair.org/index.php/jair/article/view/10246)).
- Biber and Duckett kept the map at five timescales and ran it for five weeks in a real
  changing environment ([RSS 2005](https://roboticsproceedings.org/rss01/p03.html)).
- Meyer-Delius et al. localized against a reference map plus temporary local maps of
  unexpected objects ([IROS 2010](https://doi.org/10.1109/IROS.2010.5648920)).
- Tipaldi, Meyer-Delius and Burgard combined a Rao-Blackwellized particle filter with a hidden
  Markov model of each cell. On a real robot it beat MCL
  ([IJRR 2013](https://doi.org/10.1177/0278364913502830)).
- DT-NDT-MCL tracks against a static map and a short-term map and chooses the better one
  locally ([Valencia et al., ICRA 2014](https://www.iri.upc.edu/publications/show/1499)).
- Episodic non-Markov localization separates permanent mapped features, unmapped static
  objects such as chairs, and moving people. It kept CMU's CoBots driving more than 1,000 km
  in office buildings ([Biswas & Veloso, RAS 2017](https://www.ri.cmu.edu/publications/episodic-non-markov-localization);
  [CoBots, IJCAI 2015](https://www.ijcai.org/Abstract/15/656)).
- Akai's line of work adds a reliability estimate and failure detection to MCL
  ([als_ros README and papers](https://github.com/NaokiAkai/als_ros)).
- One study found AMCL degrades "drastically" when the map deviates from the building, but it
  is simulation only ([Ogm2Pgbm, 2023](https://arxiv.org/abs/2308.05443)).

**Slip-robust odometry.**
- rf2o estimates planar motion from consecutive scans in about 0.9 ms
  ([Jaimez et al., ICRA 2016](https://cvai.cit.tum.de/_media/spezial/bib/jaimez2016icra.pdf)).
  Its tracker reports failures in long corridors and in pure rotation
  ([#16](https://github.com/MAPIRlab/rf2o_laser_odometry/issues/16),
  [#28](https://github.com/MAPIRlab/rf2o_laser_odometry/issues/28)).
- KISS-ICP is 3D-only ([Vizzo et al., RA-L 2023](https://arxiv.org/abs/2209.15397)).
- Kinematic-ICP adds wheeled-robot kinematic constraints to ICP and weighs lidar against wheel
  odometry dynamically. It accepts a 2D `LaserScan`, runs on Jazzy, and is MIT-licensed. Its
  authors report it in Dexory's warehouse fleet, which is their own claim
  ([Guadagnino et al., 2024](https://arxiv.org/abs/2410.10277);
  [repo](https://github.com/PRBonn/kinematic-icp)).
- Skid-steer slip is real and varies with speed and curvature, which is why
  instantaneous-centre models exist
  ([Mandow et al., IROS 2007](https://portaldelainvestigacion.uma.es/documentos/60d69da493a5ba1687c98f83)).
- All of these reduce drift. None of them pins the rover to the stored map.

### Commercial practice: what ships, as distinct from what is claimed

- **Robot vacuums.** iRobot introduced camera vSLAM in the Roomba 980 in 2015
  ([press release](https://media.irobot.com/2015-09-16-iRobot-Enters-the-Smart-Home-with-Roomba-980-Vacuum-Cleaning-Robot);
  algorithm in [Karlsson et al., ICRA 2005](https://www.academia.edu/10650610/The_vSLAM_Algorithm_for_Robust_Localization_and_Mapping)).
  Its 2025 lineup moved every model to lidar
  ([Vacuum Wars, 2025](https://vacuumwars.com/irobots-new-2025-lineup-buyers-guide)). Vacuum
  makers do not publish their localizers. Patents are the best evidence, and they show claimed
  methods, not proof of what ships. Ecovacs' EP 3 808 512, now litigated against Roborock,
  covers relocalizing after the robot is picked up or dragged. It matches against the stored
  grid by stepping through candidate poses (5 cm and 5° in its example) at several
  resolutions, refined by ICP
  ([EP3808512B1](https://patents.google.com/patent/EP3808512B1/en);
  [litigation report](https://www.smarthomeassistent.de/ecovacs-vs-roborock-saugroboter-hersteller-im-patentstreit/)).
  UBTech's US 11,045,953 uses template matching against the known map plus ICP, and explicitly
  sets itself against AMCL ([patent](https://patents.google.com/patent/US11045953B2/en)).
  Correlative search against the stored grid is what home-robot makers patent.
- **A supermarket cleaning robot.** Gaussian Robotics' robot has a SICK TiM571 2D lidar and
  an 8 GB i5. It localizes by branch-and-bound scan-to-submap matching against a
  multi-session map and trims stale submaps. It was deployed for over a month in a
  supermarket. The authors admit that "sudden, drastic" changes are not handled
  ([Zhao et al., IROS 2021](https://arxiv.org/abs/2111.10946)).
- **slam_toolbox's localization mode** was built for Simbe Robotics' retail robots
  ([JOSS 2021](https://joss.theoj.org/papers/10.21105/joss.02783)). Its author calls it a
  niche technique that needs "high quality odometry", and for general use he recommends AMCL
  ([#285](https://github.com/SteveMacenski/slam_toolbox/issues/285),
  [#732](https://github.com/SteveMacenski/slam_toolbox/issues/732), README).
- **ROS-based integrators** ship slam_toolbox for mapping and AMCL for localization.
  Clearpath does this in [clearpath_nav2_demos](https://docs.clearpathrobotics.com/docs/ros/tutorials/navigation_demos/overview),
  and TurtleBot 4 on a Raspberry Pi 4 with [stock AMCL parameters](https://github.com/turtlebot/turtlebot4/blob/jazzy/turtlebot4_navigation/config/localization.yaml).
  Nav2 recommends fusing wheels and IMU with robot_localization's EKF for `odom -> base_link`
  ([Nav2 guide](https://docs.nav2.org/setup_guides/odom/setup_robot_localization.html)).
  Locus Robotics publishes [fuse](https://github.com/locusrobotics/fuse), a factor-graph
  fusion stack. None of these vendors publishes the localizer it runs in production.

### Open source

| Package | What it is | Licence | Maintenance (GitHub, 2026-10-21) | Reported failure modes |
|---|---|---|---|---|
| [nav2_amcl](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/others/configuring_amcl/) | AMCL with KLD, likelihood field, beam skipping, augmented-MCL recovery | LGPL-2.1+ | active (Nav2) | needs alpha tuning; recovery injection off by default (`recovery_alpha_* = 0`); a 2025 report of a crash on setting them ([#5358](https://github.com/ros-navigation/navigation2/issues/5358), closed) |
| [Beluga AMCL](https://github.com/Ekumen-OS/beluga) | C++17 re-implementation of nav2_amcl, parameter-compatible, with a 2D NDT-AMCL node | Apache-2.0 | active (pushed 2026-10-20); Humble/Jazzy/Kilted | benchmarks vs nav2_amcl are the project's own and use synthetic data ([index](https://index.ros.org/p/beluga_amcl/)) |
| [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) 2.8.5 | Karto pose graph: mapping, localization mode, experimental lifelong and mode switching | LGPL-2.1 | active | localization degrades over time with loop closure on ([#334](https://github.com/SteveMacenski/slam_toolbox/issues/334)); "snaps out" or lags in turns ([#285](https://github.com/SteveMacenski/slam_toolbox/issues/285)); graph "explodes" ([#732](https://github.com/SteveMacenski/slam_toolbox/issues/732)); per-scan cost grows with revisits ([#903](https://github.com/SteveMacenski/slam_toolbox/issues/903)); `bad_alloc` loading a >100 MB graph on a Jetson ([#765](https://github.com/SteveMacenski/slam_toolbox/issues/765)) |
| [Cartographer](https://github.com/cartographer-project/cartographer) / [ROS 2 fork](https://github.com/ros2/cartographer_ros) | submaps, CSM, branch-and-bound pure localization | Apache-2.0 | upstream "no longer actively maintained"; ROS fork "limited capacity" | pose jumps at look-alike places and of about 40 cm ([#1441](https://github.com/cartographer-project/cartographer/issues/1441)); jumps after small map changes ([ros #1759](https://github.com/cartographer-project/cartographer_ros/issues/1759)); localization costs more CPU than SLAM ([#1603](https://github.com/cartographer-project/cartographer/issues/1603)); submaps keep growing ([ros #1815](https://github.com/cartographer-project/cartographer_ros/issues/1815)) |
| [LaMa / iris_lama_ros](https://github.com/iris-ua/iris_lama_ros) | likelihood-field scan-match localizer | BSD-3 | stale; ROS 2 branches only for Dashing/Eloquent | none relevant found |
| [emcl2_ros2](https://github.com/ryuichiueda/emcl2_ros2) | MCL with expansion resetting | LGPL-3.0 | Humble only, last push 2024 | resetting is automatic (see R-NAV-3 below) |
| [als_ros](https://github.com/NaokiAkai/als_ros) | MCL with reliability and failure detection | Apache-2.0 | ROS 1 only | not usable on Jazzy without a port |
| [rf2o](https://github.com/MAPIRlab/rf2o_laser_odometry) | scan-to-scan odometry | GPL-3.0 | ros2 branch, last push 2024 | corridors and pure rotation, above |
| [Kinematic-ICP](https://github.com/PRBonn/kinematic-icp) | lidar + wheel odometry with kinematic constraints | MIT | 2025, Jazzy | 2D use is a flag; untested here |
| [robot_localization](https://github.com/cra-ros-pkg/robot_localization) | EKF/UKF fusion | Apache-2.0 | active | n/a |

## The four routes for this rover

**(a) slam_toolbox localization mode for daily driving, mapping only on demand.**
- *Failure 1 (heading after the turn):* expected not fixed (read from the code, not replayed). `ProcessLocalization` is `Process` plus a
  rolling buffer that evicts old scans. It uses the same sequential match against the robot's
  own recent scans, the same ±0.25 m penalised window, the same near-chain averaging and the
  same loop closure
  ([Mapper.cpp](https://github.com/SteveMacenski/slam_toolbox/blob/jazzy/lib/karto_sdk/src/Mapper.cpp#L2868)).
- *Failure 2 (the 0.8 m slip):* expected not fixed, for the same reason: the window and penalty are unchanged.
- *CPU and memory:* the whole 234 MB graph plus 133 MB of scans is loaded (about 40 s), and
  every scan is still matched against every nearby chain of a graph that has had weeks of
  revisits ([#903](https://github.com/SteveMacenski/slam_toolbox/issues/903)). Memory stops
  growing, but the base cost does not shrink.
- *Jumps:* loop closure can still snap the pose up to `loop_search_maximum_distance` (3 m),
  which is enough to reach the next room. The author sees no value in loop closure in this
  mode, and a user fixed degradation over time by turning it off
  ([#334](https://github.com/SteveMacenski/slam_toolbox/issues/334)).
- *Map and frame:* the map is not updated during driving. Optimisation can still move loaded
  nodes in memory; #334 reports the graph shifting.
- *Integration:* switching modes needs the experimental map-and-localization node and its
  `set_localization_mode` service
  ([source](https://github.com/SteveMacenski/slam_toolbox/blob/jazzy/src/experimental/slam_toolbox_map_and_localization.cpp))
  or a restart, and each switch reloads the graph.
- *Verdict:* this bounds memory and changes nothing about tracking.

**(b) Nav2 AMCL (or Beluga) against the frozen occupancy grid, owning `map -> odom`.**
- *Failure 1:* probably fixed. Each update weighs particles against the stored map rather
  than recent scans. `update_min_a` 0.2 rad gives about 11 updates through a 127° turn, and
  α₁ 0.2 spreads heading by about 5° per update, which covers a 4.5° error.
- *Failure 2:* uncertain. A particle filter has no hard window. But with α₃ 0.2 and updates
  every 0.25 m, each update spreads translation by only about 11 cm, so the cloud follows
  odometry's 0.8 m and must be pulled back over several updates. Raising α₃ widens the cloud
  and adds jitter while driving normally. Only a replay can settle it.
- *CPU and memory:* small. On defaults that is 500-2000 particles × 60 beams per update and a
  few-megabyte distance field. It runs on a Raspberry Pi 4 in TurtleBot 4 and in
  [Beluga's demos](https://ekumen-os.github.io/beluga/).
- *Jumps:* none with recovery injection off, which is the default. Turned on, it scatters
  particles over all free space and can land in the wrong room. Leave it off.
- *Map and frame:* the frame is fixed by construction. Updating the map moves to a separate
  path (see below).
- *Weaknesses:* the likelihood field's `z_rand` and beam skipping are its only defences
  against moved furniture. Updates are discrete (every 0.25 m or 0.2 rad), and the particle
  filter's noise shows up as small steps in `map -> odom`.

**(c) A custom continuous correlative scan-to-map tracker owning `map -> odom`.**
- *Failures 1 and 2:* both held in the offline replay. At ±25 cm per step and 5 Hz, the
  window absorbs up to about 1.25 m/s of motion odometry did not report, against 0.2 m/s in
  the slip. The search is exhaustive within the window, so there is no local-minimum
  failure inside it ([Olson 2009](https://doi.org/10.1109/ROBOT.2009.5152375)).
- *CPU:* low. The rover's change watch already searches shifts of up to 0.40 m and 6° for
  each scan's returns within 3 m, and that took it from 3.4 to 6.7 ms a scan on the Orin
  (commit `b54f8fa`). A
  multi-resolution search would cost less.
- *Jumps:* bounded by the window by construction, which means no automatic wrong-room
  relocalization. The real risk is locking onto a wrong peak inside the window in a corridor
  or among furniture that moved since the map was made. That needs a score-margin test
  against the second-best peak, an odometry prior, and a "cannot say" outcome that keeps
  odometry rather than accepting a weak fit.
- *Map and frame:* as for (b).
- *Weaknesses:* it is code to own, it tracks a single hypothesis, and it has no recovery
  short of a person's refit. The last point is a feature under R-NAV-3, which makes fitting
  the rover onto its map a person's act.

**(d) Keep mapping mode and tune it (smaller buffer, eager loop closure, lower penalties).**
- *Failure 1:* a smaller buffer would likely weaken the self-reference only slightly. The confident match
  to recent scans still dominates the weighted mean, and the map's correction still waits
  for a loop closure.
- *Failure 2:* covering 0.8 m needs a window of about 1.6 m, which is (1.6/0.5)², or about
  10 times, the per-scan search. A weaker penalty brings back the 1.33 m furniture drag
  measured on 2026-10-12.
- *Jumps:* eager loop closure invites false closures, and in mapping mode a false closure
  deforms the graph permanently.
- *CPU and memory:* the graph keeps growing. Serialisation stalls of 5-20 s, 40 s reloads and
  per-scan near-chain cost
  ([#903](https://github.com/SteveMacenski/slam_toolbox/issues/903)) all get worse. The README
  calls true lifelong mapping "highly experimental" and suggests running it in the cloud.
- *Frame:* every optimisation can move geometry the world model has stored coordinates in.
- *Verdict:* this is the trade-off the rover has already measured, not a way out of it.

## Integration pitfalls for (b) and (c)

- **Exactly one publisher of `map -> odom`.** slam_toolbox publishes nothing when
  `transform_publish_period: 0`
  ([slam_toolbox_common.cpp](https://github.com/SteveMacenski/slam_toolbox/blob/jazzy/src/slam_toolbox_common.cpp#L487)),
  and AMCL stops with `tf_broadcast: false`. With two publishers, tf2 takes whichever arrived
  last, and the pose flickers between two beliefs. A mapping session switches the owner, it
  does not add a second one. The refit, the parked drift check (R-NAV-14) and
  `map_settled` currently talk to slam_toolbox, and they must be repointed to the new
  owner.
- **TF timing.** Compute each correction with odometry looked up at the scan's own stamp. At
  60°/s a 100 ms lag is 6°. Then republish the latest correction at a fixed rate, post-dated
  the way AMCL's `transform_tolerance` does. Never let the search block publishing: #903 shows
  Nav2's extrapolation errors when the stamp stops advancing, and the mapper's 5-20 s
  serialisation stall is the same fault. Feed the tracker deskewed scans during turns.
- **Keep it a tracker, not a fitter.** R-NAV-3 makes a global fit a person's act. AMCL's
  recovery alphas and emcl2's expansion resetting are automatic global fits, so they stay off.
  The tracker's window stays bounded, and anything it cannot explain becomes a report, not a
  jump.
- **Parked behaviour.** A tracker running at 5 Hz while parked can be pulled by people walking
  past, which matters for R-NAV-13 (a rover standing still does not accumulate heading).
  Either update only while moving, as AMCL's `update_min_*` does, or require a much stronger
  fit while stationary.
- **The frozen map and its frame.** Export the grid from the current graph once, so that
  stored world-model coordinates keep their meaning, and version it. After each deliberate
  mapping session, register the new grid to the old one with the same correlative matcher and
  refuse it (or transform the world store) if it moved more than about 2 cm or 0.5°.
- **Stale furniture in the frozen map.** Score walls and other cells occupied in every session
  at full weight, and furniture at less. Leave what moved to the live layer and change watch
  that already exist in `ros_nav/`. This is the static-versus-short-term split of Biber and
  Duckett, DT-NDT-MCL and EnML, made cheap.

## Recommendation

Build route (c). The continuous correlative tracker scores against the frozen, versioned grid,
weights persistent structure, keeps a bounded window with a score-margin gate and a "cannot
say" outcome, and is the only publisher of `map -> odom`. Demote slam_toolbox to supervised
mapping sessions, with transform publishing switched over while one runs. Put nav2_amcl (and
Beluga if it is easy to install) through the same replay as the control. (c) is preferred
because it is the only route already shown to hold both measured failures, its cost is a few
milliseconds on the Orin, and its bounded window fits R-NAV-3 without exceptions. AMCL is the
fallback if the benchmark shows it equal, because it is less code to keep. Odometry
improvements such as an EKF with the gyro, or Kinematic-ICP as a slip detector, are separate
work and should not be mixed into this decision.

## What a benchmark on recorded drives should measure

Run everything as replays on the desktop first (R-NAV-11), then on the rover with the full stack running.

- **Recordings.**
  - the 10-21 drives with the 127° turn and the 0.8 m slip;
  - the 10-12 shakedown among moved furniture, from its kept starting graph (the 1.33 m case);
  - the twelve turns on the spot of 10-10;
  - the 10-21 doorway, passage and charger cases, 19-35 cm out;
  - the nine M5 sessions and the multi-day benchmark mornings, for everyday driving, staged
    moves and people.
- **Ground truth.** Use the refit's wide-window answer at every stop, accepted only above its
  score floor and checked against a photo. Add the charger pose at each return, and a few of
  the owner's tape readings. Where none of these exist, report the fraction of returns lying
  on mapped walls along the track, and compare it with the same fraction at the true pose.
- **Accuracy.** Report position and heading error at each checkpoint (median, 90th percentile,
  worst). For reference, the mapper's own corrections since 10-17 were a median of 2 cm and
  1°, and 4-8 cm and 2-4° at the 90th percentile (commit `92a8341`). A proposed target is
  under 5 cm and 1° at the 90th percentile and never over 15 cm.
- **Fault injection.** During replay, add 2-10° heading errors and 0.2-1.0 m slips over 1-4 s
  to `/odom`. Measure the time and distance until the localizer is back within 5 cm and 1°,
  and the largest error on the way.
- **Jumps and wrong rooms.**
  - Measure the largest single-step correction and count the corrections over 10 cm.
  - Start the localizer 1-2 m off in the neighbouring room or the corridor and require "cannot
    say", not a confident lock.
  - Count how often the score-margin gate refuses a fit in the corridor.
- **Moved furniture.** On the staged-move sessions, compare the pose with the walls-only fit,
  and check that the change watch still logs the staged moves (no new false "changes" caused
  by a better or worse pose).
- **Cost.** On the Orin, with Nav2, perception and world state running, measure CPU per scan
  (median and 99th percentile) and resident memory. Measure the latency from scan stamp to
  published TF, and count Nav2 TF extrapolation warnings in a replay with the controller
  active.
- **Frame stability.** The charger, the doorway and fixed world-state things should keep
  their map-frame coordinates across days. After a mapping session, measure the registration
  of the new grid against the frozen one.
- **Routes in the table.** Include the deployed mapping mode (baseline), (a) with loop closure
  on and off, (b) nav2_amcl on defaults and with α₃ and α₄ raised for skid-steer, Beluga,
  (c), and the best (d) found.

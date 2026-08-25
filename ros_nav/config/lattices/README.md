# Lattice control set

`diff_5cm_0.5m.json` is Nav2's sample differential-drive minimum control set:
5 cm grid, 0.5 m turning radius, 16 headings, 112 primitives including in-place
rotations. Copied from the Jazzy package on this rover
(`nav2_smac_planner/sample_primitives/.../diff/output.json`, generated
2022-03-17) so a deploy does not depend on the share path staying put.

0.5 m is this chassis's DWB envelope (`max_vel_x / max_vel_theta` = 0.51 m) to
a centimetre. The 1 m sample is the more conservative neighbour and is not
used. `nav.launch.py` injects the absolute path; the empty `lattice_filepath`
in `nav2.yaml` is a placeholder, because yaml cannot resolve a relative one
and an empty value would load Nav2's ackermann test set instead.

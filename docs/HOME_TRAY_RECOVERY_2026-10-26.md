# HOME start refusal at the tray edge

Read-only investigation evidence: Pi runtime `/home/user/4dof_ik_pi_test`,
`output/home_returns/20261026T044534/result.json`. No device was opened and no
motion, torque, pump or navigation was performed for this investigation.
The verified code was subsequently deployed by the root agent as recorded below.

The recorded failure was `not at HOME and outside the vertical reset corridor`.
No motion phase ran. Its exact post-takeover start snapshot was omitted by the
old script, so the pre-gain and final snapshots cannot establish the exact pose
at rejection. Both recorded poses are outside the radial gate:

| Snapshot | Raw base / shoulder / elbow / wrist | Candidate radius | Candidate z | Candidate pitch |
| --- | --- | --- | --- | --- |
| Before temporary gains | 2385 / 3834 / 1390 / 2454 | 337.624 mm | 52.658 mm | -47.023 degrees |
| Final read | 2385 / 3853 / 1390 / 2454 | 334.364 mm | 43.652 mm | -48.693 degrees |

Calculations use the runtime mapping directions (-1 for all joints), fitted
zeros from `fit_full.json`, fitted lengths from `review.json`, shoulder offset
27 mm, and shoulder height 160 mm (default 40 mm robot raise). An overridden
`VACUUM_ROBOT_RAISE_MM` requires recalculation.

The ordinary corridor permits radius 140–320 mm at z 30–135 mm. Final radius
exceeds that ceiling by 14.364 mm; its height satisfies the gate. Increasing the
generic radius alone would be insufficient: the old initial lift requests a
straight-down tool, and no IK solution exists for a 30 mm lift from this final
pose at that pitch.

The separate tray-edge route admits radius above 320 mm through the existing
hover executor ceiling (355 mm), z 30–135 mm, and measured pitch from straight
down through the existing forward-tilt limit (60 degrees). It retains the
measured initial pitch for 30 mm vertical lifts until the candidate cup height
reaches at least 80 mm. It then uses the existing fully sampled continuous HOME
fold. Ordinary corridor rules, controller margins, link-height guards, endpoint
tolerances, and hold/gain checks remain in effect.

For the recorded final pose, offline candidate sampling gives:

| Segment | Candidate endpoint raw | Samples | Minimum / maximum link endpoint z |
| --- | --- | --- | --- |
| First 30 mm lift | 2385 / 3723 / 1481 / 2493 | 131 | 43.652 / 181.735 mm |
| Second 30 mm lift | 2385 / 3651 / 1464 / 2582 | 90 | 73.770 / 193.867 mm |
| Existing fold to recorded HOME | 2110 / 1823 / 3018 / 3075 | 1829 | 103.722 / 286.779 mm |

All sampled raw values satisfy the existing 16-pulse controller margins.
The lift uses the unchanged 30 mm link-height floor; the fold uses the unchanged
60 mm floor and 450 mm ceiling. `test_home_tray_recovery.py` also runs the real
planners and sampled executor preflight using an in-memory driver with no I/O
methods, then cancels before the first motion command.

Future start-selection refusals preserve the actual post-takeover snapshot,
raw positions, candidate xyz/pitch, ordinary corridor bounds and violated bound
names. The rejection message includes radius, height, and pitch.

The previous final hold was **not confirmed**: shoulder position 3853 versus
goal 3849 persisted, even though gain restoration was verified. This change does
not modify that tolerance or its reporting. Final torque readback was 1 on all
four joints. Offline FK and sampled link endpoints do not verify physical tray
clearance, an attached object's clearance, suction state, tracking, or actual
HOME recovery. A fresh supported real run must still satisfy every existing
hold, tracking, arrival, and restoration check.

Deployment: backed up to
`output/continuous_lift_home_fix_20261026T141528`, then applied to the Pi runtime.
API restart and healthy/idle status were verified. The saved failed pose is
admitted by the new bounded tray predicate; real motion was not executed.
Combined final project verification:320 tests and13 subtests passed.

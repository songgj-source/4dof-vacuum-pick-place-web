# Continuous pick-to-carry and tilted tray HOME recovery

User approved sequence: after suction, rise vertically to70mm, then continue
to the planned carry height while tilting to the planned carry pitch, without
an intermediate hold. Preserve all release checks and avoid live movement.

1. Add a pure transition builder that consumes a validated place plan and
   measured contact pose. Support vertical raise/tilt prefixes only; retain
   the legacy sequence for pull-in paths, tilted picks and wrist hold checks.
2. Add a guarded motion primitive that solves each vertical/pitch waypoint,
   samples every raw step for clearance/controller margins, then calls the
   existing fixed-rate waypoint executor once (one ramp and one final hold).
   Keep the existing total1600-pulse envelope and held rate. Tilt starts only
   above70mm; near-contact movement remains vertical and uses the contact floor.
3. Integrate in pick_auto: plan before lifting, execute once, record the merged
   phase, then run the remaining validated carry/lower/release/retreat legs.
   If preparation refuses before execution, retain the existing vertical lift.
4. Independently diagnose the latest HOME refusal from its saved snapshot.
   Add a bounded tilted-tray start route: preserve measured tilt during vertical
   clearance raises, then use the existing fully sampled HOME fold. Keep the
   ordinary reset corridor and hold tolerances unchanged; retain diagnostics.
5. Write failing tests first for transition order, preserved low vertical
   section, bounded execution, invalid prefixes/paths, resume plan and HOME.
   Run focused and complete project tests, plus model-only checks using the
   saved robot snapshot. Back up and deploy only after idle and hash checks.
   Restart API; verify health and imports without motor commands.

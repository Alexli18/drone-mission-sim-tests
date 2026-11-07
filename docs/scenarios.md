# Adding a scenario

Scenarios are YAML files in `scenarios/`; tests are parametrized over them, so
a new file is a new test with no Python change. A survey mission:

```yaml
name: survey_corridor
type: mission
description: Corridor survey, 20 x 100 m, 10 m line spacing, 20 m altitude, 8 m/s.
altitude_m: 20
speed_m_s: 8
area: {origin_north_m: 10, origin_east_m: 0, width_m: 20, height_m: 100, line_spacing_m: 10}
duration_bounds_s: {min_s: 72, max_s: 165}   # simulated seconds; derive from the flight profile
```

A failsafe scenario adds the PX4 parameters it depends on, the fault, and the
expected reaction:

```yaml
type: failsafe
px4_params: {NAV_DLL_ACT: 2, COM_DL_LOSS_T: 10, COM_DLL_EXCEPT: 0}
failure:
  kind: link_loss                           # one of framework/failures.py FAULTS
  trigger: {type: at_waypoint, value: 3}    # or {type: after_s, value: <sim s>}
expected:
  mode_reached: RETURN_TO_LAUNCH            # or leave_mode: MISSION
  allowed_before: [HOLD]
  landed: true
  disarmed: true
  max_dist_from_home_m: 5
```

The loader rejects a file with unknown keys, a missing parameter the fault
needs, or an expectation that mixes `mode_reached` and `leave_mode`, and
names the problem. After boot, every parameter is read back from PX4, so a
name PX4 does not know fails the test instead of being ignored. Files starting
with `_` (such as `_baseline.yaml`, the parameters every flight shares) are
not scenarios.

# Oracle independence in the line-survey test

A check proves something only when its expected value (or its measured value) does not
come from the code it judges. This page lists, for each pass rule of
`tests/test_line_survey.py`, where the expected and the measured values come from, what
they share with the nodes under test, and which separate test covers the shared part.

The nodes under test: `toy_sensor_node` (the simulated sensor), `georeference_node` and
`marker_detector_node`. The reference everywhere is the PX4 *estimate* of the vehicle
pose, not the simulator's ground truth (see the last section).

| Rule | Expected value from | Measured value from | Shared with the nodes under test | What it cannot catch | Covered separately by |
|---|---|---|---|---|---|
| 0. No capture lost, repeated or reordered | Ids run 0, 1, 2, ... up to the `last_capture_id` the sensor reports at its stop | Recorded `/sim/capture_truth` and `/survey/georef_capture`; the georeferencing node's `captures_seen` and `released` counters | Message definitions only | A capture the sensor never generated (the count comes from the sensor) | `tests/unit/test_survey.py` (capture stream tests: missing, repeated, swapped, trailing, empty) |
| 1. Coverage | `coverage_min` from `config.yaml` | The sensor's own true positions (PX4 estimate interpolated at the true sample times); the node's georeferenced positions | `sensor_bridge.coverage.coverage()`, `sensor_bridge.line.Line.project`, `sensor_position`, the same PX4 pose stream | A mistake in the shared algorithm: the "agreement" check compares two inputs of the same function | Hand-computed cases in `test_georeference.py`; a grid-integration reference on random captures in `test_survey.py` |
| 2. Height, cross-track, speed | The scenario (`altitude_m`, line, `speed_m_s`) | MAVSDK telemetry, projected with `framework.mission.SurveyLine` | The state estimator behind the telemetry and behind the PX4 pose messages | An estimator error, since both read the estimate | `Line` and `SurveyLine` are two implementations of the projection; a unit test checks that they agree on random points |
| 3. Markers | Marker positions painted in the scenario YAML | The detector's output on the georeferenced captures | The projection (`Line.project`) used by the sensor and the node | A projection error identical on both sides | `test_projection_for_headings`; the line-implementation cross-check above; `test_chain_with_a_wrong_lever_arm_is_caught_by_the_marker_position` (a wrong lever arm does move the markers) |
| 4. Clock offset | Line speed from telemetry x the offset from the scenario | Georeferenced minus true sample positions | The PX4 poses | A pose error common to both | `test_chain_shifts_every_marker_by_speed_times_clock_offset`, `test_clock_offset_shifts_positions_by_v_times_offset` |
| 5. Replay | The live run's decisions and markers | A fresh georeferencing node and detector reading the recorded bag | All the code | Everything except non-determinism: replay shows that the chain repeats itself, not that it is right | Rules 1 to 4 judge the live run |

## Rule of thumb used above

* A value taken from the scenario file is independent of every node.
* A value measured by MAVSDK telemetry is independent of the ROS chain (another
  transport, another projection implementation), but not of the PX4 estimator.
* Two figures computed by one function from two inputs compare the inputs, not the
  function; the function needs its own reference (an analytic case or a different
  algorithm).

## What is still not independent

The PX4 estimate stands in for the true position. How closely it follows the simulator's
ground truth is measured by `spikes/line_hold_truth.py`, which is not part of CI. A
position bias in the estimator would pass every rule above; it would show up only in that
spike. This is a residual risk of the simulated bench, to be revisited when a real
sensor and an independent position reference are available.

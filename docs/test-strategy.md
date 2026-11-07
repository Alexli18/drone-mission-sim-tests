# Test strategy: what is tested, how a verdict is reached, and what is left

This page says, per risk, what the suite does, what makes a test pass or fail, where the
truth comes from, and what is *not* covered. Everything runs in simulation (PX4 SITL with
the SIH simulator); nothing here says anything about hardware. Numbers come from the
repository's own runs; the README links them to CI runs.

## Principles

*   **The expected value does not come from the code under test.** Positions come from the
    simulator's truth, coverage from a grid integration, stream integrity from the sensor's
    own capture numbers. Per rule: [oracle-independence.md](oracle-independence.md).
*   **A limit comes from a requirement or from data, never from "make it pass".** The
    height limit (0.15 m) is the georeferencing node's own height band; the marker limit
    (0.10 m) exists to catch a 15 cm lever-arm error. When a rule fails, the cause is
    looked for first (below), and a limit is not widened without data.
*   **A check has been seen to fail.** [Deliberate-break patches](qa-notes.md#deliberate-break-checks)
    break one thing each (nearest pose, no lever arm, ignored clock offset); the unit tests
    must go red.
*   **Three layers, each for what it is cheap at.** Unit tests (seconds) for logic;
    simulator flights (minutes) for PX4 reactions; the ROS end-to-end flight for the chain
    from pose to marker. A defect is guarded at the cheapest layer that can catch it.
*   **Flakes are measured and shown**, in the README and in
    [qa-notes.md](qa-notes.md), with the run that showed them.

## Risks

| Risk | If it goes wrong | Test, layer | Pass or fail, and the source of truth | Residual risk; next level |
|---|---|---|---|---|
| GPS lost in flight | Blind flight, possible fly-away | `test_failsafe[gps_loss]`, simulator flight. The fault is MAVSDK's failure injection (GPS off) 60 s into the survey | Pass: the vehicle leaves Mission, lands and disarms. No distance limit, the descent is blind. Truth: PX4's mode stream and the landed/armed telemetry | Only a clean total loss; no degraded or jumping GPS. Next: partial-degradation faults, then hardware-in-the-loop |
| Data link to the ground lost | Vehicle continues unsupervised | `link_loss`: iptables drops the test-to-PX4 traffic at waypoint 3 (Linux only; skipped on macOS) | Pass: Return to Launch (Hold allowed before), lands, disarms, within 5 m of home | A dropped UDP path, not a radio; timers are scaled for simulated time. Next: a real link under interference |
| Remote control lost | Same | `rc_loss` (neutral sticks stop at waypoint 3) and `rc_loss_wind` (the same with a steady 5 m/s wind) | Same rule; with wind the landing must still be within 5 m of home | One wind speed and direction. Next: a wind sweep |
| Battery runs down | Crash on an empty battery | `low_battery`: the simulated battery drains during the survey | Pass: Return at the critical level, then Land at the emergency level; lands, disarms. No distance limit (the emergency Land may come before home) | A linear drain, not a real discharge curve under load. Next: a measured discharge profile |
| Vehicle leaves the geofence | Leaves the permitted area | `geofence_breach`: after take-off the test flies north at 3 m/s in offboard mode until it crosses the fence (30 m north of home) | Pass: Return to Launch, lands within 5 m of home | One rectangular fence; a predictive geofence does not exist in this PX4 version. Next: other shapes, a moving breach |
| Sensor captures lost, repeated or reordered | Holes in the survey that look like clean ground | Rule 0 of `test_line_survey` (ROS end to end), with unit tests of the checker | Pass: from the first capture both streams share, no capture is missing, repeated or out of order in either; the node's counters of captures seen and released equal the last number plus one. Truth: the sensor's own capture numbers on `/sim/capture_truth` | The checker is unit tested with injected loss, but no flight injects loss. Next: drop captures on the bench and require the failure |
| Time and georeferencing wrong | Every marker placed off the ground | `line_survey_markers`, `line_survey_clock_plus` / `minus` (20 ms), `line_survey_no_markers`; unit tests of interpolation, lever arm and clock offset | Markers within 0.10 m of the painted positions; no marker on a clean line; coverage at least 0.95 and the two coverages within 0.01; clock shift of the markers within 0.01 m of speed times offset; the replayed recording must give the same result as the flight. Truth: the simulator's pose and the painted marker positions | The pose estimate is on both sides of the chain (the estimator itself is a separate check against truth). The marker rule has a margin of 4 mm in the data. Next: a lower-noise reference pose |
| Compute overload of the payload board | Late captures, a wrong pose behind the capture | A proxy only: the sensor-lag guard (the sensor may trail the newest pose by at most 5 simulated seconds) and a 2-vCPU CI runner that is saturated at high speed factors | Pass: the lag stays under the guard; the measured CPU run queue and effective speed factor decide the speed factor of the ROS suite | Not tested on a real board. Next: the same flight with the nodes on the target hardware |
| A ROS node restarts mid-line | Lost state, a silent gap | **Not covered.** Nothing kills a node during a flight | None | Next: kill the georeferencing node mid-line and require that the gap is reported, not hidden |

## One defect, end to end: the height of the vehicle at the start of the line

The long form, with the figures, is in
[qa-notes.md](qa-notes.md#height-at-the-start-of-the-line); this is the path from symptom
to decision.

1.  **Symptom.** The line-survey test failed its height rule (the vehicle must stay within
    0.15 m of 0.4 m) in 2 of 51 measured flights (0.191 and 0.172 m); one of six line-survey
    runs in a three-fold CI run failed on it. The other flights measured 0.044 to 0.115 m.
2.  **Influence.** Either the test is flaky and a red build means nothing, or the vehicle
    really leaves the sensor's height band at the start of the line and the data are taken
    at the wrong height. The first reading would call for a wider limit, the second for
    leaving it alone, so the cause had to be found before the decision.
3.  **Reproduction.** Not on demand: the failure depends on timing. The test's number
    could be rebuilt from the PX4 flight log alone (it matched the test's value to 0.01 m
    in all 51 flights), so 68 flights could be analysed afterwards without new test code.
    They showed that the drop is real (the simulator's truth was 0.2 m low in the worst
    flight), that the vehicle often hovers at 0.2 to 0.3 m for seconds after take-off, and
    that the range finder was fused into the height estimate 2.0 s after take-off in the
    median but up to 12.2 s in the worst case.
4.  **Cause.** The estimator's range-finder "stuck" detector arms when no valid sample was
    seen for 10 s, and releases only when the readings vary by 0.1 m. The simulated
    sensor reports about 0 m with quality 0 on the ground, so after take-off the detector
    starts armed, a vehicle hovering steadily gives no variation, and the height rests on
    the barometer until something moves the vehicle. A model of that logic, written from
    the source, predicted the start of the fusion within 0.2 s in 68 of 68 flights.
5.  **Fix, and the two that did not work.** A longer approach (line start 9 m from home)
    lowered the median but kept the outliers. A 10 s hover before the mission left the late
    fusion in 7 of 17 flights and moved the flight path so that the second marker's error
    went over its limit. Both were closed. The fix that worked removes the cause in the
    simulation image: one line (`docker/px4-sitl/rng_stuck_detector.patch`) sets the
    detector's threshold to 0, which disables it.
6.  **Evidence that it worked.** 24 flights with the patch: the fusion started 0.70 to
    0.78 s after take-off in every flight (51 without it: median 1.84 s, maximum 12.2 s;
    p = 3.5e-12), and the model predicted all 24 with the detector off and none with it on.
    Flights over the 0.15 m limit: 0 of 24 against 2 of 51. That is too few flights to
    call the flake fixed.
7.  **Regression protection.** A unit test checks that every patch in the image directory is
    applied by the Dockerfile and that the detector patch changes exactly one line; the
    weekly run (three seeds, every scenario) keeps counting the height rule; the limit
    stays at 0.15 m. Not yet in place: a rule that reads the fusion start from the flight
    log and fails when it is late.
8.  **Decision.** The limit stays; the patch is part of the simulation image and is
    documented as simulation only. The finding is about the simulated sensor, and it is
    not verified for a real range finder, so it is no argument about hardware. Readiness
    of the line-survey suite: the height rule is reliable enough to gate on, with its
    rate watched weekly, not yet shown to be zero.

A second case, the crash in PX4's lockstep scheduler (a data race, an upstream fix
applied in the image), is written up in
[qa-notes.md](qa-notes.md#px4-lockstep-scheduler-crash).

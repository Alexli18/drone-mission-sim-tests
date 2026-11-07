# drone-mission-sim-tests

[![sim-tests](https://github.com/Alexli18/drone-mission-sim-tests/actions/workflows/sim-tests.yml/badge.svg?branch=main&event=push)](https://github.com/Alexli18/drone-mission-sim-tests/actions/workflows/sim-tests.yml?query=branch%3Amain+event%3Apush)

The badge is the unit job on pushes to `main`. The flight suites run on pull requests, nightly and weekly; the last full run and the weekly pass rate are in [At a glance](#at-a-glance).

Automated, simulation-based tests for drone missions. Each test starts a fresh
PX4 autopilot in software-in-the-loop simulation (SITL) with the SIH physics
model, headless and CPU-only on free GitHub Actions runners. Python tests
(pytest + MAVSDK) drive it over MAVLink and check the outcome; CI fails when a
scenario fails. The goal is to show simulation-based validation with mission
scenarios as data, failure injection, and CI quality gates.

**Status: milestones M1–M5 done.** This README claims only what CI shows. See the [design](docs/designs/drone_mission_sim_tests.md).

**Start here:** [docs/test-strategy.md](docs/test-strategy.md) (what is tested and why, with one defect followed end to end), then [docs/evidence.md](docs/evidence.md) (each claim with its CI run, raw data and limit), then [At a glance](#at-a-glance) below.

**Extension: a ROS 2 line survey.** A second suite flies a low survey line with a
toy sensor on a ROS 2 pipeline fed by PX4 over uXRCE-DDS and checks the result end to end,
plus the QA practice around it (a PX4 crash root-caused and patched, deliberate-break checks,
CI cost control). It is described in the sections [ROS 2 line survey](#ros-2-line-survey-extension)
and [Testing practice](#testing-practice); its CI evidence, including what is not stable yet, is in
[docs/evidence.md](docs/evidence.md) and [What is not stable](#what-is-not-stable).

## At a glance

*   **The risks it is built around:** a failsafe that does not fire (GPS, data link, remote control,
    battery, geofence); sensor data placed in the wrong spot (clock offset, lever arm, pose
    interpolation); sensor captures lost silently; verdicts that cannot fail (independent expected
    values, deliberate-break checks); and flakes that hide real defects. Per risk, what is tested, the
    pass rule and what is left: [docs/test-strategy.md](docs/test-strategy.md).
*   **Last full run:** every suite three times on `main`, [run 42408748342](https://github.com/Alexli18/drone-mission-sim-tests/actions/runs/42408748342) (commit `b5f9e42`,
    7 Nov 2026): 18 of 18 jobs green, 39 flight test results passed. **Weekly line-survey plan** (4 scenarios
    x 3 seeds = 12 tests per run) since the PX4-log rule was added (on `main` and on the branches that became
    `main`): 15 runs, 12 without a failure. The 3 runs with failures hit a stall of the georeferencing node
    (1 test, fixed) and the test bench (2 runs, 4 tests: three at the start of `rosapi`, fixed; one loss of
    capture ids in the recordings, not explained). The last 5 runs, with both fixes (the last two on
    `b5f9e42`: [run 42409105205](https://github.com/Alexli18/drone-mission-sim-tests/actions/runs/42409105205), [run 42409529152](https://github.com/Alexli18/drone-mission-sim-tests/actions/runs/42409529152)), were clean (60 of 60), which is too few to call the flakes
    gone. Causes and run ids: [docs/evidence.md](docs/evidence.md).
*   **One defect, end to end:** the vehicle sometimes hovered 0.2 m low at the start of the survey line.
    Cause: the estimator's range-finder "stuck" detector kept the range finder out of the height estimate
    for up to 12 s after take-off in the simulation. Found from 68 flight logs, confirmed with a one-line
    change in the simulation image (24 flights, fusion within 0.8 s), now watched by a rule that reads the
    PX4 log. [Case study](docs/test-strategy.md#one-defect-end-to-end-the-height-of-the-vehicle-at-the-start-of-the-line),
    [figures](docs/qa-notes.md#height-at-the-start-of-the-line).
*   **Every claim with its test, CI run, raw data and limit:** [docs/evidence.md](docs/evidence.md). The older,
    run-by-run tables are there too.

<details>
<summary>Terms used in this README</summary>

*   **SITL / SIH:** the PX4 autopilot software running on a computer, with a simple built-in physics model instead of a real vehicle.
*   **ULog:** PX4's flight log; the tests read it after the flight.
*   **Estimator (EKF):** the part of the autopilot that combines the sensors into the position and height it believes in. The tests compare that belief with the simulator's truth.
*   **Range finder:** the downward distance sensor that gives the height above the ground.
*   **Lever arm:** the offset of the payload sensor from the vehicle's reference point (15 cm here); a georeferencing that ignores it puts every marker 15 cm off.
*   **Georeferencing:** placing each sensor capture on the survey line from the vehicle's pose at the capture time.
*   **Fusion:** a sensor's data being used by the estimator.

</details>

## Two flights, drawn from the CI logs

![Remote control lost in a 5 m/s wind: hold, return, landing 0.6 m from home](docs/img/failsafe_rc_loss_wind.png)

*Failsafe test `rc_loss_wind`, [run 42396195239](https://github.com/Alexli18/drone-mission-sim-tests/actions/runs/42396195239): the remote control stops at waypoint 3 in a
5 m/s wind; PX4 holds for 5 s, climbs to about 30 m, flies home and lands 0.6 m from it.*

![Line survey: captures, painted and found markers, height over the line, range-finder fusion](docs/img/line_survey_markers.png)

*Line-survey test `line_survey_markers`, [run 42405381157](https://github.com/Alexli18/drone-mission-sim-tests/actions/runs/42405381157): the three painted markers are found
within 5 cm, the height stays within 9 cm of 0.40 m (limit 15 cm), and the range finder joins the height
estimate 0.75 s after lift-off (limit 3 s). Both figures are drawn by `scripts/flight_figures.py` from the
artifacts of those runs; more in [docs/evidence.md](docs/evidence.md#flight-figures).*

## Architecture

```mermaid
flowchart LR
    subgraph GHA["GitHub Actions (ubuntu-24.04)"]
        PT["pytest + framework/"] -- gRPC --> MS["mavsdk_server<br/>(one per test)"]
        MS -- "MAVLink UDP 14540" --> PX4["Docker: px4io/px4-sitl<br/>PX4 + SIH physics"]
        PT --> OUT["JUnit XML, telemetry JSONL, ULog"]
    end
    OUT --> ART["CI artifacts"]
```

## ROS 2 line survey (extension)

The same framework, extended to a drone with a payload: PX4 SIH flies a 12 m line 0.4 m above the
ground; a toy sensor in a ROS 2 Humble container takes timestamped captures over three painted
ground markers; a georeferencing node places each capture on the line from PX4's pose (received
over uXRCE-DDS), decides whether it is usable (`ok`, `outside_line`, `off_line`, `off_height`,
`no_pose`, `saturated`) and a detector finds the markers. The test talks to ROS over rosbridge
only.

```mermaid
flowchart LR
    PT["pytest + framework/"] -- MAVSDK --> PX4["PX4 SIH (patched image)"]
    PX4 -- "uXRCE-DDS" --> ROS["ROS 2 container:<br/>toy sensor, georeferencing,<br/>marker detector, bag recorder"]
    PT -- "rosbridge websocket" --> ROS
```

What the end-to-end test checks (details in [docs/line-survey.md](docs/line-survey.md)):

*   coverage of the line by usable captures is at least 95 %, measured from the sensor's own
    positions, and the georeferencing node's own figure agrees within 1 %;
*   every painted marker is found within 10 cm, and no marker where none is painted;
*   the flight holds height (+-15 cm), cross-track (30 cm) and speed (10 %), and the PX4 log shows
    the range finder joined the height estimate within 3 s of take-off;
*   with the sensor clock 20 ms ahead or behind PX4's, every georeferenced position shifts by
    speed x offset (within 1 cm);
*   replaying the recorded bag into fresh nodes gives the same usable/unusable decisions from
    takeoff on and the same markers;
*   the sensor never falls more than 5 simulated seconds behind the poses;
*   the nodes under test subscribe to no simulation-only topic.

Also in this extension: the low-altitude PX4 parameter profile with its findings and a
true-versus-estimated cross-track check from the ULog ([QA notes](docs/qa-notes.md#low-altitude-line-following)),
the bridge setup and its gotchas ([docs/ros-bridge.md](docs/ros-bridge.md)), about 200 new unit tests
(georeferencing, the toy sensor and detector, the rosbridge client against a fake websocket
server, replay helpers, the CI planner), and four scenarios (`scenarios/line_survey_*.yaml`).

## What is not stable

Causes, run ids and numbers: [docs/evidence.md](docs/evidence.md#what-is-not-stable).

*   **Height over the line (limit ±0.15 m):** 2 of 51 measured flights went over (0.191 and 0.172 m). Cause
    found: the simulated range finder tripped the estimator's stuck detector and stayed out of the height
    estimate for up to 12 s after take-off; the detector is disabled in the simulation image and a rule watches
    it. Too few flights since to call it fixed, so the limit stays.
    [QA notes](docs/qa-notes.md#height-at-the-start-of-the-line).
*   **Position of the second marker (limit 0.10 m):** the largest error measured was 0.096 m, a margin of 4 mm.
    The limit stays: it is what catches a georeferencing that ignores the 15 cm lever arm (`docs/breaks/`).
*   **A stall of the georeferencing node (fixed):** one weekly run lost 32 captures to `no_pose` although every
    pose had arrived. The hold timeout now also needs wall time; a unit test models the stall, and the stalls are
    too rare for CI alone to confirm it.
    [QA notes](docs/qa-notes.md#a-stall-of-the-georeferencing-node-turns-into-no_pose).
*   **Test-bench failures:** `rosapi` crashed when asked about a node it did not see yet (fixed, the test waits
    for the node); 15 capture ids once went missing from the test's recordings (not explained).
    [QA notes](docs/qa-notes.md#rosapi-dies-when-asked-about-a-node-it-does-not-see-yet).

## Testing practice

[docs/test-strategy.md](docs/test-strategy.md) says per risk what is tested, what makes a test pass
or fail, where the truth comes from and what is not covered (including a risk with no test yet), and
follows one real defect from symptom to decision.

[docs/qa-notes.md](docs/qa-notes.md) is the running list of what broke and how it is guarded:

*   **A flaky crash with a root cause upstream.** PX4 v1.18.0-rc1 aborted in its lockstep scheduler
    (a glibc assertion, PX4 issue 29060). The tests now run a PX4 image rebuilt with the upstream
    fix (`docker/px4-sitl/`, `scripts/build_px4_image.sh`), cached in CI by digest and directory hash.
*   **Flakes found and root-caused:** the DDS gotchas, pose loss from a small QoS queue, CPU
    saturation of a 2-vCPU runner (a lower speed factor for the ROS suite), a sensor node falling
    behind (a lag guard), a replay race on subscriber connect, a reactor-thread bug in the rosbridge
    client and a race in the unit test's own fake server found under `pytest -n 2`. The latest three:
    the late height fusion after take-off, traced to the simulated range finder tripping the
    estimator's stuck detector ([QA notes](docs/qa-notes.md#height-at-the-start-of-the-line)); a stall
    of the georeferencing node that turned good poses into `no_pose`
    ([QA notes](docs/qa-notes.md#a-stall-of-the-georeferencing-node-turns-into-no_pose)); and
    `rosapi` crashing when asked about a node it does not see yet
    ([QA notes](docs/qa-notes.md#rosapi-dies-when-asked-about-a-node-it-does-not-see-yet)).
*   **Measuring the truth, not only the estimate.** `spikes/line_hold_truth.py` projects the
    simulator's ground truth from the ULog on the line and compares it with what PX4 estimates, which
    is what the test sees.
*   **Deliberate breaks.** `docs/breaks/*.patch` each break one thing the suite guards;
    `scripts/check_breaks.sh` proves the unit tests go red for every one, in the unit job of every CI run.
*   **CI cost control.** Unit-only on push to `main`, path filters per suite, a `full-ci` label,
    docs-only changes ignored, a nightly run only if `main` changed, a weekly multi-seed pass-rate
    run, cancelled superseded pull-request runs, cached images and pip, `pytest-xdist`.

## Pinned versions

| Component | Pin |
|---|---|
| PX4 SITL image | `px4io/px4-sitl:v1.18.0-rc1@sha256:39d5a950…` (release candidate; see limitations), with its `px4` binary rebuilt from the same source plus two patches: the upstream lockstep fix (`lockstep_scheduler.patch`) and a simulation-only change that disables the range finder's stuck detector (`rng_stuck_detector.patch`, one line) (`px4-sitl-patched:39d5a950208a`) |
| ROS (extension) | `ros:humble-ros-base` by digest, Micro XRCE-DDS Agent v2.4.3 by commit, `px4_msgs` by commit, `rosbridge_server`; `roslibpy==1.7.0` in the tests |
| Vehicle | `sihsim_quadx` |
| MAVSDK-Python | `mavsdk-grpc==4.0.5` |
| Python | 3.11 |
| Runner | `ubuntu-24.04` |

All pins live in `config.yaml` and `pyproject.toml`.

## Running

**In CI:** the `sim-tests` workflow decides per event what to run (`scripts/ci_plan.sh`): a push
to `main` runs the unit job only; a pull request runs the unit job and the suites whose paths it
changes (all of them with the `full-ci` label); documentation-only changes run nothing; a nightly
schedule runs every suite only if `main` changed in the last 24 hours, and a weekly one runs the
ROS survey three times with different seeds. It can also be started by hand from the Actions tab
(`gh workflow run sim-tests.yml -f runs=3` runs every suite 3 times, as the pass-rate evidence
above did; `-f weekly=true` runs the weekly plan: the ROS survey, three seeds, all four scenarios). The report job writes the results table. See
[CI cost control](docs/qa-notes.md#ci-cost-control).

Every flight job also writes Allure results (`allure-results-<suite>-<run>` artifacts), and the `allure`
job merges them into one HTML report, the `allure-report` artifact of the run: download it, unzip it and
open `index.html`. It shows each test once per matrix cell (suite, run, seed offset, scenario and PX4
image are parameters), the line survey as steps (start, take off, mission, land and drain, final
figures, pass rules, replay), the recorded metrics, and on a failure the PX4 mode history and the last
telemetry. ULogs, rosbags and telemetry stay in the `results-*` artifacts, matched by the test id and
the run id in the test's description. See [Allure report](docs/qa-notes.md#allure-report).

**Locally (Linux, Docker, Python 3.11; `link_loss` needs `sudo iptables`):**

```sh
pip install -e ".[dev,ros]"
scripts/build_px4_image.sh        # pulls the pinned image and rebuilds px4 with the lockstep fix
pytest -m unit
pytest -m sitl tests/test_smoke.py tests/test_basic_flight.py tests/test_survey_mission.py tests/test_failsafes.py
```

The ROS line survey additionally needs the two ROS images (the base image takes a while to build once):

```sh
scripts/build_ros2_images.sh base && scripts/build_ros2_images.sh thin
pytest tests/test_line_survey.py -k line_survey_markers
```

`scripts/check_breaks.sh` applies each deliberate-break patch to a clean copy and requires the unit
tests to fail (CI runs it in the unit job).

**Locally on macOS (Docker Desktop, Apple Silicon):** the same commands work;
the image runs natively on arm64 and its entrypoint routes MAVLink to the Mac.
SIH reaches only about half the requested speed there (×3.7 of ×10; ×8.7 on
the CI runner). The flight tests still pass; the smoke test's speed-floor check
is enforced only in CI and reported as a warning locally. The ROS suite on a Mac needs the
`--dds-own-ip` workaround (applied automatically; see [docs/ros-bridge.md](docs/ros-bridge.md)). Timing numbers in
this README come from CI, which is the source of truth.

## Adding a scenario

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

## Known limitations

*   SIH physics is simplified; there are no real sensors.
*   PX4 v1.18.0-rc1 is a release candidate, chosen because it is the first
    version where SIH supports GPS failure injection. The image is pinned by
    digest.
*   The simulation runs at ×10. Timeouts in config are wall-clock seconds;
    scenario durations are measured on PX4's simulated clock.
*   Duration bounds come from a hand calculation calibrated on one measured
    flight (PX4 nearly stops at each 90° corner); they catch skipped
    waypoints or a stuck vehicle, not small timing changes.
*   PX4 v1.18.0-rc1 answers `UNSUPPORTED` to the built-in data link and RC
    failure injections (spike S5). Link loss is therefore produced with
    `iptables` (Linux only; the test is skipped on macOS), and RC loss by
    stopping the MAVLink manual-control stream.
*   PX4 rejects any mission, fence upload during a mission, or goto target
    that crosses the geofence ("Geofence violation for waypoint 3", spike
    S9). The geofence test therefore leaves the fence with offboard velocity
    setpoints, not by flying a mission.
*   Barometer and magnetometer failure injection is accepted by PX4 in SIH
    but has no visible effect (no warning, no mode change), and losing a
    motor makes the quadrotor fall with no mode change. None of these is a
    scenario; see the design's "Additions after M5".
*   After GPS loss the landing position is not checked: the reported
    position is the estimator's dead-reckoning guess, which ended 93–519 m
    from home in CI runs 41719048626 and 41819419053.
*   GitHub disables scheduled workflows after 60 days without repository
    activity (this stops the nightly runs).
*   The extension runs PX4 with simulation-only settings (`EKF2_IMU_CTRL 0`,
    `UXRCE_DDS_SYNCT 0`); a real vehicle needs the IMU bias states and DDS time
    synchronization. The toy sensor reads PX4's position *estimate*, so the end-to-end
    test checks the data path, not the estimator; the estimator is measured against
    ground truth by a separate spike.
*   The patched PX4 image carries two patches. The lockstep fix is a stopgap until the pinned PX4
    release contains the upstream fix; building the image compiles PX4 and is slow on a cache miss.
    The stuck-detector patch changes the simulated autopilot's behaviour so that the simulated range
    finder (which reports about 0 m on the ground) does not keep itself out of the height estimate; it
    is not verified for a real range finder.
*   This is a simulation suite. Nothing here is hardware-in-the-loop or a flight with a real vehicle.

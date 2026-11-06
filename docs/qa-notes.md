# QA notes: what broke, how it was found, and what guards it now

A running list of the flakes and surprises met while building this suite, each with its
root cause and the check that would now catch it. Numbers are measurements from
development runs (Apple Silicon Mac with Docker Desktop, and the free 2-vCPU GitHub
runner); they are examples, not a guarantee. The README lists what CI has shown.

## Summary

| # | Symptom | Root cause | Fix and guard |
|---|---|---|---|
| 1 | PX4 aborted mid-flight in one of three repeated runs of the ROS suite | A data race in PX4's lockstep scheduler (glibc assertion), PX4 issue 29060 | Rebuilt PX4 with the upstream fix (PR 29061); see below |
| 2 | No pose at the ROS side, or a pose in the wrong time base | DDS domain overwritten, time sync on, localhost-only, Docker Desktop UDP | Four settings, each with a guard: [ros-bridge.md](ros-bridge.md) |
| 3 | Coverage 0.55 from one node, 0.88 from the other | Poses lost from a best-effort queue of depth 5 | Depth 500, timer-driven sensor, cross-checks of the two coverages |
| 4 | Pass rule failed and the CPU run queue was about 9 on 2 vCPUs | PX4 plus the ROS stack saturate a 2-vCPU runner at speed factor 10 | `speed_factor_ros: 4` |
| 5 | Captures became `no_pose` late in the flight | The sensor node fell behind the poses by several seconds | Sensor lag guard; per-capture work batched and moved off the pose callback |
| 6 | Replay disagreed with the live run on the first captures | `bag play --delay` raced the subscription match | `bag_player` waits for subscribers; compare only from takeoff |
| 7 | The second websocket connection of a session never opened | `roslibpy` object created off the Twisted reactor thread | Create and close on the reactor thread |
| 8 | A unit test failed only under `pytest -n 2` | The fake websocket server counted the connection after answering | Count first, then answer |
| 9 | The vehicle wandered 0.35 to 0.44 m off the line while the test saw 0.26 to 0.30 m | The estimator was wrong in a way the control loop follows | Parameter profile plus a true-versus-estimated check |

## PX4 lockstep scheduler crash

**Symptom.** In one of three repeated runs PX4 aborted right after the mission started
(about 40 simulated seconds), with `Fatal glibc error: tpp.c:83
(__pthread_tpp_change_priority): assertion failed`; the test then timed out. The PX4 log
shows the message once, as its last line. An earlier crash on the Mac, when the DDS
client was restarted while PX4 ran, fit the same pattern.

**Narrowing it down.** The message talks about priority-protect mutexes, but PX4 sets
none, so the suspicion moved from Docker scheduling priorities to memory that is no
longer a mutex. The earlier suites (no DDS agent) never showed it: none of the PX4 logs of the
same CI run had the message. It did not reproduce locally in 300 s at speed factor 4 with
the agent and CPU load, nor in 59 DDS client restarts: it needs the timing of a loaded
runner.

**Root cause.** In PX4 v1.18.0-rc1, `LockstepScheduler::set_absolute_time()` first notes
the waiters whose simulated timeout has passed and only then locks each waiter's mutex to
wake it. A waiter woken in between by its own signal (a uORB publish posting the
semaphore of `px4_poll()`) sees its timeout flag set, skips the wait for
`set_absolute_time()` and returns. The mutex and condition variable lived on its stack, so
the scheduler then locks a reused stack slot. The uXRCE-DDS client polls uORB with a 1 ms
timeout and is woken constantly once the agent is connected, so it is the usual victim.
This is [PX4 issue 29060](https://github.com/PX4/PX4-Autopilot/issues/29060); the fix is
on PX4 `main`, and [PR 29061](https://github.com/PX4/PX4-Autopilot/pull/29061) backports
it to `release/1.18`.

**Fix.** `docker/px4-sitl` rebuilds the `px4` binary of the pinned image from the same
v1.18.0-rc1 source with the PR's diff applied (`lockstep_scheduler.patch`), natively per
architecture (`scripts/build_px4_image.sh`). The result is checked against the original:
same boot output, and the smoke and hover tests pass. The CI cache key is the pinned
digest plus the hash of `docker/px4-sitl/`, so the image is built once per change; the
priming job fills the cache once per workflow run. The patched image is used for *all*
PX4 runs, not only the ROS suite. Drop it once the pinned PX4 release contains the fix.

**Lesson.** A crash in a dependency can look like a test or environment flake. What
separated them: the log (one line, last), the absence in the runs without a DDS client,
and reading the scheduler code against the assertion.

## DDS and the bridge

See [ros-bridge.md](ros-bridge.md) for the four settings (domain through the
environment, time synchronization off, localhost-only off, the Docker Desktop address)
and the replay and websocket details.

## CPU saturation on a 2-vCPU runner

With `speed_factor: 10` the whole ROS stack plus PX4 does not fit on the free runner.
A CI job logged `nproc` (2) and a one-second `vmstat` over the test (kept as an
artifact): run queue about 9, 10 % idle. Symptoms were lost poses, `rosapi` not
answering in 30 s, and an effective speed factor that varied from 5.7 to 7.7 between
runs. Runs with the ROS container use `px4.speed_factor_ros` (4); the M1 to M5 suites
stay at 10. The `vmstat` log and the effective speed factor (recorded per test) are the
evidence to look at before touching any timeout. The same family showed up once
locally when another job was using the machine: the first `/rosapi/node_details` call
timed out after 30 s (rosbridge serves service calls on its main thread by default, so
it is the first thing that stops answering). A repeat on an idle machine passed; the call
keeps its hard timeout rather than a retry, because a retry would hide exactly the
condition the load check is there to show.

## The sensor node falling behind

An earlier, heavier sensor model cost about 0.4 to 0.5 ms of CPU per capture on the
runner and kept the node busy for 90 to 96 % of the wall time. `rclpy` serves one pose
per executor cycle, so its pose queue filled up and the node saw poses about 3 s of wall
time late. Its captures then reached the georeferencing node after that node's pose
history (10 s of simulated time) no longer held the poses they needed: `no_pose`, and
coverage lost. The positions of the samples were fine; the *timing* between nodes was not.

Fix: compute the samples of a capture as one batch, and move the work off the pose
callback (the callback appends; a timer advances). Guard: the test samples the **sensor
lag** during the flight (the georeferencing node's newest pose time from `/survey/status`
minus the newest true sample time from `/sim/capture_truth`) and fails above 5 simulated
seconds, half the pose history. The lag is read from topics the sensor and the node
under test already publish, so the guard needs no instrumentation of either.
The toy sensor is light enough that it never trips the guard; the guard is kept because
the failure mode is generic, and `sensor_lag_s` is unit tested.

## Replay race on subscriber connect

See [ros-bridge.md](ros-bridge.md#replay-bag_player). In short: a fixed delay before
playing a bag is a race with the subscription match; wait for the match instead.

## Races in the test code itself

* **Twisted reactor.** `tests/unit/test_rosbridge.py` reproduces the "second connection
  never opens" bug with a fake server, with no ROS and no Docker.
* **The fake server's counter.** Under `pytest -n 2` a fast client could connect, close
  and assert before the server thread had counted the connection. The server now counts
  before it answers the handshake. When a unit test is flaky only under xdist, suspect
  the test's own fixtures before the code under test.

## Low-altitude line following

The scenarios fly 0.4 m above the ground. The parameter set in
[`scenarios/_low_altitude.yaml`](../scenarios/_low_altitude.yaml) (a `profile`) was found
by two experiments on a 20 m line at 0.5 and 1 m/s.

**First experiment: can PX4 SIH hold 0.4 m at all?** With PX4's defaults: no false landing
detection at 0.4, 0.6 or 1.0 m, but the estimate led the vehicle up to 1 m off the line,
the height error was 0.17 to 0.36 m, and with GNSS as the height reference the vehicle
sometimes stayed on the ground while the estimate showed 0.5 to 1.5 m. The set that worked:
barometer as the height reference (`EKF2_HGT_REF 0`), range aiding over a nearly fixed
terrain (`EKF2_RNG_CTRL 2`, `EKF2_TERR_NOISE 0.5`), raised estimator noise
(`EKF2_ACC_NOISE 1.0`, `EKF2_GYR_NOISE 0.1`) and stiffer position and velocity loops
(`MPC_XY_P 1.8`, `MPC_XY_VEL_P_ACC 4`, `MPC_XY_VEL_I_ACC 2`, `MPC_Z_P 1.5`, ...). Height
error of the estimate fell to 0.09 to 0.11 m.

**Second experiment: does it really hold the line?** The test's cross-track check reads
PX4's *estimate* (MAVSDK telemetry). An estimator that is wrong in a way the controller
follows passes that check while the vehicle wanders: the controller corrects the
estimate, not the truth. SIH exposes the simulator's ground truth in the ULog
(`vehicle_local_position_groundtruth`), so `spikes/line_hold_truth.py` flies the line the
way the test does, projects both the truth and the estimate on the line, and reports the
true maximum, RMS and dominant period of the sideways motion, and estimate minus truth.

* With the first set the true cross-track reached 0.35 to 0.44 m (RMS 0.12 to 0.15 m)
  while the estimate showed 0.26 to 0.30 m and the test's check passed. Estimate minus
  truth was up to 0.14 to 0.19 m.
* The wander was not the velocity loop (one-at-a-time changes of `MPC_XY_VEL_P_ACC`,
  `_I_ACC`, `_D_ACC`, `MPC_VELD_LP` left the true maximum at 0.34 to 0.44 m; PX4's default
  loops were worse), and not a simulation-speed artefact (speed factor 1 gave the same as
  4). It was the estimator: the attitude estimate differed from the simulator's by 0.8
  degrees RMS in roll, 0.13 m/s^2 of lateral acceleration, and the setpoint stayed on
  the line.
* Two changes helped, each alone only partly: the output-predictor time constants
  `EKF2_TAU_POS` and `EKF2_TAU_VEL` at their minimum 0.1 s (default 0.25 s: the estimate
  lags the vehicle) and `EKF2_IMU_CTRL 0` (no gyro-bias, accel-bias or gravity-vector
  states). Together: true maximum 0.19 to 0.22 m (RMS 0.08 m) over 21 flights, estimate
  maximum 0.20 to 0.23 m, against the 0.3 m limit of the test.
* Further changes on top (`EKF2_GYR_NOISE`, `EKF2_ACC_NOISE`, loop gains) moved the
  maximum by at most 0.02 m, so they stay as they were.
* Reproduced on this repository's 12 m scenario (`line_survey_markers`, Mac, speed
  factor 4): with the estimator at its defaults (`EKF2_IMU_CTRL 1`, `EKF2_TAU_POS` and
  `EKF2_TAU_VEL` 0.25) two flights gave a true maximum of 0.442 and 0.260 m against
  0.306 and 0.186 m for the estimate (the test's own check read 0.309 and 0.233 m, so
  one flight would have passed the 0.3 m limit at a true 0.26 m); with the profile one
  flight gave 0.179 m true and 0.187 m estimated.

> **`EKF2_IMU_CTRL 0` is a simulation-only setting.** SIH's simulated IMU has no bias, so
> the bias states only add attitude error there. A real IMU has bias, and a real
> autopilot needs those states. The profile says so at the top; a unit test asserts the
> warning stays in the file and that the setting is never in the shared baseline.
> `UXRCE_DDS_SYNCT 0` is likewise a simulation-only divergence.

**The technique.** *The test checks the estimate, but we also measure the ground truth.*
Whenever a PX4 image or parameter changes, re-run `spikes/line_hold_truth.py`; if the
true cross-track comes back above 0.25 m while the test still passes, the estimate has
started to hide the problem. PX4 SIH is deterministic on one machine, so a plain repeat
gives the same flight; the harness imitates CI timing by waiting `K * 1.3 s` in the hover
before the mission starts (`--seeds K`).

The end-to-end test deliberately uses the estimate on both sides (the sensor reads the
estimated position, the node under test georeferences with the estimated pose): it
tests the chain, not the estimator, and the chain's result is then exact enough to
check clock offsets to a centimetre.

## Deliberate-break checks

A test suite that has never been seen to fail proves little. `docs/breaks/` holds
patches that each break one thing the suite claims to guard; `scripts/check_breaks.sh`
applies every patch to a clean copy of the repository and requires the unit tests to go
red (and, first, green without the patch).

| Patch | Breaks | Caught by |
|---|---|---|
| `nearest_pose.patch` | The georeferencing uses the nearest pose instead of interpolating | 6 unit tests: the position, interpolation and clock-offset tests of `test_georeference.py` |
| `no_lever_arm.patch` | The georeferencing ignores the sensor's body-frame offset | 4 unit tests: the offset test of `test_georeference.py` and the in-process chain tests of `test_markers.py` (markers 12 to 15 cm off) |
| `ignore_clock_offset.patch` | The sensor stamps captures with the true time, not the sensor clock | 3 unit tests in `test_markers.py` (stamps and marker shift) |

Not every break is caught by the same layer, on purpose: the flight scenarios use a
lever arm and a clock offset so that the end-to-end test catches these too, but the unit
tests catch them in a second, not in a minute of simulator run. The first is also
checked at the flight layer: with `no_lever_arm.patch` applied and the ROS image rebuilt,
`line_survey_clock_plus` failed with `a marker found within 0.1 m of s = 2.50 m | nearest
2.379 m` (and for 9.50 m), `false markers at s = 2.38 m, 9.37 m` and a clock shift of
-0.13 m against +0.02 m expected. A break that only the
flight test can catch (for example a pose queue that is too small) is the end-to-end
test's job. The CI gate itself was checked the same way in the base project (a throwaway
branch that disables a failsafe turns the workflow red, see the README).

## CI cost control

The flight suites take minutes per job and the ROS suite needs a PX4 image and a ROS
image, so the workflow decides per event what to run (`scripts/ci_plan.sh`, unit tested
with throw-away git repositories in `tests/unit/test_ci_plan.py`):

| Event | Runs |
|---|---|
| Push to `main` | Unit job only: the pull request already ran the rest |
| Pull request | Unit job; the flight suites when `framework/`, scenarios, scripts, the PX4 image or the config change; the ROS jobs when the ROS workspace, `framework/`, tests, scenarios, scripts, docker or CI change, with only the main scenario (smoke test) |
| Pull request with the `full-ci` label | Everything; adding another label starts nothing |
| Documentation-only change (`*.md`, `docs/designs`, `docs/research`) | Nothing (`paths-ignore`); the deliberate-break patches are code and not ignored |
| Nightly | Everything, but only if `main` had a commit in the last 24 hours |
| Weekly | The ROS survey three times with different seeds, plus the weekly-only scenarios: a pass rate, not a single result |
| Manual dispatch | Everything; `runs=3` runs every suite three times |

Also: a new push to a pull request cancels the older run (`concurrency`,
`cancel-in-progress` for pull requests only; scheduled and manual runs are never
cancelled); the unit job uses the pip cache and `pytest-xdist -n 2` (two vCPUs); one
"image priming" job fills the PX4 image cache once per run (`lookup-only`: on a hit it
checks the entry exists and downloads nothing), so the matrix jobs do not each build it
on a miss; the report job runs whenever at least one test job ran, tolerates skipped
ones, and reads JUnit-only artifacts; scheduled times are off the hour, because GitHub
delays or drops runs at the top of the hour; the actions are at their Node 24 majors.
Scheduled workflows are disabled by GitHub after 60 days without repository activity.

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
| 10 | Height over the line above the 0.15 m limit in 2 of 51 flights | The estimator's range-finder "stuck" detector kept the range finder out of the height estimate for up to 12 s after take-off (simulated sensor) | The detector is disabled in the simulation image; a rule reads the PX4 log and fails when the fusion starts later than 3 s after take-off |
| 11 | 32 captures `no_pose` and coverage 0.86 in one weekly flight although every pose had arrived | The georeferencing node stalled for about 0.4 s; the simulated-time hold timeout fired on a stale "now" while the poses were still queued | The timeout also needs wall time (`hold_min_wall_s`); unit tests model the stall; stalls are counted in the node's log |
| 12 | `/rosapi/node_details` got no reply within 30 s, or did not exist | Humble's `rosapi` crashes when asked about a node it does not see yet; the test asked 0.4 s after the start | The test waits until `rosapi` lists the nodes (`wait_for_nodes`) |

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
it is the first thing that stops answering). A repeat on an idle machine passed. That
explanation was incomplete: see "rosapi dies when asked about a node it does not see
yet" below, which is what the later failures of this call were.

## rosapi dies when asked about a node it does not see yet

**Symptom.** The first service call of a line-survey test, `/rosapi/node_details`, got no reply
within 30 s (twice on [42400058966](https://github.com/Alexli18/drone-mission-sim-tests/actions/runs/42400058966), once earlier on [42374481708](https://github.com/Alexli18/drone-mission-sim-tests/actions/runs/42374481708)) or
failed with "Service /rosapi/node_details does not exist" ([42399318960](https://github.com/Alexli18/drone-mission-sim-tests/actions/runs/42399318960)). Before the flight, so
nothing to do with the vehicle; a retry on an idle machine passed.

**What the log shows.** In both "no reply" jobs of [42400058966](https://github.com/Alexli18/drone-mission-sim-tests/actions/runs/42400058966) the ROS log of the test holds
a traceback of the `rosapi` node, `TypeError: cannot unpack non-iterable NoneType object` in
`get_node_details`, followed by `[rosapi_node-3]: process has died ... exit code 1`. After that no
`/rosapi` service exists, so the test waits its 30 s.

**Why.** In ROS 2 Humble (checked in the public source of `rosbridge_suite`, branch `humble`;
the installed file differs by two lines) `get_node_details` unpacks the result of
`proxy.get_node_info(name)`, and that returns `None` when the node is not in the graph that
`rosapi` sees. The exception is not caught, the executor stops and the process exits. The test asked
0.4 s after the launch started, right after "SURVEY READY", which waits for poses, not for the
graph: on a slow runner the discovery of the two nodes under test comes later than that. So it is a
race between the start of the nodes and the first question, and a crash of `rosapi` that no retry
can survive (the process is gone). It also explains why the earlier explanation, load, fitted: load
widens the race.

**The third failure is not explained.** The "does not exist" job of [42399318960](https://github.com/Alexli18/drone-mission-sim-tests/actions/runs/42399318960) has no
traceback in its log; the likeliest reading is that `rosapi` had not advertised its services yet
when the call came, but that is not confirmed.

**Fix.** Before the first `node_details` call the test waits until `/rosapi/nodes` lists both
nodes (`wait_for_nodes` in `framework/rosbridge.py`, `timeouts_wall_s.node_discovery`, 30 s). That call
only lists names and is safe early; a call that fails because `rosapi` has not started is tried again.
The question itself, and the rule that the nodes subscribe to no `/sim/*` topic, are unchanged. If a
node never appears, the test fails with the nodes it was waiting for and the ones listed, instead of
30 s of silence. This is waiting for the precondition of the check, not retrying the check. Five unit
tests with a scripted fake `rosapi` (the nodes appear late, `rosapi` not started, a node that never
appears, `rosapi` that never answers); making the wait accept at once turns two of them red.

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

## A stall of the georeferencing node turns into no_pose

**Symptom.** In the weekly run [42396898449](https://github.com/Alexli18/drone-mission-sim-tests/actions/runs/42396898449), seed 3 job, `line_survey_clock_minus` failed the
coverage rule: 0.8631 against 0.95, a gap of 0.87 m. 32 captures had the reason `no_pose`, in two
bursts of 15 and 17 captures in the middle of the line; two other tests of the job lost 3 captures each,
the fourth none. Two other weekly runs (24 flights) had no `no_pose` capture.

**Not a lost pose.** PX4 published `vehicle_local_position` without a gap (the largest gap in the ULog
is 12 ms). The node's own counters: `poses=1788 dropped=0 pose_gaps=0`, the largest pose interval
24 ms. The sensor never fell behind (0.07 s). All 32 captures were released by the simulated-time
hold timeout (`release: timeout`), none by a missing pose.

**A stall, seen in the bag.** The bag records the wall time of every `/survey/capture` and of the
`/survey/georef_capture` the node published for it. Normally the node publishes within a millisecond
of the capture. In the flight, captures kept arriving every 10 to 20 ms while the node published nothing,
and then released a whole batch within 30 ms; the latency of the batch falls linearly from 0.41 s to
0. Six such stalls in the flight (0.09, 0.16, 0.20, 0.22, 0.41 and 0.42 s); the two long ones produced
the `no_pose` captures, the shorter ones none.

**Why a stall loses data.** A held capture is stamped with the node's "now" (the newest pose time) when
it arrives, and released as `no_pose` once the pose clock has advanced `hold_timeout_s` (0.5 simulated
seconds) beyond that stamp without covering the capture. After a stall, the captures are processed
before the backlog of poses: they are stamped with a pose clock that is old, so a capture more than
0.5 s ahead of it times out before its own pose is processed, although the pose is in the queue. With
the real `HoldQueue` code and a stall of 1.5 simulated seconds (0.4 s of wall time at the speed
factor 3.7), the first 10 captures are fine and the rest are `no_pose`/`timeout`; processed in time
order, none is lost (a model run once, not yet a test in the repository; the flight shows 15 and 17 of 32 and 33).

**What stalled it.** The stalls coincide with disk wait on the runner: the two long ones at
17:36:07 and 17:36:08, the seconds with 13 % and 26 % wait in the job's `vmstat` (five tasks blocked
in the first of them); the shorter stalls fall in seconds with 6 to 11 % wait, so the wait alone does not
predict the length of a stall. The node writes
`decisions.jsonl` (one `open` and `write` per released capture) and its log in the callback, on a
runner whose disk was busy with PX4's log and the bag. That is a correlation, not a proof; a
stall in the DDS transport would look the same from here.

**Fix.** The timeout also needs `hold_min_wall_s` (1 s) of wall time, so a backlog that is worked
off within milliseconds cannot time a capture out; the node counts stalls (callbacks 0.15 s apart or more)
in its stats line (`stalls=`, `max_stall_s=`). Unit tests model a stall of 1.5 simulated seconds: all
30 captures are `covered`/`ok` with the guard, 20 are `no_pose` without it, and removing the guard from
the code turns two tests red. Stalls are rare (1 flight in 24), so CI can only show that nothing breaks:
on the fix branch, 4 weekly runs (48 flights) had no `no_pose` capture and coverage 1.0 in every
flight, and the node's own stall counter read 0 to 1 stalls of 0.16 to 0.17 s in the logs looked at.

**A different failure in the same window.** Two other weekly runs on the branch failed in the test
bench, not in the node: [42399318960](https://github.com/Alexli18/drone-mission-sim-tests/actions/runs/42399318960) (`/rosapi/node_details` did not exist in one job; in another
job 15 consecutive capture ids were missing from the test's recordings of the truth and of the
georeferenced captures, while the node's counters show 645 of 645 captures released, none `no_pose`) and
[42400058966](https://github.com/Alexli18/drone-mission-sim-tests/actions/runs/42400058966) (`/rosapi/node_details` gave no reply within 30 s, twice). Rule 0 caught the missing
ids as intended. The two control runs on `main` right after were 24 of 24, and the other four runs on the
branch 48 of 48, so it looks like a spell of a slow runner pool, but that is a guess: rosbridge serves
service calls and the recordings on its main thread, and a stall there looks the same from the test.

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

### Height at the start of the line

Two of 51 line-survey flights went over the 0.15 m height limit (0.191 and 0.172 m); the
other 49 measured 0.044 to 0.115 m. The test's number can be rebuilt from the ULog alone
(`vehicle_global_position` minus `home_position`; it matched the junit value to 0.01 m in all
51 flights), and the drop is real: in the worst flight the simulator's truth was 0.2 m low.

What the logs of 68 flights show. After lift-off the vehicle often hovers at 0.2 to 0.3 m, not
at the commanded 0.4 m, for seconds (the median truth height over the first 10 s is 0.33 m).
The test starts the mission as soon as the height reaches 0.25 m (0.4 minus the 0.15 m
limit), about 1 s after lift-off. Until the range finder is fused into the height estimate,
the estimate rests on the barometer. The fusion starts 2.0 s after lift-off in the median
(90th percentile 11.6 s, maximum 12.2 s), even though the range readings are correct and the
estimator does not reject them (`innovation_rejected` 0, test ratio near 0,
`cs_rng_kin_consistent` 1). The longer the fusion is missing, the lower the vehicle hovers
(Spearman -0.76, n = 68). The profile sets `EKF2_RNG_CTRL 2`, which PX4 v1.18.0-rc1 defines
as range aiding always on, so the speed limit `EKF2_RNG_A_VMAX` does not apply, and no
flight lost the fusion at speed.

Cause, confirmed by an experiment (below). The range finder's "stuck" detector
(`sensor_range_finder.cpp`, `updateStuckCheck`) is armed when no valid sample was seen for
10 s, and releases only when the readings vary by 0.1 m. The simulated sensor reports about
0 m with quality 0 on the ground, so after take-off the detector starts armed; a vehicle
hovering steadily at 0.25 m, low because the barometer is biased, produces no 0.1 m of
variation and stays stuck, until it moves vertically (for instance when the mission
starts). An offline model of that logic, written from the source, predicted the time the
fusion started to within 0.13 to 0.20 s in 68 of 68 flights. The PX4 log cannot show it
directly: `_is_stuck` is not logged, and the flag `cs_rng_stuck` is never set in this
version. With a real sensor the detector would normally not be armed (not verified here).

Experiments so far, all on CI. Starting the line 9 m from home instead of 5 m lowered the
median error (0.070 against 0.083 m) but not the outliers. Hovering 10 s after take-off
before the mission gave 0.064 to 0.079 m in 17 flights, but the fusion still started after
the end of the hover in 7 of them, and the shifted flight path pushed the position of the
second marker over its 0.10 m limit in several flights (that rule had a margin of 4 mm to
begin with).

The decisive experiment sets the stuck detector's threshold to 0, which disables it, in the
simulation image (`docker/px4-sitl/rng_stuck_detector.patch`; one line). Four CI runs with
three seeds each gave 24 flights, all passed, and the ULogs show:

| | Flights | Fusion delay after lift-off, median | Maximum | Over 3 s |
|---|---|---|---|---|
| Detector on (5 m and 9 m series) | 51 | 1.84 s | 12.2 s | 6 |
| Detector off (patched image) | 24 | 0.72 s | 0.78 s | 0 |

The difference is not chance (Mann-Whitney U = 0, p = 3.5e-12). The offline model, written
before the patch existed, was run on the 24 new flights in both settings: with the detector
off it predicted the start of the fusion to within 0.3 s in 24 of 24 flights (median error
-0.14 s); with it on, in 0 of 24 (it predicted 2.8 s in the median). For the "off" setting the
model skips the stuck check, as PX4 does for a threshold of 0 or less; that is the only change
to it.

What this does not show. The typical height error did not move (median 0.078 m against
0.081 m), only the tail of the fusion delay is gone, and the hover height in the first 10 s
is the same (median 0.33 m against 0.34 m, p = 0.99). Flights above 0.15 m were 2 of 51
before and 0 of 24 now, and flights above 0.10 m were 6 of 51 and 1 of 24; with a failure
rate that low this is not enough to call the flake fixed, only to say that its cause is
found and removed. The line-survey test now also reads the PX4 log after the flight and fails
when the range finder joined the height estimate later than 3 s after lift-off
(`rng_fusion_max_s`; `framework/ulog_checks.py`). The limit sits between the 0.78 s maximum of
the 24 flights with the patch and the 12.2 s of the flights without it; the same rule applied to
those 51 flights fails 6 of them, so a return of the late fusion shows as a red test with the
cause named, not only as a height error. The patch is a property of the simulation image; whether a real range
finder arms the detector in the same way is not tested here.

## Deliberate-break checks

A test suite that has never been seen to fail proves little. `docs/breaks/` holds
patches that each break one thing the suite claims to guard; `scripts/check_breaks.sh`
applies every patch to a clean copy of the repository and requires the unit tests to go
red (and, first, green without the patch). The script runs in the `unit` job of every CI run.

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
| Manual dispatch | Every suite and every ROS scenario except the weekly-only ones; `runs=3` runs every suite three times; `weekly=true` runs the weekly plan instead (the ROS survey, three seeds, every scenario) |

Also: a new push to a pull request cancels the older run (`concurrency`,
`cancel-in-progress` for pull requests only; scheduled and manual runs are never
cancelled); the unit job uses the pip cache and `pytest-xdist -n 2` (two vCPUs); one
"image priming" job fills the PX4 image cache once per run (`lookup-only`: on a hit it
checks the entry exists and downloads nothing), so the matrix jobs do not each build it
on a miss; the report job runs whenever at least one test job ran, tolerates skipped
ones, and reads JUnit-only artifacts; scheduled times are off the hour, because GitHub
delays or drops runs at the top of the hour; the actions are at their Node 24 majors.
Scheduled workflows are disabled by GitHub after 60 days without repository activity.

## Allure report

The flight jobs (`sim`, `ros-survey`) run pytest with `--alluredir=allure-results` next to the JUnit
output and upload that folder as `allure-results-<suite>-<run>` (`allure-results-ros-survey-<run>`),
even after a failure. The `allure` job runs after them whenever one of them ran, downloads all
`allure-results-*` artifacts into one folder and builds one report with Allure 3 (`allure@3.22.1`
through `npx`; it needs Node 22.18 or newer, so the job sets up Node 24). The result is the
`allure-report` artifact: a single `index.html`, no Pages and no hosting; the job summary links to it.
A missing results artifact (a job that died early) only makes the report smaller.

- Identity. Allure treats results with the same test name and parameters as retries of one test, and
  the matrix runs the same tests in several cells. `framework/reporting.py` therefore adds the
  parameters `suite`, `run`, `scenario`, `px4_image` (from `config.yaml`) and, for the ROS line
  survey, `seed_offset`; `SIM_SUITE` and `SIM_RUN` come from the job's `env`. Recorded metrics are
  attachments, never parameters: a value that changes between runs would make every run a new test.
- Content. `test_line_survey` has one step per numbered stage, plus the replay check; the other flight
  tests are one entry with the parameters. Each flight test gets a `recorded metrics` attachment (the
  `record_property` values) and, when it fails with a recorder, the PX4 mode history and the last
  telemetry snapshot.
- Not in the report: ULog, rosbag, telemetry files. They are in `results-<suite>-<run>` under
  `artifacts/`; the description of each Allure result gives the run id and the test id to look for.
- Locally: `pytest -m sitl ... --alluredir=allure-results`, then `npx allure@3.22.1 generate
  allure-results --config allurerc.mjs --output allure-report`. Without `--alluredir` the Allure plugin
  writes nothing, so `pytest -m unit` is unchanged.

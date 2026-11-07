# Evidence index

Each row is a claim the repository makes, the test or rule behind it, a CI run that shows it, where
the raw data of that run is, and what the claim does **not** cover. Everything runs in simulation
(PX4 v1.18.0-rc1 SIH SITL on free 2-vCPU GitHub runners); nothing here says anything about hardware.
The reasoning behind the tests is in [test-strategy.md](test-strategy.md), the defects and flakes in
[qa-notes.md](qa-notes.md).

## Claims

| Claim | Test or rule | CI run | Raw data | Does not cover |
|---|---|---|---|---|
| Every suite passes three times in a row on `main` | Manual dispatch with `runs=3` (`.github/workflows/sim-tests.yml`) | [run 42396195239](https://github.com/Alexli18/drone-mission-sim-tests/actions/runs/42396195239) at `abfa411`: 18 of 18 jobs green, 13 tests x 3 = 39 flight test results passed, 0 retries | `junit-<suite>-<n>`, `results-<suite>-<n>`, `allure-report` | Three repetitions do not measure a rare failure rate |
| PX4 reacts to GPS loss, link loss, RC loss (also in a 5 m/s wind), a draining battery and a geofence breach as required | `test_failsafe[...]`; the pass rule is `framework/assertions.py`, covered by `tests/unit/test_assertions.py` | [run 42396195239](https://github.com/Alexli18/drone-mission-sim-tests/actions/runs/42396195239): 6 failsafe tests x 3 passed. Per-fault figures: [run 41719048626](https://github.com/Alexli18/drone-mission-sim-tests/actions/runs/41719048626) | `results-failsafes-<n>`: telemetry, ULog, container logs | Faults are injected (MAVSDK failure injection, dropped packets, neutral sticks); total loss only, one wind |
| A broken scenario turns the workflow red with a readable message | Throwaway branch with the link-loss failsafe disabled | [run 41720870716](https://github.com/Alexli18/drone-mission-sim-tests/actions/runs/41720870716): red, `mission finished first (failsafe never fired)`; the other four failsafe tests passed | The run log | One demonstrated break |
| Survey missions flown from YAML pass their waypoint and duration bounds | `test_survey_mission[...]`: lawnmower, corridor, crosswind | [run 42396195239](https://github.com/Alexli18/drone-mission-sim-tests/actions/runs/42396195239): 3 scenarios x 3 passed | `results-survey-<n>` | Waypoint tolerance is 3 m |
| The line survey works end to end: markers found, coverage, flight, clock offsets, replay equals flight, range finder fused early | `test_line_survey[...]` (four scenarios), rules in [line-survey.md](line-survey.md) | Weekly plan, three seeds, on `main` at `abfa411`: [run 42397488794](https://github.com/Alexli18/drone-mission-sim-tests/actions/runs/42397488794) 12 of 12 passed; [run 42396898449](https://github.com/Alexli18/drone-mission-sim-tests/actions/runs/42396898449) 11 of 12 (one coverage failure, below); the earlier weekly [run 42376540886](https://github.com/Alexli18/drone-mission-sim-tests/actions/runs/42376540886) 12 of 12 | `results-ros-survey-<n>`: rosbag, decisions, markers, ULog | A toy sensor; the PX4 estimate is on both sides of the chain (the estimator itself is checked against truth separately) |
| No sensor capture is lost, repeated or reordered | Rule 0 of the line survey; `capture_stream_failures` and its unit tests | Branch run [run 42374976630](https://github.com/Alexli18/drone-mission-sim-tests/actions/runs/42374976630): 17 of 17 jobs, three seeds | `recordings/` in `results-ros-survey-<n>` | No flight injects capture loss; the checker is tested with injected loss in unit tests only |
| The expected values do not come from the code they judge | [oracle-independence.md](oracle-independence.md); grid-integration reference for coverage; two line-projection implementations compared in `tests/unit/test_survey.py` | Unit job of any run | none (unit tests) | The PX4 estimate against the truth is a residual risk, written there |
| The suite goes red when something it guards is broken | `docs/breaks/*.patch`, `scripts/check_breaks.sh`: the unit tests go red for each patch (3, 6 and 4 tests) | **Run locally, not in CI** (last on 2026-11-07); the lever-arm break was also run through a flight, see [qa-notes.md](qa-notes.md#deliberate-break-checks) | Terminal output only | Not a CI gate; unit level for the three breaks |
| The late height after take-off has a cause, it is removed in the simulation image, and a rule watches it | Stuck-detector patch (`docker/px4-sitl/rng_stuck_detector.patch`); ULog rule `rng_fusion_max_s` = 3 s | Patched series [run 42387851452](https://github.com/Alexli18/drone-mission-sim-tests/actions/runs/42387851452), [run 42388459207](https://github.com/Alexli18/drone-mission-sim-tests/actions/runs/42388459207), [run 42388812279](https://github.com/Alexli18/drone-mission-sim-tests/actions/runs/42388812279), [run 42389139074](https://github.com/Alexli18/drone-mission-sim-tests/actions/runs/42389139074): 24 flights, fusion 0.70 to 0.78 s after lift-off (without the patch: median 1.84 s, maximum 12.2 s of 51); the rule in a flight: [run 42394245687](https://github.com/Alexli18/drone-mission-sim-tests/actions/runs/42394245687) and [run 42394901644](https://github.com/Alexli18/drone-mission-sim-tests/actions/runs/42394901644) (0.64 to 0.78 s, 7 flights) | `results-ros-survey-<n>`: the ULog of every flight | Simulated range finder only; 24 flights with the patch are too few to call the height flake fixed |
| One Allure report per run, matrix cells kept apart | `framework/reporting.py`, `allure` job | [run 42396195239](https://github.com/Alexli18/drone-mission-sim-tests/actions/runs/42396195239): artifact `allure-report`; an earlier pull-request run, [run 42379457810](https://github.com/Alexli18/drone-mission-sim-tests/actions/runs/42379457810): 12 results, 0 retries | `allure-report` (a single `index.html`) | No hosted page: the report is an artifact |
| Documentation-only changes cost nothing in CI, other changes run only what they touch | `scripts/ci_plan.sh`, 13 test functions in `tests/unit/test_ci_plan.py`; `paths-ignore` | Pull requests #8 and #9 (documentation only) started no checks | n/a | Not every GitHub event type is exercised by the planner tests |

## What is not stable

Measured on the runs above and written without softening.

*   **Line survey, weekly run [42396898449](https://github.com/Alexli18/drone-mission-sim-tests/actions/runs/42396898449), seed 3 job, `line_survey_clock_minus`: coverage
    0.8631 against 0.95. Fixed in the node, confirmed by a model, not by CI.** The georeferencing node
    stopped twice for about 0.4 s; the captures that arrived meanwhile were released in one burst and 32
    of them got `no_pose` although every pose had arrived. The stalls coincide with disk wait on the
    runner. After the fix (hold timeout also needs wall time): 4 weekly runs on the fix branch, 48
    flights, no `no_pose`, coverage 1.0 in all. How it was found:
    [qa-notes.md](qa-notes.md#a-stall-of-the-georeferencing-node-turns-into-no_pose).
*   **Test bench, two weekly runs on the fix branch ([42399318960](https://github.com/Alexli18/drone-mission-sim-tests/actions/runs/42399318960), [42400058966](https://github.com/Alexli18/drone-mission-sim-tests/actions/runs/42400058966)): 4 of 24 tests
    failed in rosbridge, not in the node.** `/rosapi/node_details` did not exist (1) or gave no reply
    within 30 s (2); 15 consecutive capture ids were missing from the test's recordings (1; rule 0 caught
    it, the node had released 645 of 645). The two "no reply" jobs have the crash of `rosapi` in the log
    (a `TypeError` on a node it does not see yet, an upstream bug in Humble; the test asked 0.4 s after
    the start). The test now waits until `rosapi` lists the nodes. Not explained: the "does not exist"
    case and the lost ids. The control on `main` (2 runs) and the other four runs on the branch were
    clean (72 of 72). [qa-notes.md](qa-notes.md#rosapi-dies-when-asked-about-a-node-it-does-not-see-yet).
*   The height rule, the marker margin and the `/rosapi/node_details` flake are in the README
    section "What is not stable" and in [qa-notes.md](qa-notes.md).

## Flight figures

Two figures, committed as `docs/img/*.png` and drawn from the `results-*` artifacts of the CI runs named
under them, so each can be redrawn and checked against the run:

| Figure | Test | Run | Draws |
|---|---|---|---|
| `failsafe_rc_loss_wind.png` | `test_failsafe[rc_loss_wind]` | [42396195239](https://github.com/Alexli18/drone-mission-sim-tests/actions/runs/42396195239) (`results-failsafes-1`) | The track seen from above and the height over time, shaded by flight mode (from the ULog: simulator truth and `vehicle_status.nav_state`) |
| `line_survey_markers.png` | `test_line_survey[line_survey_markers]` | [42405381157](https://github.com/Alexli18/drone-mission-sim-tests/actions/runs/42405381157) (`results-ros-survey-1`) | Captures along the line against the painted and the found markers (decisions and markers of the node), height over the line against its limit, and when the range finder joined the height estimate (ULog) |

```sh
pip install -e ".[plots]"
scripts/flight_figures.py failsafe <ulog> out.png --title "..." --source "..."
scripts/flight_figures.py survey <unpacked results-ros-survey-N> line_survey_markers out.png --title "..." --source "..."
```

The numbers on the figures are the test's own: height 0.092 m, marker errors -1.5, +4.9 and +2.0 cm,
fusion after 0.75 s, 246 usable captures, landing 0.6 m from home (the recorded metrics of the same runs).

How flight logs are looked at in practice: a ULog goes to a log viewer (PX4 Flight Review, run locally or
self-hosted for real flights) and a ROS bag to a tool such as Foxglove, with the layout kept next to the
code. This repository uses neither: nothing is uploaded to a public service, and no Foxglove layout is
provided, because one could not be checked here. The two figures are drawn with plain scripts so
that a reader sees a flight without installing anything.

## Raw data

Every flight job uploads `results-<suite>-<n>` (telemetry, ULog, rosbag for the line survey, container
logs, a `vmstat` log for the ROS jobs) and `junit-<suite>-<n>`; the `allure` job merges the results
into `allure-report`. GitHub keeps artifacts for 90 days unless the repository setting says
otherwise, and the workflow sets no retention, so the older runs below lose their raw data
after that time; the run pages and job summaries stay.

## Older runs, in full

The two tables below are the detailed chronology that used to be the top of the README.

### The M1 to M5 suites, run by run

| Check | Evidence |
|---|---|
| PX4 SITL boots headless in a pinned container and reaches "connected, GPS fix, home set" | `tests/test_smoke.py`, [run 41708569365](https://github.com/Alexli18/drone-mission-sim-tests/actions/runs/41708569365): boot to ready 2.2 s |
| Every baseline PX4 parameter is read back after boot with the value the test set | `assert_params` in [run 41708569365](https://github.com/Alexli18/drone-mission-sim-tests/actions/runs/41708569365) |
| The simulation runs at least half the requested ×10 speed | [run 41708569365](https://github.com/Alexli18/drone-mission-sim-tests/actions/runs/41708569365): effective speed factor 8.7 |
| The PX4 image comes from the Actions cache, not Docker Hub, in the test jobs | [run 41708569365](https://github.com/Alexli18/drone-mission-sim-tests/actions/runs/41708569365): `Loaded image: px4-sitl-pinned:39d5a950208a` (at the time; the local tag is now `px4-sitl-patched`, see [Testing practice](../README.md#testing-practice)) |
| Basic flight: arm, take off to 10 m, hover 10 simulated seconds within ±1 m, land, disarm, with no mode other than Ready/Takeoff/Hold/Land | `tests/test_basic_flight.py`, green on 5 consecutive manual runs with no retries: [1](https://github.com/Alexli18/drone-mission-sim-tests/actions/runs/41710667007), [2](https://github.com/Alexli18/drone-mission-sim-tests/actions/runs/41710677497), [3](https://github.com/Alexli18/drone-mission-sim-tests/actions/runs/41710688146), [4](https://github.com/Alexli18/drone-mission-sim-tests/actions/runs/41710698804), [5](https://github.com/Alexli18/drone-mission-sim-tests/actions/runs/41710708798); max hover altitude error 0.37–0.68 m |
| Survey mission from YAML: a 60 × 40 m lawnmower pattern (14 waypoints) is generated, uploaded and flown; every waypoint is passed within 3 m and the mission duration is inside its bounds | `tests/test_survey_mission.py` (parametrized over all `mission` scenarios), [run 41713216856](https://github.com/Alexli18/drone-mission-sim-tests/actions/runs/41713216856): largest waypoint miss 1.05 m, duration 138.0 simulated s (bounds 113–242 s) |
| GPS loss 60 simulated s into a 100 × 100 m survey (MAVSDK failure injection): the vehicle leaves Mission, lands and disarms | `test_failsafe[gps_loss]`, [run 41719048626](https://github.com/Alexli18/drone-mission-sim-tests/actions/runs/41719048626): Land 5.6 s after the fault |
| Data link loss at waypoint 3 (`iptables` drops test → PX4 traffic; telemetry keeps flowing): Return, landing near home, disarm | `test_failsafe[link_loss]`, [run 41719048626](https://github.com/Alexli18/drone-mission-sim-tests/actions/runs/41719048626): Hold 95.2 s after the fault, Return 5 s later, landed 0.1 m from home |
| RC loss at waypoint 3 (the MAVLink manual-control stream stops): Return, landing near home, disarm | `test_failsafe[rc_loss]`, [run 41719048626](https://github.com/Alexli18/drone-mission-sim-tests/actions/runs/41719048626): Hold after 5.6 s, Return after 10.6 s, landed 0.7 m from home |
| Low battery (fast simulated drain): Return at the critical level, then Land at the emergency level, disarm | `test_failsafe[low_battery]`, [run 41719048626](https://github.com/Alexli18/drone-mission-sim-tests/actions/runs/41719048626): Hold 120.2 s after mission start, Return at 125.2 s, Land at 142.2 s |
| Geofence breach (offboard push north out of an inclusion fence): Return, landing near home, disarm | `test_failsafe[geofence_breach]`, [run 41719048626](https://github.com/Alexli18/drone-mission-sim-tests/actions/runs/41719048626): Hold 14.4 s after Offboard started, Return 5 s later, landed 0.4 m from home |
| Every failsafe test also fails if a failsafe fires before the fault, if the mission finishes without one, or if the wrong mode comes first (for example Land instead of Return) | pass rule in `framework/assertions.py`, covered by `tests/unit/test_assertions.py` |
| The quality gate: breaking a scenario turns the workflow red with a clear message | throwaway branch with the link loss failsafe disabled (`NAV_DLL_ACT: 0`), [run 41720870716](https://github.com/Alexli18/drone-mission-sim-tests/actions/runs/41720870716): red, `expected: the failsafe fires during the mission \| actual: mission finished first (failsafe never fired)`; the other 4 failsafe tests passed |
| A `report` job writes a results table (result, wall time, recorded metrics) to the run's job summary | `scripts/summarize.py`, [run 41720851144](https://github.com/Alexli18/drone-mission-sim-tests/actions/runs/41720851144) |
| Every simulator job stays far under the 30-minute job timeout (`timeout-minutes: 30` in the workflow) | [run 41720851144](https://github.com/Alexli18/drone-mission-sim-tests/actions/runs/41720851144): whole workflow 2 min 56 s; longest job `failsafes` 2.2 min (5 flights), `survey` 0.8 min, `basic` 0.6 min, `unit` 0.3 min. |
| More scenarios as YAML only: a 20 × 100 m corridor survey at 8 m/s, the lawnmower survey under a 5 m/s crosswind (`SIH_WIND_E`), and RC loss under the same crosswind | [run 41819419053](https://github.com/Alexli18/drone-mission-sim-tests/actions/runs/41819419053), 3 runs each: corridor max waypoint miss 0.64–0.78 m, 86.8–88.8 simulated s (bounds 72–165); crosswind survey max miss 0.37–1.1 m, 148.6–151.4 s (about 12 s longer than without wind); RC loss under wind landed 0.1–0.9 m from home |
| Pass rate with every suite run 3 times: 11 of 11 tests passed 3/3, no retries | manual 3× run (same matrix as the nightly run), [run 41819419053](https://github.com/Alexli18/drone-mission-sim-tests/actions/runs/41819419053): 9 simulator jobs in parallel, whole workflow 3 min 3 s (an earlier 3× run with 8 tests: [run 41816091197](https://github.com/Alexli18/drone-mission-sim-tests/actions/runs/41816091197)). Three runs show the tests are not obviously flaky; they do not measure a rare failure rate |
| Config, scenario loading and validation, lawnmower geometry, coordinate conversion, failsafe pass rule, report generator | `tests/unit/` (40 tests when this table was written; the unit job of any CI run shows today's count, for example [run 42379859214](https://github.com/Alexli18/drone-mission-sim-tests/actions/runs/42379859214) with 239) |

The open questions that blocked M1 were answered by spike experiments in CI
([run 41707210020](https://github.com/Alexli18/drone-mission-sim-tests/actions/runs/41707210020));
the M4 fault mechanisms by spikes S5–S9
([run 41714506246](https://github.com/Alexli18/drone-mission-sim-tests/actions/runs/41714506246),
[run 41715170080](https://github.com/Alexli18/drone-mission-sim-tests/actions/runs/41715170080)).
The results are in the design's Open Questions section.

### The line survey, run by run

Measured on `main` at `09a091c` unless noted. The figures are what the runs show, including the failures.

| Check | Evidence |
|---|---|
| End to end, four scenarios (markers, sensor clock +20 ms, clock -20 ms, a clean line) with three noise seeds each | `tests/test_line_survey.py`, [run 42376540886](https://github.com/Alexli18/drone-mission-sim-tests/actions/runs/42376540886) (`gh workflow run sim-tests.yml -f weekly=true`): 12 of 12 passed, 71 to 75 s per job, 4 min 9 s for the workflow |
| Every suite three times | [run 42376597214](https://github.com/Alexli18/drone-mission-sim-tests/actions/runs/42376597214): the flight suites 33 of 33 passed; the line survey 5 of 6 (two scenarios, three seeds); the one failure is the height rule, see below |
| No capture lost, repeated or reordered between the sensor, the georeferencing node and the recordings | `capture_stream_failures` in `framework/survey.py`; unit tests with a missing, repeated, swapped and trailing id (a mutation that drops the missing-id search turns them red, checked locally); on CI (a branch run) [run 42374976630](https://github.com/Alexli18/drone-mission-sim-tests/actions/runs/42374976630): 17 of 17 jobs, three seeds |
| The expected values do not come from the code they judge | [docs/oracle-independence.md](oracle-independence.md): per rule, where the expected and measured values come from; coverage is checked against a grid-integration reference and the two line-projection implementations against each other (`tests/unit/test_survey.py`) |
| One Allure report per run, matrix cells kept apart | pull-request [run 42379457810](https://github.com/Alexli18/drone-mission-sim-tests/actions/runs/42379457810): artifact `allure-report`, 12 results, 0 retries (a test's identity includes suite, run, scenario, seed offset and the PX4 image) |

### Local runs before CI (not CI evidence)

Local evidence for the extension (Apple Silicon Mac, Docker Desktop, PX4 speed factor 4, effective 1.9; **not CI
evidence**): the four `line_survey_*` scenarios passed in 3 of 3 repeats with different noise seeds, each in 23 s
(47 s with the replay check); coverage 1.0 from both nodes, markers found 3 of 3 with errors up to 7 cm and no false
marker (none on the clean line), height error 0.06 to 0.08 m, cross-track 0.17 to 0.25 m, line speed 0.97 to 0.99 m/s,
clock shift +0.0195 m and -0.0195 m for +20 and -20 ms, sensor lag at most 0.07 s. With the lever-arm break applied the
flight test fails with the markers 12 cm off (`docs/breaks/`). The earlier suites still pass locally (10 passed, the
iptables-based `link_loss` skipped as documented).

# drone-mission-sim-tests: spec for Claude Code

## Context
I'm a QA Automation Tech Lead building a small public portfolio project: an automated test framework for drone missions that runs against PX4 SITL in GitHub Actions. The goal is to have a real, working demo of simulation-based validation: missions, failure injection, CI quality gates.

The project must run on free infrastructure only: GitHub Actions ubuntu runners on CPU, with no GPU and no display. Locally I'm on a Mac (Apple Silicon). If local PX4 SITL is impractical on the Mac, CI is the source of truth.

## Rules for you (the agent)
- Before writing build/run commands, **check the current PX4 docs** (docs.px4.io) for the exact SITL target names, the Docker image names and the failure-injection parameters. Names change between PX4 versions. Pin the PX4 version / image tag you use and write it in the README.
- Prefer the lightest simulator that works headless: **PX4 SIH** (simulation-in-hardware, no Gazebo). Use Gazebo only if SIH can't do something we need, and run it headless.
- Work in small, verifiable steps (milestones below). After each milestone, CI must be green before moving on.
- Don't invent results. The README only claims what CI actually shows.
- Keep it simple and readable: this is a portfolio project that will be read by a hiring manager.

## Stack
- PX4 Autopilot SITL (pinned version), SIH model, headless
- Python 3.11+, MAVSDK-Python, pytest, pytest-asyncio, PyYAML
- GitHub Actions (ubuntu-latest), Docker if it simplifies the PX4 setup
- Optional later: ROS 2 + uXRCE-DDS bridge

## Repository structure (target)
```
drone-mission-sim-tests/
  README.md                 # what it is, how to run, what is tested, CI badge
  pyproject.toml
  docker/                   # PX4 SITL container setup (if used)
  scripts/
    start_sitl.sh           # start PX4 SITL headless, wait until ready
  scenarios/                # missions as data
    hover_and_land.yaml
    survey_lawnmower.yaml
    geofence_breach.yaml
    gps_loss.yaml
    link_loss.yaml
    low_battery.yaml
  framework/
    sitl.py                 # connect, wait for health/home position, reset between tests
    mission.py              # load YAML -> MAVSDK mission items
    assertions.py           # position tolerance, flight mode, landed state, timeouts
    failures.py             # failure injection helpers
    telemetry_log.py        # record telemetry per test -> artifact
  tests/
    test_basic_flight.py
    test_survey_mission.py
    test_failsafes.py
  .github/workflows/sim-tests.yml
```

## Milestones

### M1: SITL boots in CI
- GitHub Actions job builds or pulls PX4 SITL, starts it headless (SIH) and waits until MAVSDK reports it is connected and ready (GPS fix / home position).
- A smoke test connects, reads telemetry, and passes.
- **Done when:** CI is green and the job time is reasonable (cache the PX4 build or the Docker image).

### M2: Basic flight test
- `test_basic_flight.py`: arm → takeoff to N m → hover T s → land → disarm.
- Assertions: altitude within tolerance, landed state, no failsafe triggered, timeout guard on every await.
- **Done when:** the test is green 5 runs in a row (no flakiness).

### M3: Scenario-driven survey mission
- `survey_lawnmower.yaml` describes a lawnmower (zig-zag) survey over a rectangle: area size, line spacing, altitude, speed.
- `mission.py` turns the YAML into mission items, and the test uploads and flies the mission.
- Assertions: every waypoint reached within X m, mission completes, total duration within bounds.
- Tests are parametrized over all scenario files of type `mission`.

### M4: Failure injection and failsafes
- Use PX4's built-in failure injection (check the docs for the parameter that enables it and the MAVSDK `failure` plugin API).
- Scenarios:
  - GPS loss mid-mission → expected failsafe mode (per the PX4 config)
  - Data link / RC loss → expected failsafe (RTL or land)
  - Low battery → expected failsafe
  - Geofence breach → RTL
- Each scenario YAML holds: trigger point (time or waypoint), failure type, **expected** behavior and a timeout.
- Assertions check the flight-mode transition and the final state.

### M5: Reports and quality gate
- JUnit XML report and a short Markdown summary per run (scenario, result, duration, key metrics) in the GitHub Actions job summary.
- Upload per-test telemetry logs (and PX4 ulog files if available) as artifacts.
- The workflow fails if any scenario fails. Add a README badge.
- Track flakiness: run the suite 3× in a nightly scheduled job and report the pass rate.

### Extension (optional): bridge to a ROS 2 sensor pipeline
- A ROS 2 node (via uXRCE-DDS) subscribes to vehicle position during a survey line and publishes timestamped "sensor captures" from a toy sensor that sees a few painted ground markers.
- A georeferencing node places the captures on the line using the vehicle pose, and a detector consumes them. The test checks that every marker is found near its true position and none elsewhere.
- Only start this after M1–M5 are stable. Implemented: see `docs/line-survey.md`.

## Engineering requirements
- Every async wait has a timeout. No bare `sleep` for synchronization: poll the state with a deadline.
- Tests are independent: reset or restart SITL between tests (or use a fixture that guarantees a clean state).
- Configuration (PX4 version, tolerances, timeouts) lives in one place.
- Clear failure messages: expected vs. actual mode/position plus the last telemetry snapshot.

## README must include
- One paragraph: what the project shows (simulation-based validation of drone missions, failure injection, CI gates).
- Architecture diagram (simple Mermaid is fine): GitHub Actions → PX4 SITL (SIH) ↔ MAVSDK ↔ pytest → reports/artifacts.
- How to run it in CI and locally.
- A table of scenarios and the expected behavior for each.
- Known limitations (SIH physics is simplified, there are no real sensors, etc.).
# PX4 SITL + SIH + MAVSDK (Python): verified notes (researched 2026-11-03)

How I checked these facts: I read the official docs as raw Markdown from the PX4 repo (`docs/en/...` on `release/1.17` and `main`), the PX4 source at tag **v1.17.0**, the MAVSDK C++ source on `main`, the PyPI JSON API, and the Docker Hub API. I also unpacked the `mavsdk` and `mavsdk-grpc` wheels and read their contents.

URL shorthand used below:
- `SRC17` = https://github.com/PX4/PX4-Autopilot/blob/v1.17.0/
- `SRCMAIN` = https://github.com/PX4/PX4-Autopilot/blob/main/

Anything marked **UNVERIFIED** could not be confirmed from a primary source.

> **The two big changes that affect the whole framework design**
> 1. **v1.17.0 versus main/v1.18 differ a lot for SIH.** In v1.17 there is no `px4_sitl_sih` board, and the SIH sensor simulators do not handle GPS/baro/mag failure injection. Both exist on main and in v1.18.0-rc1. See Q2 and Q5.
> 2. **The `mavsdk` package on PyPI changed in 4.x.** Since 4.0, `pip install mavsdk` installs a *native* ctypes binding with a different API: `Mavsdk(Configuration...)` and `Action(system)`, with no gRPC and no `mavsdk_server`. The old asyncio/gRPC `System().connect()` API is now published as `mavsdk-grpc` and imported as `mavsdk_grpc`. See Q10.

---

## Q1. PX4 releases

These come from the GitHub API: https://api.github.com/repos/PX4/PX4-Autopilot/releases and https://github.com/PX4/PX4-Autopilot/releases

- **The latest stable release is `v1.17.0`, published 2026-05-13** (GitHub marks it "Latest").
- The previous stable releases are `v1.16.2` (2026-04-22, a 1.16 maintenance release) and `v1.16.1` (2026-01-21). `v1.16.0` came out on 2025-08-06.
- Pre-releases of the next version: `v1.18.0-rc1` (2026-09-10), `v1.18.0-beta2` (2026-08-09), `v1.18.0-beta1` (2026-07-08) and `v1.18.0-alpha1` (2026-05-13). **No stable v1.18.0 had been released as of 2026-11-03.**

## Q2. SIH in SITL

### Make target
- **v1.17 docs:** `make px4_sitl sihsim_quadx`. Other targets are `sihsim_hex`, `sihsim_airplane`, `sihsim_xvert`, `sihsim_standard_vtol` and `sihsim_rover_ackermann`.
  Source: https://github.com/PX4/PX4-Autopilot/blob/release/1.17/docs/en/sim_sih/index.md ("SIH as SITL (no FC)")
- **main docs:** `make px4_sitl_sih sihsim_quadx`. The docs say: *"Use the `px4_sitl_sih` build target! The `px4_sitl` target will work, but will also build Gazebo libraries."*
  Source: https://docs.px4.io/main/en/sim_sih/
- `boards/px4/sitl/sih.px4board` **does not exist at v1.17.0** (HTTP 404). It does exist on main and in v1.18.0-beta1 and rc1. The file disables `CONFIG_MODULES_SIMULATION_GZ_BRIDGE` and the other GZ modules. On v1.17, use `px4_sitl_default` (the same as `px4_sitl`).
  Source: `SRCMAIN` boards/px4/sitl/sih.px4board
- On main, only the quadrotor is "Stable". The other vehicles are "Experimental": *"Only the quadrotor vehicle type is stable and recommended for development."* (main sim_sih docs)
- On v1.17, the gz_bridge module is built only if `find_package(gz-transport)` succeeds, so Gazebo is optional for `px4_sitl_default` (`SRC17` src/modules/simulation/gz_bridge/CMakeLists.txt). Reading the CMakeLists only shows that the find_package call is there. **That the build succeeds without Gazebo installed is UNVERIFIED in practice.**

### Running the binary directly (v1.17 source)
- The make target expands to `env PX4_SIM_MODEL=sihsim_quadx PX4_SIMULATOR=sihsim <build>/bin/px4`, run with working directory `<build>/rootfs` (`SITL_WORKING_DIR = ${PX4_BINARY_DIR}/rootfs`).
  Sources: `SRC17` src/modules/simulation/simulator_sih/CMakeLists.txt (`add_custom_target(sihsim_${model} COMMAND ${CMAKE_COMMAND} -E env PX4_SIM_MODEL=sihsim_${model} PX4_SIMULATOR=sihsim $<TARGET_FILE:px4> WORKING_DIRECTORY ${SITL_WORKING_DIR}`) and `SRC17` platforms/posix/cmake/sitl_target.cmake
- The binary is at `build/px4_sitl_default/bin/px4`, or `build/px4_sitl_sih/bin/px4` on main.
- Usage from `SRC17` platforms/posix/src/px4/common/main.cpp:
  `px4 [-h|-d] [-s <startup_file>] [-t <test_data_directory>] [<rootfs_directory>] [-i <instance>] [-w <working_directory>]`
  - `-d` is daemon mode ("don't start pxh shell").
  - `-i` is the instance id.
  - If you give no arguments, it defaults to `data_path=PX4_BINARY_DIR/etc` and `working_directory=PX4_BINARY_DIR/rootfs`.
- PX4's own runner (main) does this: `cwd=<build>/<tmp>/rootfs; nice --20 <build>/bin/px4 <build>/etc -s etc/init.d-posix/rcS -t <ws>/test_data -d` with env `PX4_SIM_MODEL=<model>` and `PX4_SIM_SPEED_FACTOR=<n>`. It clears and recreates the rootfs for each test.
  Source: `SRCMAIN` test/mavsdk_tests/integration_test_runner/process_helper.py (`class Px4Runner`)
- **Airframe autostart IDs** (v1.17 `ROMFS/px4fmu_common/init.d-posix/airframes/`):
  - 10040 `sihsim_quadx`
  - 10041 `sihsim_airplane`
  - 10042 `sihsim_xvert`
  - 10043 `sihsim_standard_vtol`
  - 10044 `sihsim_hex`
  - 10045 `sihsim_rover_ackermann`
- `PX4_SYS_AUTOSTART` is honoured. The rcS script checks, in order: `PX4_SIM_MODEL=shell`, then `PX4_SYS_AUTOSTART` (*"env SYS_AUTOSTART: ${PX4_SYS_AUTOSTART}"*), then `PX4_SIM_MODEL` (it finds the `[0-9]+_${PX4_SIM_MODEL}` airframe file). Either `PX4_SIM_MODEL=sihsim_quadx` or `PX4_SYS_AUTOSTART=10040` works.
  Source: `SRC17` ROMFS/px4fmu_common/init.d-posix/rcS lines 35-62
- The airframe sets `PX4_SIMULATOR=${PX4_SIMULATOR:=sihsim}`, and `px4-rc.simulator` starts SIH when `PX4_SIMULATOR=sihsim` (or when SYS_AUTOSTART is 0). It then starts `simulator_sih` plus `sensor_baro_sim`, `sensor_gps_sim` and `sensor_mag_sim`, because the airframe sets `SENS_EN_GPSSIM`, `SENS_EN_BAROSIM` and `SENS_EN_MAGSIM` to 1.
  Sources: `SRC17` ROMFS/.../airframes/10040_sihsim_quadx, px4-rc.simulator, px4-rc.sihsim
- **Overriding parameters through the environment:** rcS turns every `PX4_PARAM_<NAME>=<value>` env var into `param set NAME value`. This is in v1.17.0 (`SRC17` rcS lines 129-135). PX4 CI uses it, for example `"PX4_PARAM_EKF2_EN": 0` (`SRCMAIN` test/mavsdk_tests/configs/sih-sitl.json).
- `PX4_HOME_LAT`, `PX4_HOME_LON` and `PX4_HOME_ALT` set `SIH_LOC_LAT0`, `SIH_LOC_LON0` and `SIH_LOC_H0` (`SRC17` px4-rc.sihsim).
- **Parameters persist** in `parameters.bson` in the working directory (rootfs), and rcS imports that file on boot. For a clean state, delete it or use a fresh rootfs (`SRC17` rcS: `set PARAM_FILE parameters.bson`).

### Headless, lockstep and speed-up
- **Headless:** *"SIH is intentionally headless by default."* (main sim_sih docs). No GUI or X server is needed. On SIH it optionally streams `HIL_STATE_QUATERNION` to UDP 19410 for the jMAVSim display or Hawkeye.
- **Lockstep:** SITL builds use the lockstep scheduler by default (`SRC17` boards/px4/sitl/sitl.cmake: `set(ENABLE_LOCKSTEP_SCHEDULER yes)` unless building for replay). A `nolockstep` board variant exists.
  - SIH has a `lockstep_loop()` used when `ENABLE_LOCKSTEP_SCHEDULER` is defined, otherwise `realtime_loop()` (`SRC17` src/modules/simulation/simulator_sih/sih.cpp lines 84-160).
  - The main docs say: *"The simulation runs in lockstep with PX4, ensuring deterministic and reproducible results."*
- **Speed factor:** `PX4_SIM_SPEED_FACTOR` is supported. In sih.cpp, wall-time step = `sim_interval_us / speed_factor` (*"Simulation with %.1fx speedup"*). The sim rate comes from IMU_GYRO_RATEMAX / IMU_INTEG_RATE (default 400 Hz, clamped to 200-2000 Hz).
  - Docs example: `PX4_SIM_SPEED_FACTOR=10 make px4_sitl sihsim_airplane` (v1.17 docs).
  - In lockstep, if the CPU can't keep up, the simulation just runs slower than the requested factor.
  - When `PX4_SIM_SPEED_FACTOR` is set, rcS also scales `COM_DL_LOSS_T` (×10), `COM_RC_LOSS_T` (×1.0), `COM_OF_LOSS_T` and `COM_OBC_LOSS_T` (×5). **This uses `bc`** (`SRC17` rcS lines 199-216).
  - `bc` is installed only by the sim-tools part of `ubuntu.sh` (see Q4).
- PX4 CI runs its SIH MAVSDK tests at `--speed-factor 10` (`SRCMAIN` .github/workflows/sitl_tests.yml).

## Q3. MAVLink ports and multiple instances

Port setup in `SRC17` ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink:

| Link | PX4 local (listen) port | Remote port PX4 sends to | Notes |
|---|---|---|---|
| GCS link | `18570+i` | 14550 | Normal mode, 4 Mbit/s |
| API/Offboard link | `14580+i` | `14540+i` | `-m onboard`; for instance >9 the remote port is 14549 |
| Camera | `14280+i` | `14030+i` | |
| Gimbal | `13030+i` | `13280+i` | |
| SIH display (SIH only) | `19450+i` | `19410+i` | |

- The docs say the same: *"PX4's remote UDP Port 14540 is used for communication with offboard APIs ... allocated sequentially from 14540 to 14549 (additional instances all use port 14549)"* (https://github.com/PX4/PX4-Autopilot/blob/release/1.17/docs/en/simulation/index.md). The main docs have a port table in https://docs.px4.io/main/en/sim_sih/ ("Port Reference").
- MAVSDK connects with `udpin://0.0.0.0:14540`. This is the default in PX4's `conftest.py` and in `sih-sitl.json` (`"mavlink_connection": "udpin://0.0.0.0:14540"`).
- *"By default, MAVLink only listens on localhost. Set parameter `MAV_{i}_BROADCAST = 1` to enable network access."* (https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/simulation/px4_sitl_prebuilt_packages.md)
- **Multiple instances:** use `-i N`. The instance sets `MAV_SYS_ID=N+1` and `UXRCE_DDS_KEY=N+1` (`SRC17` rcS lines 138-141). Run each instance in its own working directory.
  - v1.17 script `Tools/simulation/sitl_multiple_run.sh` runs `cd build/px4_sitl_default/instance_$n; ../bin/px4 -i $n -d "$build_path/etc"`, but it hardcodes `PX4_SIM_MODEL=gazebo-classic_iris`.
  - The main docs give: `./Tools/simulation/sitl_multiple_run.sh 3 sihsim_quadx px4_sitl_sih`, or manually `./build/px4_sitl_sih/bin/px4 -i 1 -d ./build/px4_sitl_sih/etc`.
  - So on v1.17, set `PX4_SIM_MODEL=sihsim_quadx` yourself.

## Q4. Docker images and CI build

### Recommended build container (main docs)
Source: https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/test_and_ci/docker.md
- *"The `px4-dev` container is the recommended container for building PX4 firmware. ... based on Ubuntu 24.04"*, multi-arch (amd64 and arm64).
- It is published as `px4io/px4-dev` (Docker Hub) and `ghcr.io/px4/px4-dev`.
- *"The older per-distro containers from PX4/PX4-containers (e.g. `px4-dev-nuttx-jammy`, ...) are no longer recommended."*

### px4io/px4-dev tags (Docker Hub API)
- Releases: `v1.17.0` (2026-05-13), `v1.16.2`, `v1.16.1`, `v1.16.0` and `v1.15.4`.
- Pre-releases: `v1.18.0-rc1` (2026-09-10) and others.
- Every tag also has `-amd64` and `-arm64` variants.
- It is built from `Tools/setup/Dockerfile`: `FROM ubuntu:24.04` ... `RUN bash /tmp/ubuntu.sh --no-sim-tools` (`SRC17` Tools/setup/Dockerfile).
  - It includes the NuttX toolchain. Since `--no-sim-tools` skips `bc`, add `apt-get install bc` if you use `PX4_SIM_SPEED_FACTOR`.

### Legacy images (frozen)
`px4io/px4-dev-base-jammy`, `px4-dev-nuttx-jammy` and `px4-dev-simulation-jammy` last got tags `latest` and `2024-05-18` on 2024-06-12. Images also exist for focal and bionic.

### Prebuilt SITL images (no build needed)
Sources: https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/simulation/px4_sitl_prebuilt_packages.md and the Docker Hub API.
- `px4io/px4-sitl` is SIH-only, headless, about 100 MB, amd64 and arm64. Docker Hub describes it as: *"PX4 autopilot lightweight SITL simulation (SIH). One command, no dependencies. Headless, multi-arch."*
  - Tags: `latest` (built from main), `v1.18.0-rc1`, `-beta2`, `-beta1` and `-alpha1`.
  - **There is no `v1.17.0` tag** (the API returns 404).
- `px4io/px4-sitl-sih` is an older name. Its last tag is `latest`, from 2026-04-02.
- `px4io/px4-sitl-gazebo`, `px4-sitl-ros2` and `px4-sitl-gazebo-ros2` also exist.
- Run it with `docker run --rm -it --network host px4io/px4-sitl:latest` on Linux, choosing the vehicle with `-e PX4_SIM_MODEL=sihsim_quadx`.
  - In this image the default destinations are 14550 (QGC) and 14540 (MAVSDK).
  - For daemon mode (CI) the docs give `PX4_SIM_MODEL=sihsim_quadx px4 -d`.
  - The runtime directory is `$XDG_DATA_HOME/px4/rootfs/<instance>/fs/` (parameters, dataman, logs).
- The docs also describe `.deb` packages (`px4` for SIH, `px4-gazebo`) for Jammy and Noble. However, **the v1.17.0 and v1.18.0-rc1 GitHub release assets contain no `px4_*.deb`.** The only .deb is `voxl-px4_1.18.0.rc1_arm64.deb`. **Where these .debs are published is UNVERIFIED.**

### Building in CI
- PX4's own SITL CI uses the container `px4io/px4-dev:<tag>` and runs `make px4_sitl_default` (`SRCMAIN` .github/workflows/sitl_tests.yml).
- A local SIH deb build is `make px4_sitl_sih; cd build/px4_sitl_sih && cpack -G DEB` (main docs).

### Ubuntu 24.04 dependencies (`Tools/setup/ubuntu.sh`, `SRC17`)
- The script header says *"Ubuntu LTS (24.04, 22.04)"*.
- Flags: `--no-nuttx` skips the arm-none-eabi toolchain. `--no-sim-tools` skips `bc` and Gazebo Harmonic, plus gstreamer, opencv and so on.
- The general apt packages it installs: `astyle build-essential ccache cmake cppcheck file g++ gcc gdb git lcov libssl-dev libxml2-dev libxml2-utils make ninja-build python3 python3-dev python3-pip python3-setuptools python3-wheel rsync shellcheck unzip zip`.
- Then `pip install -r Tools/setup/requirements.txt` (adding `--break-system-packages` on Python ≥3.11). That file lists empy<4, jinja2, kconfiglib, pyyaml, pymavlink, pyulog, nunavut, lark, numpy, and others.
- **Recommendation for SIH-only CI:** run `bash Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools`, then `apt install bc`.

## Q5. Failure injection

### Enabling it
- `SYS_FAILURE_EN` defaults to 0 (`SRC17` src/lib/systemlib/system_params.c: `PARAM_DEFINE_INT32(SYS_FAILURE_EN, 0)`).
- **In SITL, rcS sets it to 1**: `param set-default SYS_FAILURE_EN 1` (`SRC17` rcS line ~186).
- If it is disabled, MAVLink `MAV_CMD_INJECT_FAILURE` is answered `DENIED` (`SRC17` mavlink_receiver.cpp line ~606). The MAVSDK 4 test expects `FailureResult.DISABLED` in this case (`SRCMAIN` test_multicopter_failure_injection.py).

### Shell syntax
`failure <component> <failure_type> [-i <instance_number>]`. Instance 0 (the default) means all instances.
Source: https://github.com/PX4/PX4-Autopilot/blob/release/1.17/docs/en/debug/failure_injection.md

- Components: `gyro accel mag baro gps optical_flow vio distance_sensor airspeed battery motor servo avoidance rc_signal mavlink_signal`
- Types: `ok off stuck garbage wrong slow delayed intermittent`
- These match `SRC17` src/systemcmds/failure/failure.cpp.
- Docs note: *"Many failure types are not broadly implemented. In those cases the command will return with an 'unsupported' message."*
- For a motor failure, `CA_FAILURE_MODE` must also be set.
- From the host you can use the client symlink: `build/.../bin/px4-failure gps off [--instance N]`. The usage text in main.cpp says: *"px4-MODULE [--instance <instance>] command using symlink"*.

### What is actually handled at v1.17.0 (grep of `VEHICLE_CMD_INJECT_FAILURE` handlers)
- **Motor** (`ok`, `off`, `stuck` = no ESC telemetry, `wrong`) is handled in commander's `FailureInjector.cpp`. It works with any simulator, so it works with SIH.
- **Battery** (`ok` and `off` with instance 0) is handled in `battery_simulator`, which runs in SITL when `SIM_BAT_ENABLE=1`. It works with SIH. `off` means *"Force battery empty"*.
- **GPS, accel, gyro, mag, baro, airspeed and VIO** are handled **only in `simulator_mavlink`**, which is used by Gazebo Classic and jMAVSim. **They are not handled in SIH, `sensor_gps_sim` or `sensor_baro_sim` at v1.17.0.** I found no other handler.
  - My inference: with SIH on v1.17, `failure gps off` gets no ACK, so MAVSDK probably returns TIMEOUT. **The runtime behaviour is UNVERIFIED.**
- **On main and on v1.18.0-rc1**, SIH has failure injection: `SensorGpsSim::publishWithFailures` (gnss), `SensorBaroSim` and `SensorMagSim` (`off` and `stuck`), and `sih.cpp` (airspeed and distance sensor). The source is `SRCMAIN` src/modules/simulation/sensor_*_sim and simulator_sih/sih.cpp.
  - PX4 main CI runs `test_land_on_gps_lost_during_mission` on SIH quadx.
- **`rc_signal` and `mavlink_signal`:** I found no handler in v1.17.0 src. The `rc_signal` example in the docs is therefore **UNVERIFIED for SITL**.
- **GPS-loss workaround for v1.17 SIH:** set `SIM_GPS_USED` (default 10) below 4. `sensor_gps_sim` then publishes `fix_type = 0 // No fix` (`SRC17` src/modules/simulation/sensor_gps_sim/SensorGpsSim.cpp lines 132-150). PX4 CI uses `PX4_SIM_GPS_USED=0` in a test env on main.

### MAVSDK API
- **mavsdk ≥4 (native)** (from the wheel's `mavsdk/plugins/failure/failure.py`):
  - Import with `from mavsdk.plugins.failure import Failure, FailureUnit, FailureType, FailureResult, FailureError`.
  - Call `Failure(system).inject(unit, type, instance)`. It blocks and raises `FailureError` (`e.result`).
  - The async variant is `mavsdk.asyncio...FailureAsync.inject`.
  - `FailureUnit` values: SENSOR_GYRO=0, SENSOR_ACCEL, SENSOR_MAG, SENSOR_BARO, SENSOR_GPS=4, SENSOR_OPTICAL_FLOW, SENSOR_VIO, SENSOR_DISTANCE_SENSOR, SENSOR_AIRSPEED=8, SYSTEM_BATTERY=100, SYSTEM_MOTOR, SYSTEM_SERVO, SYSTEM_AVOIDANCE, SYSTEM_RC_SIGNAL, SYSTEM_MAVLINK_SIGNAL=105.
  - `FailureType` values: OK, OFF, STUCK, GARBAGE, WRONG, SLOW, DELAYED, INTERMITTENT.
  - `FailureResult` values: UNKNOWN, SUCCESS, NO_SYSTEM, CONNECTION_ERROR, UNSUPPORTED, DENIED, DISABLED, TIMEOUT.
- **mavsdk-grpc / mavsdk<4:** `await drone.failure.inject(FailureUnit.SENSOR_GPS, FailureType.OFF, 0)`. The enum names are the same; they are generated from the same proto. **The exact grpc enum module path was not re-checked (UNVERIFIED).**

## Q6. Data-link loss and RC loss

Parameter definitions are from `SRC17` src/modules/commander/commander_params.c. Docs: https://github.com/PX4/PX4-Autopilot/blob/release/1.17/docs/en/config/safety.md

### Data link (GCS) loss
- `NAV_DLL_ACT` (default 0): 0 Disabled, 1 Hold, 2 Return, 3 Land, 5 Terminate, 6 Disarm.
- `COM_DL_LOSS_T`: default 10 s, range 5-300.
- `COM_DLL_EXCEPT`: bits are 0 Mission, 1 Hold, 2 Offboard.
- `COM_FAIL_ACT_T`: default 5 s. The vehicle sits in Hold for this long before the failsafe action runs.

### How PX4 detects GCS loss (v1.17 source)
- GCS loss is declared when **no HEARTBEAT with `type == MAV_TYPE_GCS`** has been seen for `COM_DL_LOSS_T`, counting all links (`Commander.cpp` lines ~2780 and ~2847, `telemetry.heartbeat_type_gcs`).
- It is only armed after the first GCS heartbeat: `_datalink_last_heartbeat_gcs != 0`.
- MAVSDK sends `MAV_TYPE_GCS` heartbeats when its component type is GroundStation. That is the default for compid 190; with compid 196 it is CompanionComputer, which sends `MAV_TYPE_ONBOARD_CONTROLLER`.
  Source: https://github.com/mavlink/MAVSDK/blob/main/cpp/src/mavsdk/core/mavsdk.cpp `component_type_for_component_id`, and `mavsdk_impl.cpp` `mav_type_for_component_type`
- **Simulating DL loss while keeping telemetry (my design inference, not a documented recipe; UNVERIFIED in practice):**
  1. Connect the test's MAVSDK as a companion computer: `Configuration.create_with_component_type(ComponentType.COMPANION_COMPUTER)`, so its heartbeats don't count as GCS.
  2. Send GCS heartbeats from a separate GCS-type MAVSDK instance or a pymavlink socket. That could be a second link, such as `udpin://:14550`.
  3. Stop that second source and wait for `COM_DL_LOSS_T` to expire.
  - Also possible: `mavlink stop -u 18570` kills the GCS link instance, but that only matters if the GCS heartbeat comes in on that link.
- `failure mavlink_signal off` has no handler at v1.17.0 (see Q5).
- Arming with `NAV_DLL_ACT>0` and no GCS fails: *"Preflight Fail: No connection to the GCS"* (`SRC17` HealthAndArmingChecks/checks/rcAndDataLinkCheck.cpp).

### RC / manual-control loss
- `NAV_RCL_ACT` (default 2): 1 Hold, 2 Return, 3 Land, 5 Terminate, 6 Disarm.
- `COM_RC_LOSS_T`: default 0.5 s.
- `COM_RCL_EXCEPT`: bits are 0 Mission, 1 Hold, 2 Offboard, 3 External Mode, 4 Altitude Cruise.
- `COM_RC_IN_MODE` values: 0 RC only, 1 MAVLink only, 2 RC or MAVLink with fallback, 3 RC or MAVLink keep first, 4 Disable manual control, 5-8 priority variants. **SITL sets the default to 1 (MAVLink only)** (`SRC17` rcS: *"Don't require RC calibration and configuration"*).
- **Meaningful in SITL?** Only if you feed MAVLink manual control.
  - Failsafe logic: `_manual_control_lost_at_arming = status_flags.manual_control_signal_lost`. When this is true the RC-loss failsafe is ignored: *"If manual control was lost and arming was allowed, consider it optional until we regain manual control"* (`SRC17` src/modules/commander/failsafe/failsafe.cpp lines 459-496, 645-653).
  - So with no joystick nothing happens. To test it: stream `ManualControl.set_manual_control_input(x, y, z, r)` continuously (PX4 tests use 50 Hz), arm, then stop streaming. After `COM_RC_LOSS_T` the `NAV_RCL_ACT` action runs.
  - That last step is derived from the source logic. **The end-to-end behaviour is UNVERIFIED.**

## Q7. GPS / position loss failsafe

Docs: https://github.com/PX4/PX4-Autopilot/blob/release/1.17/docs/en/config/safety.md ("Position (GNSS) Loss Failsafe").

### Triggers
- A timeout since the last fused velocity or position measurement, set by `EKF2_NOAID_TOUT` (default 5,000,000 µs).
- Estimated horizontal error above `COM_POS_FS_EPH` (default 5 m, -1 disables; only for MC and VTOL in hover).

### Action
- The docs say: *"Multicopters will switch to Altitude mode if a height estimate is available, otherwise Stabilized mode."*
- In the source, for an **auto mode without position** (Mission/RTL): if the intended mode can't run, the fallback is RTL, then Land, then Descend (`SRC17` failsafe.cpp `checkModeFallback`, and framework.cpp `case Action::RTL/Land/Descend` fallthrough). In other words, it does a blind descend (`NAVIGATION_STATE_DESCEND`) and then disarms.
- The PX4 test confirms this: *"With no position aiding left, a blind descend is performed. MAVSDK has no Descend flight mode, so we just wait for the disarm after landing."* (`SRCMAIN` test/mavsdk_tests/test_multicopter_failsafe.py)
- Fixed-wing uses `FW_GPSF_LT` (loiter time) and `FW_GPSF_R` (roll).
- `COM_ARM_WO_GPS` (default 1): 0 Deny arming, 1 Warning only, 2 Disabled.
- **`COM_POSCTL_NAVL` and `NAV_GPSF_*` do not exist in v1.17.0**; a grep of src finds no match.

## Q8. Battery simulation and low-battery failsafe

### Simulated battery (`SRC17` src/modules/simulation/battery_simulator/battery_simulator_params.c and BatterySimulator.cpp)
- `SIM_BAT_ENABLE` (default 1). rcS runs `battery_simulator start` when it is 1.
- `SIM_BAT_DRAIN` (default 60 s): the time to drain from 100% to 0% while armed.
- `SIM_BAT_MIN_PCT` (default 50%): the floor. The docs say it *"can be used to alter the battery level ... on the fly. Particularly useful for testing different low-battery behaviour."*
- The battery resets to 100% when disarmed.
- `BAT1_N_CELLS` is set to 4 in SITL.

### Thresholds (`SRC17` src/lib/battery/module.yaml; units are a fraction, "norm")
- `BAT_LOW_THR`: default 0.15
- `BAT_CRIT_THR`: default 0.07
- `BAT_EMERGEN_THR`: default 0.05

### Action
- `COM_LOW_BAT_ACT` (default 0): 0 Warning, 2 Land, 3 *"Return at critical level, land at emergency level"*.
- **SITL sets the default to 3**: *"Enable low-battery actions by default for (automated) testing. Battery sim does not go below 50% by default, but failure injection can trigger failsafes."* (`SRC17` rcS)

### Can MAVSDK inject a battery failure?
Yes, with `FailureUnit.SYSTEM_BATTERY` and `FailureType.OFF` at instance 0, which empties the battery; `OK` restores it. This is handled by battery_simulator at v1.17.0, so it works with SIH.

Alternatively, set `SIM_BAT_MIN_PCT` low (for example 5) and `SIM_BAT_DRAIN` short.

## Q9. Geofence

### Parameters (`SRC17` src/modules/navigator/geofence_params.c)
- `GF_ACTION` (default 2): 0 None, 1 Warning, 2 Hold, 3 Return, 4 Terminate, 5 Land.
- `GF_MAX_HOR_DIST` and `GF_MAX_VER_DIST` default to 0, which means disabled. Units are metres, range 0-10000, measured from Home.
- Docs (safety.md): *"Setting GF_ACTION to terminate will kill the vehicle on violation of the fence"*. Termination is also gated by `CBRK_FLIGHTTERM`. Other parameters: `GF_SOURCE`, `GF_PREDICT`.

### MAVSDK ≥4 geofence plugin (from the wheel, `mavsdk/plugins/geofence/geofence.py`)
```python
from mavsdk.plugins.geofence import Geofence, Point, Polygon, Circle, GeofenceData, FenceType
fence = Polygon(points=[Point(lat, lon), ...], fence_type=FenceType.INCLUSION)  # or EXCLUSION
Geofence(system).upload_geofence(GeofenceData(polygons=[fence], circles=[]))
# also: download_geofence(), clear_geofence()
```
`GeofenceResult` values: SUCCESS, ERROR, TOO_MANY_GEOFENCE_ITEMS, BUSY, TIMEOUT, INVALID_ARGUMENT, NO_SYSTEM.

### Works in SITL?
Yes. PX4 main CI has `test_multicopter_rtl_with_geofence.py`, which loads a mission with geofence polygons and sets `GF_ACTION` 1, and it runs in the SIH "multicopter" test filter.

A test that checks a *breach failsafe* specifically (Hold/RTL on breach) was not found. My expectation is that it works, because geofence runs in navigator on the estimated global position. **UNVERIFIED by a PX4 test.**

## Q10. MAVSDK Python: package, versions and API

### PyPI (https://pypi.org/pypi/mavsdk/json, checked 2026-11-03)
- **`mavsdk` 4.1.0 (2026-10-27)** is the latest. It needs Python `>=3.8`; the classifiers list 3.8 to 3.14.
- Wheels are `cp38-abi3` for manylinux_2_28 x86_64 and aarch64, macOS 15 arm64 and x86_64, and Windows.
- The 4.x wheel bundles `mavsdk/lib/libcmavsdk.so`. **There is no mavsdk_server**: the migration guide says *"no gRPC and no server process"* (https://mavsdk.mavlink.io/main/en/python/migration.html).
- manylinux_2_28 is fine on ubuntu-latest (glibc 2.39).
- 4.x releases: 4.0.0 on 2026-09-22, 4.0.1-4.0.6 in late September to early October, then 4.1.0.
- The last 3.x release is 3.17.4 (2026-09-17). Its wheels are `py3-none-manylinux1_x86_64` and similar, and it uses the old gRPC API with a bundled `mavsdk_server`.
- **`mavsdk-grpc` 4.0.5** (2026-10-07) is the old MAVSDK-Python under its new name. The wheel contains `mavsdk_grpc/bin/mavsdk_server` (about 50 MB, which I confirmed by unpacking the manylinux1_x86_64 wheel). It requires Python `>=3.7`.
  - README (https://github.com/mavlink/MAVSDK-Python): *"Install `mavsdk-grpc` and use `import mavsdk_grpc as mavsdk` — the code is identical."* *"Or pin `mavsdk<4`."*
- PX4 main pins `mavsdk~=4.1.0` and `pytest>=8,<10` (`SRCMAIN` Tools/setup/test-requirements.txt).

### Native API (mavsdk ≥4), as used by PX4's own pytest suite (`SRCMAIN` test/mavsdk_tests/autopilot_tester.py)
```python
from mavsdk import Mavsdk, Configuration, ComponentType, ConnectionResult
from mavsdk.plugins.action import Action, ActionError
from mavsdk.plugins.telemetry import Telemetry, FlightMode, LandedState
from mavsdk.plugins.mission import Mission
from mavsdk.plugins.param import Param
mav = Mavsdk(Configuration.create_with_component_type(ComponentType.GROUND_STATION))
assert mav.add_any_connection("udpin://0.0.0.0:14540") == ConnectionResult.SUCCESS
system = mav.first_autopilot(timeout_s=25.0)   # None on timeout
tel = Telemetry(system)
tel.flight_mode()        # FlightMode.{UNKNOWN,READY,TAKEOFF,HOLD,MISSION,RETURN_TO_LAUNCH,LAND,OFFBOARD,FOLLOW_ME,MANUAL,ALTCTL,POSCTL,ACRO,STABILIZED,RATTITUDE}
tel.health_all_ok()
tel.health()             # .is_gyrometer_calibration_ok .is_accelerometer_calibration_ok .is_magnetometer_calibration_ok
                         # .is_local_position_ok .is_global_position_ok .is_home_position_ok .is_armable
tel.home(); tel.in_air(); tel.armed(); tel.battery(); tel.position()
tel.landed_state()       # LandedState.{UNKNOWN,ON_GROUND,IN_AIR,TAKING_OFF,LANDING}
Mission(system).mission_progress().current   # .total
Param(system).set_param_int("NAV_DLL_ACT", 2); Param(system).set_param_float("SIM_BAT_MIN_PCT", 5.0)
```
- Everything above is a blocking getter. There are also `subscribe_*(callback)` methods, `set_rate_*` and `*_async` variants.
- An asyncio API lives in `mavsdk.asyncio` (`TelemetryAsync`, `FailureAsync`, and so on).
- Exceptions now expose `e.result` (the old API used `e._result.result`).
- PX4's tester notes: *"health_all_ok() can be true before the global position is considered valid"*. It therefore also waits for `health().is_armable`.
- `Mavsdk.remove_connection(handle)` exists, together with `add_any_connection_with_handle`.
- **Old API (mavsdk<4 / mavsdk_grpc):** `drone = System(); await drone.connect(system_address="udpin://0.0.0.0:14540")`, then `drone.telemetry.flight_mode()` (an async generator), `drone.param.set_param_int(...)` and `drone.failure.inject(...)`. `System(mavsdk_server_address=None, port=50051, sysid=245, compid=190)` auto-starts the bundled mavsdk_server (https://github.com/mavlink/MAVSDK-Python/blob/main/mavsdk/system.py).

## Q11. ULog location and SDLOG_MODE

- **v1.17:** SITL has `CONFIG_BOARD_ROOT_PATH="."` (`SRC17` boards/px4/sitl/default.px4board) and the logger root is `CONFIG_BOARD_ROOT_PATH "/log"` (`SRC17` src/modules/logger/logger.h). Logs therefore go to **`<working dir>/log/<YYYY-MM-DD>/<HH_MM_SS>.ulg`**, which with make is `build/px4_sitl_default/rootfs/log/...`. The v1.17 CI uploads `build/px4_sitl_default/tmp_mavsdk_tests/rootfs/log/**/*.ulg`.
- **main:** `CONFIG_BOARD_ROOT_PATH="./fs"`, so logs go to `rootfs/fs/log/...`. The main CI uploads `.../rootfs/fs/log/**/*.ulg`. The prebuilt packages put them under `$XDG_DATA_HOME/px4/rootfs/<instance>/fs/`.
- `SDLOG_MODE` (`SRC17` src/modules/logger/module.yaml) defaults to 0. Values:
  - 0: when armed until disarm
  - 1: from boot until disarm
  - 2: from boot until shutdown
  - 3: while manual input AUX1 >30%
  - 4: from 1st armed until shutdown
- **SITL sets the default to 1** (*"By default log from boot until first disarm"*), with `SDLOG_PROFILE 131` and `SDLOG_DIRS_MAX 7` (`SRC17` rcS).
- `SDLOG_BACKEND` defaults to 3 (SD card plus MAVLink).

## Q12. Resetting state between tests

- **A MAVLink reboot is NOT supported in SITL.**
  - Commander only handles `PREFLIGHT_REBOOT_SHUTDOWN` param1=1 (reboot) under `#if defined(CONFIG_BOARDCTL_RESET)`. Otherwise the command falls through to `VEHICLE_CMD_RESULT_DENIED` (`SRC17` src/modules/commander/Commander.cpp lines 1244-1285).
  - `CONFIG_BOARDCTL_RESET` is not set for SITL; a grep of boards/px4/sitl and platforms/posix finds no match.
  - So `action.reboot()` should come back denied. That is my inference from the source. **The exact MAVSDK result is UNVERIFIED.**
- **Shutdown (param1=2) is supported.** SITL defines `BOARD_HAS_POWER_CONTROL`, and `board_power_off()` prints *"Exiting NOW."* and calls `system_exit(0)`. So `action.shutdown()` (when disarmed) makes the px4 process exit.
- **Recommended reset:** kill and restart the px4 process with a fresh rootfs. That removes `parameters.bson`, dataman and logs. PX4's runner does exactly this per test case: `clear_rootfs()` / `create_rootfs()`, plus *"Each test case is started by mavsdk_test_runner.py against a fresh PX4 instance"* (`SRCMAIN` test/mavsdk_tests/pytest.ini).
- **Boot-to-ready time for SIH: UNVERIFIED.** No official figure was found.
  - PX4 tests allow 25 s for discovery, 30 s for `health_all_ok` and 45 s for `is_armable` (autopilot_tester.py). These are upper bounds, not typical times.
  - rcS sets `EKF2_REQ_GPS_H 0.5` with the comment *"Speedup SITL startup"*.

## Q13. PX4's own CI using SIH

### main branch: `.github/workflows/sitl_tests.yml`
Source: https://github.com/PX4/PX4-Autopilot/blob/main/.github/workflows/sitl_tests.yml
- Runs in container `px4io/px4-dev:v1.16.0-rc1-258-g0369abd556`, on RunsOn AWS runners. The file says you can uncomment `ubuntu-latest`.
- Matrix: quadx (AddressSanitizer), hex (ThreadSanitizer), standard_vtol (Coverage/RelWithDebInfo) and xvert (UBSan).
- Steps:
  1. `make px4_sitl_default`
  2. `pip3 install -r Tools/setup/test-requirements.txt` (with `PIP_BREAK_SYSTEM_PACKAGES=1`)
  3. `test/mavsdk_tests/mavsdk_test_runner.py --speed-factor 10 --abort-early --model <model> test/mavsdk_tests/configs/sih-sitl.json --verbose --force-color` (45 min timeout)
  4. Upload `.ulg` and `.log` files on failure.
- `configs/sih-sitl.json` sets `"model_prefix": "sihsim_"` and `"mavlink_connection": "udpin://0.0.0.0:14540"`. Its test filters, with optional per-test env such as `PX4_PARAM_*`:
  - `vision_target`
  - `multicopter or offboard or offboard_attitude` (quadx)
  - `sih_home_yaw`
  - `motor_failure_injection` (hex)
  - `vtol`
  - `sih_xvert_transition`
- The tests are now **pytest + mavsdk 4 (native)**: `test/mavsdk_tests/conftest.py`, `pytest.ini`, `autopilot_tester*.py` and `test_*.py`.
  - The `tester` fixture connects to `--url` (default `udpin://0.0.0.0:14540`).
  - This is the closest official template for the planned framework.
- Other main workflows that reference SIH: `build_deb_package.yml` (builds the deb packages and the px4-sitl containers) and `ros_integration_tests.yml`.

### v1.17.0: `.github/workflows/sitl_tests.yml`
- Still **Gazebo Classic**, not SIH: container `px4io/px4-dev-simulation-focal:2021-09-08`, `make px4_sitl_default sitl_gazebo-classic mavsdk_tests`.
- Uses MAVSDK C++ `libmavsdk-dev` 3.7.0 (`test/mavsdk_tests/MAVSDK_VERSION`), with C++/Catch2 tests and `configs/sitl.json`.
- There is no SIH config at v1.17.0 (`SRC17` .github/workflows/sitl_tests.yml, test/mavsdk_tests/configs/).

---

## UNVERIFIED items
1. That the v1.17 `px4_sitl_default` build succeeds with no Gazebo installed. The CMake `find_package` is optional, but I did not test the build.
2. Runtime result when injecting GPS, baro or mag failures on v1.17 SIH. No handler exists in the source, and I expect a MAVSDK TIMEOUT.
3. Whether `failure rc_signal` and `failure mavlink_signal` do anything in SITL (no handler found in v1.17 src).
4. The DL-loss simulation recipe: a companion-type MAVSDK plus a separate GCS heartbeat source that is then stopped. It is built from source logic, not a documented procedure.
5. End-to-end RC-loss triggering by streaming and then stopping MAVSDK `ManualControl` input.
6. Exact MAVSDK result codes for `action.reboot()` in SITL (PX4 answers DENIED).
7. Typical SIH boot-to-ready time.
8. A geofence *breach failsafe* test (Hold/RTL on breach) in SITL. Only geofence-aware RTL is tested by PX4.
9. Where the documented `px4_*.deb` SITL packages are published (they are not in the v1.17.0 or v1.18.0-rc1 release assets).
10. Enum import paths in `mavsdk_grpc` (the old API). Names are assumed identical to the native binding.

# Known limitations

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
*   PX4 rejects a mission, a fence upload during a mission, or a goto target
    if it crosses the geofence ("Geofence violation for waypoint 3", spike
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
    release contains the upstream fix. The stuck-detector patch changes the simulated autopilot: the
    simulated range finder reports about 0 m on the ground, and the detector no longer keeps it out of the
    height estimate; this is not verified for a real range finder. Building the image compiles PX4 and is
    slow on a cache miss.
*   This is a simulation suite. Nothing here is hardware-in-the-loop or a flight with a real vehicle.

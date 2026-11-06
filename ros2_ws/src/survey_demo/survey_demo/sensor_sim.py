"""The simulated survey sensor (pure; no ROS): captures from PX4 poses in true time.

Driven by the arriving PX4 poses: ``advance_to(newest_us, history)`` emits every
capture whose last sample is at or before the newest pose time, using the pose
interpolated at each sample's true time; it never extrapolates. Once the newest pose
is past a sample's true time its bracketing poses are final, so the captures depend
only on the poses and the true times, not on how the callbacks are batched (a unit
test checks that).

The sensor reads the PX4 *estimate* of its own position, like the georeferencing node:
the pipeline test checks the chain from pose to marker position (timing, interpolation,
lever arm, projection), while how well PX4 estimates the position is measured against
the simulator's ground truth in ``spikes/line_hold_truth.py``.

Each sample's stamp is the true time plus the clock offset plus Gaussian jitter, so a
clock offset moves the georeferenced position by v·offset.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from sensor_bridge.line import sensor_position

from survey_demo.markers import reading


@dataclass(frozen=True)
class SensorParams:
    capture_rate_hz: float
    samples_per_capture: int
    sample_spacing_s: float
    footprint_sigma_m: float
    noise: float


@dataclass
class CaptureData:
    capture_id: int
    sample_stamps_us: list[int]
    readings: list[float]
    saturated: bool
    stamp_us: int                   # sensor clock of the first sample


@dataclass
class TruthData:
    capture_id: int
    sample_true_us: list[int]
    sample_stamp_us: list[int]
    sample_s: list[float]           # NaN without bracketing poses
    sample_d: list[float]


class SensorSim:
    def __init__(self, params: SensorParams, markers, line, offset, clock_offset_ms: float,
                 clock_jitter_ms: float, rng_noise, rng_jitter, start_us: int,
                 sensor_id: str = "survey0"):
        self.params = params
        self.markers = list(markers)
        self.line = line
        self.offset = offset
        self.clock_offset_us = clock_offset_ms * 1000.0
        self.clock_jitter_us = clock_jitter_ms * 1000.0
        self.rng_noise = rng_noise
        self.rng_jitter = rng_jitter
        self.sensor_id = sensor_id
        self.start_us = int(start_us)
        self.period_us = 1e6 / params.capture_rate_hz
        self.spacing_us = params.sample_spacing_s * 1e6
        self.next_id = 0
        self.stopped = False
        self.without_pose = 0       # samples emitted without bracketing poses

    def _true_time_us(self, capture_id: int, sample: int) -> int:
        return round(self.start_us + capture_id * self.period_us + sample * self.spacing_us)

    @property
    def last_capture_id(self) -> int | None:
        return self.next_id - 1 if self.next_id else None

    def stop(self) -> int | None:
        """No further captures; returns the id of the last one emitted (None: none)."""
        self.stopped = True
        return self.last_capture_id

    def advance_to(self, newest_us: int, history):
        """``(CaptureData, TruthData)`` of every capture due by ``newest_us``."""
        out = []
        n = self.params.samples_per_capture
        while not self.stopped and self._true_time_us(self.next_id, n - 1) <= newest_us:
            out.append(self._capture(self.next_id, history))
            self.next_id += 1
        return out

    def _capture(self, capture_id: int, history):
        n = self.params.samples_per_capture
        true_us, stamps, readings, ss, ds = [], [], [], [], []
        for j in range(n):
            t = self._true_time_us(capture_id, j)
            jitter = self.rng_jitter.normal(0.0, self.clock_jitter_us) \
                if self.clock_jitter_us else 0.0
            true_us.append(t)
            stamps.append(max(round(t + self.clock_offset_us + jitter), 0))
            pose = history.interpolate(t)
            if pose is None:
                self.without_pose += 1
                s = d = math.nan
                value = 0.0
            else:
                north, east, _ = sensor_position(pose, self.offset)
                s, d = self.line.project(north, east)
                value = reading(s, d, self.markers, self.params.footprint_sigma_m)
            noisy = value + self.rng_noise.normal(0.0, self.params.noise)
            readings.append(float(np.clip(noisy, 0.0, 1.0)))
            ss.append(s)
            ds.append(d)
        capture = CaptureData(capture_id=capture_id, sample_stamps_us=stamps, readings=readings,
                              saturated=any(r >= 1.0 for r in readings), stamp_us=stamps[0])
        truth = TruthData(capture_id=capture_id, sample_true_us=true_us, sample_stamp_us=stamps,
                          sample_s=ss, sample_d=ds)
        return capture, truth

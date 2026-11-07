"""The flight-log check: when the range finder joins the height estimate."""

import struct

import pytest

pytest.importorskip("pyulog")

from framework.ulog_checks import (
    find_ulog,
    load_fusion_inputs,
    rng_fusion_check,
    rng_fusion_delay_s,
)

pytestmark = pytest.mark.unit


def test_delay_counts_from_the_first_sample_above_the_ground():
    delay = rng_fusion_delay_s([0, 1, 2, 3], [0.0, 0.0, 0.2, 0.4],
                               [0, 1, 2, 3, 4], [False, False, False, True, True])
    assert delay == pytest.approx(1.0)


def test_fusion_on_the_ground_does_not_count():
    # Fused while still on the ground, lost, and fused again 1.5 s after lift-off.
    delay = rng_fusion_delay_s([0, 1, 2, 3], [0.0, 0.0, 0.2, 0.4],
                               [0, 1, 2, 3, 3.5], [True, True, False, False, True])
    assert delay == pytest.approx(1.5)


def test_fused_at_lift_off_is_a_zero_delay():
    assert rng_fusion_delay_s([0, 1], [0.0, 0.3], [0.5, 1.0], [True, True]) == pytest.approx(0.0)


@pytest.mark.parametrize("height, fused", [([0.0, 0.0, 0.0], [True, True]),
                                           ([0.0, 0.2, 0.4], [False, False])])
def test_no_lift_off_or_no_fusion_gives_none(height, fused):
    assert rng_fusion_delay_s([0, 1, 2], height, [1, 2], fused) is None


# A hand-built log: the format messages, the two topics and their samples. Only what
# pyulog needs to read these topics.
def _message(kind: bytes, payload: bytes) -> bytes:
    return struct.pack("<HB", len(payload), kind[0]) + payload


def _write_ulog(path, truth, aid, with_aid=True):
    """``truth``: (t_us, z) samples; ``aid``: (t_us, fused) samples."""
    out = b"ULog\x01\x12\x35\x01" + struct.pack("<Q", 0)
    out += _message(b"B", bytes(8) + bytes(8) + bytes(24))
    # The definitions come first: the format messages, then the subscriptions.
    out += _message(b"F", b"vehicle_local_position_groundtruth:uint64_t timestamp;float z;")
    if with_aid:
        out += _message(b"F", b"estimator_aid_src_rng_hgt:uint64_t timestamp;bool fused;")
    out += _message(b"A", struct.pack("<BH", 0, 0) + b"vehicle_local_position_groundtruth")
    if with_aid:
        out += _message(b"A", struct.pack("<BH", 0, 1) + b"estimator_aid_src_rng_hgt")
    for t_us, z in truth:
        out += _message(b"D", struct.pack("<HQf", 0, t_us, z))
    if with_aid:
        for t_us, fused in aid:
            out += _message(b"D", struct.pack("<HQ?", 1, t_us, fused))
    path.write_bytes(out)
    return path


def _flight_dir(tmp_path, **kwargs):
    log_dir = tmp_path / "px4" / "rootfs" / "log" / "2026-01-01"
    log_dir.mkdir(parents=True)
    truth = [(1_000_000, 0.0), (2_000_000, -0.2), (3_000_000, -0.4)]
    aid = [(1_500_000, False), (2_500_000, False), (3_500_000, True)]
    _write_ulog(log_dir / "00_00_00.ulg", truth, aid, **kwargs)
    return tmp_path


def test_a_log_is_read_and_the_delay_found(tmp_path):
    data_dir = _flight_dir(tmp_path)
    assert rng_fusion_delay_s(*load_fusion_inputs(find_ulog(data_dir))) == pytest.approx(1.5)


def test_check_passes_within_the_limit_and_fails_over_it(tmp_path):
    data_dir = _flight_dir(tmp_path)
    delay, failure = rng_fusion_check(data_dir, 2.0)
    assert delay == pytest.approx(1.5) and failure is None
    delay, failure = rng_fusion_check(data_dir, 1.0)
    assert delay == pytest.approx(1.5)
    assert "within 1.0 s of lift-off" in failure[0] and "1.50 s after lift-off" in failure[1]


def test_a_missing_log_is_a_failure_not_a_skip(tmp_path):
    delay, failure = rng_fusion_check(tmp_path, 3.0)
    assert delay is None and "could not be read" in failure[1]


def test_a_log_without_the_estimator_topic_is_a_failure(tmp_path):
    delay, failure = rng_fusion_check(_flight_dir(tmp_path, with_aid=False), 3.0)
    assert delay is None and "estimator_aid_src_rng_hgt" in failure[1]


def test_a_file_that_is_not_a_log_is_a_failure(tmp_path):
    log_dir = tmp_path / "px4" / "rootfs" / "log" / "d"
    log_dir.mkdir(parents=True)
    (log_dir / "x.ulg").write_bytes(b"not a log")
    delay, failure = rng_fusion_check(tmp_path, 3.0)
    assert delay is None and "could not be read" in failure[1]


def test_the_largest_log_is_the_flight(tmp_path):
    data_dir = _flight_dir(tmp_path)
    small = data_dir / "px4" / "rootfs" / "log" / "2026-01-02"
    small.mkdir()
    (small / "short.ulg").write_bytes(b"ULog")
    assert find_ulog(data_dir).name == "00_00_00.ulg"

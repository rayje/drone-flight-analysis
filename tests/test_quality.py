"""Regression tests for conservative quality-aware trajectory metrics."""

import pytest

from drone_flight_analysis.models import PositionSample, Topic
from drone_flight_analysis.quality import analyze_quality


def position_samples(*timestamps_us):
    return [
        PositionSample(timestamp_us, index, 0.0, -index)
        for index, timestamp_us in enumerate(timestamps_us)
    ]


def test_continuous_trajectory_preserves_raw_distance():
    samples = position_samples(0, 100_000, 200_000)

    result = analyze_quality(samples, [])

    assert result.quality_distance_m == pytest.approx(result.raw_distance_m)
    assert result.excluded_edges == ()
    assert result.recorded_duration_s == pytest.approx(0.2)
    assert result.armed_duration_s is None
    assert result.airborne_duration_s is None


def test_gap_is_excluded_without_changing_raw_distance():
    samples = position_samples(0, 100_000, 200_000, 600_000)

    result = analyze_quality(samples, [])

    assert result.excluded_edges == ((200_000, 600_000, "gap"),)
    assert result.quality_distance_m < result.raw_distance_m
    assert result.excluded_distance_m == pytest.approx(
        result.raw_distance_m - result.quality_distance_m
    )


def test_simultaneous_reset_counters_count_as_one_event():
    samples = position_samples(0, 100_000, 200_000)
    position_topic = Topic(
        "vehicle_local_position",
        0,
        {
            "timestamp": [0, 100_000, 200_000],
            "xy_reset_counter": [4, 4, 5],
            "z_reset_counter": [4, 4, 5],
            "ref_timestamp": [123, 123, 123],
        },
    )

    result = analyze_quality(samples, [position_topic])

    assert result.reset_count == 1
    assert result.excluded_edges == (
        (100_000, 200_000, "reset_or_reference_change"),
    )


def test_reference_change_marks_discontinuous_edge():
    samples = position_samples(0, 100_000, 200_000)
    position_topic = Topic(
        "vehicle_local_position",
        0,
        {
            "timestamp": [0, 100_000, 200_000],
            "ref_timestamp": [123, 123, 124],
        },
    )

    result = analyze_quality(samples, [position_topic])

    assert result.reset_count == 1
    assert result.quality_distance_m < result.raw_distance_m


def test_armed_and_airborne_durations_are_separate():
    samples = position_samples(0, 100_000, 200_000, 300_000, 400_000)
    armed_topic = Topic(
        "actuator_armed",
        0,
        {
            "timestamp": [0, 100_000, 400_000],
            "armed": [False, True, False],
        },
    )
    landed_topic = Topic(
        "vehicle_land_detected",
        0,
        {
            "timestamp": [0, 200_000, 300_000],
            "landed": [True, False, True],
        },
    )

    result = analyze_quality(samples, [armed_topic, landed_topic])

    assert result.armed_duration_s == pytest.approx(0.3)
    assert result.airborne_duration_s == pytest.approx(0.1)
    assert result.max_height_above_takeoff_m == pytest.approx(2.0)


def test_single_sample_without_state():
    samples = position_samples(100_000)

    result = analyze_quality(samples, [])

    assert result.raw_distance_m == 0
    assert result.quality_distance_m == 0
    assert result.recorded_duration_s == 0
    assert result.max_height_above_takeoff_m is None


@pytest.mark.parametrize(
    "gap_factor, minimum_gap_s",
    [(1.0, 0.15), (float("nan"), 0.15), (1.5, 0.0)],
)
def test_invalid_gap_parameters(gap_factor, minimum_gap_s):
    samples = position_samples(0, 100_000)

    with pytest.raises(ValueError):
        analyze_quality(
            samples,
            [],
            gap_factor=gap_factor,
            minimum_gap_s=minimum_gap_s,
        )


def test_nonmonotonic_samples_rejected():
    samples = position_samples(100_000, 0)

    with pytest.raises(ValueError):
        analyze_quality(samples, [])


def test_missing_state_does_not_invent_airborne_duration():
    samples = position_samples(0, 100_000, 200_000)

    result = analyze_quality(samples, [])

    assert result.armed_duration_s is None
    assert result.airborne_duration_s is None


def test_gap_and_reset_on_same_edge_are_both_reported():
    samples = position_samples(0, 100_000, 500_000)
    position_topic = Topic(
        "vehicle_local_position",
        0,
        {
            "timestamp": [0, 100_000, 500_000],
            "xy_reset_counter": [0, 0, 1],
        },
    )

    result = analyze_quality(samples, [position_topic])

    assert result.excluded_edges == (
        (100_000, 500_000, "gap+reset_or_reference_change"),
    )

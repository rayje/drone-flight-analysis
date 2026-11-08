"""Regression tests for conservative quality-aware metrics."""
import pytest

from drone_flight_analysis.models import PositionSample, Topic
from drone_flight_analysis.quality import analyze_quality


def points(*times):
    return [PositionSample(t, i, 0.0, -i) for i,t in enumerate(times)]


def test_no_discontinuities_preserves_raw_distance():
    result=analyze_quality(points(0,100000,200000),[])
    assert result.quality_distance_m == pytest.approx(result.raw_distance_m)
    assert result.excluded_edges == ()
    assert result.recorded_duration_s == .2
    assert result.armed_duration_s is None
    assert result.airborne_duration_s is None


def test_gap_is_excluded_but_raw_preserved():
    result=analyze_quality(points(0,100000,200000,600000),[])
    assert result.excluded_edges == ((200000,600000,"gap"),)
    assert result.quality_distance_m < result.raw_distance_m
    assert result.excluded_distance_m == pytest.approx(result.raw_distance_m-result.quality_distance_m)


def test_reset_crossing_edge_excluded():
    position=Topic("vehicle_local_position",0,dict(
        timestamp=[0,100000,200000],xy_reset_counter=[4,4,5],
        z_reset_counter=[4,4,5],ref_timestamp=[123,123,123]))
    result=analyze_quality(points(0,100000,200000),[position])
    assert result.reset_count == 1
    assert result.excluded_edges == ((100000,200000,"reset_or_reference_change"),)


def test_reference_change_excluded():
    position=Topic("vehicle_local_position",0,dict(
        timestamp=[0,100000,200000],ref_timestamp=[123,123,124]))
    result=analyze_quality(points(0,100000,200000),[position])
    assert result.reset_count == 1


def test_armed_and_airborne_duration_are_distinct():
    p=points(0,100000,200000,300000,400000)
    armed=Topic("actuator_armed",0,dict(timestamp=[0,100000,400000],armed=[False,True,False]))
    landed=Topic("vehicle_land_detected",0,dict(timestamp=[0,200000,300000],landed=[True,False,True]))
    result=analyze_quality(p,[armed,landed])
    assert result.armed_duration_s == pytest.approx(.3)
    assert result.airborne_duration_s == pytest.approx(.1)
    assert result.max_height_above_takeoff_m == pytest.approx(2)


def test_single_point_and_missing_state():
    result=analyze_quality(points(100000),[])
    assert result.raw_distance_m == 0
    assert result.quality_distance_m == 0
    assert result.recorded_duration_s == 0
    assert result.max_height_above_takeoff_m is None


def test_bad_threshold_and_nonmonotonic_samples():
    with pytest.raises(ValueError):
        analyze_quality(points(0,100000),[],gap_factor=1)
    with pytest.raises(ValueError):
        analyze_quality(points(100000,0),[])

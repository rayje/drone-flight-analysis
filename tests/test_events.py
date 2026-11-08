"""Tests for read-only single-aircraft event reconstruction."""
import pytest

from drone_flight_analysis.events import reconstruct_events
from drone_flight_analysis.models import Topic


def topic(name, fields, instance=0):
    return Topic(name, instance, fields)


def test_transition_is_bracketed_not_assumed_instantaneous():
    events = reconstruct_events([topic("vehicle_status", {
        "timestamp": [100, 200, 300], "nav_state": [3, 3, 18]
    })])
    assert len(events) == 1
    event = events[0]
    assert (event.earliest_us, event.timestamp_us) == (200, 300)
    assert (event.previous_value, event.value) == (3, 18)
    assert event.as_dict()["latest_us"] == 300


def test_events_sorted_across_independent_topics():
    events = reconstruct_events([
        topic("vehicle_command", {"timestamp": [250], "command": [20]}),
        topic("actuator_armed", {"timestamp": [100, 200], "armed": [False, True]}),
        topic("vehicle_status", {"timestamp": [100, 300], "nav_state": [3, 18]}),
    ])
    assert [e.timestamp_us for e in events] == [200, 250, 300]
    assert [e.event_type for e in events] == [
        "arming_changed", "command_observed", "navigation_state_changed"
    ]


def test_missing_topics_do_not_invent_events():
    assert reconstruct_events([]) == ()
    assert reconstruct_events([topic("unknown", {"timestamp": [1]})]) == ()


def test_position_instance_selection():
    events = reconstruct_events([
        topic("vehicle_local_position", {
            "timestamp": [1, 2], "xy_reset_counter": [0, 1]
        }, instance=0),
        topic("vehicle_local_position", {
            "timestamp": [1, 2], "xy_reset_counter": [0, 2]
        }, instance=1),
    ], position_instance=1)
    assert len(events) == 1
    assert events[0].value == 2
    assert events[0].source_instance == 1


def test_command_ack_preserves_result_and_does_not_claim_acceptance():
    events = reconstruct_events([topic("vehicle_command_ack", {
        "timestamp": [100], "command": [20], "result": [1]
    })])
    assert events[0].event_type == "command_ack_observed"
    assert events[0].previous_value == 1


def test_unsorted_source_preserves_sample_index():
    events = reconstruct_events([topic("vehicle_status", {
        "timestamp": [300, 100, 200], "nav_state": [2, 1, 1]
    })])
    assert len(events) == 1
    assert events[0].sample_index == 0
    assert events[0].earliest_us == 200


def test_duplicate_timestamps_rejected():
    with pytest.raises(ValueError, match="Duplicate timestamps"):
        reconstruct_events([topic("vehicle_status", {
            "timestamp": [100, 100], "nav_state": [1, 2]
        })])


def test_invalid_lengths_rejected():
    with pytest.raises(ValueError, match="length mismatch"):
        reconstruct_events([topic("actuator_armed", {
            "timestamp": [100, 200], "armed": [True]
        })])


def test_failsafe_boolean_transitions():
    events = reconstruct_events([topic("failsafe_flags", {
        "timestamp": [100, 200, 300],
        "local_position_invalid": [False, True, False],
        "counter": [0, 1, 2],
    })])
    assert len(events) == 2
    assert all(e.event_type == "failsafe_flag_changed" for e in events)


def test_does_not_mutate_input():
    fields = {"timestamp": [300, 100], "nav_state": [18, 3]}
    reconstruct_events([topic("vehicle_status", fields)])
    assert fields["timestamp"] == [300, 100]

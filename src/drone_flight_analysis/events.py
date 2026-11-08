"""Read-only, single-aircraft PX4 event extraction and boot-clock timeline.

Events are observations, not diagnoses. State transitions are bracketed by the
previous and current samples; command records are observations of logged
messages, not proof of command acceptance or execution.
"""
from dataclasses import dataclass
from typing import Iterable

from .models import Topic


@dataclass(frozen=True)
class FlightEvent:
    """An observed event in PX4 boot microseconds, with source provenance."""
    event_type: str
    timestamp_us: int
    earliest_us: int
    source_topic: str
    source_instance: int
    source_field: str
    sample_index: int
    previous_value: object = None
    value: object = None

    def as_dict(self) -> dict:
        """Return a JSON-compatible, explicitly timed event."""
        return {
            "type": self.event_type,
            "timestamp_us": self.timestamp_us,
            "earliest_us": self.earliest_us,
            "latest_us": self.timestamp_us,
            "source_topic": self.source_topic,
            "source_instance": self.source_instance,
            "source_field": self.source_field,
            "sample_index": self.sample_index,
            "previous_value": self.previous_value,
            "value": self.value,
        }


# Each entry is an observed transition, not an interpretation of its cause.
TRANSITIONS = {
    "actuator_armed": {"armed": "arming_changed"},
    "vehicle_land_detected": {"landed": "landed_state_changed"},
    "vehicle_status": {
        "nav_state": "navigation_state_changed",
        "arming_state": "arming_state_changed",
        "failsafe": "failsafe_state_changed",
    },
    "failsafe_flags": {},  # fields differ by firmware; opt-in below
    "vehicle_local_position": {
        "xy_reset_counter": "position_xy_reset",
        "z_reset_counter": "position_z_reset",
        "ref_timestamp": "position_reference_changed",
    },
}

COMMAND_TOPICS = {
    "vehicle_command": ("command", "command_observed"),
    "vehicle_command_ack": ("command", "command_ack_observed"),
}


def _series(topic: Topic, field: str) -> tuple[tuple[int, int, object], ...]:
    """Validate and sort one field by (boot timestamp, original sample index)."""
    timestamps = topic.fields.get("timestamp")
    values = topic.fields.get(field)
    if timestamps is None or values is None:
        return ()
    if len(timestamps) != len(values):
        raise ValueError(f"{topic.name}[{topic.instance}].{field} length mismatch")
    rows = []
    for index, (timestamp, value) in enumerate(zip(timestamps, values)):
        if isinstance(timestamp, bool) or not isinstance(timestamp, int) or timestamp < 0:
            raise ValueError(f"Invalid timestamp in {topic.name}.{field}")
        # Numpy integer types from pyulog support __index__.
        rows.append((int(timestamp), index, value.item() if hasattr(value, "item") else value))
    rows.sort(key=lambda row: (row[0], row[1]))
    if any(a[0] == b[0] for a, b in zip(rows, rows[1:])):
        raise ValueError(f"Duplicate timestamps in {topic.name}.{field}")
    return tuple(rows)


def reconstruct_events(
    topics: Iterable[Topic],
    *,
    position_instance: int = 0,
    include_failsafe_flags: bool = True,
) -> tuple[FlightEvent, ...]:
    """Extract source-traceable transitions and commands from one aircraft log.

    The event timestamp is the first sample showing the new value. For a
    transition, earliest_us is the previous observation; the actual transition
    is only known to lie in (earliest_us, timestamp_us]. No synthetic initial
    state is emitted. Unknown topics and missing fields are ignored.
    """
    events = []
    seen = set()
    for topic in topics:
        key = (topic.name, topic.instance)
        if key in seen:
            raise ValueError(f"Duplicate topic instance: {key}")
        seen.add(key)
        if topic.name == "vehicle_local_position" and topic.instance != position_instance:
            continue
        if topic.instance != 0 and topic.name != "vehicle_local_position":
            continue
        fields = dict(TRANSITIONS.get(topic.name, {}))
        if topic.name == "failsafe_flags" and include_failsafe_flags:
            # Only boolean-valued flags; exclude timestamps and numeric counters.
            for field, values in topic.fields.items():
                if field != "timestamp" and values and all(
                    isinstance(v, bool) or type(v).__name__ in ("bool_",)
                    for v in values
                ):
                    fields[field] = "failsafe_flag_changed"
        for field, kind in fields.items():
            rows = _series(topic, field)
            for previous, current in zip(rows, rows[1:]):
                if previous[2] != current[2]:
                    events.append(FlightEvent(
                        kind, current[0], previous[0], topic.name, topic.instance,
                        field, current[1], previous[2], current[2],
                    ))
        if topic.name in COMMAND_TOPICS:
            field, kind = COMMAND_TOPICS[topic.name]
            for timestamp, index, value in _series(topic, field):
                events.append(FlightEvent(
                    kind, timestamp, timestamp, topic.name, topic.instance,
                    field, index, None, value,
                ))
            # A command acknowledgment is observed, not necessarily accepted.
            if topic.name == "vehicle_command_ack" and "result" in topic.fields:
                for event_index in range(len(events)):
                    event = events[event_index]
                    if event.source_topic == topic.name and event.source_field == "command":
                        result = topic.fields["result"]
                        if len(result) != len(topic.fields["command"]):
                            raise ValueError("vehicle_command_ack.result length mismatch")
                        events[event_index] = FlightEvent(
                            event.event_type, event.timestamp_us, event.earliest_us,
                            event.source_topic, event.source_instance, event.source_field,
                            event.sample_index, result[event.sample_index], event.value,
                        )
    return tuple(sorted(events, key=lambda e: (
        e.timestamp_us, e.source_topic, e.source_instance, e.sample_index, e.source_field
    )))

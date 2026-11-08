"""Production analysis of local-position telemetry quality for flight investigations.

This module is part of the analysis pipeline, NOT the integration-test harness.
It supplements trajectory reconstruction by identifying position-sampling gaps
and estimator reference/reset discontinuities that could distort sampled path
length. Future incident reports can consume QualityMetrics to flag unreliable
trajectory segments alongside synchronized event timelines. It does not infer
root cause or correct the aircraft trajectory, and the current CLI does not
include these metrics yet.

All timestamps use PX4 boot microseconds. Positions are local NED metres.
Quality-filtered distance is a diagnostic, not reconstructed ground truth.
"""

from bisect import bisect_right
from dataclasses import dataclass
from math import dist, isfinite
from statistics import median
from typing import Iterable

from .models import PositionSample, Topic


@dataclass(frozen=True)
class QualityMetrics:
    recorded_duration_s: float
    armed_duration_s: float | None
    airborne_duration_s: float | None
    raw_distance_m: float
    quality_distance_m: float
    excluded_distance_m: float
    excluded_edges: tuple[tuple[int, int, str], ...]
    gap_threshold_s: float
    max_gap_s: float
    reset_count: int
    min_height_above_origin_m: float
    max_height_above_origin_m: float
    max_height_above_takeoff_m: float | None


def _find_topic(
    topics: tuple[Topic, ...],
    name: str,
    instance: int = 0,
) -> Topic | None:
    """Find a single named topic instance, rejecting ambiguous duplicate datasets."""
    matches = [
        topic
        for topic in topics
        if topic.name == name and topic.instance == instance
    ]

    if len(matches) > 1:
        raise ValueError(f"Ambiguous topic: {name} instance {instance}")

    return matches[0] if matches else None


def _state_events(
    topic: Topic | None,
    field: str,
) -> tuple[tuple[int, bool], ...] | None:
    """Read timestamped boolean state transitions; return None when unavailable."""
    if topic is None or field not in topic.fields:
        return None

    timestamps = topic.fields.get("timestamp")
    values = topic.fields[field]

    if timestamps is None or len(timestamps) != len(values):
        raise ValueError(f"Invalid {topic.name}.{field} series")

    events = sorted(
        (int(timestamp), bool(value))
        for timestamp, value in zip(timestamps, values)
    )

    if any(timestamp < 0 for timestamp, _ in events):
        raise ValueError("Negative state timestamp")

    return tuple(events)


def _active_duration(
    events: tuple[tuple[int, bool], ...] | None,
    start_us: int,
    end_us: int,
) -> float | None:
    """Sum intervals in the active state within the observed position window."""
    if not events:
        return None

    duration_us = 0
    previous_us = max(start_us, events[0][0])
    active = False

    for timestamp_us, value in events:
        if timestamp_us < start_us:
            active = value
            previous_us = start_us
            continue

        if timestamp_us > end_us:
            break

        if active and timestamp_us > previous_us:
            duration_us += timestamp_us - previous_us

        previous_us = timestamp_us
        active = value

    if active and end_us > previous_us:
        duration_us += end_us - previous_us

    return duration_us / 1_000_000


def _discontinuity_times(position_topic: Topic | None) -> tuple[int, ...]:
    """Identify timestamps where reset counters or local reference timestamps change."""
    if position_topic is None:
        return ()

    fields = position_topic.fields
    timestamps = fields.get("timestamp", ())
    changes = set()

    for field_name in (
        "xy_reset_counter",
        "z_reset_counter",
        "ref_timestamp",
    ):
        values = fields.get(field_name)

        if values is None:
            continue

        if len(values) != len(timestamps):
            raise ValueError(f"Mismatched {field_name} lengths")

        for index in range(1, len(timestamps)):
            if values[index] != values[index - 1]:
                changes.add(int(timestamps[index]))

    return tuple(sorted(changes))


def _takeoff_relative_height(
    points: tuple[PositionSample, ...],
    landed_events: tuple[tuple[int, bool], ...] | None,
) -> float | None:
    """Estimate maximum height relative to the position immediately before takeoff."""
    if not landed_events:
        return None

    start_us = points[0].timestamp_us
    end_us = points[-1].timestamp_us

    for previous, current in zip(landed_events, landed_events[1:]):
        takeoff_us, is_landed = current

        if not previous[1] or is_landed:
            continue

        if not start_us <= takeoff_us <= end_us:
            continue

        ground_samples = [
            sample for sample in points
            if sample.timestamp_us <= takeoff_us
        ]
        airborne_samples = [
            sample for sample in points
            if sample.timestamp_us >= takeoff_us
        ]

        if not ground_samples or not airborne_samples:
            return None

        ground_height_m = -ground_samples[-1].down_m

        return max(
            -sample.down_m - ground_height_m
            for sample in airborne_samples
        )

    return None


def analyze_quality(
    samples: Iterable[PositionSample],
    topics: Iterable[Topic],
    *,
    gap_factor: float = 1.5,
    minimum_gap_s: float = 0.15,
    position_instance: int = 0,
) -> QualityMetrics:
    """Calculate diagnostic trajectory quality metrics for incident reconstruction.

    Exclude reset/gap-crossing edges only from quality-filtered distance.

    The gap threshold is the greater of minimum_gap_s and gap_factor times
    the median position interval. Neither parameter is a validated Firefly
    aircraft limit. Original samples and raw distance remain unchanged.
    """
    points = tuple(samples)
    datasets = tuple(topics)

    if not points:
        raise ValueError("No position samples")

    if (
        not isfinite(gap_factor)
        or gap_factor <= 1
        or not isfinite(minimum_gap_s)
        or minimum_gap_s <= 0
    ):
        raise ValueError("Invalid gap threshold")

    for point in points:
        coordinates = (point.north_m, point.east_m, point.down_m)

        if point.timestamp_us < 0 or not all(map(isfinite, coordinates)):
            raise ValueError("Invalid position sample")

    intervals_s = []

    for previous, current in zip(points, points[1:]):
        delta_us = current.timestamp_us - previous.timestamp_us

        if delta_us <= 0:
            raise ValueError("Position timestamps must increase")

        intervals_s.append(delta_us / 1_000_000)

    gap_threshold_s = minimum_gap_s

    if intervals_s:
        gap_threshold_s = max(
            minimum_gap_s,
            gap_factor * median(intervals_s),
        )

    position_topic = _find_topic(
        datasets, "vehicle_local_position", instance=position_instance
    )
    discontinuities = _discontinuity_times(position_topic)

    raw_distance_m = 0.0
    quality_distance_m = 0.0
    excluded_edges = []

    for previous, current, interval_s in zip(
        points,
        points[1:],
        intervals_s,
    ):
        previous_xyz = (
            previous.north_m,
            previous.east_m,
            previous.down_m,
        )
        current_xyz = (
            current.north_m,
            current.east_m,
            current.down_m,
        )
        edge_distance_m = dist(previous_xyz, current_xyz)
        raw_distance_m += edge_distance_m

        reasons = []

        if interval_s > gap_threshold_s:
            reasons.append("gap")

        # Find the first reset strictly after the preceding sample.
        reset_index = bisect_right(discontinuities, previous.timestamp_us)

        if (
            reset_index < len(discontinuities)
            and discontinuities[reset_index] <= current.timestamp_us
        ):
            reasons.append("reset_or_reference_change")

        if reasons:
            excluded_edges.append(
                (
                    previous.timestamp_us,
                    current.timestamp_us,
                    "+".join(reasons),
                )
            )
        else:
            quality_distance_m += edge_distance_m

    armed_events = _state_events(
        _find_topic(datasets, "actuator_armed"),
        "armed",
    )
    landed_events = _state_events(
        _find_topic(datasets, "vehicle_land_detected"),
        "landed",
    )

    start_us = points[0].timestamp_us
    end_us = points[-1].timestamp_us

    airborne_events = None

    if landed_events:
        airborne_events = tuple(
            (timestamp_us, not is_landed)
            for timestamp_us, is_landed in landed_events
        )

    heights_m = [-point.down_m for point in points]

    return QualityMetrics(
        recorded_duration_s=(end_us - start_us) / 1_000_000,
        armed_duration_s=_active_duration(armed_events, start_us, end_us),
        airborne_duration_s=_active_duration(airborne_events, start_us, end_us),
        raw_distance_m=raw_distance_m,
        quality_distance_m=quality_distance_m,
        excluded_distance_m=raw_distance_m - quality_distance_m,
        excluded_edges=tuple(excluded_edges),
        gap_threshold_s=gap_threshold_s,
        max_gap_s=max(intervals_s, default=0.0),
        reset_count=len(discontinuities),
        min_height_above_origin_m=min(heights_m),
        max_height_above_origin_m=max(heights_m),
        max_height_above_takeoff_m=_takeoff_relative_height(
            points,
            landed_events,
        ),
    )

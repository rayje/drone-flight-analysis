"""Additive, conservative quality-aware local-position metrics.

The original reconstruction and CLI output remain unchanged. All timestamps are
PX4 boot microseconds; coordinates are local NED metres. No gap interpolation,
geographic conversion, or unverified estimator reset correction is performed.
"""
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


def _topic(topics: tuple[Topic, ...], name: str, instance: int = 0) -> Topic | None:
    matches = [t for t in topics if t.name == name and t.instance == instance]
    if len(matches) > 1:
        raise ValueError(f"Ambiguous topic: {name} instance {instance}")
    return matches[0] if matches else None


def _events(topic: Topic | None, field: str) -> tuple[tuple[int, bool], ...] | None:
    if topic is None or field not in topic.fields:
        return None
    timestamps = topic.fields.get("timestamp")
    values = topic.fields[field]
    if timestamps is None or len(timestamps) != len(values):
        raise ValueError(f"Invalid {topic.name}.{field} series")
    result = sorted((int(t), bool(v)) for t, v in zip(timestamps, values))
    if any(t < 0 for t, _ in result):
        raise ValueError("Negative state timestamp")
    return tuple(result)


def _state_durations(events: tuple[tuple[int, bool], ...] | None,
                     start: int, end: int) -> float | None:
    if not events:
        return None
    # Do not extrapolate state before the first observation.
    duration = 0
    previous = max(start, events[0][0])
    active = False
    for t, value in events:
        if t < start:
            active = value
            previous = start
            continue
        if t > end:
            break
        if active and t > previous:
            duration += t - previous
        previous, active = max(t, start), value
    if active and end > previous:
        duration += end - previous
    return duration / 1_000_000


def analyze_quality(samples: Iterable[PositionSample], topics: Iterable[Topic],
                    *, gap_factor: float = 1.5,
                    minimum_gap_s: float = 0.15) -> QualityMetrics:
    """Flag discontinuities; never silently bridge resets or significant gaps.

    The gap threshold is max(minimum_gap_s, gap_factor * median sample interval).
    Both position reset counters and local reference changes mark an edge.
    Distance excluded from the quality estimate is reported, not discarded.
    """
    points, datasets = tuple(samples), tuple(topics)
    if not points:
        raise ValueError("No position samples")
    if not isfinite(gap_factor) or gap_factor <= 1 or not isfinite(minimum_gap_s) or minimum_gap_s <= 0:
        raise ValueError("Invalid gap threshold")
    if any(not all(isfinite(v) for v in (p.north_m, p.east_m, p.down_m)) or p.timestamp_us < 0 for p in points):
        raise ValueError("Invalid position sample")
    if any(b.timestamp_us <= a.timestamp_us for a, b in zip(points, points[1:])):
        raise ValueError("Position timestamps must increase")
    position = _topic(datasets, "vehicle_local_position")
    reset_times = set()
    if position is not None:
        f = position.fields
        ts = f.get("timestamp", ())
        for field in ("xy_reset_counter", "z_reset_counter", "ref_timestamp"):
            if field not in f:
                continue
            if len(f[field]) != len(ts):
                raise ValueError(f"Mismatched {field} lengths")
            for i in range(1, len(ts)):
                if f[field][i] != f[field][i-1]:
                    reset_times.add(int(ts[i]))
    intervals = [(b.timestamp_us-a.timestamp_us)/1_000_000 for a,b in zip(points,points[1:])]
    threshold = max(minimum_gap_s, gap_factor*median(intervals)) if intervals else minimum_gap_s
    raw = kept = 0.0
    excluded = []
    for a,b,delta in zip(points,points[1:],intervals):
        length = dist((a.north_m,a.east_m,a.down_m),(b.north_m,b.east_m,b.down_m))
        raw += length
        reasons = []
        if delta > threshold:
            reasons.append("gap")
        if any(a.timestamp_us < t <= b.timestamp_us for t in reset_times):
            reasons.append("reset_or_reference_change")
        if reasons:
            excluded.append((a.timestamp_us,b.timestamp_us,"+".join(reasons)))
        else:
            kept += length
    armed = _events(_topic(datasets,"actuator_armed"),"armed")
    landed = _events(_topic(datasets,"vehicle_land_detected"),"landed")
    start,end = points[0].timestamp_us,points[-1].timestamp_us
    armed_duration = _state_durations(armed,start,end)
    airborne_duration = _state_durations(
        tuple((t,not value) for t,value in landed) if landed else None,start,end)
    # Takeoff reference requires a transition from landed to airborne and a
    # preceding sampled ground position. Otherwise the value is unknown.
    takeoff_height = None
    if landed:
        transitions = [t for (prev_t,prev),(t,now) in zip(landed,landed[1:])
                       if prev and not now and start <= t <= end]
        if transitions:
            t0=transitions[0]
            before=[p for p in points if p.timestamp_us <= t0]
            if before:
                ground=-before[-1].down_m
                airborne_points=[p for p in points if p.timestamp_us >= t0]
                if airborne_points:
                    takeoff_height=max(-p.down_m-ground for p in airborne_points)
    heights=[-p.down_m for p in points]
    return QualityMetrics((end-start)/1_000_000,armed_duration,airborne_duration,
                          raw,kept,raw-kept,tuple(excluded),threshold,
                          max(intervals,default=0.0),len(reset_times),
                          min(heights),max(heights),takeoff_height)

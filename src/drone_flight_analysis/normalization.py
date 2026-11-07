"""Normalize PX4 local position to microsecond timestamps and NED metres."""

from math import isfinite
from numbers import Integral
from typing import Iterable

from .models import PositionSample, Topic


def normalize_local_position(topics: Iterable[Topic], instance: int = 0) -> tuple[PositionSample, ...]:
    """Select one vehicle_local_position instance; reject ambiguous/invalid data.

    PX4 x/y/z are north/east/down in metres, not latitude/longitude/altitude.
    Optional xy_valid/z_valid flags exclude invalid estimates. No interpolation
    or coordinate-origin reset correction is performed.
    """
    matches = [t for t in topics if t.name == "vehicle_local_position" and t.instance == instance]
    if len(matches) != 1:
        raise ValueError("Expected exactly one vehicle_local_position topic for requested instance")
    fields = matches[0].fields
    required = ("timestamp", "x", "y", "z")
    if any(key not in fields for key in required):
        raise ValueError("Local position requires timestamp, x, y, z")
    keys = required + tuple(k for k in ("xy_valid", "z_valid") if k in fields)
    if len({len(fields[key]) for key in keys}) != 1:
        raise ValueError("Local position field lengths differ")
    samples = []
    for i, timestamp in enumerate(fields["timestamp"]):
        if any(not fields[k][i] for k in ("xy_valid", "z_valid") if k in fields):
            continue
        if isinstance(timestamp, bool) or not isinstance(timestamp, Integral) or timestamp < 0:
            raise ValueError("Timestamps must be nonnegative integer microseconds")
        xyz = tuple(float(fields[k][i]) for k in ("x", "y", "z"))
        if not all(isfinite(v) for v in xyz):
            raise ValueError("Positions must be finite")
        samples.append(PositionSample(int(timestamp), *xyz))
    samples.sort(key=lambda sample: sample.timestamp_us)
    if len({s.timestamp_us for s in samples}) != len(samples):
        raise ValueError("Duplicate position timestamps")
    if not samples:
        raise ValueError("No valid local position samples")
    return tuple(samples)

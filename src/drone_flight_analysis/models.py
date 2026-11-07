"""Shared immutable records, independent of the log reader."""

from dataclasses import dataclass
from typing import Mapping, Sequence


@dataclass(frozen=True)
class Topic:
    name: str
    instance: int
    fields: Mapping[str, Sequence]


@dataclass(frozen=True)
class PositionSample:
    timestamp_us: int
    north_m: float
    east_m: float
    down_m: float


@dataclass(frozen=True)
class Flight:
    """Local NED trajectory; altitude is relative to the local origin."""

    samples: tuple[PositionSample, ...]
    duration_s: float
    distance_m: float
    max_height_above_origin_m: float

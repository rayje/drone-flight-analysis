"""Reconstruct a sampled trajectory without aircraft-specific assumptions."""

from math import dist, isfinite
from typing import Iterable

from .models import Flight, PositionSample


def reconstruct_flight(samples: Iterable[PositionSample]) -> Flight:
    """Summarize a strictly time-ordered trajectory (no arming segmentation).

    Distance is the sampled 3D path length and may include estimator resets.
    """
    points = tuple(samples)
    if not points:
        raise ValueError("Cannot reconstruct an empty trajectory")
    if any(not all(isfinite(v) for v in (s.north_m, s.east_m, s.down_m)) for s in points):
        raise ValueError("Positions must be finite")
    if any(s.timestamp_us < 0 for s in points):
        raise ValueError("Timestamps must be nonnegative")
    if any(b.timestamp_us <= a.timestamp_us for a, b in zip(points, points[1:])):
        raise ValueError("Samples must have strictly increasing timestamps")
    distance = sum(dist((a.north_m, a.east_m, a.down_m), (b.north_m, b.east_m, b.down_m))
                   for a, b in zip(points, points[1:]))
    return Flight(points, (points[-1].timestamp_us - points[0].timestamp_us) / 1_000_000,
                  distance, max(-s.down_m for s in points))

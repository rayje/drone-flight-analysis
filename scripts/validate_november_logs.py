"""Validate quality metrics against the three November 3 Firefly logs.

Run from an installed editable checkout with pyulog available:
    python scripts/validate_november_logs.py /path/to/log-directory

The script reads logs without modifying them and writes no artifacts.
"""
import argparse
import hashlib
from pathlib import Path

from drone_flight_analysis import normalize_local_position, read_ulog
from drone_flight_analysis.quality import analyze_quality
from drone_flight_analysis.reconstruction import reconstruct_flight


EXPECTED_LOGS = {
    "log_91_2026-11-3-20-13-06.ulg": {
        "sha256": "4adcd2d727b8bc5991a4dcd12a550abc8743055d2df92b2d1160ec9f0e4b1dc2",
        "reset_count": 0,
        "gap_count": 0,
    },
    "log_108_2026-11-3-20-13-14.ulg": {
        "sha256": "778358785158f3c2acf29d1d6c598e89ef7d1a389aaa0cbee309bc34a05d492a",
        "reset_count": 0,
        "gap_count": 3,
    },
    "log_94_2026-11-3-20-13-18.ulg": {
        "sha256": "979a2cfeb4ac15b56ccedf86ee682f911598cab91a6bb4e58a878bef1af08e61",
        "reset_count": 1,
        "gap_count": 0,
    },
}


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()

    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)

    return digest.hexdigest()


def validate_log(path: Path, expected: dict) -> None:
    actual_hash = sha256_file(path)

    if actual_hash != expected["sha256"]:
        raise AssertionError(f"Source hash mismatch: {path.name}")

    topics = read_ulog(path)
    samples = normalize_local_position(topics)
    original = reconstruct_flight(samples)
    quality = analyze_quality(samples, topics)

    gap_count = sum(
        "gap" in reason
        for _, _, reason in quality.excluded_edges
    )

    if quality.reset_count != expected["reset_count"]:
        raise AssertionError(
            f"{path.name}: expected {expected['reset_count']} resets, "
            f"observed {quality.reset_count}"
        )

    if gap_count != expected["gap_count"]:
        raise AssertionError(
            f"{path.name}: expected {expected['gap_count']} gaps, "
            f"observed {gap_count}"
        )

    if abs(quality.raw_distance_m - original.distance_m) > 1e-6:
        raise AssertionError(f"{path.name}: raw distance regression")

    if quality.quality_distance_m > quality.raw_distance_m + 1e-6:
        raise AssertionError(f"{path.name}: quality distance exceeds raw")

    print(f"PASS {path.name}")
    print(f"  samples: {len(samples)}")
    print(f"  raw distance: {quality.raw_distance_m:.3f} m")
    print(f"  quality distance: {quality.quality_distance_m:.3f} m")
    print(f"  excluded distance: {quality.excluded_distance_m:.3f} m")
    print(f"  gap edges: {gap_count}; reset events: {quality.reset_count}")
    print(f"  recorded duration: {quality.recorded_duration_s:.3f} s")
    print(f"  armed duration: {quality.armed_duration_s} s")
    print(f"  airborne duration: {quality.airborne_duration_s} s")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log_directory", type=Path)
    args = parser.parse_args()

    for filename, expected in EXPECTED_LOGS.items():
        path = args.log_directory / filename

        if not path.is_file():
            parser.error(f"Missing required log: {path}")

        validate_log(path, expected)


if __name__ == "__main__":
    main()

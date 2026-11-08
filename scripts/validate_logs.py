"""Validate configured ULog cases without hard-coded aircraft or log names.

Usage:
    python scripts/validate_logs.py --config config/log_validation.json --keys log91 log94
    python scripts/validate_logs.py --config config/log_validation.json --all

Log files are read-only. No output files are written.
"""

import argparse
import hashlib
import json
from pathlib import Path

from drone_flight_analysis import normalize_local_position, read_ulog
from drone_flight_analysis.quality import analyze_quality
from drone_flight_analysis.reconstruction import reconstruct_flight


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()

    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)

    return digest.hexdigest()


def load_cases(config_path: Path) -> dict:
    with config_path.open(encoding="utf-8") as source:
        configuration = json.load(source)

    if not isinstance(configuration, dict):
        raise ValueError("Configuration must be a JSON object")

    cases = configuration.get("cases")

    if not isinstance(cases, dict) or not cases:
        raise ValueError("Configuration must contain a nonempty 'cases' object")

    for key, case in cases.items():
        if not isinstance(case, dict) or not isinstance(case.get("file"), str):
            raise ValueError(f"Case {key!r} must define a file path")

        if not case["file"]:
            raise ValueError(f"Case {key!r} has an empty file path")

        expected = case.get("expected", {})

        if not isinstance(expected, dict):
            raise ValueError(f"Case {key!r} expected must be an object")

        unknown = set(expected) - {"sha256", "reset_count", "gap_count"}

        if unknown:
            raise ValueError(f"Case {key!r} has unknown expectations: {sorted(unknown)}")

    return cases


def validate_case(
    key: str,
    case: dict,
    log_directory: Path,
) -> None:
    source_path = Path(case["file"])

    if not source_path.is_absolute():
        source_path = log_directory / source_path

    source_path = source_path.resolve(strict=True)
    expected = case.get("expected", {})

    if "sha256" in expected:
        digest = sha256_file(source_path)

        if digest.lower() != expected["sha256"].lower():
            raise AssertionError(f"{key}: source SHA-256 mismatch")

    topics = read_ulog(source_path)
    samples = normalize_local_position(topics, instance=case.get("instance", 0))

    original = reconstruct_flight(samples)
    quality = analyze_quality(
        samples,
        topics,
        gap_factor=case.get("gap_factor", 1.5),
        minimum_gap_s=case.get("minimum_gap_s", 0.15),
    )

    gap_count = sum(
        "gap" in reason
        for _, _, reason in quality.excluded_edges
    )

    observed = {
        "reset_count": quality.reset_count,
        "gap_count": gap_count,
    }

    for metric, expected_value in expected.items():
        if metric == "sha256":
            continue

        if observed[metric] != expected_value:
            raise AssertionError(
                f"{key}: {metric}: expected {expected_value}, "
                f"observed {observed[metric]}"
            )

    if abs(quality.raw_distance_m - original.distance_m) > 1e-6:
        raise AssertionError(f"{key}: raw distance regression")

    if quality.quality_distance_m > quality.raw_distance_m + 1e-6:
        raise AssertionError(f"{key}: quality distance exceeds raw distance")

    print(f"PASS {key}: {source_path.name}")
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
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument(
        "--log-directory",
        type=Path,
        help="Base directory for relative log paths (default: config directory)",
    )
    selection = parser.add_mutually_exclusive_group(required=True)
    selection.add_argument("--keys", nargs="+", metavar="KEY")
    selection.add_argument("--all", action="store_true")
    args = parser.parse_args()

    cases = load_cases(args.config)
    selected_keys = list(cases) if args.all else args.keys

    unknown_keys = [key for key in selected_keys if key not in cases]

    if unknown_keys:
        parser.error(f"Unknown validation keys: {', '.join(unknown_keys)}")

    log_directory = args.log_directory or args.config.resolve().parent
    failures = []

    for key in dict.fromkeys(selected_keys):
        try:
            validate_case(key, cases[key], log_directory)
        except (AssertionError, OSError, ValueError) as error:
            failures.append(key)
            print(f"FAIL {key}: {error}")

    if failures:
        parser.exit(1, f"Validation failed for: {', '.join(failures)}\n")


if __name__ == "__main__":
    main()

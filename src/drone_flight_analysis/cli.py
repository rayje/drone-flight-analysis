"""Command-line investigation reports with source and code provenance."""

import argparse
import hashlib
import json
from pathlib import Path
import subprocess

from .normalization import normalize_local_position
from .parsing import read_ulog
from .reconstruction import reconstruct_flight


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path)
    parser.add_argument("--instance", type=int, default=0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    source = args.log.resolve(strict=True)
    if args.output.resolve() == source:
        parser.error("Output must differ from the original log")
    # Resolve code provenance from the package checkout, not the caller's cwd.
    root = Path(__file__).resolve().parents[2]
    commit = subprocess.check_output(["git", "-C", str(root), "rev-parse", "HEAD"], text=True).strip()
    if subprocess.check_output(["git", "-C", str(root), "status", "--porcelain"], text=True).strip():
        parser.error("Commit analysis changes before recording an investigation")
    hasher = hashlib.sha256()
    with source.open("rb") as log_file:
        for chunk in iter(lambda: log_file.read(1024 * 1024), b""):
            hasher.update(chunk)
    digest = hasher.hexdigest()
    flight = reconstruct_flight(normalize_local_position(read_ulog(source), args.instance))
    report = {
        "git_commit": commit, "source_log": str(source), "source_sha256": digest,
        "parameters": {"instance": args.instance, "frame": "NED", "position_unit": "m"},
        "sample_count": len(flight.samples), "duration_s": flight.duration_s,
        "distance_m": flight.distance_m,
        "max_height_above_origin_m": flight.max_height_above_origin_m,
    }
    with args.output.open("x") as output:
        json.dump(report, output, indent=2)
        output.write("\n")

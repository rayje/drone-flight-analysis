"""Generate a reproducible JSON timeline from one PX4 ULog.

Run via `drone-flight-events flight.ulg --output events.json`.
"""
import argparse
import hashlib
import json
from pathlib import Path
from importlib.metadata import version

from .events import reconstruct_events
from .parsing import read_ulog


def main() -> None:
    """Parse a read-only ULog and write a provenance-bearing event timeline."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path)
    parser.add_argument("--instance", type=int, default=0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    source = args.log.resolve(strict=True)
    if args.output.resolve() == source:
        parser.error("Output must differ from the original log")
    hasher = hashlib.sha256()
    with source.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            hasher.update(chunk)
    events = reconstruct_events(read_ulog(source), position_instance=args.instance)
    report = {
        "schema_version": 1,
        "analyzer_version": version("drone-flight-analysis"),
        "source_log": str(source),
        "source_sha256": hasher.hexdigest(),
        "parameters": {"position_instance": args.instance, "timebase": "px4_boot_us"},
        "event_count": len(events),
        "events": [event.as_dict() for event in events],
    }
    with args.output.open("x") as output:
        json.dump(report, output, indent=2)
        output.write("\n")


if __name__ == "__main__":
    main()

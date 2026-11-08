"""CLI integration tests without a Git checkout or real flight log."""
import json
import sys

import pytest

from drone_flight_analysis import events_cli
from drone_flight_analysis.models import Topic


def test_cli_runs_without_git_and_records_provenance(tmp_path, monkeypatch):
    source = tmp_path / "flight.ulg"
    source.write_bytes(b"example flight log bytes")
    output = tmp_path / "events.json"
    monkeypatch.setattr(events_cli, "read_ulog", lambda path: [
        Topic("vehicle_status", 0, {
            "timestamp": [100, 200], "nav_state": [3, 18]
        })
    ])
    monkeypatch.setattr(sys, "argv", [
        "drone-flight-events", str(source), "--output", str(output)
    ])
    events_cli.main()
    result = json.loads(output.read_text())
    assert result["event_count"] == 1
    assert result["events"][0]["type"] == "navigation_state_changed"
    assert result["parameters"]["timebase"] == "px4_boot_us"
    assert result["analyzer_version"]
    assert len(result["source_sha256"]) == 64
    with pytest.raises(FileExistsError):
        events_cli.main()


def test_cli_refuses_to_overwrite_original_log(tmp_path, monkeypatch):
    source = tmp_path / "flight.ulg"
    source.write_bytes(b"flight log")
    monkeypatch.setattr(sys, "argv", [
        "drone-flight-events", str(source), "--output", str(source)
    ])
    with pytest.raises(SystemExit):
        events_cli.main()
    assert source.read_bytes() == b"flight log"

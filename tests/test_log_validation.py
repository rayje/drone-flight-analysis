"""Tests for config-driven log validation independent of source ULogs."""

import importlib.util
import json
from pathlib import Path

import pytest

from drone_flight_analysis.models import Topic


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts" / "validate_logs.py"
SPEC = importlib.util.spec_from_file_location("validate_logs", SCRIPT_PATH)
VALIDATOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(VALIDATOR)


def write_config(tmp_path, cases):
    path = tmp_path / "cases.json"
    path.write_text(json.dumps({"cases": cases}), encoding="utf-8")
    return path


@pytest.mark.parametrize(
    "case",
    [
        {"file": ""},
        {"file": "flight.ulg", "instance": -1},
        {"file": "flight.ulg", "instance": True},
        {"file": "flight.ulg", "gap_factor": "large"},
        {"file": "flight.ulg", "expected": {"gap_count": "3"}},
        {"file": "flight.ulg", "expected": {"reset_count": True}},
        {"file": "flight.ulg", "expected": {"sha256": "not-a-hash"}},
        {"file": "flight.ulg", "expected": {"unknown": 1}},
    ],
)
def test_invalid_configuration_rejected(tmp_path, case):
    with pytest.raises(ValueError):
        VALIDATOR.load_cases(write_config(tmp_path, {"case": case}))


def test_config_keys_are_independent_of_filenames(tmp_path, monkeypatch, capsys):
    log_file = tmp_path / "arbitrary.ulg"
    log_file.write_bytes(b"fake test source")
    digest = VALIDATOR.sha256_file(log_file)

    topics = (
        Topic(
            "vehicle_local_position",
            1,
            {
                "timestamp": [0, 100_000, 200_000],
                "x": [0.0, 1.0, 2.0],
                "y": [0.0, 0.0, 0.0],
                "z": [0.0, 0.0, 0.0],
                "xy_reset_counter": [0, 0, 0],
            },
        ),
        Topic(
            "vehicle_local_position",
            0,
            {
                "timestamp": [0, 100_000, 200_000],
                "x": [0.0, 1.0, 2.0],
                "y": [0.0, 0.0, 0.0],
                "z": [0.0, 0.0, 0.0],
                "xy_reset_counter": [0, 0, 1],
            },
        ),
    )
    monkeypatch.setattr(VALIDATOR, "read_ulog", lambda path: topics)

    case = {
        "file": log_file.name,
        "instance": 1,
        "expected": {"sha256": digest, "reset_count": 0, "gap_count": 0},
    }
    cases = VALIDATOR.load_cases(write_config(tmp_path, {"my-flight": case}))

    VALIDATOR.validate_case("my-flight", cases["my-flight"], tmp_path)

    assert "PASS my-flight" in capsys.readouterr().out


def test_hash_mismatch_rejected_before_parsing(tmp_path, monkeypatch):
    log_file = tmp_path / "flight.ulg"
    log_file.write_bytes(b"source")
    monkeypatch.setattr(
        VALIDATOR,
        "read_ulog",
        lambda path: pytest.fail("Parser must not be invoked on a hash mismatch"),
    )

    with pytest.raises(AssertionError, match="SHA-256 mismatch"):
        VALIDATOR.validate_case(
            "case",
            {"file": log_file.name, "expected": {"sha256": "0" * 64}},
            tmp_path,
        )

import struct

import pytest

from drone_flight_analysis import normalize_local_position, read_ulog, reconstruct_flight
from drone_flight_analysis.models import PositionSample, Topic


def message(kind, payload):
    return struct.pack('<HB', len(payload), ord(kind)) + payload


@pytest.fixture
def ulog(tmp_path):
    # Minimal real ULog with format, subscription, and two position messages.
    data = b'ULog\x01\x12\x35' + bytes([1]) + struct.pack('<Q', 0)
    data += message('F', b'vehicle_local_position:uint64_t timestamp;float x;float y;float z;')
    data += message('A', struct.pack('<BH', 0, 7) + b'vehicle_local_position')
    for values in [(1_000_000, 0, 0, 0), (3_000_000, 3, 4, -12)]:
        data += message('D', struct.pack('<HQfff', 7, *values))
    path = tmp_path / 'flight.ulg'
    path.write_bytes(data)
    return path


def test_real_ulog_pipeline_preserves_source(ulog):
    original = ulog.read_bytes()
    topics = read_ulog(ulog)
    assert topics[0].instance == 0
    flight = reconstruct_flight(normalize_local_position(topics))
    assert flight.duration_s == 2
    assert flight.distance_m == 13
    assert flight.max_height_above_origin_m == 12
    assert len(flight.samples) == 2
    assert ulog.read_bytes() == original


def topic(**overrides):
    fields = dict(timestamp=[2, 1], x=[3, 0], y=[4, 0], z=[-12, 0])
    fields.update(overrides)
    return Topic('vehicle_local_position', 0, fields)


def test_normalization_sorts_and_filters():
    assert [s.timestamp_us for s in normalize_local_position([topic()])] == [1, 2]
    assert len(normalize_local_position([topic(xy_valid=[True, False])])) == 1


@pytest.mark.parametrize('fields', [dict(timestamp=[1, 1]), dict(x=[0]),
                                    dict(x=[float('nan'), 0]), dict(timestamp=[-1, 0]),
                                    dict(timestamp=[1.5, 2]), dict(z_valid=[False, False])])
def test_invalid_telemetry(fields):
    with pytest.raises(ValueError):
        normalize_local_position([topic(**fields)])


def test_missing_topic_and_instance():
    with pytest.raises(ValueError):
        normalize_local_position([])
    with pytest.raises(ValueError):
        normalize_local_position([topic()], instance=1)
    with pytest.raises(ValueError):
        normalize_local_position([topic(), topic()])
    with pytest.raises(ValueError):
        normalize_local_position([Topic('vehicle_local_position', 0, {})])


@pytest.mark.parametrize('samples', [[], [PositionSample(2, 0, 0, 0), PositionSample(1, 0, 0, 0)],
                                     [PositionSample(0, float('inf'), 0, 0)]])
def test_invalid_reconstruction(samples):
    with pytest.raises(ValueError):
        reconstruct_flight(samples)


def test_single_sample():
    result = reconstruct_flight([PositionSample(0, 1, 2, -3)])
    assert result.distance_m == result.duration_s == 0


def test_missing_log(tmp_path):
    with pytest.raises(FileNotFoundError):
        read_ulog(tmp_path / 'missing.ulg')


def test_cli_report_and_overwrite_protection(ulog, tmp_path, monkeypatch):
    import hashlib
    import json
    from drone_flight_analysis import cli

    output = tmp_path / 'report.json'
    monkeypatch.setattr('sys.argv', ['drone-flight-analysis', str(ulog), '--output', str(output)])
    monkeypatch.setattr(cli.subprocess, 'check_output',
                        lambda command, **kwargs: '' if 'status' in command else 'a' * 40 + '\n')
    cli.main()
    report = json.loads(output.read_text())
    assert report['git_commit'] == 'a' * 40
    assert report['source_sha256'] == hashlib.sha256(ulog.read_bytes()).hexdigest()
    assert report['distance_m'] == 13
    with pytest.raises(FileExistsError):
        cli.main()
    monkeypatch.setattr('sys.argv', ['drone-flight-analysis', str(ulog), '--output', str(ulog)])
    original = ulog.read_bytes()
    with pytest.raises(SystemExit):
        cli.main()
    assert ulog.read_bytes() == original


def test_cli_rejects_dirty_checkout(ulog, tmp_path, monkeypatch):
    from drone_flight_analysis import cli

    monkeypatch.setattr('sys.argv', ['drone-flight-analysis', str(ulog), '--output', str(tmp_path / 'out.json')])
    monkeypatch.setattr(cli.subprocess, 'check_output',
                        lambda command, **kwargs: ' M analysis.py' if 'status' in command else 'a' * 40)
    with pytest.raises(SystemExit):
        cli.main()
    assert not (tmp_path / 'out.json').exists()

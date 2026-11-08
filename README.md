# drone-flight-analysis

Initial Python framework for investigating PX4 ULog (`.ulg`)
flight logs. Requires Python 3.10 or newer. No aircraft-specific calibration
or hardware access is required.

## Install and test

```sh
python -m venv .venv
. .venv/bin/activate
python -m pip install -e '.[dev]'
python -m pytest
```

## Usage

Commit analysis changes before an investigation. Keep source logs outside the
checkout or in an ignored location, and write reports to a separate directory:

```sh
mkdir -p investigations
drone-flight-analysis /path/to/flight.ulg --instance 0 --output investigations/flight.json
```

The command requires a clean Git checkout and records the full analysis commit,
source SHA-256, source path, and parameters. It refuses to overwrite an existing
report or the input log. Run from an editable source installation so provenance
can be resolved from this repository. Wheel installations without Git metadata
are not supported for investigation reports.

Python components can also be used independently:

```python
from drone_flight_analysis import read_ulog, normalize_local_position, reconstruct_flight

topics = read_ulog('/path/to/flight.ulg')
samples = normalize_local_position(topics, instance=0)
flight = reconstruct_flight(samples)
print(flight.duration_s, flight.distance_m)
```

## Architecture and limits

- `parsing`: read-only pyulog adapter preserving topic names, instances, and fields.
- `models`: shared telemetry and trajectory records.
- `normalization`: selects `vehicle_local_position`, sorts integer microsecond
  timestamps, validates field lengths and finite values, rejects duplicate
  timestamps, and filters invalid estimates when validity flags exist.
- `reconstruction`: sampled 3D path, elapsed duration, and maximum height above
  the PX4 local origin. Coordinates are north/east/down in metres; height is
  negative down, not height above terrain or takeoff.
- `cli`: JSON investigation reports with Git and log provenance.

The initial framework analyzes one local trajectory. It does not segment armed
flights, correct estimator origin resets, interpolate gaps, perform geodetic
conversion, or interpret specific channels. Distance can include jumps
from resets or telemetry gaps. No real logs are bundled; tests exercise
a generated ULog through pyulog and deterministic telemetry cases. Validate
against representative aircraft logs before drawing operational conclusions.

See [AGENTS.md](AGENTS.md) for development and review rules.

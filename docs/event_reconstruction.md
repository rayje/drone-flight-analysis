# Synchronized flight-event reconstruction (Phase 3)

This is a **single-aircraft**, read-only, PX4 boot-clock event timeline. It is
not a causal investigation, cross-aircraft synchronization, or proof that a
command was accepted or executed. No Firefly-specific DGF semantics are assumed.

## Use

Install from a clean Git checkout using `python -m pip install -e '.[dev]'`,
then:

```sh
drone-flight-events /path/to/flight.ulg --instance 0 --output /path/to/events.json
```

The existing `drone-flight-analysis` trajectory command is unchanged. The new
command refuses to overwrite its output or source log, requires a clean Git
checkout, and records the analysis Git commit, source SHA-256, selected local
position instance and PX4 boot-time units.

Python API:

```python
from drone_flight_analysis import read_ulog
from drone_flight_analysis.events import reconstruct_events

events = reconstruct_events(read_ulog("/path/to/flight.ulg"))
for event in events:
    print(event.as_dict())
```

## Events and interpretation

- `actuator_armed.armed`, `vehicle_land_detected.landed`:
  observed arming and landed-state transitions.
- `vehicle_status.nav_state`, `arming_state`, `failsafe` (when present):
  raw integer/boolean state transitions, **not** firmware-specific decoded
  meanings.
- `vehicle_local_position.xy_reset_counter`, `z_reset_counter`,
  `ref_timestamp`: position reset/reference changes for selected instance.
- `vehicle_command.command`: each recorded command message.
- `vehicle_command_ack.command`: each acknowledgment message with its
  separately recorded `command_result` if present. An acknowledgment is not
  proof of execution; raw result codes are not decoded here.
- `failsafe_flags`: observed transitions of boolean fields when present.
  Field availability depends on PX4/Firefly firmware.

Each event includes source topic, instance, field, original sample index,
timestamp_us, earliest_us and latest_us. A state transition was observed
**after** the previous sample and **by** the current sample; the actual
transition time is unknown within that interval. Command records are point
observations, not transition intervals. Initial state is not inferred.

All timestamps are PX4 **boot microseconds**, not UTC or time since takeoff.
Events from different aircraft **must not** be compared by boot timestamps.
No interpolation, causal inference, missing-topic substitution, GPS clock
conversion, or cross-aircraft alignment is performed.

## Limits and validation

The parser retains original topics and instances. Unknown/missing fields are
ignored; malformed lengths, negative timestamps, duplicate timestamps in an
extracted field, or duplicate topic instances raise errors. Nonzero instances
for non-position topics are not yet supported. Original logs are never modified.

Run `python -m pytest` for synthetic regression tests covering ordering,
transition brackets, commands/acknowledgments, reset instances, malformed
inputs, missing telemetry and input immutability. Before operational use,
validate extracted events against representative real Firefly Gen2 logs and
their firmware-specific topic schemas.

Future phases can consume this immutable event model for trajectory/setpoint
correlation, estimator/power diagnostics, multi-aircraft time alignment and
evidence-backed root-cause hypotheses.

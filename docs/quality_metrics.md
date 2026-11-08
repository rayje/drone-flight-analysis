# Quality-aware trajectory metrics (experimental)

**Production vs test separation:** `src/drone_flight_analysis/quality.py` is
production analysis code, not an integration test. It measures how trustworthy
sampled local-position path metrics are by flagging gaps and estimator
reference/reset changes. These diagnostics can inform future incident timelines
and root-cause investigations, but are not themselves a root-cause finding.
`tests/integration/validate_logs.py` is the test-only runner that checks
these calculations against configured known logs. It is not imported by the
production flight-analysis modules.

The original `reconstruct_flight` and CLI report are intentionally unchanged.
`drone_flight_analysis.quality.analyze_quality` is an **additive API** for
conservative local-position diagnostics. It does not establish physical ground
truth and is not yet integrated into the investigation JSON schema.

```python
from drone_flight_analysis import read_ulog, normalize_local_position
from drone_flight_analysis.quality import analyze_quality

topics = read_ulog("/path/to/flight.ulg")
samples = normalize_local_position(topics, instance=0)
metrics = analyze_quality(samples, topics, position_instance=0)
print(metrics.raw_distance_m, metrics.quality_distance_m)
print(metrics.excluded_edges)
```

- **Recorded duration** is elapsed time between first and last retained local
  position samples, including ground time.
- **Armed/airborne duration** uses logged `actuator_armed.armed` and
  `vehicle_land_detected.landed` state transitions, clipped to the position
  observation window. Missing state yields `None`. State is not extrapolated
  before its first observation.
- **Raw distance** is the original sum of sampled three-dimensional NED
  displacements.
- **Quality distance** omits sample-to-sample edges that cross an observed
  local-position reset-counter/reference-timestamp change or exceed a
  configurable gap threshold. Excluded edges and their raw lengths are retained
  in diagnostic totals. This is not a reconstructed or corrected physical path.
- **Gap threshold** defaults to the greater of 0.15 seconds and 1.5 times the
  median interval of retained position samples. These are *provisional*
  diagnostics, not Firefly-specific validated limits.
- **Height** remains relative to the PX4 local origin. An estimated
  takeoff-relative height is only provided when a logged landed-to-airborne
  transition and preceding position sample exist; it is not terrain clearance.
- **Reset detection** examines `xy_reset_counter`, `z_reset_counter`, and
  `ref_timestamp` in the raw `vehicle_local_position` topic. No reset delta
  is applied. The quality calculation excludes crossing edges rather than
  guessing the true displacement.
- **Limitations**: timestamp alignment between topics assumes the same PX4
  boot clock; state and position sample boundaries are not interpolated;
  filtering invalid position samples can create apparent gaps; data dropouts
  shorter than the gap threshold may not be detected; the method does not use
  ULog dropout records; origin latitude/longitude shifts without
  `ref_timestamp` changes are not yet detected; no cross-aircraft coordinate
  transform is performed. A valid estimator position is not independent
  evidence of true physical position.

This module is intended as the first step toward incident timeline reconstruction,
not a root-cause determination. Keep original logs outside Git.

## Configuration-driven log validation

The test-only integration validator at `tests/integration/validate_logs.py` contains no aircraft names or log filenames. Define
named cases in a JSON file (see `config/november_log_validation.json`).
Each case requires `file` and can optionally include `instance`,
`gap_factor`, `minimum_gap_s`, and `expected` with any combination of
`sha256`, `reset_count`, or `gap_count`.

```sh
python -m tests.integration.validate_logs \
  --config config/november_log_validation.json \
  --log-directory /path/to/logs \
  --keys log91 log94

python -m tests.integration.validate_logs \
  --config config/november_log_validation.json \
  --log-directory /path/to/logs --all
```

For an unfamiliar log, omit `expected` to calculate diagnostics without
asserting a baseline. Relative file paths resolve under `--log-directory`,
or the configuration directory if the option is omitted. Original logs remain
unmodified. The program prints results and exits nonzero on any failure.

The example configuration records expected observations from earlier independent
inspection, **not** results of executing the new validator. Do not commit
source ULogs. Tests in CI use synthetic telemetry; real-log validation is a
separate operational checkpoint.

## Analysis component responsibilities

- `parsing.py` reads ULog datasets without modifying source logs.
- `normalization.py` selects and validates the requested local-position topic instance.
- `reconstruction.py` computes baseline sampled trajectory metrics without quality filtering.
- `quality.py` supplements that baseline with observed gaps, resets, state durations,
  and conservative quality-filtered distance; it is analysis code intended for
  future incident reporting, not a test utility.
- `tests/integration/validate_logs.py` executes configured regression checks
  using those production modules; it never supplies analysis logic to them.

The named configuration key (for example `log91`) is a user-facing case selector.
The `file` field independently identifies the ULog, so renaming a case does not
change which file is read. The integration test also checks that the configured
position-topic instance is honored, rather than silently using instance zero.

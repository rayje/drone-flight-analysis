# Development rules

- Keep parsing, normalization, reconstruction, and interfaces loosely coupled. Exchange explicit data models; keep pyulog details inside the parser adapter.
- Preserve approved functionality. Discuss breaking behavior changes before implementing them.
- Work on feature branches. Never merge to main without explicit approval.
- Include automated tests for new behavior and bug fixes. Run `python -m pytest` before requesting review.
- Keep original flight logs unchanged. Read logs without modifying, renaming, or replacing them; write derived artifacts to separate paths.
- Record the full Git commit for every investigation, together with the source log SHA-256 and analysis parameters. Commit analysis changes before producing investigation artifacts.
- Use the existing isolated checkout; do not create Git worktrees unless requested.
- Install development dependencies with `python -m pip install -e '.[dev]'` in a virtual environment.
- Document coordinate frames, units, missing telemetry, and assumptions. Do not infer Firefly-specific calibrations without verified data.

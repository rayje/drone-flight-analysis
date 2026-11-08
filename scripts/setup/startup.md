# Development setup

Use the existing isolated checkout; do not create Git worktrees unless requested.
Read `AGENTS.md` and work on a feature branch.

Install Python 3.10 or newer with the `venv` module and a `python` command on
your PATH. From the repository root, run:

```sh
bash scripts/setup/install.sh
```

The Bash script creates or reuses the ignored `.venv` directory, installs the
editable package and its declared development dependencies, checks dependency
consistency, and runs all existing tests. It locates the repository relative to
itself, so it also works when invoked by path from another directory. Re-running
it refreshes the installation; dependency versions follow `pyproject.toml`.
Package installation requires access to the configured Python package index.

For subsequent Bash sessions, from the repository root:

```sh
source .venv/bin/activate
python -m pip check
python -m pytest
drone-flight-analysis --help
```

No background services, credentials, or aircraft hardware are required for
development and tests. Do not commit virtual environments, credentials, or
temporary files. No real aircraft logs are bundled; tests use a generated ULog.

Before an investigation, commit analysis changes and ensure the checkout is
clean. Keep original flight logs unchanged and write reports to a separate
location. Replace the example paths below; the output must not already exist:

```sh
drone-flight-analysis /path/to/flight.ulg --instance 0 --output /path/to/reports/flight.json
```

Create the report's parent directory first. Reports record the full analysis
commit, source SHA-256, and analysis parameters. Positions use north/east/down
coordinates in metres; height is relative to the PX4 local origin. Validate
against representative aircraft logs before drawing operational conclusions.

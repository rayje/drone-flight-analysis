#!/usr/bin/env bash
set -euo pipefail

# Resolve the checkout from this script, independent of the caller's directory.
repository_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
cd -- "$repository_root"

python -m venv .venv
.venv/bin/python -m pip install -e '.[dev]'
.venv/bin/python -m pip check
.venv/bin/python -m pytest

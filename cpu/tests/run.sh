#!/usr/bin/env bash
# Run the cocotb regression. ROS (or other) pytest plugins on PYTHONPATH break collection,
# so run isolated. Usage: ./run.sh [pytest args...]   e.g. ./run.sh system_tests/test_csr.py -x
set -euo pipefail
here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$here"
exec env -u PYTHONPATH PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PATH="$HOME/.local/bin:$PATH" \
    "$here/../.venv/bin/python" -m pytest -p cocotb_test.plugin -p xdist.plugin -q "$@"

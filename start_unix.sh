#!/usr/bin/env sh
set -eu
cd -- "$(dirname -- "$0")"
if [ ! -x .venv/bin/python ]; then
  echo 'First run: follow README.md to create .venv and install requirements.' >&2
  exit 1
fi
exec .venv/bin/python -m robot_lab

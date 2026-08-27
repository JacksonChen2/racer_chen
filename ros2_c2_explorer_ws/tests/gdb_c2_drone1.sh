#!/usr/bin/env bash
set -euo pipefail

for argument in "$@"; do
  if [[ "${argument}" == "__node:=c2_explorer_1" ]]; then
    exec gdb -q -batch \
      -ex "set pagination off" \
      -ex "run" \
      -ex "thread apply all bt" \
      --args "$@"
  fi
done
exec "$@"

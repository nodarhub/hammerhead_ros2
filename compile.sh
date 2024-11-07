#!/usr/bin/env bash
HERE="$(
  cd -- "$(dirname "$0")" >/dev/null 2>&1
  pwd -P
)"
pushd $HERE
colcon build
popd
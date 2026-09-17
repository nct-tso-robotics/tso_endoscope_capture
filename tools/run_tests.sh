#!/usr/bin/env bash
set -euo pipefail
if [ "$#" -lt 1 ]; then
  echo "Usage: $0 BUILD_DIRECTORY [CTEST_ARGUMENT ...]" >&2
  exit 2
fi
capture_test_build="$(cd -- "$1" && pwd)"
shift
if [ ! -f "$capture_test_build/CTestTestfile.cmake" ]; then
  echo "No configured CTest tests in $capture_test_build; run build-capture first." >&2
  exit 2
fi
capture_test_temp="$(mktemp -d "${TMPDIR:-/tmp}/endoscope-check-XXXXXXXX")"
trap 'rm -rf -- "$capture_test_temp"' EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
mkdir -- "$capture_test_temp/tmp" "$capture_test_temp/ros"
export TMPDIR="$capture_test_temp/tmp" ROS_LOG_DIR="$capture_test_temp/ros"
export PYTHONDONTWRITEBYTECODE=1
# CTest writes logs below its top-level directory. Refer to the actual build as
# a subdirectory so working directories and pre-existing build logs stay intact.
capture_test_delimiter='='
while [[ "$capture_test_build" == *"]$capture_test_delimiter]"* ]]; do
  capture_test_delimiter+='='
done
printf 'subdirs([%s[%s]%s])\n' "$capture_test_delimiter" "$capture_test_build" \
  "$capture_test_delimiter" > "$capture_test_temp/CTestTestfile.cmake"
cd -- "$capture_test_temp"
ctest --output-on-failure "$@"

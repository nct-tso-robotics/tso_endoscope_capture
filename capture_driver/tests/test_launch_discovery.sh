#!/usr/bin/env bash
# Resolve the real ROS launch and executable without starting ROS or hardware.
set -euo pipefail
capture_test_root="$(cd -- "$1" && pwd)"
capture_test_devel="$(cd -- "$2" && pwd)"
capture_test_node="$capture_test_devel/lib/endoscope_capture/endoscope_capture_node"
test -f "$capture_test_root/package.xml"
test -f "$capture_test_devel/.catkin"
test -x "$capture_test_node"
export CMAKE_PREFIX_PATH="$capture_test_devel${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export ROS_PACKAGE_PATH="$capture_test_root${ROS_PACKAGE_PATH:+:$ROS_PACKAGE_PATH}"
export PYTHONDONTWRITEBYTECODE=1
capture_test_files="$(roslaunch --files endoscope_capture endoscope.launch)"
test "$capture_test_files" = "$capture_test_root/launch/endoscope.launch"
capture_test_nodes="$(roslaunch --nodes endoscope_capture endoscope.launch)"
test "$capture_test_nodes" = /endoscope/capture
capture_test_arguments="$(roslaunch --args=/endoscope/capture endoscope_capture endoscope.launch)"
capture_test_arguments="${capture_test_arguments#'ROS_NAMESPACE="/endoscope" '}"
if [[ "$capture_test_arguments" != "$capture_test_node "* &&
      "$capture_test_arguments" != "'$capture_test_node' "* &&
      "$capture_test_arguments" != "\"$capture_test_node\" "* ]]; then
  echo "ROS selected an unexpected capture executable: $capture_test_arguments" >&2
  exit 1
fi
# Match ROS resource lookup's hidden-directory pruning. Visible copies of the
# executable in another build/install tree must not silently shadow this build.
while IFS= read -r -d '' capture_test_candidate; do
  if [ "$capture_test_candidate" != "$capture_test_node" ]; then
    echo "Duplicate visible capture executable: $capture_test_candidate" >&2
    exit 1
  fi
done < <(find "$capture_test_root" -mindepth 1 -type d -name '.*' -prune -o \
  -type f -name endoscope_capture_node -perm -u=x -print0)
echo "PASS one source launch file and one current devel capture executable"

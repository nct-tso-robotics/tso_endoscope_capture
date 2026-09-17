#!/usr/bin/env bash
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
# Catkin-generated POSIX scripts use /usr/bin/env sh.
# Include the host's POSIX tools alongside the Pixi build tools.
export PATH="$PATH:/usr/bin:/bin"
compiler_target="$("$CONDA_PREFIX/bin/c++" -dumpmachine)"
case "${PIXI_ENVIRONMENT_NAME:-}" in
  ros2) capture_ros_version=2 ;;
  ros1) capture_ros_version=1 ;;
  *) capture_ros_version="${ROS_VERSION:-1}" ;;
esac
export ROS_VERSION="$capture_ros_version"
capture_build_dir="$project_dir/build/ros"
capture_cmake_args=(
  -DCMAKE_BUILD_TYPE=RelWithDebInfo
  -DCMAKE_INSTALL_MESSAGE=LAZY
  "-DCMAKE_PREFIX_PATH=$CONDA_PREFIX"
  "-DCMAKE_CXX_COMPILER=$CONDA_PREFIX/bin/c++"
  "-DCMAKE_AR=$CONDA_PREFIX/bin/$compiler_target-ar"
  "-DCMAKE_RANLIB=$CONDA_PREFIX/bin/$compiler_target-ranlib"
  "-DCAPTURE_ROS_VERSION=$capture_ros_version"
  -DBUILD_TESTING=ON
)
if [ "$capture_ros_version" = 2 ]; then
  capture_build_dir="$project_dir/build/ros2"
  capture_cmake_args+=("-DCMAKE_C_COMPILER=$CONDA_PREFIX/bin/cc" "-DPython3_EXECUTABLE=$CONDA_PREFIX/bin/python")
else
  capture_cmake_args+=("-DPYTHON_EXECUTABLE=$CONDA_PREFIX/bin/python" "-DCATKIN_DEVEL_PREFIX=$capture_build_dir/devel" -DCATKIN_ENABLE_TESTING=OFF)
fi
capture_cmake_args+=("-DCMAKE_INSTALL_PREFIX=$capture_build_dir/.install")
cmake "$@" -S "$project_dir" -B "$capture_build_dir" -G Ninja "${capture_cmake_args[@]}"
cmake --build "$capture_build_dir" --parallel 2
if [ "$capture_ros_version" = 2 ]; then
  cmake --install "$capture_build_dir"
fi

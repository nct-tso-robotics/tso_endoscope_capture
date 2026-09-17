# C++ tests

The backend tests use fake DeckLink devices and frames through injected SDK
interfaces. Checks remain active in release builds.

From the repository root:

```sh
cmake -S capture_driver -B build/capture-tests -DCAPTURE_BUILD_ROS=OFF
cmake --build build/capture-tests -j2
bash tools/run_tests.sh build/capture-tests
```

For address, leak, and undefined-behavior checks, use a separate build:

```sh
cmake -S capture_driver -B build/capture-tests-sanitize \
  -DCAPTURE_BUILD_ROS=OFF -DCAPTURE_SANITIZE=ON -DCMAKE_BUILD_TYPE=Debug
cmake --build build/capture-tests-sanitize -j2
ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 UBSAN_OPTIONS=halt_on_error=1 \
  bash tools/run_tests.sh build/capture-tests-sanitize
```

The backend suite covers retained frame ownership, row padding and destination
boundaries, failed or null SDK byte buffers, invalid dimensions, concurrent
frame turnover, no-input timeout and shutdown, signal-loss handling, format
change latching, frame accounting, startup rollback, restart, and cleanup after
SDK failures. A fake callback registration can retain its callback and then
return an error to exercise rollback after partial effects. The message-pool
suite checks that storage cannot be recycled while readers still own it.

Concurrency tests exercise producer, consumer and shutdown threads. The timeout
test checks that waiting for input consumes little CPU.

The full builds test stereo geometry, image processing and publication-rate
estimation. ROS1 includes launch discovery; ROS2 adds a synthetic publisher and
subscriber test in an isolated local DDS domain:

```sh
pixi run -e ros1 test-capture
pixi run -e ros2 test-capture
```

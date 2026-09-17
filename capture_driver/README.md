# DeckLink capture implementation

The integrated node acquires BGRA frames from DeckLink, splits alternating
stereo rows and publishes images, calibration and frequency status. ROS1 uses
C++14; ROS2 uses C++17. Both compile the same backend and stereo processor.
Build and run the selected version from the repository root:

```bash
pixi run -e ros1 build-capture
pixi run -e ros1 roslaunch endoscope_capture endoscope.launch publish_rate:=30
```

```bash
pixi run -e ros2 build-capture
pixi run -e ros2 ros2 launch endoscope_capture endoscope.launch.xml publish_rate:=30
```

The host supplies Desktop Video and its kernel driver. Run one acquisition
process per card.

## Launch parameters

| Argument | Default | Meaning |
| --- | --- | --- |
| `namespace` | `/endoscope` | Namespace containing `capture`, `left` and `right` |
| `publish_rate` | `30` | Requested output frequency, 0.1–240 Hz |
| `device_index` | `0` | SDK device enumeration index |
| `mode` | `1080p5994` | Input mode for the session |
| `frame_id` | `camera` | Image, CameraInfo and status frame ID |
| `buffer_count` | `4` | Slots per output pool, 2–32 |
| `left_calibration`, `right_calibration` | Packaged YAML files | Calibration at separated input resolution |
| `scale_x`, `scale_y` | `0.5`, `1.0` | Per-eye scaling, each in `(0, 1]` |
| `opencv_threads` | `1` | Process-wide OpenCV worker count |
| `camera_info_url` | empty | Optional combined-image calibration with matching dimensions |
| `duration_seconds` | `0` | Bounded session duration; zero runs until shutdown |
| `metrics_file` | empty | Capture JSON report on orderly shutdown |
| `split_metrics_file` | empty | Splitter JSON report on orderly shutdown |
| `qos_reliability` (ROS2) | `reliable` | Raw-topic QoS: `reliable` or `best_effort` |
| `qos_depth` (ROS2) | `4` | Middleware history depth per topic, 1–32 frames |

Arguments become node parameters at startup. ROS2 marks capture parameters
read-only for the running session. For example:

```bash
pixi run -e ros1 roslaunch endoscope_capture endoscope.launch \
  publish_rate:=20 namespace:=/camera_test duration_seconds:=30
```

This produces `/camera_test/capture/image_raw`, sibling combined `camera_info`,
`status` and `diagnostics`, plus `/camera_test/left/image_raw`,
`/camera_test/right/image_raw` and both eye CameraInfo topics. The default combined
CameraInfo is conventionally uncalibrated (`K[0] == 0`).

`publish_rate` limits publication independently of the selected input mode.
An input slower than the requested rate yields fewer images. The node consumes
each retained input once and avoids republishing an old frame. Rate changes take
effect on the next launch.

## Ownership, copies and timing

The SDK callback retains a frame reference and records software arrival time.
A one-slot handoff keeps the newest available input. A condition variable wakes
the consumer on input or shutdown. Frame dimensions, format and row stride are
validated; SDK byte access is checked before copying. The consumer copies pixel
bytes into a pooled ROS image and releases the SDK reference after use.

Capture passes that combined image directly to the shared `StereoProcessor`. Alternating
row views separate the eyes without copying whole rows. Reusable conversion
scratch holds each BGR eye; `INTER_AREA` writes the resized output directly into
pooled image storage. Odd source rows are left and even rows are right.

Image and stereo CameraInfo buffers use fixed pools. Entries are reused after
readers release their shared references. Exhaustion drops and counts the frame,
preserving buffers retained by subscribers.

Calibration dimensions must match the full-width separated eyes. Output
sizes round to the nearest integer, ties to even. The actual output dimensions
scale every entry in the first two rows of K/P, including translation. D/R are
preserved, with zero binning and ROI. Headers propagate from the combined image.

Timestamps represent **software SDK callback arrival** on the host system
clock before copying. Hardware exposure time requires a separate clock mapping.
Capture requires system ROS time; repeated or backwards source stamps are
counted and rejected.

## Status and diagnostics

`status` publishes `endoscope_capture/CaptureStatus` for each published combined
frame with the same timestamp and frame ID. Its fields are:

- `configured_publish_hz`: requested publication frequency.
- `measured_publish_hz` and `measured_rate_valid`: recent achieved frequency,
  estimated with a steady clock; the first sample is invalid with rate zero.
- `rate_window_samples` and `rate_window_seconds`: evidence behind the estimate.
- `source_sequence`: accepted SDK callback frame sequence.

The estimator restarts after missing input or a stall. Per-frame status stops
when frames stop; the separate JSON `diagnostics` heartbeat continues while
waiting for input. Image/CameraInfo/status timestamps can drive lightweight
freshness checks. ROS1 assigns header sequence numbers independently of the
SDK source sequence. ROS2 exposes the SDK sequence through `CaptureStatus`.

Diagnostics and shutdown reports expose frame replacements, delivery, invalid
input, absent signal, SDK errors, format changes, timeouts, buffer exhaustion,
processing errors, copied bytes and timing. `overwritten_before_delivery`
includes intentional selection when input arrives faster than publication.
For example, approximately 59.94 Hz input replaces frames with a 30 Hz output
target. This counter also includes replacements during processing delays.
Frame counts and sequence gaps measured by subscribers describe delivery after
publication.

Both splitter reports count processed/rejected frames and exhausted pools.
The ROS2 report includes the middleware's actual publisher `qos_depth`.
The ROS1 splitter also reports header sequence gaps, repeated/backwards stamps
and processing duration. Capture reports include CPU and peak RSS for the whole
process: SDK/runtime, ROS/OpenCV, scratch and output pools.
The capture report's `publish_ms_*` fields measure stereo processing and all
output publication calls in both ROS versions. Reliable ROS2 delivery can wait
for middleware resources, increasing publication latency.

## Failure handling and checks

RAII releases frames and SDK interfaces after normal shutdown or initialization
failure. Callback registration, stream startup and byte access are checked.
Signal loss clears pending data. A format-change notification latches failure
and requires explicit reconfiguration and restart. Bounded waits allow shutdown
while input is absent.

Backend tests use injected SDK devices/frames. A backend-only sanitizer build is
available without ROS:

```bash
cmake -S capture_driver -B build/capture-sanitize \
  -DCAPTURE_BUILD_ROS=OFF -DCAPTURE_SANITIZE=ON -DCMAKE_BUILD_TYPE=Debug
cmake --build build/capture-sanitize --parallel 2
bash tools/run_tests.sh build/capture-sanitize
```

The full Pixi CTest builds add stereo processing and publication-rate checks.
ROS2 tests use synthetic frames to exercise the production publisher and DDS
subscriptions. The [project README](../README.md) contains build, launch and
test commands.

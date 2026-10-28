# Endoscope capture and stereo splitting

Acquire a DeckLink BGRA stream and publish the combined frame and calibrated
stereo views from one C++ node. Choose ROS1 Noetic or ROS2 Jazzy through Pixi.
Both use the same DeckLink backend, stereo processor and rate estimator. Capture
and splitting share frame buffers within the process. The host supplies Blackmagic
Desktop Video and its kernel driver; Pixi supplies the ROS, OpenCV and build dependencies.

See [Blackmagic driver requirements](patches/README.md) for Ubuntu packages
and the patch needed with Desktop Video 16.4 on PREEMPT_RT.

## Build and launch

Run the commands for your ROS version from the repository root.

**ROS1 Noetic:**

```bash
pixi install -e ros1
pixi run -e ros1 build-capture
pixi run -e ros1 roslaunch endoscope_capture endoscope.launch publish_rate:=30
```

**ROS2 Jazzy:**

```bash
pixi install -e ros2
pixi run -e ros2 build-capture
pixi run -e ros2 ros2 launch endoscope_capture endoscope.launch.xml publish_rate:=30
```

Both launch files accept `publish_rate` and `namespace`, for example
`publish_rate:=15 namespace:=/camera`. The valid range is 0.1–240 Hz.
Actual delivery also depends on input frequency
and processing capacity. The `mode` argument independently selects the DeckLink
input mode; its default is `1080p5994`. Use one acquisition process per card.

For ROS1, set `ROS_MASTER_URI` and `ROS_IP` for a remote master and unset a
conflicting `ROS_HOSTNAME`. For ROS2, use the same `ROS_DOMAIN_ID` as the consumers
and configure DDS discovery for your network. Pixi activates the matching built
overlay; each ROS version has its own environment and build directory.

ROS2 publishes raw images with volatile, keep-last QoS and `qos_depth:=4`. Its default
`qos_reliability:=reliable` supports reliable and best-effort subscribers. Use
`qos_reliability:=best_effort` with best-effort subscribers when occasional
delivery loss is acceptable. Queue depth accepts 1–32 frames; larger queues use
more middleware memory and accommodate longer subscriber delays. ROS2 parameters
are configured at startup.

## Topics and rate information

With the default namespace, the node publishes:

The table uses ROS1 type names; ROS2 inserts `/msg/`, such as
`sensor_msgs/msg/Image` and `endoscope_capture/msg/CaptureStatus`.

| Topic | Type | Content |
| --- | --- | --- |
| `/endoscope/capture/image_raw` | `sensor_msgs/Image` | Combined BGRA frame |
| `/endoscope/capture/camera_info` | `sensor_msgs/CameraInfo` | Combined-frame metadata |
| `/endoscope/left/image_raw`, `/endoscope/right/image_raw` | `sensor_msgs/Image` | BGR eye images |
| `/endoscope/left/camera_info`, `/endoscope/right/camera_info` | `sensor_msgs/CameraInfo` | Scaled per-eye calibration |
| `/endoscope/capture/status` | `endoscope_capture/CaptureStatus` | Per-frame configured and measured frequency |
| `/endoscope/capture/diagnostics` | `std_msgs/String` | JSON heartbeat and capture counters |

`CaptureStatus` carries the image timestamp and frame ID,
`configured_publish_hz`, `measured_publish_hz`, `measured_rate_valid`, the
measurement window size/duration and the SDK source sequence. The rate estimate
uses recent steady-clock publication intervals and restarts after input stalls.
The first sample has no valid measured rate. Per-frame status and CameraInfo
support lightweight freshness monitoring; diagnostics continues during missing
input. Match related image, calibration and status messages by timestamp.

## Image and calibration behavior

- Zero-based even rows form the right eye; odd rows form the left eye.
- Separate the rows, convert each eye to BGR, then downsample using `INTER_AREA`.
- Default scaling is horizontal `0.5`, vertical `1.0`: a 1920x1080 combined
  frame produces two 960x540 images. These dimensions describe pixel storage.
- Preserve the source timestamp and frame ID in both eye images and CameraInfo.
- Scale the first two rows of K and P, including translation terms, to the
  actual rounded output dimensions. Preserve D and R; binning and ROI stay zero.
- Select calibration YAML for your camera with `left_calibration` and
  `right_calibration`. Its dimensions must match the full-width separated eyes.

Image timestamps represent software SDK callback arrival before copying.
Hardware exposure timing requires a separate clock mapping. The default combined
CameraInfo is uncalibrated; `camera_info_url` can supply a matching calibration.
See the [capture implementation](capture_driver/README.md) for buffer ownership,
parameters, diagnostics and failure behavior.

## Tests

```bash
pixi run -e ros1 test-capture
pixi run -e ros2 test-capture
```

The C++ tests cover DeckLink ownership and errors, bounded buffer pools,
stereo row ordering, area filtering, calibration scaling, timestamps and
publication-rate estimation. ROS2 integration tests exercise the production
publisher and a subscriber in an isolated local DDS domain with synthetic
frames and both reliability modes. Test logs are temporary and removed
when the tests finish.

## License and attribution

Licensed under [MIT](LICENSE). Maintained by
[Lorenzo Mazza](mailto:lorenzo.mazza@nct-dresden.de) at
[Translational Surgical Oncology (TSO), NCT Dresden](https://www.nct-dresden.de/en/research/departments-and-groups/translational-surgical-oncology).

[NOTICE.md](NOTICE.md) records source attributions.
The ROS [Image](https://raw.githubusercontent.com/ros/common_msgs/noetic-devel/sensor_msgs/msg/Image.msg)
and [CameraInfo](https://raw.githubusercontent.com/ros/common_msgs/noetic-devel/sensor_msgs/msg/CameraInfo.msg)
contracts define message geometry and timing. Downsampling follows OpenCV's
[area interpolation](https://docs.opencv.org/4.x/da/d54/group__imgproc__transform.html).

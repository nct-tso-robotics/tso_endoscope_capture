# Blackmagic driver requirements

For an Intensity Pro 4K on Ubuntu:

| Package | Provides |
| --- | --- |
| `desktopvideo` | Blackmagic runtime and `blackmagic-io` kernel driver |
| `dkms` | Rebuilds the driver for each installed kernel |
| `build-essential` | Compiler and build tools for the driver |
| `linux-headers-$(uname -r)` | Headers matching the running kernel |

Install the build dependencies:

```bash
sudo apt install dkms build-essential "linux-headers-$(uname -r)"
```

Install the `desktopvideo` package from Blackmagic's **Desktop Video for Linux**
download. Pixi supplies the capture application's ROS and OpenCV dependencies.

## PREEMPT_RT

Ubuntu's `linux-realtime` package installs the RT kernel and its matching headers.

**Desktop Video 16.4a1 with kernel 6.8.1-1015-realtime on x86_64 requires the
[included driver patch](blackmagic-io-16.4-rt-notify-layout.patch).** Apply it to the
driver's DKMS build and rebuild for that kernel before rebooting into RT.

The patch fixes a notification-structure layout mismatch between the driver's C
code and its precompiled component, which causes a crash on the first frame.
Capture and stop/restart were verified at 30 FPS with the patched driver.

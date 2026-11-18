# Checks that need the camera attached

This page is the checklist for a session with a camera. It covers the node's stamps, which offline tests cannot
settle, and the functions that we tested only with recordings. [timestamps.md](timestamps.md) explains the stamps.

Each check states what to run, what to look for, what to expect and how to judge the result. Record the results with
date, setup, mode and numbers in [timestamps.md](timestamps.md), and remove the check here.

Use this mode for all checks unless a check states another: 1080P MJPEG colour, NFOV unbinned depth, 30 frames per
second and auto exposure. Then repeat the check with manual exposure.

## 1. Delay from the middle of the exposure to the stamp

This check measures the bias of the stamps.

**Run.**

1. Put a monitor next to the camera that shows a millisecond clock of the camera PC. A full-screen page works, or a
   small OpenGL or SDL program that draws `CLOCK_REALTIME` in ms as large digits at every vsync. Use a monitor with
   120-240 Hz.
2. Point the colour camera at the monitor and publish `color`.
3. Save 300 frames with their stamps. Use `xblick snapshot` in a loop or a short recording with
   `xblick record`.

**Look for.** Compare the clock value in each image with the stamp of the frame. Read the value by OCR, or by hand
for 20 frames. The shown value is the time at which the monitor drew it, so subtract the latency of the monitor. Take
the input lag from its data sheet or measure it with a photodiode. With auto exposure, note the exposure time, which
`k4a` reports per image.

**Expect.** The stamp minus the shown time is the mean delay from the middle of the exposure to the arrival. It is a
constant of a few tens of ms. The scatter stays within one refresh period of the monitor in each direction.

**Judge.** Put a constant offset into the clock as a fixed correction, measured per mode (`clock.fixed_delay_us`). A
spread much larger than the refresh period means that the `mean` estimator is not good enough. Continue with check 5.

**A more exact variant.** The PC drives an LED through a GPIO or through an Arduino on USB with a known delay. The
LED flashes at known `CLOCK_REALTIME` times. Find the first frame that shows it lit.

## 2. Colour and depth of one capture

**Run.** Start `xblick-azure-kinect`. `xblick diagnostics` shows per report "colour - depth stamp
ms", "colour exposure ms", "depth exposure ms" and "predicted if start-aligned ms".
[parameters.md](parameters.md#diagnostics) explains them. Use auto exposure in a bright and in a dim room. Then use
manual exposure: set `color.exposure_time_us:=8000` and `:=30000` with `xblick-azure-kinect-parameters --set`.

**Look for.** Check whether "colour - depth stamp ms" follows "predicted if start-aligned ms" as the exposure changes,
or whether it stays near 0. Check whether "depth exposure ms" is the SDK's value or carries "(spec)".

**Expect.** Microsoft states that the images of a capture are aligned at the middles of their exposures by default
(`depth_delay_off_color_usec` 0). The difference should therefore be near 0. One camera showed 11-17 ms on 2026-11-03,
which is the reason for `--max-offset-ms 20` in the pairing of its streams. A likely cause is auto exposure, because the
alignment holds for the configured exposure.

**Judge.**

| Result | Action |
| --- | --- |
| Near 0 with manual exposure, 11-17 ms with auto exposure | Document it. Consider manual exposure, or pair by capture where the node now pairs by stamp. |
| Non-zero with manual exposure too | Report it to the SDK's issue tracker and keep the max offset. |

## 3. Offset drift over an hour

**Run.** Let the camera work for 60 min under normal load. Log the offset of the clock (`DeviceClock::offset`) and
`snaps` once a second. Both are internal now, so add them to the node's diagnostics for this check.

**Look for.** The slope of the offset, which is the device crystal against the host clock, and the number of snaps.

**Expect.** A slope of a few µs/s, which is the crystal tolerance. The low-pass follows it smoothly. Snaps occur only
at the start.

**Judge.** Snaps during the hour mean that arrival delays over 10 ms happen under load. Check 5 then matters. If the
filter lags behind the slope by more than 1 ms, raise alpha or use the `min` estimator.

## 4. USB reconnect and restart

**Run.** Pull and replug the USB cable while the node is active. Also stop and start the node.

**Look for.** Check whether the node restarts the device, as [design.md](design.md#stalls) describes, and look at the
first stamps afterwards.

**Expect.** The device timestamp restarts near 0. The first image snaps the offset. The stamps keep increasing.

**Judge.** Fix the restart path if a stamp goes backwards or if a gap is longer than the outage.

## 5. Load on CPU, USB and GPU: mean against min

**Run.** Start the node beside a full Durchblick load, with Boxer, SAM 3 and nvblox on the same PC. Add
`stress-ng --cpu` on all cores for 2 min. Do this once with `--clock-offset mean`, the default, and once with
`--clock-offset min`. Log these diagnostics once a second: "arrival delay p50/p95/max ms", "clock snaps", "clock
resets" and "clock offset s".

**Look for.** The tail of the arrival delay, and how the offset moves. With `mean` the offset follows the load. It
snaps on arrivals that are more than 10 ms late, and the stamp of such an image is late by the extra delay. With
`min` the offset should stay put.

**Expect.** On synthetic delays, `min` kept the stamps within 0.3 ms of the middle of the exposure plus the fixed
delay. `mean` was 3.6 ms late on average and up to 40 ms late for outliers. Microsoft's ROS driver resets its offset
under heavy load, which gives non-monotonic stamps (Azure_Kinect_ROS_Driver issue #228). `min` avoids this.

**Judge.** Make `min` the default (`clock.offset_estimator`) if its offset stays within 1 ms while the offset of
`mean` moves by more than 2 ms or snaps. Keep `mean` and report it if the offset of `min` jumps, which means resets
without a reason, or if it lags the drift.

## 6. Host clock steps (chrony, NTP)

**Run.** On a test PC, with the node active, step the host clock: `date -s` a few seconds off, then
`chronyc makestep`. Also slew it with `chronyc` and a small offset.

**Look for.** The stamps at the step.

**Expect.** A step snaps the offset, because the wall clock moved. The stamps jump with the host clock, as for every
other reader of the host clock. The low-pass follows a slew.

**Judge.** The stamps must follow the wall clock, because consumers compare them with other wall-clock stamps.

## 7. Several PCs

**Run.** `chronyc tracking` and `chronyc sources` on every PC that reads the streams. Compare the clocks, for example
with `clockdiff` or with a Zenoh ping that records send and receive wall times in both directions.

**Look for.** The offsets between the PCs.

**Expect.** Below 1 ms with chrony on a LAN, and below 0.1 ms with PTP. Every stamp age that a TUI or Rerun on
another PC shows includes this offset.

**Judge.** Above 1 ms, configure chrony or PTP before you compare ages across PCs.

## 8. Colour controls

**Run.** `xblick-azure-kinect-parameters --node <base> --describe` prints the ranges and defaults that the device
reports. Then set each control in turn: `--set color.<control>:=<min>`, `:=<max>` and a value between. For exposure
and white balance, also set `color.auto_*:=true` and back to `false`. Watch `color` in `xblick rerun`.

**Expect.** The image changes as the control says. `--get` returns the value that the camera uses. The camera rounds
the exposure to a supported step, at most the frame period. The node refuses a value out of range and names the
range.

**Judge.** Add to [design.md](design.md#special-cases-of-the-azure-kinect) every control that changes nothing, and
every range that the device refuses although it reported it. Also check that a restart of the node sets the controls
back, because they persist on the device.

## 9. Functions that we tested only offline

No device and no recording of `k4arecorder` was at hand during development. The playback test uses a recording that
the SDK wrote without a device. Check these with a camera:

- the node with the camera and `--cuda-device`,
- the status line: captures per second, dropped captures, added Huffman tables,
- `xblick rerun` with the camera's images,
- the depth images from the depth engine in its EGL context, which we tested only with the transformation,
- the point cloud node with the real camera.

## Sources

- SDK reference, `k4a_image_get_device_timestamp_usec`: the device timestamp is the middle of the exposure.
- <https://learn.microsoft.com/en-us/azure/kinect-dk/capture-device-synchronization>: the images of a capture are
  aligned at the middles of their exposures by default, and `depth_delay_off_color_usec`.
- <https://learn.microsoft.com/en-us/azure/kinect-dk/multi-camera-sync>: several devices within 100 µs, and manual
  exposure for precise timing.
- <https://github.com/microsoft/Azure_Kinect_ROS_Driver/issues/228>: the offset reset under load.

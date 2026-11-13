# Timestamps of durchblick-azure-kinect

This page explains how the node computes the stamp of a frame and how exact that stamp is. Durchblick's
[timestamps page](https://github.com/xblick/durchblick/blob/main/docs/concepts/timestamps.md)
defines the fields that every Durchblick sample has: `header.stamp`, and `seq`, `publish_called_ns` and `publish_ns`
in the attachment. The code is in `src/clock.hpp`.

## The stamp

The camera has its own clock. The SDK reports for every image the device timestamp, which is the time of that clock
at the middle of the exposure (`k4a_image_get_device_timestamp_usec`). The node must place this time on the wall
clock of the host.

```
header.stamp = device timestamp + offset - clock.fixed_delay_us
```

The node estimates the offset from the images themselves. For each image the SDK also reports the system timestamp,
which is the host's monotonic clock at the arrival of the image. The node converts it to wall-clock time with the
difference between the two host clocks, which it reads at each image. The difference between this arrival time and
the device timestamp is one sample of the offset.

An image arrives some time after its exposure. This delay has a fixed part and a part that varies with the load. The
fixed part covers half the exposure, the readout and the USB transfer. The two estimators treat the varying part
differently.

| Estimator | Selected by | How it computes the offset | What the stamp means |
| --- | --- | --- | --- |
| `mean` | the default | It low-pass filters the samples with 10 % per image. | The middle of the exposure plus the mean delay, load included. |
| `min` | `--clock-offset min` | It takes the smallest sample of the last `clock.window_s` (2 s) of device time. | The middle of the exposure plus the fixed delay. |

The `mean` estimator sets the offset directly to a sample that differs from it by more than 10 ms. Microsoft's ROS
driver does the same. An image that arrives more than 10 ms late therefore moves the offset. The stamp of that image
is late by the extra delay, and the next image moves the offset back.

The `min` estimator restarts its window when the device time jumps.

The parameter `clock.fixed_delay_us` is the delay from the middle of the exposure to the earliest arrival. The node
subtracts it with both estimators. Its value is 0 until somebody measures it, which is check 1 in
[hardware-checks.md](hardware-checks.md). Until then, stamps are late by this delay when you compare them with other
clocks, such as the robot state or another camera.

## Accuracy on synthetic delays

`tests/test_parameters.cpp` feeds both estimators with synthetic arrivals for 60 s at 30 frames per second. The
delays have a fixed part of 20 ms, an exponential load with a mean of 3 ms, 2 % outliers of +30 ms, a drift of
50 ppm, and one restart. The table shows the error of the stamps against the middle of the exposure plus the fixed
delay.

| Estimator | Mean error | p95 | Maximum |
| --- | --- | --- | --- |
| `mean` | +3.6 ms | 6.7 ms | 40.7 ms |
| `min` | +0.1 ms | 0.2 ms | 0.3 ms, and 3.2 ms once at the restart |

The default stays `mean` until check 5 of [hardware-checks.md](hardware-checks.md) compares both with a camera.

## One clock for all streams

One `DeviceClock` stamps colour, depth, IR and the IMU samples. The difference between two stamps of the camera is
therefore the difference of the device clock, exact to the microsecond.

Depth and IR of one capture share one stamp. Colour has the stamp of the middle of its own exposure. The diagnostics
report the difference as "colour - depth stamp ms", which [parameters.md](parameters.md#diagnostics) describes.

## Bodies

Each result on `<base>/bodies` carries `stamp_ns`. It is the stamp of the depth frame that the tracker worked on.

## Attachments

Durchblick's publishers write the attachments of the camera streams. `durchblick-k4abt` writes the attachment of the
bodies itself, as the same object with these fields:

| Field | Value |
| --- | --- |
| `v` | the version of the attachment |
| `seq` | a counter with one step per sample |
| `publish_called_ns` | the time at which the tracker has its result, before it serializes it |
| `publish_ns` | the time right before the put |
| `source` | the source name |

# Streams of durchblick-azure-kinect

This page describes every stream of the node, the calibration files, the rectified colour image and the point
clouds. The README has the short table.

## The streams

`<base>` is the base name of the node. Its default is `robot/azurekinect`.

The node produces every representation only while a subscriber asks for it. A stream without a subscriber costs
nothing.

### Colour

| Topic | Encoding | Content |
| --- | --- | --- |
| `<base>/color` | `rgb8` | The colour camera. |
| `<base>/color_rect` | `rgb8` | The colour image without lens distortion. |

The camera sends MJPEG. The node passes those bytes on unchanged as `image_raw/compressed`. It decodes a frame only
for subscribers that read pixels, which are the transports `raw`, `shm` and `cuda`.

`color_rect` is the rectified colour image, as ROS's `image_proc` produces it. The section "Rectified colour" below
explains it.

### Depth and IR

| Topic | Encoding | Content |
| --- | --- | --- |
| `<base>/depth` | `16UC1`, mm | The depth camera in its own geometry, 640 x 576 in NFOV unbinned. |
| `<base>/ir` | `mono16` | The IR image. It has the same camera, exposure and stamp as the depth frame. |
| `<base>/depth_to_color` | `16UC1`, mm | The depth as the colour camera sees it. |
| `<base>/depth_to_color_rect` | `16UC1`, mm | The depth as the camera of `color_rect` sees it. |

The node computes `depth_to_color` and `depth_to_color_rect` on the CPU (`src/depth_to_color.hpp`). With
`--sdk-depth-to-color` the SDK's transformation computes `depth_to_color`, which is OpenGL work in the depth engine.

`depth_to_color_rect` has the `CameraInfo` of `color_rect`, so it has no distortion. A pixel outside the raw image
has the depth 0. On the CPU the node projects the depth into that camera directly. With `--sdk-depth-to-color` the
node rectifies `depth_to_color` with the same map as the colour image and takes the nearest pixel.

Use `depth_to_color_rect` for nodes that read `color_rect`. Examples are RGB-D nodes and nvblox.

### Calibration, transforms and IMU

| Topic | Type | Content |
| --- | --- | --- |
| `<base>/k4a_calibration` | JSON, queryable | The calibration record: the raw device calibration, the depth mode and the colour resolution. |
| `<base>/tf_static` | `tf2_msgs/TFMessage`, also queryable | The poses of the colour camera and of the IMU in the depth camera's frame. |
| `<base>/imu` | `sensor_msgs/Imu` | One message per IMU sample, about 1600 per second, while a subscriber exists. |

The poses in `tf_static` come from the factory calibration, as in ROS's `/tf_static`. The pose of the camera on its
mount belongs to the model of the robot or rig that carries it.

An IMU message contains the acceleration in m/s^2 and the angular velocity in rad/s. Both are in the frame of the
accelerometer, and the node turns the axes of the gyroscope into that frame. The frame is `AzureKinectImu` by default,
and `--imu-frame-id` changes it. The message contains no orientation. `--no-imu` turns the stream off.

### Diagnostics

`<base>/diagnostics` is a `diagnostic_msgs/DiagnosticArray`. Three programs publish on it, about once a second.

| Status | Publisher | Content |
| --- | --- | --- |
| `azure_kinect` | the camera node | Captures per second, drops, restarts, rate and time per stream, IMU rate, temperature, firmware. |
| `body_tracking` | `durchblick-k4abt` | Rates, latency, unpaired frames, busy frames, rate-limited frames, settings. |
| `point_cloud` | Durchblick's point cloud node | The status of that node. |

The status `azure_kinect` uses the serial number of the device as `hardware_id`. Its level is WARN after drops, after
restarts, or while the device delivers less than 90 % of its frame rate. Its level is ERROR while no captures arrive.
[parameters.md](parameters.md#diagnostics) lists the further values.

`blick diagnostics robot --connect <camera host>` prints the diagnostics.

### Bodies

`durchblick-k4abt` publishes its results as JSON on `<base>/bodies`. The results use the
model `K4ABT_BODY_32` with its 32 joints, millimetres, and the depth camera's frame. Each result
carries `stamp_ns`, the stamp of the depth frame that the tracker worked on.

## CameraInfo

Every image stream carries its `CameraInfo`. The values come from the calibration of the device or from a calibration
file.

| Streams | Distortion model | D |
| --- | --- | --- |
| `color`, `depth`, `ir`, `depth_to_color` | `rational_polynomial` | k1 k2 p1 p2 k3 k4 k5 k6 |
| `color_rect`, `depth_to_color_rect` | `plumb_bob` | no distortion, and K equals P |

The stamp of a frame is the device timestamp of the image in wall-clock time. [timestamps.md](timestamps.md) explains
how the node computes it.

## Calibration files

A calibration file replaces the intrinsics of the device, like the `camera_info_url` of a ROS driver.

| Option | Camera |
| --- | --- |
| `--color-calibration FILE` | colour |
| `--depth-calibration FILE` | depth, which is also the camera of the IR image |

Each file is a ROS calibration YAML as `camera_calibration` writes it. Durchblick reads it with
`CameraInfo::read_file` in C++ and `CameraInfo.read_file` in Python.

The node puts K and D of the file into the SDK's calibration. `depth_to_color`, the body tracker and the point clouds
therefore use them. The tracker receives them with the calibration record. The streams carry the `CameraInfo` of the
file with its own P.

The node stops at its start unless the file meets all of these conditions:

- The image size is the size of the camera's mode.
- The model is `rational_polynomial` with 8 coefficients, which is the SDK's model, or `plumb_bob` with 5.
- The matrix K has no skew.
- R is the identity.

The extrinsics in `tf_static` stay the values of the device.

## Rectified colour

Each pixel of `color_rect` shows the ray of the camera that P describes. The node takes its value from the colour
image at the place where the lens distortion puts that ray. It interpolates bilinearly. A pixel is black where that
place is outside the image. OpenCV's `initUndistortRectifyMap` with R as the identity computes the same map.

With the calibration of the device, P equals K, so the focal length and the centre stay the same. A calibration file
has its own P. `camera_calibration` sets that P from its scaling parameter alpha.

The node computes the map once at the start. Per frame it decodes the colour image once for `color` and `color_rect`.
It then remaps the image with NPP's `nppiRemap` on the GPU when `--cuda-device` names a GPU, and on the CPU otherwise.

The `offline` test checks `color_rect` against the SDK's own projection (`k4a_calibration_3d_to_2d`). It uses the
calibration of the device and calibration files, on the CPU and on the GPU. Every checked pixel is within 1 of the
colour image at the SDK's point. Every pixel is black where that point is outside the image.
[performance.md](performance.md#rectified-colour) has the cost.

## Point clouds

This node computes no point clouds. Durchblick's point cloud node `durchblick-points` reads the depth and colour
streams like any other reader and publishes `<base>/points`.

```bash
# on the camera's host: clouds in the depth camera's geometry
durchblick-points --depth robot/azurekinect/depth --color robot/azurekinect/color
# clouds in the colour camera's geometry
durchblick-points --depth robot/azurekinect/depth_to_color --color robot/azurekinect/color \
  --output robot/azurekinect/points_color
```

| Property | Value |
| --- | --- |
| Message | `sensor_msgs/PointCloud2` with x, y, z and rgb |
| Layout | organized like the depth image |
| Frame | the depth camera's frame |
| Colour | from `color` |
| When | only while a reader exists |

With `depth_to_color` as its depth stream, the node makes a cloud in the colour camera's geometry. That cloud has
1280 x 720 points at 720P, and every point has the colour of its own pixel.

The `offline` test checks the clouds against the SDK. The point cloud node is a separate process that reads this
node's streams. In CUDA builds it reads them from the CUDA rings.

| Check | Result |
| --- | --- |
| Position against `k4a_calibration_2d_to_3d` | every point, at most 0.012 mm apart |
| Colour against the pixel that `k4a_calibration_3d_to_2d` gives | equal for 99.96 % of the points |
| Colour with `depth_to_color` as depth stream, on the recording | equal for all points |

Durchblick's
[point cloud guide](https://github.com/xblick/durchblick/blob/main/docs/guides/point-clouds.md) has
the details of that node.

## Viewing

A viewer that connects to the router of the camera host receives every program there: the camera, the tracker and the
point cloud node.

```bash
blick list --connect <camera host>
blick rerun robot/azurekinect/color robot/azurekinect/depth robot/azurekinect/ir \
  robot/azurekinect/depth_to_color --points robot/azurekinect/points --spawn --connect <camera host>
```

Name the streams before `--connect`. `blick rerun` shows the following:

- colour, depth and `depth_to_color` as images, and the two depth streams also as point clouds in 3D,
- the IR image,
- the point clouds of `<base>/points`, a few per second,
- the bodies as skeletons in 3D and in every image,
- IMU and diagnostics as time series and as a text log.

The viewer uses `tf_static` to place the colour camera beside the depth camera. Durchblick's
[viewer guide](https://github.com/xblick/durchblick/blob/main/docs/guides/viewer.md) describes the
viewer.

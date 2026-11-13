# Parameters of durchblick-azure-kinect

This page lists the camera's parameters and explains how you set them at the start and at run time. It also lists the
values that the node reports in its diagnostics.

The parameters follow the model of ROS 2. The node uses Durchblick's `durchblick::Parameters` and declares each
parameter with an `rcl_interfaces/msg/ParameterDescriptor` in `src/settings.hpp`. Durchblick's
[parameters reference](https://github.com/xblick/durchblick/blob/main/docs/reference/parameters.md)
describes the services, the keys, the params files and the checks.

## The parameters

| Parameter | Type | Set at | Meaning |
| --- | --- | --- | --- |
| `color.exposure_time_us`, `color.auto_exposure` | integer (us), bool | run time | The exposure. The camera rounds it to a supported value, at most the frame period. |
| `color.white_balance_k`, `color.auto_white_balance` | integer (K), bool | run time | The white balance. |
| `color.brightness`, `color.contrast`, `color.saturation`, `color.sharpness`, `color.backlight_compensation`, `color.gain` | integer | run time | The other colour controls. A control has an `auto_*` parameter only where the device has that mode. |
| `color.powerline_frequency` | integer | run time | 1 means 50 Hz, 2 means 60 Hz. |
| `fps` | integer (5, 15, 30) | start | The frame rate. |
| `depth.mode` | string | start | The depth mode, for example `NFOV_UNBINNED`. |
| `color.resolution` | string | start | The colour resolution, for example `720P`. |
| `depth.delay_off_color_us` | integer, within one frame period | start | How long the middle of the depth exposure follows the middle of the colour exposure. It is the SDK's `depth_delay_off_color_usec`. |
| `synchronized_images_only` | bool | start | The device delivers only captures with both images. |
| `disable_streaming_indicator` | bool | start | The white LED stays off. |
| `clock.offset_estimator` | string (`mean`, `min`) | start | The estimator of the clock offset. See [timestamps.md](timestamps.md). |
| `clock.window_s` | double (s) | start | The window of the `min` estimator. |
| `clock.fixed_delay_us` | integer (us) | start | The fixed delay that the node subtracts from every stamp. |

The device reports the ranges and defaults of the colour controls (`k4a_device_get_color_control_capabilities`).

When you set the value of a control, the control becomes manual, and its `auto_*` parameter becomes false.

The device keeps its controls until somebody power-cycles it. The node therefore sets every control at the start. It
uses the value you gave, and the SDK's default otherwise. The exception is `color.brightness`, which the node sets to
128.

The node applies the start values in the order of declaration. It applies the value of a control before its `auto_*`
mode, so a control whose default is automatic starts automatic.

A start parameter is `read_only`. A set at run time returns "read-only: applies at the next start".

A recording has `fps`, `depth.mode` and `color.resolution` as somebody recorded it, and it has no controls.

## Set parameters at the start

| Option | Effect |
| --- | --- |
| `--params-file FILE` | Read a ROS params file. You can repeat the option. |
| `--param NAME:=VALUE...` | Set values. A value is a YAML scalar, which the node reads for the type of the parameter. These values win over the files. |
| `--dump-params FILE` | Write the values in effect as a params file. |
| `--params-read-only` | Serve the parameters without accepting changes. Use it for a camera that others rely on. |
| `--fps`, `--depth-mode`, `--color-resolution`, `--brightness`, `--clock-offset` | Short forms for `fps`, `depth.mode`, `color.resolution`, `color.brightness` and `clock.offset_estimator`. |

In a params file the node is `/<base>`, for example `/robot/azurekinect`. The key `/**` matches any node.

```yaml
/robot/azurekinect:
  ros__parameters:
    fps: 30
    color:
      exposure_time_us: 10000   # manual exposure: color.auto_exposure becomes false
      white_balance_k: 4500
    clock:
      offset_estimator: min
```

The node stops at the start and names the parameter in each of these cases:

- The node does not have the parameter.
- The value has another type.
- The value is out of range.
- The value is not one of the choices.

## Set parameters at run time

The node serves ROS 2's parameter services as Zenoh queryables below the base. Request and response are CDR-encoded
`rcl_interfaces/srv/*` messages. The bytes are identical to those in rosbags, and `tests/test_parameters.cpp` checks
the core's types again.

| Key | Content |
| --- | --- |
| `<base>/list_parameters` | service |
| `<base>/describe_parameters` | service |
| `<base>/get_parameters` | service |
| `<base>/set_parameters` | service |
| `<base>/set_parameters_atomically` | service |
| `<base>/get_parameter_types` | service |
| `parameter_events` | Each change as `rcl_interfaces/msg/ParameterEvent`. All nodes publish on this one key, as on ROS's `/parameter_events`. |
| `<base>/parameters` | A liveliness token that shows the node. |

The client works like `ros2 param`:

```bash
durchblick-azure-kinect-parameters --node robot/azurekinect --list
durchblick-azure-kinect-parameters --node robot/azurekinect --describe color.exposure_time_us
durchblick-azure-kinect-parameters --node robot/azurekinect --get
durchblick-azure-kinect-parameters --node robot/azurekinect --set color.exposure_time_us:=8000 color.gain:=10
```

Durchblick's Python command `blick param` does the same, and so does the Parameters tab of Durchblick's
terminal interface. The C++ client here is for a camera PC that has only the C++ part of Durchblick.

## How the descriptor expresses our needs

| Need | In ROS's descriptor |
| --- | --- |
| The parameter needs a restart. | `read_only`. In ROS such a parameter is set only at the start. |
| Unit and choices | `additional_constraints` over an integer or string type, for example `unit: Hz; choices: 5, 15, 30`. |
| Range | `IntegerRange` or `FloatingPointRange` with a step. |
| Automatic mode | A bool parameter `auto_*` beside the value. |

ROS defines a range as `from + n * step`. The exposure values of the SDK form no such grid, and the default 15625 is
off it. The node therefore gives the step of the exposure as 1.

## Diagnostics

Besides rates and drops, the status `azure_kinect` on `<base>/diagnostics` contains the following values.

| Value | Meaning |
| --- | --- |
| every parameter | its current value |
| "parameter changes" | the number of changes, and the last change |
| "colour - depth stamp ms" | median, minimum and maximum of the difference between the device timestamps of colour and depth |
| "colour exposure ms" | the exposure of the colour image |
| "depth exposure ms" | the exposure of the depth image. It is the SDK's value, or the value of the hardware specification for the mode, which the node marks "(spec)". |
| "predicted if start-aligned ms" | (colour exposure - depth exposure) / 2 |
| "clock estimator" | `mean` or `min` |
| "clock offset s" | the current offset |
| "arrival delay p50/p95/max ms" | arrival time - device timestamp - offset, which is the jitter that the estimator sees |
| "clock snaps" | how often the `mean` estimator set the offset directly |
| "clock resets" | how often the `min` estimator restarted |

The node computes the exposure values per report period from the captures that have colour and depth. Exposures that
are aligned at their centres give a stamp difference of 0, and this is the SDK's default. Exposures that are aligned
at their starts give the predicted value.

Colour and depth share one clock. With both streams, a constant difference of their delays therefore shows in the
arrival delay.

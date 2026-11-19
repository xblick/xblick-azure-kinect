# xblick-azure-kinect

[![xblick core](https://img.shields.io/pypi/v/xblick?label=xblick%20core&color=blue)](https://github.com/xblick/xblick)
[![checks](https://github.com/xblick/xblick-azure-kinect/actions/workflows/checks.yml/badge.svg)](https://github.com/xblick/xblick-azure-kinect/actions/workflows/checks.yml)
[![licence](https://img.shields.io/badge/licence-Apache--2.0-blue)](LICENSE)
[![platform](https://img.shields.io/badge/platform-Linux%20x86__64-lightgrey)](README.md#requirements)

`xblick-azure-kinect` publishes the colour, depth, IR and IMU data of an Azure Kinect as
[Durchblick](https://github.com/xblick/xblick) streams, at up to 30 captures per second.
Body tracking is a separate repository: [xblick-k4abt](https://github.com/xblick/xblick-k4abt) reads these streams
and publishes the bodies that the Azure Kinect Body Tracking SDK finds.

The node produces each representation of a stream only while a subscriber asks for it. It needs no X server, so it
runs on a headless camera PC.

Installed from its Python wheel, the node needs one setup command after the install, `xblick-azure-kinect-setup`. It
downloads Microsoft's depth engine, which is under Microsoft's licence terms and not part of the wheel
([Optional: the Python wheel](https://github.com/xblick/xblick-azure-kinect#optional-the-python-wheel)).

## What it publishes

The node publishes below one base name. The default base is `cam/azurekinect`.

| Topic | Type | Content |
| --- | --- | --- |
| `<base>/color` | image, `rgb8` | The colour camera. JPEG subscribers get the camera's own JPEG bytes. |
| `<base>/color_rect` | image, `rgb8` | The colour image without lens distortion. |
| `<base>/depth` | image, `16UC1`, mm | The depth camera in its own geometry (640 x 576 in NFOV unbinned). |
| `<base>/ir` | image, `mono16` | The IR image of the depth camera, with the stamp of the depth frame. |
| `<base>/depth_to_color` | image, `16UC1`, mm | The depth as the colour camera sees it. |
| `<base>/depth_to_color_rect` | image, `16UC1`, mm | The depth as the camera of `color_rect` sees it. |
| `<base>/imu` | `sensor_msgs/Imu` | One message per IMU sample, about 1600 per second. |
| `<base>/tf_static` | `tf2_msgs/TFMessage` | The poses of the colour camera and the IMU in the depth camera's frame. |
| `<base>/k4a_calibration` | JSON, queryable | The calibration record of the device. |
| `<base>/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Rates, drops, restarts, temperature and settings, about once a second. |

Every image stream carries its `CameraInfo`. Durchblick's point cloud node computes coloured point clouds from these
streams on `<base>/points`. [xblick-k4abt](https://github.com/xblick/xblick-k4abt) publishes the tracked bodies on
`<base>/bodies`. [docs/streams.md](docs/streams.md) describes every stream, the calibration files and the point
clouds.

## Requirements

| Need | For |
| --- | --- |
| Linux on x86-64 | everything |
| CMake 3.24 or newer, Ninja, a C++20 compiler | the build |
| Durchblick 0.5.5 or a later 0.5, C++ package | everything |
| zenoh-c and zenoh-cpp 1.10 | everything |
| Azure Kinect Sensor SDK 1.4 with `k4arecord` and the depth engine | everything |
| nlohmann_json, libjpeg or libjpeg-turbo, libpng | everything |
| NVIDIA GPU and driver with EGL, GLVND's `libGL` and `libX11` | the depth engine without an X server |
| CUDA toolkit 12 or newer with nvJPEG and NPP | optional: colour decoding on the GPU, CUDA rings, `xblick-points` |

Every program connects to the Zenoh router of its PC (`zenohd` on port 7447). Durchblick's
[sessions page](https://github.com/xblick/xblick/blob/main/docs/concepts/sessions.md) explains
the routers.

## Install

`tools/setup.sh` installs the prerequisites, builds the programs, runs the tests and installs the programs in
`install/`. It does not use `sudo`.

```bash
git clone https://github.com/xblick/xblick-azure-kinect.git
cd xblick-azure-kinect
tools/setup.sh
```

- If Ubuntu packages are missing, the script stops and prints the `sudo apt install` command for them.
- If the udev rule for the camera is missing, the script prints the commands that install it and goes on. Without
  the rule, only root can open the camera.
- The script builds the CUDA parts when `nvidia-smi` lists a GPU. They need a CUDA toolkit, 12 or newer, at
  `/usr/local/cuda` or at `--cuda-toolkit DIR`. `--no-gpu` builds without them.
- A second call does only what changed. `tools/setup.sh --help` lists the options.
- Body tracking has its own setup script in [xblick-k4abt](https://github.com/xblick/xblick-k4abt).

The script keeps the prerequisites in Durchblick's cache directory, by default `~/.cache/xblick/azure-kinect`.
Every download has a pinned version and a sha256 checksum. Microsoft's Sensor SDK comes from Microsoft's package
repository and is under Microsoft's licence terms. The script prints the name of the terms and their link when it
starts that download.
[docs/build.md](docs/build.md) lists every download with its source and terms, the manual build with CMake and what
each test checks.

### Optional: the Python wheel

The wheel `xblick-azure-kinect` holds the two programs, built, with the libraries that they need. It needs no
compiler and no CUDA toolkit. Install a wheel file, built by `tools/build_wheel.sh` or by the CI (artifact
`xblick-azure-kinect-wheel`), with `uv` or `pip`:

```bash
uv tool install --with-executables-from xblick ./xblick_azure_kinect-0.4.1-py3-none-manylinux_2_28_x86_64.whl
xblick-azure-kinect-setup     # once per PC: the depth engine from Microsoft, and what needs sudo
xblick router &               # once per PC, unless a router runs already
xblick-azure-kinect
```

- `xblick-azure-kinect-setup` downloads Microsoft's package `libk4a1.4` 1.4.1 with the pinned sha256 that
  `tools/setup.sh` uses, prints the name and link of Microsoft's terms, and unpacks the depth engine into
  `~/.cache/xblick/azure-kinect/depth-engine-1.4.1/`. The node loads it from there at its start. Without it, the node
  stops and names the setup command. `XBLICK_HOME` moves the directory, as for every node.
- The setup command prints the `sudo` commands for what is missing: the udev rule for the camera and the system
  libraries `libgl1`, `libx11-6`, `libegl1` and `libudev1`.
- The wheel needs Linux on x86-64 with glibc 2.28 or newer, Python 3.10 to 3.13, and an NVIDIA GPU with its driver,
  525 or newer, for EGL and CUDA 12. NVIDIA's nvJPEG, NPP and CUDA runtime come from PyPI as dependencies of the
  wheel, about 290 MB to download, most of it NPP.
- `--cuda-device 0` is the default, as in a CUDA build. Without a usable GPU, the node stops with one line;
  `--cuda-device none` decodes the colour on the CPU.
- The wheel contains no SDK tools such as `k4arecorder`, no `xblick-points` and not the CMake package `xblick_k4a`.
  These come with `tools/setup.sh`.
- `pip install ./xblick_azure_kinect-<version>-py3-none-manylinux_2_28_x86_64.whl` in a venv installs the same
  commands. `xblick` comes with it as a dependency.

## Run

In each new shell, put the programs on `PATH` first. Start one Zenoh router per PC, unless one runs already.

```bash
source install/env.sh
zenohd --no-multicast-scouting -l tcp/0.0.0.0:7447 &
```

Start the camera node. It prints `ready` when the streams are up.

```bash
xblick-azure-kinect
```

The other programs read the streams of that node.

```bash
# a recording of k4arecorder in place of the camera
xblick-azure-kinect --playback take.mkv --loop
# body tracking: xblick-k4abt from https://github.com/xblick/xblick-k4abt
xblick-k4abt
# coloured point clouds on cam/azurekinect/points, with Durchblick's point cloud node (CUDA builds)
xblick-points --depth cam/azurekinect/depth --color cam/azurekinect/color
# a viewer on another host
xblick rerun cam/azurekinect/color cam/azurekinect/depth cam/azurekinect/ir \
  cam/azurekinect/depth_to_color --points cam/azurekinect/points --spawn --connect <camera host>
```

Most users change these options:

| Option | Default | Effect |
| --- | --- | --- |
| `--base NAME` | `cam/azurekinect` | The base name of all topics. |
| `--cuda-device N\|none` | `0` in CUDA builds | The GPU that decodes the colour image and holds the CUDA rings. |
| `--color-resolution MODE` | `720P` | The colour resolution. |
| `--depth-mode MODE` | `NFOV_UNBINNED` | The depth mode. |
| `--params-file FILE`, `--param NAME:=VALUE` | none | Start values of the parameters. |
| `--color-calibration FILE`, `--depth-calibration FILE` | none | Calibration files that replace the device's intrinsics. |

`--help` lists every option of a program, and a program stops at an unknown option. Every option of the camera is
also a read-only parameter of the node `<base>`. Durchblick's terminal interface (`xblick`, tab Parameters) shows them, and
`--param NAME:=VALUE` or `--params-file FILE` set them at the start.
[docs/options.md](docs/options.md) lists every option. [docs/parameters.md](docs/parameters.md) lists the camera
settings that you can change while the node runs, such as exposure and white balance.

## Why it is fast

At 1080P, decoding a colour frame on the CPU takes 4.39 ms. This node spends 0.006 ms per frame for a JPEG
subscriber and 1.20 ms for a subscriber that reads pixels on the GPU.

| Change | Effect |
| --- | --- |
| The node passes the camera's MJPEG bytes on unchanged. | A JPEG subscriber costs no decoding. |
| nvJPEG decodes the colour image into GPU memory. | 1080P pixels cost 1.20 ms, against 4.39 ms on the CPU. |
| The node computes `depth_to_color` on the CPU. | The camera keeps 30 captures per second on a GPU that other programs use. |
| Four threads publish the streams in parallel. | A slow upload of one stream does not delay the next capture. |

[docs/performance.md](docs/performance.md) has every measurement with its setup.

## Limits

- Nobody has measured the fixed delay between the exposure and the arrival of an image. Stamps are late by that delay.
- The calibration files replace the intrinsics only. The extrinsics stay the factory values.
- One program can open a device. No other driver may open the same camera.

[docs/hardware-checks.md](docs/hardware-checks.md) is the checklist for a session with the camera attached.

## Documentation

| Page | Content |
| --- | --- |
| [docs/streams.md](docs/streams.md) | Every stream in detail, calibration files, rectified colour, point clouds, viewing. |
| [docs/options.md](docs/options.md) | Every command-line option of the two programs. |
| [docs/parameters.md](docs/parameters.md) | The camera's parameters, the parameter services, the client, the diagnostics. |
| [docs/timestamps.md](docs/timestamps.md) | How the node computes the stamp of a frame. |
| [docs/performance.md](docs/performance.md) | Cost per frame and per capture. |
| [docs/design.md](docs/design.md) | How the node works inside, the decisions and the special cases of the Azure Kinect. |
| [docs/build.md](docs/build.md) | Dependencies, CMake options, tests. |
| [docs/hardware-checks.md](docs/hardware-checks.md) | The checks that still need a camera. |

Durchblick's concept pages explain the terms that these pages use:
[streams and topics](https://github.com/xblick/xblick/blob/main/docs/concepts/streams-and-topics.md),
[transports](https://github.com/xblick/xblick/blob/main/docs/concepts/transports.md),
[rings](https://github.com/xblick/xblick/blob/main/docs/concepts/rings.md) and
[timestamps](https://github.com/xblick/xblick/blob/main/docs/concepts/timestamps.md).

## Licence and credits

The code of this repository is under the Apache License 2.0 ([LICENSE](LICENSE)). The test calibration
`tests/data/k4a_test_calibration.json` comes from Microsoft's Azure Kinect Sensor SDK, under the MIT licence. The
repository does not contain the Sensor SDK or its closed-source depth engine. You install them separately, and they
have Microsoft's licence terms. [NOTICE](NOTICE) has the details.

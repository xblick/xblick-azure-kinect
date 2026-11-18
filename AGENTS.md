# AGENTS.md

Azure Kinect as Durchblick streams: `xblick-azure-kinect` (camera or MKV playback). Body tracking (`xblick-k4abt`,
reads these streams) is the repository github.com/xblick/xblick-k4abt. C++20, plain CMake project
`xblick_azure_kinect`, installs the CMake package `xblick_k4a` for programs that build on the node. No point clouds here: Durchblick's
generic node computes them. Human documentation: `README.md`, then `docs/` (streams, options, parameters, timestamps,
performance, design, build, hardware-checks). Transport-wide traps (Zenoh sessions and routers, CUDA rings, hanging
tests): `AGENTS.md` of the core repository (github.com/xblick/xblick). Style of all documentation: the core's
`docs/project/writing-docs.md`.

## Commands

```bash
# Everything from scratch (prerequisites in xblick::cache_dir("azure-kinect"), build in build-setup/, ctest, install/
# with install/env.sh). No sudo: it prints the apt command and stops. Options: --help. Details: docs/build.md.
tools/setup.sh [--cuda | --no-gpu] [--cuda-toolkit DIR]
# By hand. Build on a local disk, never on a shared filesystem. <...> = install prefixes. Options: docs/build.md.
cmake -S . -B build -G Ninja -DCMAKE_PREFIX_PATH="<xblick>;<zenoh>;<k4a>/usr;<nlohmann_json>" \
  -DXBLICK_K4A_CUDA=ON -DCUDAToolkit_ROOT=<cuda-12.8>
cmake --build build
ctest --test-dir build --output-on-failure   # no LD_LIBRARY_PATH needed with the SDK laid out as Microsoft's package
cmake --install build --prefix <xblick_k4a>   # the CMake package xblick_k4a (xblick_k4a_DIR)
tools/check_references.sh                         # the reference check of the CI
```

Durations: not recorded. Give `ctest` a timeout.

- Tests need no camera: `offline`, `parameters`. Content per test: docs/build.md.
- Benchmark: `tools/k4a_node_bench.cpp` (CUDA builds).

## Rules

- Fail fast. A wrong calibration file, an unknown option, a missing depth engine or an unsupported GPU stops the
  program with the cause.
- The camera program defines its options with Durchblick's `xblick::Options` (`--help` lists them), and every option
  is also a parameter of the node. Keep the header comment of `src/main_camera.cpp` and docs/options.md in sync with
  them.
- Read docs/design.md before you change build or run-time details. Each decision has a reason there (EGL depth engine,
  RPATH, diagnostics). Keep that table current. Do not undo a decision without reading its
  reason.
- Run-time libraries come from RPATH. Never put them on a workspace-wide `LD_LIBRARY_PATH`: that would shadow, for
  example, PyTorch's CUDA libraries for every program of the workspace. The depth engine, which `libk4a` opens
  by name, comes through libk4a's own RUNPATH `$ORIGIN/libk4a1.4` (the layout of Microsoft's package); only a
  depth engine elsewhere needs `LD_LIBRARY_PATH`. `install/env.sh` sets PATH only.
- `tools/setup.sh` pins every download (file, URL, sha256 in `pins`). A new version needs its sha256 and the table
  in docs/build.md. No sudo in it: what needs root it checks and prints. Microsoft's and NVIDIA's binaries: print the
  terms' name and link when the download starts (no accept flag).
- A capture failure ends the program.
- The calibration record has version 2. Camera nodes and trackers of another version refuse each other. xblick-k4abt
  reads it with a copy of `src/calibration.*`: a change of the record needs the same change there.
- Documentation: Markdown only in `README.md` and `docs/`, one purpose per page, history in commit messages.

## Traps

| Trap | Symptom | Fix |
| --- | --- | --- |
| Depth engine without X | SDK error 204. Without the plugin the SDK's transformation returns a null handle, no error. | `libxblick_k4a_egl.so` (`src/egl_glx.cpp`) provides the 15 Xlib/GLX functions with EGL on the NVIDIA GPU. The node checks the handle. |
| RPATH lost at install | Installed programs do not find CUDA or the SDK. | `CMAKE_INSTALL_RPATH_USE_LINK_PATH` must stay on. |
| MJPEG without Huffman tables | nvJPEG and other decoders fail. | The node inserts libjpeg's standard tables. Keep that step. |
| Body tracking build switch | `-DXBLICK_K4A_BODY_TRACKING=ON` stops the configuration. | Body tracking is the repository xblick-k4abt since 0.4.0. |
| Two programs on one device | Only one program can open a device. | No other driver may open the same camera. |
| CUDA ring in its own process | The process that created a CUDA ring cannot open it. | The `offline` test reads it from `test_k4a_cuda_reader`, a separate process. |
| NFS | Do not build or write large outputs on NFS. | Build and write large outputs on a local disk. |

## Layout

| Path | Content |
| --- | --- |
| `src/main_camera.cpp`, `node.*`, `source.*` | camera/playback program, publishing, capture loop |
| `src/settings.*`, `main_parameters.cpp` | parameters, the parameter client |
| `src/calibration.*`, `clock.*`, `rectify.*` | calibration files, device clock, `color_rect` |
| `src/depth_to_color.*` | `depth_to_color(_rect)` on the CPU (SDK's transformation only with `--sdk-depth-to-color`) |
| `src/jpeg.*`, `nvjpeg_decoder.*` | MJPEG handling, nvJPEG decoding |
| `src/egl_glx.cpp` | EGL stand-in for the depth engine's Xlib/GLX |
| `tests/`, `tests/data/k4a_test_calibration.json` | `offline`, `parameters` tests |
| `tools/` | `setup.sh`, benchmark, `depth_to_color` check and mirror, `check_references.sh` |
| `docs/` | human documentation |

## Defaults

- Base: `cam/azurekinect`. Topics below it: `color`, `color_rect`, `depth`, `ir`, `depth_to_color`,
  `depth_to_color_rect`, `imu`, `tf_static`, `k4a_calibration`, `diagnostics`. `points` comes from
  `xblick-points`, `bodies` from `xblick-k4abt` (other repository).
- Mode: `fps` 30, `depth.mode` NFOV_UNBINNED, `color.resolution` 720P, `color.brightness` 128,
  `clock.offset_estimator` mean. Ring slots per stream: 32.
- `--cuda-device`: 0 in CUDA builds, else none. `--depth-engine-gpu`: the CUDA device, else 0.
- Frames: `AzureKinectDepthCamera`, `AzureKinectCamera`, `AzureKinectImu`. `source`: `cam`, with playback
  `recording`.
- Zenoh: the PC's router at 127.0.0.1:7447, or `--connect`, `--listen`, `--zenoh-config`.
- Standalone start: `source install/env.sh`, `zenohd`, then `xblick-azure-kinect [--cuda-device 0]` (or `--playback take.mkv --loop`),
  `xblick-points --depth <base>/depth --color <base>/color`.
  Look with `xblick rerun ... --spawn` and `xblick diagnostics cam`.
- Checks that need a camera: docs/hardware-checks.md.

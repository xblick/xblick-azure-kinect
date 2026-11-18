# Build and test xblick-azure-kinect

This page lists the dependencies, what `tools/setup.sh` installs and from where, the manual build with CMake, the
CMake options and the tests.

## Dependencies

| Dependency | Needed for |
| --- | --- |
| CMake 3.24 or newer (Durchblick needs it), Ninja, a C++20 compiler | the build |
| Durchblick's C++ package (`find_package(xblick 0.5.5)`), version 0.5.5 or a later 0.5 | everything |
| zenoh-c and zenoh-cpp 1.10 | everything |
| Azure Kinect Sensor SDK 1.4 (`find_package(k4a)`, `k4arecord`) with the depth engine | everything |
| nlohmann_json | everything |
| libjpeg or libjpeg-turbo, libpng (Durchblick needs it), EGL | everything |
| CUDA toolkit 12 or newer with nvJPEG and NPP | optional: decoding and rectification on the GPU, `xblick-points` |

Microsoft ships the depth engine as a binary package under its own licence terms. This repository does not contain
it. Body tracking is the repository [xblick-k4abt](https://github.com/xblick/xblick-k4abt).

The programs use the Zenoh router of the PC as their default session. When Durchblick has its point cloud node, the
`offline` test also checks that node against the SDK.

## The setup script

`tools/setup.sh` installs everything in two places. It does not use `sudo` and writes nothing outside them.

| Place | Content |
| --- | --- |
| Durchblick's cache directory `cache_dir("azure-kinect")`: `$XBLICK_HOME/cache/azure-kinect`, else `$XDG_CACHE_HOME/xblick/azure-kinect`, else `~/.cache/xblick/azure-kinect` | the prerequisites, one directory per package and version; `downloads/` with the downloaded files; `src/` and `build/` of the packages that the script builds |
| the repository | `build-setup/`, the build of the node; `install/`, the installed programs and `install/env.sh` |

`install/env.sh` puts the node's programs, Durchblick's `xblick-points` and `zenohd` on `PATH`. It sets no
`LD_LIBRARY_PATH`, because the programs find every library through their RPATH.

The script downloads these files. It checks each one against its sha256 and stops on a mismatch.

| Package | Version | Source | Terms |
| --- | --- | --- | --- |
| zenoh-c, standalone build | 1.10.1 | github.com/eclipse-zenoh/zenoh-c, release | EPL 2.0 or Apache 2.0 |
| zenoh-cpp, source | 1.10.1 | github.com/eclipse-zenoh/zenoh-cpp, tag | EPL 2.0 or Apache 2.0 |
| `zenohd`, the router | 1.10.1 | github.com/eclipse-zenoh/zenoh, release | EPL 2.0 or Apache 2.0 |
| Durchblick, source | 0.5.14 | github.com/xblick/xblick, tag `v0.5.14` | Apache 2.0 |
| nlohmann_json | 3.12.0 | github.com/nlohmann/json, release | MIT |
| `cuda.h` from `nvidia-cuda-runtime` (without CUDA only) | 13.4.92 | PyPI | [NVIDIA CUDA Toolkit EULA](https://docs.nvidia.com/cuda/eula/index.html) |
| `libk4a1.4`, `libk4a1.4-dev`: Sensor SDK with the depth engine | 1.4.1 | packages.microsoft.com, `ubuntu/18.04/prod` | [Microsoft Software License Terms, Azure Kinect Sensor SDK](https://www.nuget.org/packages/Microsoft.Azure.Kinect.Sensor/1.4.1/License) |
| `99-k4a.rules`, the udev rule | 1.4.2 | github.com/microsoft/Azure-Kinect-Sensor-SDK, tag | MIT |

The script prints the terms' name and link when it starts the download of a file that is not under an open-source
licence. It unpacks Microsoft's packages with `dpkg-deb -x` and
does not install them. Microsoft's packages ask for the acceptance of the licence terms only when `dpkg` or `apt`
installs them.
Microsoft's page on the depth engine
([docs/depthengine.md](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/v1.4.2/docs/depthengine.md)) names
the extraction from the package as the way to get the depth engine without installing the package.

A file that is already in `downloads/` under the name that the script uses is not downloaded again. A file obtained
in another way, for example on a PC without internet access, can therefore be put there. The script checks its
sha256 in the same way.

What needs root, the script checks and leaves to you:

| What | Check | When it is missing |
| --- | --- | --- |
| Ubuntu packages: `cmake ninja-build g++ curl unzip xz-utils libjpeg-turbo8-dev libpng-dev libegl-dev libgl-dev libgl1 libx11-6 libudev1` | `dpkg-query` | The script stops and prints `sudo apt install` with the missing packages. |
| A CUDA toolkit, 12 or newer, with nvJPEG and NPP | `<toolkit>/version.json`, `bin/nvcc`, the nvJPEG and NPP files | The script stops and names the package `cuda-toolkit-12-8` of NVIDIA's repository, `--cuda-toolkit DIR` and `--no-gpu`. |
| The udev rule for the camera | a rule for the device `045e:097c` in `/etc/udev/rules.d` or `/lib/udev/rules.d` | The script prints the commands that copy the rule there and goes on. |

| Option | Effect |
| --- | --- |
| none | The script builds the CUDA parts when `nvidia-smi` lists a GPU. |
| `--cuda` | The script always builds the CUDA parts. |
| `--no-gpu` | The script builds without CUDA. The depth engine still needs an NVIDIA GPU with EGL at run time. |
| `--cuda-toolkit DIR` | The CUDA toolkit, 12 or newer. The default is `/usr/local/cuda`. |

With the CUDA parts, the script builds Durchblick with its point cloud node and takes `cuda.h` from the toolkit.
Without them, it takes `cuda.h` from NVIDIA's package `nvidia-cuda-runtime`, as Durchblick's own CI does.

A second call does only what changed. Each package directory has a stamp `.setup-done`. The node is configured
again when the options change, built when a file changed, and tested after each build. The logs of the node's steps
are `build-setup/.setup-configure.log`, `.setup-build.log` and `.setup-ctest.log`.

## Manual build with CMake

Replace each `<...>` with the install prefix of that package.

```bash
cmake -S . -B build -G Ninja -DCMAKE_PREFIX_PATH="<xblick>;<zenoh>;<k4a>/usr;<nlohmann_json>" \
  -DXBLICK_K4A_CUDA=ON -DCUDAToolkit_ROOT=<cuda-12.8>
cmake --build build
ctest --test-dir build --output-on-failure
cmake --install build --prefix <xblick_k4a>   # the CMake package xblick_k4a, for other projects
```

Here `<k4a>` is the directory into which Microsoft's packages were unpacked, or `/` after `apt`
installed them. `libk4a` finds the depth engine through its own `RUNPATH` `$ORIGIN/libk4a1.4`, which is where the
package puts it. If the depth engine is elsewhere, its directory has to be on `LD_LIBRARY_PATH`.

Build in a directory on a local disk. Do not build on a shared filesystem.

## CMake options

| Option | Default | Effect |
| --- | --- | --- |
| `XBLICK_K4A_CUDA` | `OFF` | Decode with nvJPEG and rectify with NPP on the GPU. Needs `CUDAToolkit_ROOT`. |

## Tests

The tests need no camera.

| Test | What it checks |
| --- | --- |
| `offline` | The clock. The calibration, from the SDK's test calibration. The MJPEG handling. The node end to end over Zenoh. nvJPEG against libjpeg, also through the CUDA ring into another process. `depth_to_color` from the depth engine in its EGL context, against the SDK's CPU projection. `color_rect` against the SDK's projection. Durchblick's point cloud node against the SDK's projection. The playback of a recording that the SDK writes from such captures, also in a loop, with its stamps. |
| `parameters` | The parameters, the parameter types and the clock estimators on synthetic delays. |
The `offline` test reads the CUDA ring from `test_k4a_cuda_reader`, a separate process, because the process that
created a CUDA ring cannot open it.

## Tools

| Tool | Purpose |
| --- | --- |
| `tools/k4a_node_bench.cpp` | The cost per capture with subscribers. |
| `tools/k4a_depth_to_color_check.cpp` | Compares a running node's `depth_to_color` with the SDK's transformation. |
| `tools/k4a_depth_to_color_mirror.cpp` | Publishes the other method's `depth_to_color` below another base. |

#!/usr/bin/env bash
# The wheel xblick-azure-kinect for PyPI, tagged manylinux_2_28_x86_64: what the CI's wheel job builds (an optional
# install beside tools/setup.sh). Runs in the image quay.io/pypa/manylinux_2_28_x86_64 (AlmaLinux 8, glibc 2.28):
#
#   dnf install -y --setopt=tsflags= libjpeg-turbo-devel libpng-devel libglvnd-devel systemd-devel openssl-devel \
#       libuuid-devel libX11-devel libXrandr-devel libXinerama-devel libXcursor-devel libXi-devel \
#       gcc-toolset-9-gcc gcc-toolset-9-gcc-c++          # as root; tsflags: with libjpeg-turbo's README.ijg
#   tools/build_wheel.sh <xblick checkout> <out dir> <work dir>
#
# Needs on PATH cargo and rustc of the version zenoh-c's rust-toolchain.toml names (1.97.1 for zenoh-c 1.10.1), and
# network access: the sources of zenoh-c, zenoh-cpp, nlohmann_json and the Azure Kinect Sensor SDK with its git
# submodules (GitHub, chromium.googlesource.com for libyuv), Rust crates, the pip packages nvidia-cuda-runtime (cuda.h
# for Durchblick's C++ library), CMake 3.31 (the SDK's glfw submodule needs a CMake before 4) and the build
# requirements (pyproject.toml: NVIDIA's nvJPEG, for the CUDA module), and NVIDIA's redistributable archives of nvcc
# and the CUDA runtime 12.8 (developer.download.nvidia.com, pinned sha256: nvcc compiles src/gpu_rectify.cu; nothing of
# them goes into the wheel). No GPU, no installed CUDA toolkit,
# no camera. <work dir> must not exist or be empty; <out dir> gets the repaired wheel only;
# <work dir>/lgpl-sources/ gets the source archive of the LGPL libraries in libk4a and libk4arecord (below).
#
# 1. zenoh-c from source: its prebuilt release needs glibc 2.34 (GLIBC_2.34 symbols), more than manylinux_2_28 allows.
# 2. zenoh-cpp's headers, nlohmann_json's headers. 3. Durchblick's C++ library (camera API, static) from
#    <xblick checkout>.
# 4. libk4a and libk4arecord from the Sensor SDK's source (MIT; v1.4.2 with the submodule commits of
#    licenses/k4a-sdk-sources.txt), with GCC 9 (gcc-toolset-9): the SDK's code and its submodules build with -Werror,
#    which newer compilers fail. Not Microsoft's prebuilt libraries: those are under Microsoft's licence terms, whose
#    conditions for distribution a public wheel cannot meet. The depth engine (closed source) is not built or
#    contained: the setup command fetches it on the user's PC.
# 5. The wheel (pyproject.toml). 6. auditwheel repair: libjpeg and libpng of AlmaLinux 8 go into
#    xblick_azure_kinect.libs/ (their licences: licenses/); NVIDIA's libraries stay out (--exclude: the wheel depends
#    on NVIDIA's packages, it does not contain them), and so do libEGL (GLVND: its vendor libraries come with the GPU
#    driver) and libudev (systemd, on every PC that has udev).
# 7. Checks: the platform tag, no NVIDIA file and no depth engine in the wheel, the dependencies on NVIDIA's packages,
#    every library of the programs resolves without LD_LIBRARY_PATH, the module's RPATH, the programs need no CUDA
#    library, --help.
set -euo pipefail

if [ $# -ne 3 ]; then
  echo "usage: $0 <xblick checkout> <out dir> <work dir>" >&2
  exit 2
fi
CORE="$(cd "$1" && pwd)"
OUT="$2"
WORK="$3"
NODE="$(cd "$(dirname "$0")/.." && pwd)"
ZENOH_VERSION="${ZENOH_VERSION:-1.10.1}"
JSON_VERSION=3.12.0
JSON_SHA256=42f6e95cad6ec532fd372391373363b62a14af6d771056dbfc86160e6dfff7aa
K4A_TAG=v1.4.2
K4A_COMMIT=cd2b3e9894b26fbeebfe34a5cab5b26efab0939a
# nvcc and the CUDA runtime's headers for the CUDA module (redistrib_12.8.1.json of NVIDIA's redist directory)
CUDA_REDIST=https://developer.download.nvidia.com/compute/cuda/redist
CUDA_NVCC=cuda_nvcc/linux-x86_64/cuda_nvcc-linux-x86_64-12.8.93-archive.tar.xz
CUDA_NVCC_SHA256=9961b3484b6b71314063709a4f9529654f96782ad39e72bf1e00f070db8210d3
CUDA_CUDART=cuda_cudart/linux-x86_64/cuda_cudart-linux-x86_64-12.8.90-archive.tar.xz
CUDA_CUDART_SHA256=8d566b5fe745c46842dc16945cf36686227536decd2302c372be86da37faca68
PLAT=manylinux_2_28_x86_64
PYTHON=/opt/python/cp312-cp312/bin/python
GCC9=/opt/rh/gcc-toolset-9/root/usr/bin

if [ "${AUDITWHEEL_PLAT:-}" != "$PLAT" ]; then
  echo "run this in quay.io/pypa/${PLAT} (AUDITWHEEL_PLAT is '${AUDITWHEEL_PLAT:-}')" >&2
  exit 1
fi
packages=(libjpeg-turbo-devel libpng-devel libglvnd-devel systemd-devel openssl-devel libuuid-devel libX11-devel
          libXrandr-devel libXinerama-devel libXcursor-devel libXi-devel gcc-toolset-9-gcc gcc-toolset-9-gcc-c++)
for package in "${packages[@]}"; do
  if ! rpm -q "$package" > /dev/null; then
    echo "$package is missing: dnf install -y --setopt=tsflags= ${packages[*]} (as root; tsflags: with README.ijg)" >&2
    exit 1
  fi
done
command -v cargo > /dev/null || { echo "cargo is missing (Rust: see zenoh-c's rust-toolchain.toml)" >&2; exit 1; }
if [ -e "$WORK" ] && [ -n "$(ls -A "$WORK")" ]; then
  echo "work dir $WORK is not empty" >&2
  exit 1
fi
mkdir -p "$WORK" "$OUT"
WORK="$(cd "$WORK" && pwd)"
ZENOH="$WORK/zenoh"
JOBS="$(nproc)"

check_licence() {  # <file of this build> <file in licenses/>: the wheel carries the second, which must be the first
  if ! cmp -s "$1" "$NODE/licenses/$2"; then
    echo "licenses/$2 differs from $1: copy it again" >&2
    exit 1
  fi
}

echo "== zenoh-c ${ZENOH_VERSION} from source"
curl -sSfL -o "$WORK/zenoh-c.tar.gz" \
  "https://github.com/eclipse-zenoh/zenoh-c/archive/refs/tags/${ZENOH_VERSION}.tar.gz"
tar xzf "$WORK/zenoh-c.tar.gz" -C "$WORK"
ZENOHC_SRC="$WORK/zenoh-c-${ZENOH_VERSION}"
toolchain="$(sed -n 's/^channel = "\(.*\)"$/\1/p' "$ZENOHC_SRC/rust-toolchain.toml")"
rustc_version="$(rustc --version | cut -d' ' -f2)"
if [ "$rustc_version" != "$toolchain" ]; then
  echo "rustc is $rustc_version, zenoh-c ${ZENOH_VERSION} names $toolchain (rust-toolchain.toml)" >&2
  exit 1
fi
# the features of zenoh-c's prebuilt release (its zenoh_configure.h): shared memory and the unstable API
cmake -S "$ZENOHC_SRC" -B "$WORK/zenoh-c-build" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$ZENOH" \
  -DZENOHC_BUILD_WITH_SHARED_MEMORY=TRUE -DZENOHC_BUILD_WITH_UNSTABLE_API=TRUE > /dev/null
cmake --build "$WORK/zenoh-c-build" --config Release -j "$JOBS"
cmake --install "$WORK/zenoh-c-build" > /dev/null
# the Rust crates in libzenohc.so and their licence texts: the wheel carries licenses/zenoh-c-crates.txt, which must
# be what this build links
"$PYTHON" "$NODE/tools/zenoh_c_crates.py" "$WORK/zenoh-c-build/release" > "$WORK/zenoh-c-crates.txt"
if ! cmp -s "$WORK/zenoh-c-crates.txt" "$NODE/licenses/zenoh-c-crates.txt"; then
  diff "$NODE/licenses/zenoh-c-crates.txt" "$WORK/zenoh-c-crates.txt" | head -20 >&2 || true
  echo "licenses/zenoh-c-crates.txt is not the list of this build: regenerate it with tools/zenoh_c_crates.py" >&2
  exit 1
fi
check_licence "$ZENOHC_SRC/LICENSE" zenoh-c-LICENSE
check_licence "$ZENOHC_SRC/NOTICE.md" zenoh-c-NOTICE.md
# libjpeg-turbo and libpng of this image go into the wheel (auditwheel): their licences (NOTICE names the versions)
echo "   libjpeg-turbo $(rpm -q --qf "%{VERSION}" libjpeg-turbo), libpng $(rpm -q --qf "%{VERSION}" libpng)"
check_licence /usr/share/licenses/libjpeg-turbo/LICENSE.md libjpeg-turbo-LICENSE.md
check_licence /usr/share/doc/libjpeg-turbo/README.ijg libjpeg-turbo-README.ijg
check_licence /usr/share/licenses/libpng/LICENSE libpng-LICENSE

echo "== zenoh-cpp ${ZENOH_VERSION} (headers), nlohmann_json ${JSON_VERSION} (headers)"
curl -sSfL -o "$WORK/zenoh-cpp.tar.gz" "https://github.com/eclipse-zenoh/zenoh-cpp/archive/refs/tags/${ZENOH_VERSION}.tar.gz"
tar xzf "$WORK/zenoh-cpp.tar.gz" -C "$WORK"
cmake -S "$WORK/zenoh-cpp-${ZENOH_VERSION}" -B "$WORK/zenoh-cpp-build" -DCMAKE_INSTALL_PREFIX="$ZENOH" \
  -DCMAKE_PREFIX_PATH="$ZENOH" -DZENOHCXX_ZENOHPICO=OFF > /dev/null
cmake --install "$WORK/zenoh-cpp-build" > /dev/null
curl -sSfL -o "$WORK/json.tar.xz" "https://github.com/nlohmann/json/releases/download/v${JSON_VERSION}/json.tar.xz"
echo "$JSON_SHA256  $WORK/json.tar.xz" | sha256sum --check --quiet
mkdir "$WORK/json-src"
tar xJf "$WORK/json.tar.xz" -C "$WORK/json-src"
check_licence "$WORK/json-src/json/LICENSE.MIT" nlohmann-json-LICENSE.MIT
cmake -S "$WORK/json-src/json" -B "$WORK/json-build" -DCMAKE_INSTALL_PREFIX="$WORK/json" -DJSON_BuildTests=OFF \
  > /dev/null
cmake --install "$WORK/json-build" > /dev/null

echo "== Durchblick's C++ library"
# cuda.h from the pip package nvidia-cuda-runtime, as Durchblick's own build takes it (libcuda.so.1 is opened at run time)
"$PYTHON" -m venv "$WORK/tools"
"$WORK/tools/bin/pip" install --quiet "nvidia-cuda-runtime>=13,<14" "cmake==3.31.6" "ninja==1.13.0"
CUDA_INCLUDE="$("$WORK/tools/bin/python" -c "import nvidia, os; print(os.path.join(list(nvidia.__path__)[0], 'cu13', 'include'))")"
cmake -S "$CORE" -B "$WORK/core-build" -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$ZENOH" \
  -DXBLICK_CAMERA=ON -DXBLICK_CUDA_INCLUDE_DIR="$CUDA_INCLUDE" -DBUILD_TESTING=OFF > /dev/null
cmake --build "$WORK/core-build" -j "$JOBS"
cmake --install "$WORK/core-build" --prefix "$WORK/core-install" > /dev/null

echo "== Azure Kinect Sensor SDK ${K4A_TAG} (libk4a, libk4arecord) from source"
git -c advice.detachedHead=false clone --quiet --depth 1 --branch "$K4A_TAG" --recurse-submodules \
  --shallow-submodules https://github.com/microsoft/Azure-Kinect-Sensor-SDK.git "$WORK/k4a-src"
K4A_SRC="$WORK/k4a-src"
if [ "$(git -C "$K4A_SRC" rev-parse HEAD)" != "$K4A_COMMIT" ]; then
  echo "the tag $K4A_TAG is $(git -C "$K4A_SRC" rev-parse HEAD), not $K4A_COMMIT" >&2
  exit 1
fi
# the sources the wheel's notice names (licenses/k4a-sdk-sources.txt): the SDK's commit and every submodule's
{
  echo "# The Azure Kinect Sensor SDK $K4A_TAG (commit $K4A_COMMIT, https://github.com/microsoft/Azure-Kinect-Sensor-SDK)"
  echo "# and its git submodules, as tools/build_wheel.sh builds libk4a and libk4arecord from them: commit, path."
  git -C "$K4A_SRC" submodule status --recursive | awk '{print $1, $2}'
} > "$WORK/k4a-sdk-sources.txt"
check_licence "$WORK/k4a-sdk-sources.txt" k4a-sdk-sources.txt
# the licences of the code in libk4a and libk4arecord: the SDK's and the submodules' linked into them
check_licence "$K4A_SRC/LICENSE" Azure-Kinect-Sensor-SDK-LICENSE
check_licence "$K4A_SRC/extern/azure_c_shared/src/LICENSE" azure-c-shared-utility-LICENSE
check_licence "$K4A_SRC/extern/cjson/src/LICENSE" cJSON-LICENSE
check_licence "$K4A_SRC/extern/libjpeg-turbo/src/LICENSE.md" k4a-libjpeg-turbo-LICENSE.md
check_licence "$K4A_SRC/extern/libjpeg-turbo/src/README.ijg" k4a-libjpeg-turbo-README.ijg
check_licence "$K4A_SRC/extern/libusb/src/COPYING" libusb-COPYING
check_licence "$K4A_SRC/extern/libuvc/src/LICENSE.txt" libuvc-LICENSE.txt
check_licence "$K4A_SRC/extern/libyuv/src/LICENSE" libyuv-LICENSE
check_licence "$K4A_SRC/extern/libyuv/src/PATENTS" libyuv-PATENTS
check_licence "$K4A_SRC/extern/libebml/src/LICENSE.LGPL" libebml-LICENSE.LGPL
check_licence "$K4A_SRC/extern/libmatroska/src/LICENSE.LGPL" libmatroska-LICENSE.LGPL
check_licence "$K4A_SRC/extern/spdlog/src/LICENSE" spdlog-LICENSE
check_licence "$K4A_SRC/extern/spdlog/src/include/spdlog/fmt/bundled/LICENSE.rst" fmt-LICENSE.rst
# libusb, libebml and libmatroska (LGPL 2.1) are linked into libk4a and libk4arecord: their source code at the
# commits built, as one archive that each release carries beside the wheel (NOTICE)
version="$(sed -n 's/^project(xblick_azure_kinect VERSION \([0-9.]*\).*/\1/p' "$NODE/CMakeLists.txt")"
sources="xblick-azure-kinect-$version-lgpl-sources"
mkdir -p "$WORK/lgpl-sources/$sources"
for library in libusb libebml libmatroska; do
  commit="$(git -C "$K4A_SRC/extern/$library/src" rev-parse HEAD)"
  git -C "$K4A_SRC/extern/$library/src" archive --format=tar --prefix="$sources/$library-$commit/" HEAD |
    tar xf - -C "$WORK/lgpl-sources"
done
cp "$NODE/licenses/k4a-sdk-sources.txt" "$WORK/lgpl-sources/$sources/"
# how the SDK builds them (its CMake files for the three, MIT): the scripts that control their compilation
for library in libusb libebml libmatroska; do
  mkdir -p "$WORK/lgpl-sources/$sources/sdk-build/$library"
  cp "$K4A_SRC/extern/$library/CMakeLists.txt" "$WORK/lgpl-sources/$sources/sdk-build/$library/"
done
cp "$K4A_SRC/LICENSE" "$WORK/lgpl-sources/$sources/sdk-build/LICENSE"
tar czf "$WORK/lgpl-sources/$sources.tar.gz" -C "$WORK/lgpl-sources" "$sources"
rm -r "$WORK/lgpl-sources/$sources"
echo "   $WORK/lgpl-sources/$sources.tar.gz"
check_licence "$K4A_SRC/scripts/99-k4a.rules" ../python/xblick_azure_kinect/99-k4a.rules
# CMake 3.31: glfw (configured with the rest, not built) sets a policy to OLD that CMake 4 refuses. libuvc compiles
# against the libusb that the SDK links (its own submodule), not against a system libusb. OpenSSL is needed by the
# configuration of azure-c-shared-utility only: libk4a links none of its TLS code.
PATH="$WORK/tools/bin:$PATH" cmake -S "$K4A_SRC" -B "$WORK/k4a-build" -G Ninja -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_C_COMPILER="$GCC9/gcc" -DCMAKE_CXX_COMPILER="$GCC9/g++" \
  -DCMAKE_C_FLAGS="-I$K4A_SRC/extern/libusb/src/libusb" -DCMAKE_INSTALL_PREFIX="$WORK/k4a" \
  -DCMAKE_INSTALL_LIBDIR=lib -DBUILD_TESTING=OFF -DK4A_VALIDATE_CLANG_FORMAT=OFF > "$WORK/k4a-configure.log" ||
  { tail -40 "$WORK/k4a-configure.log" >&2; exit 1; }
PATH="$WORK/tools/bin:$PATH" cmake --build "$WORK/k4a-build" --target k4a k4arecord -j "$JOBS"
for component in runtime development; do
  PATH="$WORK/tools/bin:$PATH" cmake --install "$WORK/k4a-build" --component "$component" > /dev/null
done
for library in libk4a.so.1.4 libk4arecord.so.1.4; do
  test -e "$WORK/k4a/lib/$library" || { echo "the SDK's build installed no $WORK/k4a/lib/$library" >&2; exit 1; }
done
if readelf -d "$WORK/k4a/lib/libk4a.so.1.4" | grep -q "NEEDED.*\[lib\(ssl\|crypto\)"; then
  echo "libk4a links OpenSSL: its licence would have to be in the wheel" >&2
  exit 1
fi

echo "== nvcc and the CUDA runtime 12.8 (NVIDIA's redistributable archives, for the build only)"
mkdir "$WORK/cuda"
for archive in "$CUDA_NVCC|$CUDA_NVCC_SHA256" "$CUDA_CUDART|$CUDA_CUDART_SHA256"; do
  path="${archive%|*}"
  curl -sSfL -o "$WORK/${path##*/}" "$CUDA_REDIST/$path"
  echo "${archive#*|}  $WORK/${path##*/}" | sha256sum --check --quiet
  tar xJf "$WORK/${path##*/}" -C "$WORK/cuda" --strip-components 1
done
# nvcc looks for the runtime in lib64, the archives have lib
ln -s lib "$WORK/cuda/lib64"
"$WORK/cuda/bin/nvcc" --version | tail -1

echo "== the wheel"
CMAKE_PREFIX_PATH="$WORK/core-install:$ZENOH:$WORK/json:$WORK/k4a" "$PYTHON" -m pip wheel --no-deps --no-cache-dir \
  "$NODE" -w "$WORK/wheelhouse" -C build-dir="$WORK/node-build" -C cmake.define.CUDAToolkit_ROOT="$WORK/cuda" \
  -C cmake.define.CMAKE_CUDA_COMPILER="$WORK/cuda/bin/nvcc"
wheel="$(ls "$WORK"/wheelhouse/xblick_azure_kinect-*.whl)"
excluded=(libnvjpeg.so.12 libcudart.so.12 libEGL.so.1 libudev.so.1)
auditwheel repair --plat "$PLAT" "${excluded[@]/#/--exclude=}" -w "$WORK/repaired" "$wheel"
repaired="$(ls "$WORK"/repaired/xblick_azure_kinect-*-py3-none-${PLAT}.whl)"
auditwheel show "$repaired"

echo "== checks"
mkdir "$WORK/check"
(cd "$WORK/check" && "$PYTHON" -m zipfile -e "$repaired" .)
listing="$("$PYTHON" -m zipfile -l "$repaired")"
# no code of NVIDIA (neither its libraries nor anything under nvidia/) and no part of the depth engine in the wheel
if grep -E -i "libnvjpeg|libnpp|libcudart|libcuda\.|(^|/)nvidia/" <<< "$listing"; then
  echo "$repaired contains NVIDIA's files" >&2
  exit 1
fi
if grep -E -i "depthengine|depth_engine" <<< "$listing"; then
  echo "$repaired contains the depth engine" >&2
  exit 1
fi
lib="$WORK/check/xblick_azure_kinect/lib"
for file in libk4a.so.1.4 libk4arecord.so.1.4 libzenohc.so libxblick_k4a_egl.so libxblick-azure-kinect-cuda.so; do
  test -f "$lib/$file" || { echo "the wheel has no xblick_azure_kinect/lib/$file" >&2; exit 1; }
done
for name in xblick-azure-kinect xblick-azure-kinect-parameters; do
  program="$WORK/check/xblick_azure_kinect/bin/$name"
  chmod +x "$program"
  env -u LD_LIBRARY_PATH ldd "$program"
  # the system's libEGL and libudev (excluded above) are not in this image's library path everywhere: not checked
  if env -u LD_LIBRARY_PATH ldd "$program" | grep "not found" | grep -v -E "libEGL\.so\.1|libudev\.so\.1"; then
    echo "$program: a library does not resolve" >&2
    exit 1
  fi
  env -u LD_LIBRARY_PATH ldd "$program" | grep -q "xblick_azure_kinect/bin/../lib/libk4a.so.1.4"
  env -u LD_LIBRARY_PATH ldd "$program" | grep -q "xblick_azure_kinect/bin/../lib/libzenohc.so"
  if env -u LD_LIBRARY_PATH ldd "$program" | grep -E "=> /(usr/)?lib.*/lib(jpeg|png|k4a)"; then
    echo "$program takes libjpeg, libpng or libk4a from the system" >&2
    exit 1
  fi
  if readelf -d "$program" | grep -q "NEEDED.*\[lib\(nvjpeg\|npp\|cudart\|cuda\)"; then
    echo "$program needs a CUDA library: it must open the module instead" >&2
    exit 1
  fi
done
# they start: the camera's --help, the parameter client's line without --node (it has no --help)
env -u LD_LIBRARY_PATH "$WORK/check/xblick_azure_kinect/bin/xblick-azure-kinect" --help > /dev/null
out="$(env -u LD_LIBRARY_PATH "$WORK/check/xblick_azure_kinect/bin/xblick-azure-kinect-parameters" 2>&1 || true)"
if ! grep -q -x "xblick-azure-kinect-parameters: --node <prefix>, e.g. cam/azurekinect" <<< "$out"; then
  echo "$out" >&2
  echo "xblick-azure-kinect-parameters does not start" >&2
  exit 1
fi
# GLVND's libraries come with the system: the EGL shim needs libEGL only
if readelf -d "$lib/libxblick_k4a_egl.so" | grep -q "NEEDED.*\[lib\(OpenGL\|GLdispatch\|GL\)\."; then
  echo "libxblick_k4a_egl.so needs a GLVND library besides libEGL: it would be grafted into the wheel" >&2
  exit 1
fi
if grep -E -i "lib(EGL|OpenGL|GLdispatch|GLX|GL|udev)[-.]" <<< "$listing"; then
  echo "$repaired contains a library that must come from the system" >&2
  exit 1
fi
module="$lib/libxblick-azure-kinect-cuda.so"
for needed in libnvjpeg.so.12 libcudart.so.12; do
  readelf -d "$module" | grep -q "NEEDED.*\[$needed\]" || { echo "$module does not need $needed" >&2; exit 1; }
done
rpath='$ORIGIN/../../nvidia/nvjpeg/lib:$ORIGIN/../../nvidia/cuda_runtime/lib'
if ! readelf -d "$module" | grep -F "(RPATH)" | grep -q -F "[$rpath]"; then
  readelf -d "$module" | grep -E "RPATH|RUNPATH" >&2 || true
  echo "$module: its RPATH is not $rpath" >&2
  exit 1
fi
# the CUDA runtime from libcudart.so.12 (NVIDIA's package), not linked in statically; the remap kernel's own code
if ! nm -D --undefined-only "$module" | grep -q -w cudaMalloc; then
  echo "$module does not take the CUDA runtime from libcudart.so.12: the static runtime would put NVIDIA's code into it" >&2
  exit 1
fi
readelf -S "$module" | grep -q "\.nv_fatbin" || { echo "$module has no kernel binaries (.nv_fatbin)" >&2; exit 1; }
metadata="$(cd "$WORK/check" && cat xblick_azure_kinect-*.dist-info/METADATA)"
for requirement in "xblick<0.6,>=0.5.14" "nvidia-nvjpeg-cu12<13,>=12.3.5" \
                   "nvidia-cuda-runtime-cu12<13,>=12.0"; do
  if ! grep -q -x "Requires-Dist: $requirement" <<< "$metadata"; then
    grep "^Requires-Dist" <<< "$metadata" >&2 || true
    echo "the wheel does not depend on $requirement" >&2
    exit 1
  fi
done
cp "$repaired" "$OUT/"
echo "== $OUT/$(basename "$repaired")"

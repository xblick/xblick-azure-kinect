# Azure Kinect cameras as Durchblick streams (README.md).
#
#   cmake -S . -B build -G Ninja -DCMAKE_PREFIX_PATH="<xblick>;<zenoh>;<k4a SDK>;<nlohmann_json>"
#         [-DXBLICK_K4A_CUDA=ON -DCUDAToolkit_ROOT=<CUDA toolkit> -DCMAKE_CUDA_COMPILER=<CUDA toolkit>/bin/nvcc]
#
# Targets: xblick-azure-kinect (the camera node), xblick-azure-kinect-parameters, test_k4a_offline (no camera needed;
# ctest). Installs the libraries as the CMake package xblick_k4a (xblick_k4a::xblick_k4a), for programs that build on
# the node. Body tracking from the node's streams is the repository xblick-k4abt.
#
# The Python wheel xblick-azure-kinect (pyproject.toml, scikit-build-core; tools/build_wheel.sh) builds the same two
# programs with the CUDA parts in a module and installs them into the package xblick_azure_kinect (below, SKBUILD).
cmake_minimum_required(VERSION 3.24)
project(xblick_azure_kinect VERSION 0.4.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
# installed programs find this project's shared libraries (lib/) and keep the directories of the libraries they link
# (CUDA, the SDKs), as the built ones do
set(CMAKE_INSTALL_RPATH "$ORIGIN/../lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH ON)

option(XBLICK_K4A_CUDA
       "decode (nvJPEG) and rectify (src/gpu_rectify.cu) the colour camera's MJPEG on the GPU (CUDA toolkit with nvcc)"
       OFF)
# body tracking left this repository with 0.4.0: stop instead of building without the tracker that was asked for
if(XBLICK_K4A_BODY_TRACKING)
  message(FATAL_ERROR "XBLICK_K4A_BODY_TRACKING: body tracking (xblick-k4abt) is the repository xblick-k4abt since "
                      "xblick-azure-kinect 0.4.0 (https://github.com/xblick/xblick-k4abt)")
endif()
if(SKBUILD AND XBLICK_K4A_CUDA)
  message(FATAL_ERROR "the wheel builds its CUDA parts as a module (tools/build_wheel.sh): build it without "
                      "XBLICK_K4A_CUDA")
endif()

# The CUDA kernel of the rectified colour (src/gpu_rectify.cu), in the CUDA build and in the wheel: one binary per
# major GPU architecture of the toolkit (all-major; with CUDA 12: 5.0 Maxwell to 12.0 Blackwell, each running on the
# later GPUs of its major version, plus PTX of the newest for later GPUs), unless CMAKE_CUDA_ARCHITECTURES names
# others. The CUDA runtime is the shared libcudart.so.12, never the static one.
if(XBLICK_K4A_CUDA OR SKBUILD)
  if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
    set(CMAKE_CUDA_ARCHITECTURES all-major)
  endif()
  set(CMAKE_CUDA_RUNTIME_LIBRARY Shared)
  enable_language(CUDA)
  set(CMAKE_CUDA_STANDARD 20)
  set(CMAKE_CUDA_STANDARD_REQUIRED ON)
  # all-major includes the architectures before 7.5, which CUDA 12 still builds and announces as deprecated
  add_compile_options($<$<COMPILE_LANGUAGE:CUDA>:-Wno-deprecated-gpu-targets>)
  find_package(CUDAToolkit 12 REQUIRED)
endif()

# Durchblick's 0.5 line from 0.5.5 (its CMake package accepts its own minor version only, so 0.5.5 means 0.5.5 or a
# later 0.5.x): publish_jpeg, pin_host,
# StaticTransformPublisher, Imu, diagnostics, CameraInfo::read_file, Options, Parameters, RVL for depth, ring keys with
# the host, cache_dir (the wheel's depth engine); point clouds (the tests use its point cloud node xblick-points when
# Durchblick was built with it)
find_package(xblick 0.5.5 REQUIRED OPTIONAL_COMPONENTS points)
find_package(k4a 1.4 REQUIRED)
find_package(k4arecord 1.4 REQUIRED)
find_package(nlohmann_json 3 REQUIRED)
find_package(JPEG REQUIRED)
find_package(OpenGL REQUIRED COMPONENTS EGL)

# the depth engine's OpenGL context through EGL, without an X server (src/egl_glx.cpp); shared, so that its Xlib/GLX
# functions are found before libX11's and libGL's
add_library(xblick_k4a_egl SHARED src/egl_glx.cpp)
target_include_directories(xblick_k4a_egl PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
                                                 $<INSTALL_INTERFACE:include/xblick_k4a>)
target_link_libraries(xblick_k4a_egl PRIVATE OpenGL::EGL)
target_compile_options(xblick_k4a_egl PRIVATE -Wall -Wextra)

add_library(xblick_k4a STATIC
  src/clock.cpp
  src/calibration.cpp
  src/jpeg.cpp
  src/source.cpp
  src/depth_to_color.cpp
  src/node.cpp
  src/rectify.cpp
  src/settings.cpp)
target_include_directories(xblick_k4a PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
                                             $<INSTALL_INTERFACE:include/xblick_k4a>)
target_link_libraries(xblick_k4a PUBLIC xblick::xblick k4a::k4a k4a::k4arecord
                                        nlohmann_json::nlohmann_json JPEG::JPEG xblick_k4a_egl)
target_compile_options(xblick_k4a PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-Wall -Wextra>)
if(XBLICK_K4A_CUDA)
  target_sources(xblick_k4a PRIVATE src/nvjpeg_decoder.cpp src/gpu_rectify.cu)
  target_compile_definitions(xblick_k4a PUBLIC XBLICK_K4A_CUDA)
  target_link_libraries(xblick_k4a PUBLIC CUDA::cudart CUDA::nvjpeg)
  # the node's cost per capture with shared-memory and CUDA subscribers on every stream (tools/k4a_node_bench.cpp)
  add_executable(k4a_node_bench tools/k4a_node_bench.cpp)
  target_link_libraries(k4a_node_bench PRIVATE xblick_k4a)
  # the GPU remap (src/gpu_rectify.cu) against NPP's nppiRemap, which the node used up to 0.4.0: pixel for pixel and
  # per frame, on synthetic images and maps (tools/remap_npp_check.cpp); only where the toolkit has NPP
  if(TARGET CUDA::nppig)
    add_executable(k4a_remap_npp_check tools/remap_npp_check.cpp)
    target_link_libraries(k4a_remap_npp_check PRIVATE xblick_k4a CUDA::nppig CUDA::nppc)
  endif()
endif()

add_executable(xblick-azure-kinect src/main_camera.cpp)
target_link_libraries(xblick-azure-kinect PRIVATE xblick_k4a)

# a node's parameters over its ROS 2 parameter services (as ros2 param), the camera's and any other node's
add_executable(xblick-azure-kinect-parameters src/main_parameters.cpp)
target_link_libraries(xblick-azure-kinect-parameters PRIVATE xblick_k4a)

if(SKBUILD)
  # The wheel: xblick_azure_kinect/bin/{xblick-azure-kinect,xblick-azure-kinect-parameters} and in
  # xblick_azure_kinect/lib the libraries they need: libzenohc.so, libk4a.so.1.4 and libk4arecord.so.1.4 (the Sensor
  # SDK built from its MIT-licensed source, tools/build_wheel.sh), libxblick_k4a_egl.so and the CUDA module
  # libxblick-azure-kinect-cuda.so. The programs find them through $ORIGIN/../lib only: no path of the build machine
  # in their RPATH. They need no CUDA library: they open the module when --cuda-device names a GPU (src/wheel.cpp).
  # The module (nvjpeg_decoder.cpp, gpu_rectify.cu) needs libnvjpeg.so.12 and libcudart.so.12, which the wheel does
  # not contain: they come from NVIDIA's pip packages nvidia-nvjpeg-cu12 and nvidia-cuda-runtime-cu12, dependencies of
  # the wheel, installed in the same site-packages (nvidia/{nvjpeg,cuda_runtime}/lib). The module's RPATH names these
  # directories relative to itself, as DT_RPATH, which goes before LD_LIBRARY_PATH: a CUDA toolkit on LD_LIBRARY_PATH
  # does not replace them. The build takes nvJPEG's header and library from its package in the build environment
  # (pyproject.toml, build-system.requires), nvcc and the CUDA runtime's headers from NVIDIA's redistributable
  # archives (CUDAToolkit_ROOT, tools/build_wheel.sh). The depth engine is not in the wheel: its setup command
  # xblick-azure-kinect-setup downloads Microsoft's package and unpacks the engine into the node's cache directory,
  # where the program loads it (src/wheel.cpp). Its version comes from the setup command's pin.
  find_package(Python 3.10 REQUIRED COMPONENTS Interpreter)
  execute_process(
    COMMAND "${Python_EXECUTABLE}" -c "import nvidia.nvjpeg; print(list(nvidia.nvjpeg.__path__)[0])"
    OUTPUT_VARIABLE NVJPEG_DIR OUTPUT_STRIP_TRAILING_WHITESPACE RESULT_VARIABLE nvidia_found ERROR_VARIABLE nvidia_error)
  if(NOT nvidia_found EQUAL 0)
    message(FATAL_ERROR "the package nvidia-nvjpeg-cu12 is not in the build environment (${Python_EXECUTABLE}, "
                        "pyproject.toml build-system.requires): ${nvidia_error}")
  endif()
  foreach(file "${NVJPEG_DIR}/include/nvjpeg.h" "${NVJPEG_DIR}/lib/libnvjpeg.so.12")
    if(NOT EXISTS "${file}")
      message(FATAL_ERROR "${file} is missing: NVIDIA's pip package has another layout than this build expects")
    endif()
  endforeach()

  file(STRINGS python/xblick_azure_kinect/sdk.py engine_pin REGEX "^K4A_PACKAGE_VERSION = \"[0-9.]+\"$")
  string(REGEX REPLACE "^K4A_PACKAGE_VERSION = \"([0-9.]+)\"$" "\\1" engine_version "${engine_pin}")
  if(NOT engine_version MATCHES "^[0-9]+\\.[0-9]+\\.[0-9]+$")
    message(FATAL_ERROR "python/xblick_azure_kinect/sdk.py has no line K4A_PACKAGE_VERSION = \"<version>\"")
  endif()

  add_library(xblick-azure-kinect-cuda MODULE src/nvjpeg_decoder.cpp src/gpu_rectify.cu)
  target_compile_definitions(xblick-azure-kinect-cuda PRIVATE XBLICK_K4A_CUDA_MODULE)
  target_compile_options(xblick-azure-kinect-cuda PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-Wall -Wextra>)
  # rectify.hpp's types come with Durchblick's headers (xblick/msg.hpp); the module links none of its code
  target_include_directories(xblick-azure-kinect-cuda PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src"
                             $<TARGET_PROPERTY:xblick::xblick,INTERFACE_INCLUDE_DIRECTORIES>)
  target_include_directories(xblick-azure-kinect-cuda SYSTEM PRIVATE "${NVJPEG_DIR}/include")
  target_link_libraries(xblick-azure-kinect-cuda PRIVATE "${NVJPEG_DIR}/lib/libnvjpeg.so.12" CUDA::cudart)
  target_link_options(xblick-azure-kinect-cuda PRIVATE -Wl,--no-undefined -Wl,--disable-new-dtags)
  set_target_properties(xblick-azure-kinect-cuda PROPERTIES
    CXX_VISIBILITY_PRESET hidden
    CUDA_VISIBILITY_PRESET hidden
    INSTALL_RPATH "$ORIGIN/../../nvidia/nvjpeg/lib:$ORIGIN/../../nvidia/cuda_runtime/lib"
    INSTALL_RPATH_USE_LINK_PATH OFF)

  target_sources(xblick_k4a PRIVATE src/wheel.cpp)
  target_compile_definitions(xblick_k4a PUBLIC XBLICK_K4A_CUDA XBLICK_K4A_WHEEL
                                        PRIVATE XBLICK_K4A_DEPTH_ENGINE_VERSION="${engine_version}")
  target_link_libraries(xblick_k4a PUBLIC ${CMAKE_DL_LIBS})
  # DT_RPATH (--disable-new-dtags): it also serves the libraries' own needs (libk4arecord needs libk4a)
  foreach(target xblick-azure-kinect xblick-azure-kinect-parameters xblick_k4a_egl)
    target_link_options(${target} PRIVATE -Wl,--disable-new-dtags)
    set_target_properties(${target} PROPERTIES INSTALL_RPATH_USE_LINK_PATH OFF)
  endforeach()
  set_target_properties(xblick_k4a_egl PROPERTIES INSTALL_RPATH "$ORIGIN")
  # OpenGL::EGL also brings GLVND's libOpenGL, which the shim does not use; as NEEDED it would be grafted into the
  # wheel (auditwheel) with libGLdispatch, a second copy beside the system's
  target_link_options(xblick_k4a_egl PRIVATE -Wl,--as-needed)
  message(STATUS "xblick-azure-kinect: nvJPEG ${NVJPEG_DIR}, CUDA ${CUDAToolkit_VERSION} at ${CUDAToolkit_LIBRARY_DIR} "
                 "(module, architectures ${CMAKE_CUDA_ARCHITECTURES}); depth engine of Microsoft's package "
                 "${engine_version}")
  install(TARGETS xblick-azure-kinect xblick-azure-kinect-parameters RUNTIME DESTINATION xblick_azure_kinect/bin)
  install(TARGETS xblick_k4a_egl LIBRARY DESTINATION xblick_azure_kinect/lib)
  install(TARGETS xblick-azure-kinect-cuda LIBRARY DESTINATION xblick_azure_kinect/lib)
  install(FILES $<TARGET_FILE:zenohc::lib> DESTINATION xblick_azure_kinect/lib)
  # the SDK's libraries under their sonames, the names the programs ask for
  foreach(sdk k4a::k4a k4a::k4arecord)
    install(FILES $<TARGET_FILE:${sdk}> DESTINATION xblick_azure_kinect/lib RENAME $<TARGET_SONAME_FILE_NAME:${sdk}>)
  endforeach()
  return()
endif()


# a running node's depth_to_color against the SDK's transformation of the same depth frames, live
# (tools/k4a_depth_to_color_check.cpp)
add_executable(k4a_depth_to_color_check tools/k4a_depth_to_color_check.cpp)
target_link_libraries(k4a_depth_to_color_check PRIVATE xblick_k4a)
# a running node's depth frames turned into depth_to_color another way (the SDK's, ...), published below another base,
# to look at both side by side (tools/k4a_depth_to_color_mirror.cpp)
add_executable(k4a_depth_to_color_mirror tools/k4a_depth_to_color_mirror.cpp)
target_link_libraries(k4a_depth_to_color_mirror PRIVATE xblick_k4a)

include(CTest)
if(BUILD_TESTING)
  add_executable(test_k4a_offline tests/test_offline.cpp)
  target_link_libraries(test_k4a_offline PRIVATE xblick_k4a)
  target_compile_definitions(test_k4a_offline PRIVATE K4A_TEST_TMP="${CMAKE_CURRENT_BINARY_DIR}")
  if(XBLICK_K4A_CUDA)
    # the CUDA ring's frames are read in a process of their own (CUDA IPC handles do not open in their own process)
    add_executable(test_k4a_cuda_reader tests/cuda_reader.cpp)
    target_link_libraries(test_k4a_cuda_reader PRIVATE xblick::xblick CUDA::cuda_driver)
    target_compile_definitions(test_k4a_offline PRIVATE K4A_CUDA_READER="$<TARGET_FILE:test_k4a_cuda_reader>")
    add_dependencies(test_k4a_offline test_k4a_cuda_reader)
  endif()
  target_compile_definitions(test_k4a_offline PRIVATE
    K4A_TEST_CALIBRATION="${CMAKE_CURRENT_SOURCE_DIR}/tests/data/k4a_test_calibration.json")
  if(xblick_points_FOUND)
    # Durchblick's point cloud node against the SDK's projection (it needs a GPU)
    find_program(K4A_POINTS_PROGRAM xblick-points HINTS "${xblick_DIR}/../../../bin" NO_DEFAULT_PATH REQUIRED)
    target_compile_definitions(test_k4a_offline PRIVATE K4A_POINTS_PROGRAM="${K4A_POINTS_PROGRAM}")
  endif()
  add_test(NAME offline COMMAND test_k4a_offline)
  # the parameters (rcl_interfaces CDR against rosbags' bytes, the services, fake controls) and the clock's estimators
  add_executable(test_k4a_parameters tests/test_parameters.cpp)
  target_link_libraries(test_k4a_parameters PRIVATE xblick_k4a)
  add_test(NAME parameters COMMAND test_k4a_parameters)
endif()

install(TARGETS xblick-azure-kinect xblick-azure-kinect-parameters RUNTIME DESTINATION bin)

# The CMake package xblick_k4a: the node's libraries with their headers (src/, without the programs' and the decoders'
# own).
set(XBLICK_K4A_EXPORTED xblick_k4a xblick_k4a_egl)
set(XBLICK_K4A_HEADERS src/calibration.hpp src/clock.hpp src/egl_glx.hpp src/node.hpp src/session.hpp
                       src/settings.hpp src/source.hpp)
install(TARGETS ${XBLICK_K4A_EXPORTED} EXPORT xblick_k4aTargets
        ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(FILES ${XBLICK_K4A_HEADERS} DESTINATION include/xblick_k4a)
install(EXPORT xblick_k4aTargets NAMESPACE xblick_k4a:: DESTINATION lib/cmake/xblick_k4a)
include(CMakePackageConfigHelpers)
configure_package_config_file(cmake/xblick_k4aConfig.cmake.in
                              ${CMAKE_CURRENT_BINARY_DIR}/xblick_k4aConfig.cmake
                              INSTALL_DESTINATION lib/cmake/xblick_k4a)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/xblick_k4aConfigVersion.cmake
                                 COMPATIBILITY SameMinorVersion)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/xblick_k4aConfig.cmake
              ${CMAKE_CURRENT_BINARY_DIR}/xblick_k4aConfigVersion.cmake
        DESTINATION lib/cmake/xblick_k4a)

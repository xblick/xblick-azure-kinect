# Azure Kinect cameras as Durchblick streams (README.md).
#
#   cmake -S . -B build -G Ninja -DCMAKE_PREFIX_PATH="<xblick>;<zenoh>;<k4a SDK>;<nlohmann_json>"
#         [-DXBLICK_K4A_CUDA=ON -DCUDAToolkit_ROOT=<CUDA toolkit with nvJPEG>]
#
# Targets: xblick-azure-kinect (the camera node), xblick-azure-kinect-parameters, test_k4a_offline (no camera needed;
# ctest). Installs the libraries as the CMake package xblick_k4a (xblick_k4a::xblick_k4a), for programs that build on
# the node. Body tracking from the node's streams is the repository xblick-k4abt.
cmake_minimum_required(VERSION 3.22)
project(xblick_azure_kinect VERSION 0.4.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
# installed programs find this project's shared libraries (lib/) and keep the directories of the libraries they link
# (CUDA, the SDKs), as the built ones do
set(CMAKE_INSTALL_RPATH "$ORIGIN/../lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH ON)

option(XBLICK_K4A_CUDA "decode (nvJPEG) and rectify (NPP) the colour camera's MJPEG on the GPU (CUDA toolkit)" OFF)
# body tracking left this repository with 0.4.0: stop instead of building without the tracker that was asked for
if(XBLICK_K4A_BODY_TRACKING)
  message(FATAL_ERROR "XBLICK_K4A_BODY_TRACKING: body tracking (xblick-k4abt) is the repository xblick-k4abt since "
                      "xblick-azure-kinect 0.4.0 (https://github.com/xblick/xblick-k4abt)")
endif()

# Durchblick's 0.5 line from 0.5.5 (its CMake package accepts its own minor version only, so 0.5.5 means 0.5.5 or a
# later 0.5.x): publish_jpeg, pin_host,
# StaticTransformPublisher, Imu, diagnostics, CameraInfo::read_file, Options, Parameters, RVL for depth, ring keys with
# the host; point clouds (the tests use its point cloud node xblick-points when Durchblick was built with it)
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
target_compile_options(xblick_k4a PRIVATE -Wall -Wextra)
if(XBLICK_K4A_CUDA)
  find_package(CUDAToolkit 12 REQUIRED)
  target_sources(xblick_k4a PRIVATE src/nvjpeg_decoder.cpp)
  target_compile_definitions(xblick_k4a PUBLIC XBLICK_K4A_CUDA)
  # nppig: the rectified colour's remap (rectify.cpp)
  target_link_libraries(xblick_k4a PUBLIC CUDA::cudart CUDA::nvjpeg CUDA::nppig CUDA::nppc)
  # the node's cost per capture with shared-memory and CUDA subscribers on every stream (tools/k4a_node_bench.cpp)
  add_executable(k4a_node_bench tools/k4a_node_bench.cpp)
  target_link_libraries(k4a_node_bench PRIVATE xblick_k4a)
endif()

add_executable(xblick-azure-kinect src/main_camera.cpp)
target_link_libraries(xblick-azure-kinect PRIVATE xblick_k4a)

# a node's parameters over its ROS 2 parameter services (as ros2 param), the camera's and any other node's
add_executable(xblick-azure-kinect-parameters src/main_parameters.cpp)
target_link_libraries(xblick-azure-kinect-parameters PRIVATE xblick_k4a)

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

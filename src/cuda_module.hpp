// The functions of the wheel's CUDA module libxblick-azure-kinect-cuda.so (nvjpeg_decoder.cpp, gpu_rectify.cpp),
// which the wheel's programs look up with dlsym after opening it (wheel.cpp). Defined only in the module build
// (XBLICK_K4A_CUDA_MODULE); the CMake build with XBLICK_K4A_CUDA calls the C++ functions directly.
#pragma once

#include "nvjpeg_decoder.hpp"
#include "rectify.hpp"

#define XBLICK_K4A_CUDA_EXPORT extern "C" __attribute__((visibility("default")))

XBLICK_K4A_CUDA_EXPORT void xblick_k4a_check_cuda_device(int device);
XBLICK_K4A_CUDA_EXPORT xblick::k4a_node::NvJpegDecoder* xblick_k4a_open_nvjpeg(int device);
XBLICK_K4A_CUDA_EXPORT xblick::k4a_node::GpuRectifier* xblick_k4a_open_rectifier(int device,
                                                                                  const xblick::k4a_node::RectifyMap* map);

// What only the programs of the Python wheel do (CMakeLists.txt, SKBUILD; XBLICK_K4A_WHEEL), beside wheel.cpp's
// make_nvjpeg_decoder, make_gpu_rectifier and check_cuda_device from the CUDA module (nvjpeg_decoder.hpp).
#pragma once

#include <filesystem>

namespace xblick::k4a_node
{
    // Where the wheel's setup command xblick-azure-kinect-setup puts the depth engine of Microsoft's package
    // (python/xblick_azure_kinect/sdk.py): <cache_dir("azure-kinect")>/depth-engine-<version>/libdepthengine.so.2.0.
    std::filesystem::path depth_engine_path();

    // Loads the depth engine from depth_engine_path(), so that libk4a's dlopen("libdepthengine.so.2.0") finds it
    // loaded (by its soname) instead of searching the library path: first thing in main, before --help can end the
    // program. Never throws; require_depth_engine says what went wrong.
    void preload_depth_engine();

    // Throws with one line naming the setup command unless preload_depth_engine loaded the engine. The program calls it
    // where the engine is needed: a camera (the SDK turns its raw depth frames into depth images with the engine), and
    // --sdk-depth-to-color (the SDK's transformation runs in the engine). A recording plays without it.
    void require_depth_engine();
}  // namespace xblick::k4a_node

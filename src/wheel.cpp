// The wheel's programs: the CUDA parts from the module libxblick-azure-kinect-cuda.so beside them
// (xblick_azure_kinect/lib), opened when --cuda-device names a GPU. The module finds nvJPEG and the CUDA runtime
// through its RPATH in NVIDIA's pip packages of the same environment (site-packages/nvidia/*/lib, CMakeLists.txt);
// without them the programs still run with --cuda-device none. The depth engine from the node's cache directory.
#include "wheel.hpp"

#include <dlfcn.h>

#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <string>

#include "cuda_module.hpp"
#include "nvjpeg_decoder.hpp"
#include "rectify.hpp"
#include "xblick/paths.hpp"

#ifndef XBLICK_K4A_DEPTH_ENGINE_VERSION
#error "XBLICK_K4A_DEPTH_ENGINE_VERSION: the version of Microsoft's package libk4a1.4 the setup command unpacks"
#endif

namespace xblick::k4a_node
{
    namespace
    {
        constexpr const char* kSetup = "xblick-azure-kinect-setup";

        struct Module
        {
            decltype(&xblick_k4a_check_cuda_device) check = nullptr;
            decltype(&xblick_k4a_open_nvjpeg) open_nvjpeg = nullptr;
            decltype(&xblick_k4a_open_rectifier) open_rectifier = nullptr;
        };

        template <typename F>
        F symbol(void* handle, const std::filesystem::path& module, const char* name)
        {
            void* address = ::dlsym(handle, name);
            if (address == nullptr)
            {
                throw std::runtime_error(module.string() + " has no " + name + ": reinstall xblick-azure-kinect");
            }
            return reinterpret_cast<F>(address);
        }

        Module load_module()
        {
            const std::filesystem::path module =
                std::filesystem::read_symlink("/proc/self/exe").parent_path().parent_path() / "lib" /
                "libxblick-azure-kinect-cuda.so";
            void* handle = ::dlopen(module.c_str(), RTLD_NOW | RTLD_LOCAL);
            if (handle == nullptr)
            {
                const std::string error = ::dlerror();
                if (not std::filesystem::exists(module))
                {
                    throw std::runtime_error(module.string() + " is missing: reinstall xblick-azure-kinect from a wheel");
                }
                throw std::runtime_error("cannot load nvJPEG and the CUDA runtime (" + error +
                                         "): the packages nvidia-nvjpeg-cu12 and nvidia-cuda-runtime-cu12 must be "
                                         "installed in the environment of xblick-azure-kinect (its dependencies)");
            }
            Module m;
            m.check = symbol<decltype(m.check)>(handle, module, "xblick_k4a_check_cuda_device");
            m.open_nvjpeg = symbol<decltype(m.open_nvjpeg)>(handle, module, "xblick_k4a_open_nvjpeg");
            m.open_rectifier = symbol<decltype(m.open_rectifier)>(handle, module, "xblick_k4a_open_rectifier");
            return m;  // the module stays loaded: the decoders' code is in it
        }

        const Module& module()
        {
            static const Module m = load_module();  // a failure throws again at the next call
            return m;
        }
    }  // namespace

    void check_cuda_device(int device) { module().check(device); }

    std::unique_ptr<NvJpegDecoder> make_nvjpeg_decoder(int device)
    {
        return std::unique_ptr<NvJpegDecoder>(module().open_nvjpeg(device));
    }

    std::unique_ptr<GpuRectifier> make_gpu_rectifier(int device, const RectifyMap& map)
    {
        return std::unique_ptr<GpuRectifier>(module().open_rectifier(device, &map));
    }

    std::filesystem::path depth_engine_path()
    {
        return xblick::cache_dir("azure-kinect") / ("depth-engine-" XBLICK_K4A_DEPTH_ENGINE_VERSION) /
               "libdepthengine.so.2.0";
    }

    namespace
    {
        std::string engine_error = "the depth engine was not loaded (preload_depth_engine)";  // empty: loaded
    }  // namespace

    void preload_depth_engine()
    {
        try
        {
            const std::filesystem::path engine = depth_engine_path();
            if (not std::filesystem::exists(engine))
            {
                engine_error = "the depth engine of Microsoft's Azure Kinect Sensor SDK is missing (" + engine.string() +
                               "): run " + kSetup + " once, which downloads it from Microsoft";
            }
            else if (::dlopen(engine.c_str(), RTLD_NOW | RTLD_LOCAL) == nullptr)
            {
                engine_error = "cannot load the depth engine (" + std::string(::dlerror()) + "): run " + kSetup +
                               ", which names the system libraries it needs";
            }
            else
            {
                engine_error.clear();
                return;
            }
        }
        catch (const std::exception& e)  // cache_dir: a relative XBLICK_HOME
        {
            engine_error = std::string("the depth engine's directory: ") + e.what();
        }
        // Without the engine, libk4a tries to load it once more when it is unloaded at the program's end and logs three
        // lines on stdout that point at the library path, also after a playback that never needed it. Set before that
        // (at exit, before the libraries' destructors), the variable keeps its logger off if nothing started it
        // before; a value the user set is kept. The program's own line (require_depth_engine) names the setup command.
        std::atexit([] { ::setenv("K4A_ENABLE_LOG_TO_STDOUT", "0", 0); });
    }

    void require_depth_engine()
    {
        if (not engine_error.empty())
        {
            throw std::runtime_error(engine_error);
        }
    }
}  // namespace xblick::k4a_node

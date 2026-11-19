// The colour image's remap on the GPU with NPP (rectify.hpp: GpuRectifier). Built with XBLICK_K4A_CUDA against a CUDA
// toolkit, or in the wheel's CUDA module against NVIDIA's pip packages (nvjpeg_decoder.hpp).
#include <stdexcept>
#include <string>

#include <cuda_runtime_api.h>
#include <nppi_geometry_transforms.h>

#include "cuda_module.hpp"
#include "rectify.hpp"

namespace xblick::k4a_node
{
    namespace
    {
        void check(cudaError_t status, const char* what)
        {
            if (status != cudaSuccess)
            {
                throw std::runtime_error(std::string("rectification: ") + what + ": " + cudaGetErrorString(status));
            }
        }

        class Rectifier final : public GpuRectifier
        {
        public:
            Rectifier(int device, const RectifyMap& map) : device_(device), width_(map.width), height_(map.height)
            {
                try
                {
                    check(cudaSetDevice(device), "cudaSetDevice");
                    const std::size_t n = map.x.size();
                    check(cudaMalloc(reinterpret_cast<void**>(&x_), n * sizeof(float)), "cudaMalloc");
                    check(cudaMalloc(reinterpret_cast<void**>(&y_), n * sizeof(float)), "cudaMalloc");
                    check(cudaMalloc(reinterpret_cast<void**>(&out_), n * 3), "cudaMalloc");
                    check(cudaMemcpy(x_, map.x.data(), n * sizeof(float), cudaMemcpyHostToDevice), "cudaMemcpy");
                    check(cudaMemcpy(y_, map.y.data(), n * sizeof(float), cudaMemcpyHostToDevice), "cudaMemcpy");
                    // NPP leaves pixels whose point is outside the image unwritten: they stay this black
                    check(cudaMemset(out_, 0, n * 3), "cudaMemset");
                    cudaDeviceProp properties{};
                    check(cudaGetDeviceProperties(&properties, device), "cudaGetDeviceProperties");
                    context_.nCudaDeviceId = device;
                    context_.nMultiProcessorCount = properties.multiProcessorCount;
                    context_.nMaxThreadsPerMultiProcessor = properties.maxThreadsPerMultiProcessor;
                    context_.nMaxThreadsPerBlock = properties.maxThreadsPerBlock;
                    context_.nSharedMemPerBlock = properties.sharedMemPerBlock;
                    context_.nCudaDevAttrComputeCapabilityMajor = properties.major;
                    context_.nCudaDevAttrComputeCapabilityMinor = properties.minor;
                }
                catch (...)
                {
                    release();
                    throw;
                }
            }

            ~Rectifier() override { release(); }

            std::uint64_t rectify(std::uint64_t src, std::uint64_t stream) override
            {
                check(cudaSetDevice(device_), "cudaSetDevice");
                NppStreamContext context = context_;
                context.hStream = reinterpret_cast<cudaStream_t>(stream);
                check(cudaStreamGetFlags(context.hStream, &context.nStreamFlags), "cudaStreamGetFlags");
                const int w = int(width_), h = int(height_);
                const NppStatus status = nppiRemap_8u_C3R_Ctx(
                    reinterpret_cast<const Npp8u*>(src), NppiSize{w, h}, w * 3, NppiRect{0, 0, w, h}, x_,
                    w * int(sizeof(float)), y_, w * int(sizeof(float)), out_, w * 3, NppiSize{w, h}, NPPI_INTER_LINEAR,
                    context);
                if (status != NPP_SUCCESS)
                {
                    throw std::runtime_error("rectification: nppiRemap_8u_C3R_Ctx: NPP status " +
                                             std::to_string(int(status)));
                }
                return reinterpret_cast<std::uint64_t>(out_);
            }

        private:
            void release()
            {
                cudaFree(x_);
                cudaFree(y_);
                cudaFree(out_);
            }

            int device_ = 0;
            std::uint32_t width_ = 0, height_ = 0;
            float* x_ = nullptr;
            float* y_ = nullptr;
            std::uint8_t* out_ = nullptr;
            NppStreamContext context_{};
        };
    }  // namespace

#ifndef XBLICK_K4A_CUDA_MODULE
    std::unique_ptr<GpuRectifier> make_gpu_rectifier(int device, const RectifyMap& map)
    {
        return std::make_unique<Rectifier>(device, map);
    }
#endif
}  // namespace xblick::k4a_node

#ifdef XBLICK_K4A_CUDA_MODULE
XBLICK_K4A_CUDA_EXPORT xblick::k4a_node::GpuRectifier* xblick_k4a_open_rectifier(int device,
                                                                                  const xblick::k4a_node::RectifyMap* map)
{
    return new xblick::k4a_node::Rectifier(device, *map);
}
#endif

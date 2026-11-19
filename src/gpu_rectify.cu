// The colour image's remap on the GPU (rectify.hpp: GpuRectifier), a CUDA kernel of this project. Built with
// XBLICK_K4A_CUDA against a CUDA toolkit, and in the wheel's CUDA module (nvjpeg_decoder.hpp); both compile this file
// with nvcc, so that both builds give the same pixels.
//
// The kernel gives the result of NPP's nppiRemap_8u_C3R_Ctx with NPPI_INTER_LINEAR, which the node used up to 0.4.0,
// bit for bit (docs/design.md, Rectified colour on the GPU):
// * a pixel is written only when its point (sx, sy) has -0.5 <= sx < width and -0.5 <= sy < height; the others keep
//   the buffer's black;
// * the point is clamped to [0, width - 1] x [0, height - 1], its right and lower neighbours to the last column and
//   row;
// * per channel: top = p00 (1 - fx) + p10 fx, bottom = p01 (1 - fx) + p11 fx, value = top (1 - fy) + bottom fy in
//   float, rounded as floor(value + 0.5). The order of these operations decides the last bit; nvcc's default
//   contraction to FMA (-fmad=true) is part of it. Do not change them without the comparison against NPP.
#include <stdexcept>
#include <string>

#include <cuda_runtime_api.h>

#include "cuda_module.hpp"
#include "rectify.hpp"

namespace xblick::k4a_node
{
    namespace
    {
        constexpr int kBlockX = 32, kBlockY = 8;

        __global__ void __launch_bounds__(kBlockX * kBlockY)
            remap_rgb8_linear(const std::uint8_t* __restrict__ src, int w, int h, const float* __restrict__ map_x,
                              const float* __restrict__ map_y, std::uint8_t* __restrict__ dst)
        {
            const int x = int(blockIdx.x) * kBlockX + int(threadIdx.x);
            const int y = int(blockIdx.y) * kBlockY + int(threadIdx.y);
            if (x >= w or y >= h)
            {
                return;
            }
            const int i = y * w + x;
            float sx = map_x[i], sy = map_y[i];
            if (not(sx >= -0.5f and sy >= -0.5f and sx < float(w) and sy < float(h)))  // also NaN
            {
                return;
            }
            sx = fminf(fmaxf(sx, 0.0f), float(w - 1));
            sy = fminf(fmaxf(sy, 0.0f), float(h - 1));
            const int x0 = int(sx), y0 = int(sy);
            const int x1 = min(x0 + 1, w - 1), y1 = min(y0 + 1, h - 1);
            const float fx = sx - float(x0), fy = sy - float(y0);
            const std::uint8_t* p00 = src + (y0 * w + x0) * 3;
            const std::uint8_t* p10 = src + (y0 * w + x1) * 3;
            const std::uint8_t* p01 = src + (y1 * w + x0) * 3;
            const std::uint8_t* p11 = src + (y1 * w + x1) * 3;
            for (int c = 0; c < 3; ++c)
            {
                const float top = float(p00[c]) * (1.0f - fx) + float(p10[c]) * fx;
                const float bottom = float(p01[c]) * (1.0f - fx) + float(p11[c]) * fx;
                dst[i * 3 + c] = std::uint8_t(floorf(top * (1.0f - fy) + bottom * fy + 0.5f));
            }
        }

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
                    // pixels whose point is outside the image are not written: they stay this black
                    check(cudaMemset(out_, 0, n * 3), "cudaMemset");
                    // before the first frame, which comes on another stream (a non-blocking one does not wait for the
                    // memset on the default stream)
                    check(cudaDeviceSynchronize(), "cudaDeviceSynchronize");
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
                const int w = int(width_), h = int(height_);
                const dim3 grid((w + kBlockX - 1) / kBlockX, (h + kBlockY - 1) / kBlockY);
                remap_rgb8_linear<<<grid, dim3(kBlockX, kBlockY), 0, reinterpret_cast<cudaStream_t>(stream)>>>(
                    reinterpret_cast<const std::uint8_t*>(src), w, h, x_, y_, out_);
                check(cudaGetLastError(), "remap kernel launch");
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

#include "rectify.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>

#ifdef DURCHBLICK_K4A_CUDA
#include <cuda_runtime_api.h>
#include <nppi_geometry_transforms.h>
#endif

namespace durchblick::k4a_node
{
    durchblick::CameraInfo rectified_info(const durchblick::CameraInfo& raw)
    {
        durchblick::CameraInfo info = durchblick::CameraInfo::pinhole(raw.width, raw.height, raw.p[0], raw.p[5],
                                                                      raw.p[2], raw.p[6], raw.header.frame_id);
        info.header = raw.header;
        info.p = raw.p;
        return info;
    }

    RectifyMap rectify_map(const durchblick::CameraInfo& raw)
    {
        const bool rational = raw.distortion_model == "rational_polynomial" and raw.d.size() == 8;
        const bool plumb_bob = raw.distortion_model == "plumb_bob" and raw.d.size() == 5;
        if (not rational and not plumb_bob)
        {
            throw std::invalid_argument("rectification: distortion model " + raw.distortion_model + " with " +
                                        std::to_string(raw.d.size()) +
                                        " coefficients; rational_polynomial (8) or plumb_bob (5)");
        }
        if (raw.r != std::array<double, 9>{1, 0, 0, 0, 1, 0, 0, 0, 1})
        {
            throw std::invalid_argument("rectification: R must be the identity (one camera, not a stereo pair)");
        }
        const auto& p = raw.p;
        if (p[1] != 0.0 or p[3] != 0.0 or p[4] != 0.0 or p[7] != 0.0 or p[8] != 0.0 or p[9] != 0.0 or
            p[10] != 1.0 or p[11] != 0.0 or not(p[0] > 0.0) or not(p[5] > 0.0))
        {
            throw std::invalid_argument("rectification: P must be [fx' 0 cx' 0; 0 fy' cy' 0; 0 0 1 0]");
        }
        const auto& k = raw.k;
        const auto& d = raw.d;
        const double k4 = rational ? d[5] : 0.0, k5 = rational ? d[6] : 0.0, k6 = rational ? d[7] : 0.0;
        RectifyMap map;
        map.width = raw.width;
        map.height = raw.height;
        map.x.resize(std::size_t(raw.width) * raw.height);
        map.y.resize(map.x.size());
        for (std::uint32_t v = 0; v < raw.height; ++v)
        {
            for (std::uint32_t u = 0; u < raw.width; ++u)
            {
                // the ray of the rectified pixel, then the camera's distortion (OpenCV's rational model)
                const double x = (u - p[2]) / p[0];
                const double y = (v - p[6]) / p[5];
                const double r2 = x * x + y * y;
                const double radial =
                    (1 + r2 * (d[0] + r2 * (d[1] + r2 * d[4]))) / (1 + r2 * (k4 + r2 * (k5 + r2 * k6)));
                const double xd = x * radial + 2 * d[2] * x * y + d[3] * (r2 + 2 * x * x);
                const double yd = y * radial + d[2] * (r2 + 2 * y * y) + 2 * d[3] * x * y;
                const std::size_t i = std::size_t(v) * raw.width + u;
                map.x[i] = float(k[0] * xd + k[1] * yd + k[2]);
                map.y[i] = float(k[4] * yd + k[5]);
            }
        }
        const int w = int(raw.width), h = int(raw.height);
        map.offset.assign(map.x.size(), -1);
        map.wx.assign(map.x.size(), 0);
        map.wy.assign(map.x.size(), 0);
        map.neighbours.assign(map.x.size(), 0);
        map.nearest.assign(map.x.size(), -1);
        for (std::size_t i = 0; i < map.x.size(); ++i)
        {
            const float sx = map.x[i], sy = map.y[i];
            if (not(sx >= 0.0f and sy >= 0.0f and sx <= float(w - 1) and sy <= float(h - 1)))
            {
                continue;
            }
            const int x0 = int(sx), y0 = int(sy);
            map.offset[i] = 3 * (y0 * w + x0);
            map.wx[i] = std::uint16_t(std::lround((sx - float(x0)) * 256.0f));
            map.wy[i] = std::uint16_t(std::lround((sy - float(y0)) * 256.0f));
            map.neighbours[i] = std::uint8_t((x0 < w - 1 ? 1 : 0) | (y0 < h - 1 ? 2 : 0));
            map.nearest[i] = std::int32_t(std::lround(sy)) * w + std::int32_t(std::lround(sx));
        }
        return map;
    }

    void remap_depth16(const RectifyMap& map, const std::uint16_t* src, std::uint16_t* dst)
    {
        for (std::size_t i = 0; i < map.nearest.size(); ++i)
        {
            dst[i] = map.nearest[i] < 0 ? std::uint16_t(0) : src[map.nearest[i]];
        }
    }

    void remap_rgb8(const RectifyMap& map, const std::uint8_t* src, std::uint8_t* dst)
    {
        const std::int32_t row = std::int32_t(map.width) * 3;
        for (std::size_t i = 0; i < map.offset.size(); ++i)
        {
            std::uint8_t* out = dst + 3 * i;
            if (map.offset[i] < 0)
            {
                out[0] = out[1] = out[2] = 0;
                continue;
            }
            const std::uint8_t* a = src + map.offset[i];
            const std::uint8_t* b = a + ((map.neighbours[i] & 1) ? 3 : 0);
            const std::uint8_t* c = a + ((map.neighbours[i] & 2) ? row : 0);
            const std::uint8_t* e = c + (b - a);
            const std::uint32_t wx = map.wx[i], wy = map.wy[i];
            const std::uint32_t wa = (256 - wx) * (256 - wy), wb = wx * (256 - wy), wc = (256 - wx) * wy, we = wx * wy;
            for (int ch = 0; ch < 3; ++ch)
            {
                out[ch] = std::uint8_t((a[ch] * wa + b[ch] * wb + c[ch] * wc + e[ch] * we + 32768) >> 16);
            }
        }
    }

#ifdef DURCHBLICK_K4A_CUDA
    namespace
    {
        void check(cudaError_t status, const char* what)
        {
            if (status != cudaSuccess)
            {
                throw std::runtime_error(std::string("rectification: ") + what + ": " + cudaGetErrorString(status));
            }
        }
    }  // namespace

    struct GpuRectifier::Impl
    {
        int device = 0;
        std::uint32_t width = 0, height = 0;
        float* x = nullptr;
        float* y = nullptr;
        std::uint8_t* out = nullptr;
        NppStreamContext context{};
    };

    GpuRectifier::GpuRectifier(int device, const RectifyMap& map) : impl_(std::make_unique<Impl>())
    {
        Impl& m = *impl_;
        m.device = device;
        m.width = map.width;
        m.height = map.height;
        check(cudaSetDevice(device), "cudaSetDevice");
        const std::size_t n = map.x.size();
        check(cudaMalloc(reinterpret_cast<void**>(&m.x), n * sizeof(float)), "cudaMalloc");
        check(cudaMalloc(reinterpret_cast<void**>(&m.y), n * sizeof(float)), "cudaMalloc");
        check(cudaMalloc(reinterpret_cast<void**>(&m.out), n * 3), "cudaMalloc");
        check(cudaMemcpy(m.x, map.x.data(), n * sizeof(float), cudaMemcpyHostToDevice), "cudaMemcpy");
        check(cudaMemcpy(m.y, map.y.data(), n * sizeof(float), cudaMemcpyHostToDevice), "cudaMemcpy");
        // NPP leaves pixels whose point is outside the image unwritten: they stay this black
        check(cudaMemset(m.out, 0, n * 3), "cudaMemset");
        cudaDeviceProp properties{};
        check(cudaGetDeviceProperties(&properties, device), "cudaGetDeviceProperties");
        m.context.nCudaDeviceId = device;
        m.context.nMultiProcessorCount = properties.multiProcessorCount;
        m.context.nMaxThreadsPerMultiProcessor = properties.maxThreadsPerMultiProcessor;
        m.context.nMaxThreadsPerBlock = properties.maxThreadsPerBlock;
        m.context.nSharedMemPerBlock = properties.sharedMemPerBlock;
        m.context.nCudaDevAttrComputeCapabilityMajor = properties.major;
        m.context.nCudaDevAttrComputeCapabilityMinor = properties.minor;
    }

    GpuRectifier::~GpuRectifier()
    {
        cudaFree(impl_->x);
        cudaFree(impl_->y);
        cudaFree(impl_->out);
    }

    std::uint64_t GpuRectifier::rectify(std::uint64_t src, std::uint64_t stream)
    {
        Impl& m = *impl_;
        check(cudaSetDevice(m.device), "cudaSetDevice");
        NppStreamContext context = m.context;
        context.hStream = reinterpret_cast<cudaStream_t>(stream);
        check(cudaStreamGetFlags(context.hStream, &context.nStreamFlags), "cudaStreamGetFlags");
        const int w = int(m.width), h = int(m.height);
        const NppStatus status = nppiRemap_8u_C3R_Ctx(
            reinterpret_cast<const Npp8u*>(src), NppiSize{w, h}, w * 3, NppiRect{0, 0, w, h}, m.x,
            w * int(sizeof(float)), m.y, w * int(sizeof(float)), m.out, w * 3, NppiSize{w, h}, NPPI_INTER_LINEAR,
            context);
        if (status != NPP_SUCCESS)
        {
            throw std::runtime_error("rectification: nppiRemap_8u_C3R_Ctx: NPP status " + std::to_string(int(status)));
        }
        return reinterpret_cast<std::uint64_t>(m.out);
    }
#endif
}  // namespace durchblick::k4a_node

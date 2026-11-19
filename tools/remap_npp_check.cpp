// The node's GPU remap of the rectified colour (src/gpu_rectify.cu, make_gpu_rectifier) against NPP's
// nppiRemap_8u_C3R_Ctx with NPPI_INTER_LINEAR, which the node used up to 0.4.0. Built in CUDA builds whose toolkit has
// NPP (CMakeLists.txt: k4a_remap_npp_check). Needs a GPU; run it after any change of the kernel.
//
//   k4a_remap_npp_check [--time]
//
// Equality: 720P and 1080P images (a full-range gradient, noise, a 1-pixel checkerboard, 8-pixel edges) through maps
// (a lens distortion like the colour camera's, points anywhere and outside, every fraction, steps of 1/256, the
// borders: -0.5, 0, width - 1, width and the values beside them). Per case the pixels NPP writes, those the kernel
// writes and those that differ; exit 1 if any differs. --time: then the median time per frame of both at 720P and
// 1080P with the distortion map (CUDA events, 20 x 100 frames after 200), for an idle GPU only.
#include <cuda_runtime_api.h>
#include <nppi_geometry_transforms.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <string>
#include <vector>

#include "rectify.hpp"

namespace
{
    using xblick::k4a_node::RectifyMap;

    void check(cudaError_t status, const char* what)
    {
        if (status != cudaSuccess)
        {
            std::fprintf(stderr, "%s: %s\n", what, cudaGetErrorString(status));
            std::exit(2);
        }
    }

    constexpr std::uint8_t kSeven = 7;  // an image of 7s shows which pixels a remap writes (unwritten ones stay 0)

    struct Device
    {
        std::uint8_t *src = nullptr, *npp = nullptr;
        float *x = nullptr, *y = nullptr;
        NppStreamContext context{};
        cudaStream_t stream = nullptr;
    };

    Device upload(const std::vector<std::uint8_t>& image, const RectifyMap& map)
    {
        const std::size_t n = std::size_t(map.width) * map.height;
        Device d;
        check(cudaStreamCreateWithFlags(&d.stream, cudaStreamNonBlocking), "cudaStreamCreate");
        check(cudaMalloc(reinterpret_cast<void**>(&d.src), n * 3), "cudaMalloc");
        check(cudaMalloc(reinterpret_cast<void**>(&d.npp), n * 3), "cudaMalloc");
        check(cudaMalloc(reinterpret_cast<void**>(&d.x), n * 4), "cudaMalloc");
        check(cudaMalloc(reinterpret_cast<void**>(&d.y), n * 4), "cudaMalloc");
        check(cudaMemcpy(d.src, image.data(), n * 3, cudaMemcpyHostToDevice), "cudaMemcpy");
        check(cudaMemcpy(d.x, map.x.data(), n * 4, cudaMemcpyHostToDevice), "cudaMemcpy");
        check(cudaMemcpy(d.y, map.y.data(), n * 4, cudaMemcpyHostToDevice), "cudaMemcpy");
        cudaDeviceProp p{};
        check(cudaGetDeviceProperties(&p, 0), "cudaGetDeviceProperties");
        d.context.hStream = d.stream;
        check(cudaStreamGetFlags(d.stream, &d.context.nStreamFlags), "cudaStreamGetFlags");
        d.context.nCudaDeviceId = 0;
        d.context.nMultiProcessorCount = p.multiProcessorCount;
        d.context.nMaxThreadsPerMultiProcessor = p.maxThreadsPerMultiProcessor;
        d.context.nMaxThreadsPerBlock = p.maxThreadsPerBlock;
        d.context.nSharedMemPerBlock = p.sharedMemPerBlock;
        d.context.nCudaDevAttrComputeCapabilityMajor = p.major;
        d.context.nCudaDevAttrComputeCapabilityMinor = p.minor;
        return d;
    }

    void release(Device& d)
    {
        cudaFree(d.src);
        cudaFree(d.npp);
        cudaFree(d.x);
        cudaFree(d.y);
        cudaStreamDestroy(d.stream);
    }

    void npp_remap(const Device& d, int w, int h)
    {
        const NppStatus status =
            nppiRemap_8u_C3R_Ctx(d.src, NppiSize{w, h}, w * 3, NppiRect{0, 0, w, h}, d.x, w * 4, d.y, w * 4, d.npp,
                                 w * 3, NppiSize{w, h}, NPPI_INTER_LINEAR, d.context);
        if (status != NPP_SUCCESS)
        {
            std::fprintf(stderr, "nppiRemap_8u_C3R_Ctx: status %d\n", int(status));
            std::exit(2);
        }
    }

    std::vector<std::uint8_t> image_of(int kind, int w, int h, std::mt19937& rng)
    {
        std::vector<std::uint8_t> image(std::size_t(w) * h * 3);
        std::uniform_int_distribution<int> byte(0, 255);
        for (int y = 0; y < h; ++y)
            for (int x = 0; x < w; ++x)
                for (int c = 0; c < 3; ++c)
                {
                    int v = 0;
                    switch (kind)
                    {
                        case 0: v = (x * 255 / (w - 1) + 85 * c) % 256; break;
                        case 1: v = byte(rng); break;
                        case 2: v = (x + y) % 2 ? 255 : 0; break;
                        default: v = (x / 8 + y / 8) % 2 ? 255 : 0; break;
                    }
                    image[(std::size_t(y) * w + x) * 3 + c] = std::uint8_t(v);
                }
        return image;
    }

    RectifyMap map_of(int kind, int w, int h, std::mt19937& rng)
    {
        RectifyMap map;
        map.width = std::uint32_t(w);
        map.height = std::uint32_t(h);
        map.x.resize(std::size_t(w) * h);
        map.y.resize(map.x.size());
        std::uniform_real_distribution<float> any_x(-2.0f, float(w) + 1.0f), any_y(-2.0f, float(h) + 1.0f);
        std::uniform_real_distribution<float> fraction(0.0f, 1.0f);
        const float cx = w / 2.0f, cy = h / 2.0f, f = 0.8f * w;
        const float bx[] = {-1.0f, -0.5f, -1e-6f, 0.0f, 1e-6f, 0.5f, w - 1.5f, w - 1.0f - 1e-4f, w - 1.0f, w - 1.0f + 1e-4f,
                            w - 0.5f, float(w)};
        const float by[] = {-1.0f, -0.5f, -1e-6f, 0.0f, 1e-6f, 0.5f, h - 1.5f, h - 1.0f - 1e-4f, h - 1.0f, h - 1.0f + 1e-4f,
                            h - 0.5f, float(h)};
        for (int y = 0; y < h; ++y)
            for (int x = 0; x < w; ++x)
            {
                float sx = float(x), sy = float(y);
                switch (kind)
                {
                    case 0:  // plumb_bob with k1, k2, p1, p2
                    {
                        const float u = (x - cx) / f, v = (y - cy) / f, r2 = u * u + v * v;
                        const float k = 1 + 0.08f * r2 - 0.05f * r2 * r2;
                        sx = cx + f * (u * k + 2 * 0.001f * u * v + 0.0005f * (r2 + 2 * u * u));
                        sy = cy + f * (v * k + 0.001f * (r2 + 2 * v * v) + 2 * 0.0005f * u * v);
                        break;
                    }
                    case 1: sx = any_x(rng); sy = any_y(rng); break;
                    case 2: sx = x + fraction(rng); sy = y + fraction(rng); break;
                    case 3: sx = x + (x % 257) / 256.0f; sy = y + (y % 257) / 256.0f; break;
                    default: sx = bx[(x + y) % 12]; sy = by[(x / 12 + y) % 12]; break;
                }
                map.x[std::size_t(y) * w + x] = sx;
                map.y[std::size_t(y) * w + x] = sy;
            }
        return map;
    }

    float median_ms(cudaStream_t stream, const auto& frame)
    {
        cudaEvent_t start, end;
        check(cudaEventCreate(&start), "cudaEventCreate");
        check(cudaEventCreate(&end), "cudaEventCreate");
        for (int i = 0; i < 200; ++i)
        {
            frame();
        }
        std::vector<float> ms;
        for (int r = 0; r < 20; ++r)
        {
            check(cudaEventRecord(start, stream), "cudaEventRecord");
            for (int i = 0; i < 100; ++i)
            {
                frame();
            }
            check(cudaEventRecord(end, stream), "cudaEventRecord");
            check(cudaEventSynchronize(end), "cudaEventSynchronize");
            float t = 0;
            check(cudaEventElapsedTime(&t, start, end), "cudaEventElapsedTime");
            ms.push_back(t / 100);
        }
        std::sort(ms.begin(), ms.end());
        cudaEventDestroy(start);
        cudaEventDestroy(end);
        return (ms[9] + ms[10]) / 2;
    }
}  // namespace

int main(int argc, char** argv)
{
    const bool time = argc > 1 and std::strcmp(argv[1], "--time") == 0;
    std::mt19937 rng(1);
    long pixels = 0, differ = 0, by_one = 0, written_apart = 0;
    for (const auto [w, h] : {std::pair{1280, 720}, std::pair{1920, 1080}})
        for (int image_kind = 0; image_kind < 4; ++image_kind)
            for (int map_kind = 0; map_kind < 5; ++map_kind)
            {
                const std::vector<std::uint8_t> image = image_of(image_kind, w, h, rng);
                const RectifyMap map = map_of(map_kind, w, h, rng);
                Device d = upload(image, map);
                const std::size_t n = std::size_t(w) * h;
                auto ours = xblick::k4a_node::make_gpu_rectifier(0, map);
                // which pixels each writes: both outputs start black (0), an image of 7s leaves 7 where they write
                std::uint8_t* sevens = nullptr;
                check(cudaMalloc(reinterpret_cast<void**>(&sevens), n * 3), "cudaMalloc");
                check(cudaMemset(sevens, kSeven, n * 3), "cudaMemset");
                check(cudaMemset(d.npp, 0, n * 3), "cudaMemset");
                check(cudaDeviceSynchronize(), "cudaDeviceSynchronize");  // the remaps run on a non-blocking stream
                std::swap(d.src, sevens);
                npp_remap(d, w, h);
                std::uint64_t out = ours->rectify(reinterpret_cast<std::uint64_t>(d.src),
                                                  reinterpret_cast<std::uint64_t>(d.stream));
                check(cudaStreamSynchronize(d.stream), "cudaStreamSynchronize");
                std::vector<std::uint8_t> npp_mask(n * 3), kernel_mask(n * 3);
                check(cudaMemcpy(npp_mask.data(), d.npp, n * 3, cudaMemcpyDeviceToHost), "cudaMemcpy");
                check(cudaMemcpy(kernel_mask.data(), reinterpret_cast<void*>(out), n * 3, cudaMemcpyDeviceToHost),
                      "cudaMemcpy");
                std::swap(d.src, sevens);
                cudaFree(sevens);
                // the image itself, into the same buffers (unwritten pixels stay 0 in both)
                check(cudaMemset(d.npp, 0, n * 3), "cudaMemset");
                check(cudaDeviceSynchronize(), "cudaDeviceSynchronize");
                npp_remap(d, w, h);
                out = ours->rectify(reinterpret_cast<std::uint64_t>(d.src), reinterpret_cast<std::uint64_t>(d.stream));
                check(cudaStreamSynchronize(d.stream), "cudaStreamSynchronize");
                std::vector<std::uint8_t> a(n * 3), b(n * 3);
                check(cudaMemcpy(a.data(), d.npp, n * 3, cudaMemcpyDeviceToHost), "cudaMemcpy");
                check(cudaMemcpy(b.data(), reinterpret_cast<void*>(out), n * 3, cudaMemcpyDeviceToHost), "cudaMemcpy");
                long case_differ = 0, case_npp = 0, case_ours = 0, case_apart = 0;
                for (std::size_t i = 0; i < n; ++i)
                {
                    const bool npp_wrote = npp_mask[i * 3] == kSeven, kernel_wrote = kernel_mask[i * 3] == kSeven;
                    int most = 0;
                    for (int c = 0; c < 3; ++c)
                    {
                        most = std::max(most, std::abs(int(a[i * 3 + c]) - int(b[i * 3 + c])));
                    }
                    case_npp += npp_wrote;
                    case_ours += kernel_wrote;
                    case_apart += npp_wrote != kernel_wrote;
                    case_differ += most != 0;
                    by_one += most == 1;
                }
                std::printf("%dx%d image %d map %d: NPP writes %ld pixels, the kernel %ld, %ld by one only; %ld differ\n",
                            w, h, image_kind, map_kind, case_npp, case_ours, case_apart, case_differ);
                differ += case_differ;
                written_apart += case_apart;
                release(d);
                pixels += long(n);
            }
    std::printf("%ld pixels: %ld differ (%ld by 1), %ld written by one of the two only\n", pixels, differ, by_one,
                written_apart);
    const bool same = differ == 0 and written_apart == 0;
    if (time)
    {
        for (const auto [w, h] : {std::pair{1280, 720}, std::pair{1920, 1080}})
        {
            const std::vector<std::uint8_t> image = image_of(1, w, h, rng);
            const RectifyMap map = map_of(0, w, h, rng);
            Device d = upload(image, map);
            auto ours = xblick::k4a_node::make_gpu_rectifier(0, map);
            const float npp = median_ms(d.stream, [&] { npp_remap(d, w, h); });
            const float kernel = median_ms(d.stream, [&] {
                ours->rectify(reinterpret_cast<std::uint64_t>(d.src), reinterpret_cast<std::uint64_t>(d.stream));
            });
            std::printf("%dx%d per frame: NPP %.4f ms, the kernel %.4f ms\n", w, h, npp, kernel);
            release(d);
        }
    }
    return same ? 0 : 1;
}

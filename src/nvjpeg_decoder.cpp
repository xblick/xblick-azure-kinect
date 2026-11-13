#include "nvjpeg_decoder.hpp"

#include <stdexcept>
#include <string>

#include <cuda_runtime.h>
#include <nvjpeg.h>

namespace xblick::k4a_node
{
    namespace
    {
        void check(cudaError_t status, const char* what)
        {
            if (status != cudaSuccess)
            {
                throw std::runtime_error(std::string(what) + ": " + cudaGetErrorString(status));
            }
        }

        void check(nvjpegStatus_t status, const char* what)
        {
            if (status != NVJPEG_STATUS_SUCCESS)
            {
                throw std::runtime_error(std::string(what) + ": nvJPEG status " + std::to_string(int(status)));
            }
        }
    }  // namespace

    struct NvJpegDecoder::Impl
    {
        int device = 0;
        nvjpegHandle_t handle = nullptr;
        nvjpegJpegState_t state = nullptr;
        cudaStream_t stream = nullptr;
        void* pixels = nullptr;
        std::size_t capacity = 0;
    };

    NvJpegDecoder::NvJpegDecoder(int device) : impl_(std::make_unique<Impl>())
    {
        Impl& m = *impl_;
        m.device = device;
        check(cudaSetDevice(device), "cudaSetDevice");
        check(cudaStreamCreateWithFlags(&m.stream, cudaStreamNonBlocking), "cudaStreamCreate");
        check(nvjpegCreateSimple(&m.handle), "nvjpegCreateSimple");
        check(nvjpegJpegStateCreate(m.handle, &m.state), "nvjpegJpegStateCreate");
    }

    NvJpegDecoder::~NvJpegDecoder()
    {
        Impl& m = *impl_;
        cudaSetDevice(m.device);
        if (m.stream != nullptr)
        {
            cudaStreamSynchronize(m.stream);
        }
        if (m.pixels != nullptr)
        {
            cudaFree(m.pixels);
        }
        if (m.state != nullptr)
        {
            nvjpegJpegStateDestroy(m.state);
        }
        if (m.handle != nullptr)
        {
            nvjpegDestroy(m.handle);
        }
        if (m.stream != nullptr)
        {
            cudaStreamDestroy(m.stream);
        }
    }

    std::uint64_t NvJpegDecoder::decode(std::span<const std::uint8_t> jpeg, std::uint32_t width, std::uint32_t height)
    {
        Impl& m = *impl_;
        check(cudaSetDevice(m.device), "cudaSetDevice");
        int components = 0;
        nvjpegChromaSubsampling_t subsampling;
        int widths[NVJPEG_MAX_COMPONENT];
        int heights[NVJPEG_MAX_COMPONENT];
        check(nvjpegGetImageInfo(m.handle, jpeg.data(), jpeg.size(), &components, &subsampling, widths, heights),
              "nvjpegGetImageInfo");
        if (std::uint32_t(widths[0]) != width or std::uint32_t(heights[0]) != height)
        {
            throw std::runtime_error("JPEG frame of " + std::to_string(widths[0]) + "x" + std::to_string(heights[0]) +
                                     ", the colour mode gives " + std::to_string(width) + "x" + std::to_string(height));
        }
        const std::size_t row = std::size_t(width) * 3;
        if (m.capacity < row * height)
        {
            if (m.pixels != nullptr)
            {
                check(cudaStreamSynchronize(m.stream), "cudaStreamSynchronize");
                check(cudaFree(m.pixels), "cudaFree");
            }
            check(cudaMalloc(&m.pixels, row * height), "cudaMalloc");
            m.capacity = row * height;
        }
        nvjpegImage_t out{};
        out.channel[0] = static_cast<unsigned char*>(m.pixels);
        out.pitch[0] = row;
        check(nvjpegDecode(m.handle, m.state, jpeg.data(), jpeg.size(), NVJPEG_OUTPUT_RGBI, &out, m.stream),
              "nvjpegDecode");
        return reinterpret_cast<std::uint64_t>(m.pixels);
    }

    std::uint64_t NvJpegDecoder::stream() const { return reinterpret_cast<std::uint64_t>(impl_->stream); }

    void NvJpegDecoder::synchronize() { check(cudaStreamSynchronize(impl_->stream), "cudaStreamSynchronize"); }
}  // namespace xblick::k4a_node

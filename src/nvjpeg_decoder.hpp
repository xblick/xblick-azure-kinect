// JPEG frames decoded on the GPU with nvJPEG into device memory (rgb8 rows, no padding), for Durchblick's CUDA ring
// without a trip through host memory: the colour frame of a consumer on the GPU (a network, a renderer) never leaves
// the device. Built with DURCHBLICK_K4A_CUDA (CUDA toolkit with nvJPEG).
#pragma once

#include <cstdint>
#include <memory>
#include <span>

namespace durchblick::k4a_node
{
    class NvJpegDecoder
    {
    public:
        explicit NvJpegDecoder(int device);
        ~NvJpegDecoder();
        NvJpegDecoder(const NvJpegDecoder&) = delete;
        NvJpegDecoder& operator=(const NvJpegDecoder&) = delete;

        // Decodes into the decoder's device buffer (valid until the next decode), on stream(); the pixels are
        // ready on that stream when this returns (work is enqueued, not waited for). The JPEG needs its Huffman
        // tables (jpeg.hpp: with_huffman_tables). Checks the expected size.
        std::uint64_t decode(std::span<const std::uint8_t> jpeg, std::uint32_t width, std::uint32_t height);
        // The CUDA stream (a cudaStream_t, usable as a CUstream) the decoded pixels are ready on.
        std::uint64_t stream() const;
        // Waits for the last decode (for a timing or a host read).
        void synchronize();

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };
}  // namespace durchblick::k4a_node

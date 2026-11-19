// JPEG frames decoded on the GPU with nvJPEG into device memory (rgb8 rows, no padding), for Durchblick's CUDA ring
// without a trip through host memory: the colour frame of a consumer on the GPU (a network, a renderer) never leaves
// the device.
//
// Two builds (CMakeLists.txt). The CMake build with XBLICK_K4A_CUDA links nvjpeg_decoder.cpp and gpu_rectify.cpp into
// the library against a CUDA toolkit. The wheel builds both as the module libxblick-azure-kinect-cuda.so against
// NVIDIA's pip packages; its programs open the module when --cuda-device names a GPU (wheel.cpp) and need no CUDA
// library themselves.
#pragma once

#include <cstdint>
#include <memory>
#include <span>

namespace xblick::k4a_node
{
    class NvJpegDecoder
    {
    public:
        virtual ~NvJpegDecoder() = default;

        // Decodes into the decoder's device buffer (valid until the next decode), on stream(); the pixels are
        // ready on that stream when this returns (work is enqueued, not waited for). The JPEG needs its Huffman
        // tables (jpeg.hpp: with_huffman_tables). Checks the expected size.
        virtual std::uint64_t decode(std::span<const std::uint8_t> jpeg, std::uint32_t width,
                                     std::uint32_t height) = 0;
        // The CUDA stream (a cudaStream_t, usable as a CUstream) the decoded pixels are ready on.
        virtual std::uint64_t stream() const = 0;
        // Waits for the last decode (for a timing or a host read).
        virtual void synchronize() = 0;
    };

    // A decoder on CUDA device `device`.
    std::unique_ptr<NvJpegDecoder> make_nvjpeg_decoder(int device);

    // Throws, naming what is missing, unless the NVIDIA driver loads, supports this build's CUDA and shows GPU
    // `device`. The program calls it at the start, before the CUDA calls whose errors would not say it (cudaSetDevice
    // says "CUDA driver version is insufficient" also when there is no driver).
    void check_cuda_device(int device);
}  // namespace xblick::k4a_node

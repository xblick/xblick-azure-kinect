// The colour image rectified (<base>/color_rect, as ROS's image_proc does it): without its lens distortion, in the
// camera that the colour CameraInfo's P describes (the device's calibration: P = K, the same focal length and centre;
// a calibration file: its own P). One map per calibration gives every rectified pixel the point of the camera's image
// it shows (OpenCV's initUndistortRectifyMap with R = I); per frame a bilinear remap, on the CPU or on the GPU
// (gpu_rectify.cu). A rectified pixel whose point is not inside the camera's image is black.
#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "xblick/msg.hpp"

namespace xblick::k4a_node
{
    // The CameraInfo of the rectified image: K = P's left 3x3, no distortion (plumb_bob, zeros), R = I, P as given.
    xblick::CameraInfo rectified_info(const xblick::CameraInfo& raw);

    struct RectifyMap
    {
        std::uint32_t width = 0, height = 0;
        std::vector<float> x, y;  // per rectified pixel: where in the camera's image (pixel centres at integers)
        // the same for the CPU's remap, precomputed: the top-left neighbour's byte offset (-1: outside, black), the
        // weights of the right and the lower neighbours in 1/256, and whether those exist (bit 0: right, 1: lower;
        // at the last column or row the point lies on it and its weight is 0)
        std::vector<std::int32_t> offset;
        std::vector<std::uint16_t> wx, wy;
        std::vector<std::uint8_t> neighbours;
        // for images whose values must not be blended: the nearest pixel's index (-1: outside)
        std::vector<std::int32_t> nearest;
    };

    // The map of a camera (plumb_bob or rational_polynomial; R must be the identity, P a monocular camera's: no
    // skew, no translation). Throws otherwise.
    RectifyMap rectify_map(const xblick::CameraInfo& raw);

    // Bilinear remap of an rgb8 image (rows without padding) of the map's size into dst (the same size).
    void remap_rgb8(const RectifyMap& map, const std::uint8_t* src, std::uint8_t* dst);

    // Nearest-neighbour remap of a depth image in the camera's geometry (rows without padding) into dst (the same size),
    // 0 (no depth) outside: blending would make up depths between a surface and what is behind it, or a hole.
    void remap_depth16(const RectifyMap& map, const std::uint16_t* src, std::uint16_t* dst);

    // The remap on the GPU (bilinear, as NPP's nppiRemap), from and to device memory (gpu_rectify.cu; in the wheel
    // part of its CUDA module, nvjpeg_decoder.hpp).
    class GpuRectifier
    {
    public:
        virtual ~GpuRectifier() = default;

        // Remaps the rgb8 image at src (device memory, rows without padding) on stream (a cudaStream_t) into the
        // rectifier's buffer, valid until the next call; the result is ready on that stream when this returns.
        virtual std::uint64_t rectify(std::uint64_t src, std::uint64_t stream) = 0;
    };

    // A rectifier on CUDA device `device` for `map`.
    std::unique_ptr<GpuRectifier> make_gpu_rectifier(int device, const RectifyMap& map);
}  // namespace xblick::k4a_node

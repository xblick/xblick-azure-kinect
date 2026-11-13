// Depth seen from the colour camera, on the CPU: <base>/depth_to_color_rect and <base>/depth_to_color without the
// SDK's transformation. The SDK computes the transformation in its closed depth engine with OpenGL; on a GPU that other
// programs keep busy (CUDA, MPS) its OpenGL work waits for its time slices: 15-18 ms a frame on a camera PC whose GPU
// other nodes shared under MPS (2026-11-10), half of a 30 Hz frame period. Here it is a projection on the CPU, a few
// milliseconds.
//
// Each depth pixel with a depth z is the point z a + t in the colour camera's frame (a: the pixel's ray, z = 1, turned
// into the colour camera by the extrinsics' rotation; t: their translation, mm), projected into the rectified colour
// camera (color_rect's pinhole camera, P). It covers the rectified pixels whose centres lie within its footprint there
// (the depth pixel's angular size, scaled by z / Z): no gaps between neighbours on one surface. Where several depth
// pixels fall on one colour pixel the nearest wins (z-buffer), as with the SDK. The value is Z, the point's z in the
// colour camera's frame [mm], as the SDK's, across the footprint interpolated along the surface: its slope in the colour
// image from the neighbouring depth pixels (left/right, above/below) where they lie on the same surface (a step of at
// most 30 mm or 3 %), so that a slanted surface (a floor, a table seen at an angle) has no steps from pixel to pixel,
// as the SDK's triangles have none; at an edge the footprint keeps its pixel's depth. Without interpolation (the
// constructor's `interpolate`) every footprint has its pixel's depth throughout: half the time, steps of a few
// millimetres on slanted surfaces.
//
// Flying pixels: at an object's edge the depth engine gives some pixels a depth between the object and what is behind
// it (a pixel that sees both), points that hover in the air. A depth pixel whose neighbours along an image axis, at
// distance 1 or 2, lie one clearly nearer and the other clearly farther (each by more than 30 mm or 3 %) is in such a
// gap and is left out (drop_flying), as the SDK's transformation leaves most of them out; an edge between two surfaces
// keeps its pixels, whose neighbour on one side is on their own surface.
//
// Sparse pixels: the SDK draws the depth image as squares between 2x2 neighbouring pixels, so a depth pixel that is
// part of no complete square on one surface (a scattered single pixel, a streak one pixel wide; often the ambiguous
// depths beyond the camera's range) does not appear in its depth_to_color. drop_sparse leaves such pixels out too. A rectified pixel whose ray is outside the colour camera's image is 0, as
// color_rect is black there.
//
// depth_to_color (the colour camera's own image, with its distortion) is the rectified result read at each colour
// pixel's rectified position (nearest pixel, a table made once): a depth is never blended with another. The colour
// image's outer pixels lie outside the rectified image (barrel distortion), so the projection runs on a rectified
// canvas with a margin around the image that is as wide as those positions need; depth_to_color_rect is its inside.
//
// The rays (depth camera) and the rectified positions (colour camera) come from the SDK's own calibration functions
// (k4a_calibration_2d_to_3d), once at construction, so that both cameras' lens models are exactly the SDK's.
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <k4a/k4a.h>

#include "xblick/msg.hpp"
#include "rectify.hpp"

namespace xblick::k4a_node
{
    class DepthToColor
    {
    public:
        // calibration: the SDK's (CalibrationRecord::calibration(), with a calibration file's intrinsics); color: the
        // colour camera's CameraInfo (CalibrationRecord::color_info()), whose P is the rectified camera; map: its
        // rectify map (rectify_map(color)), for the pixels outside the colour image. interpolate: the depth follows the
        // surface's slope across a footprint.
        DepthToColor(const k4a_calibration_t& calibration, const xblick::CameraInfo& color, const RectifyMap& map,
                     bool interpolate = true, bool drop_flying = true, bool drop_sparse = true);

        // depth pixels left out as sparse in the last rectified() frame
        std::size_t sparse() const { return sparse_; }

        // depth pixels left out as flying pixels in the last rectified() frame
        std::size_t flying() const { return flying_; }

        std::uint32_t depth_width() const { return depth_width_; }
        std::uint32_t depth_height() const { return depth_height_; }

        // The depth image (DEPTH16 of the depth camera, rows `depth_stride` bytes apart) seen from the rectified colour
        // camera, into out (the colour camera's size, rows without padding). Keeps the whole canvas for unrectify().
        // Not for two threads at once.
        void rectified(const std::uint16_t* depth, std::size_t depth_stride, std::uint16_t* out);

        // The last rectified() frame seen from the colour camera itself (with its distortion): each pixel the canvas's
        // nearest pixel at its rectified position, 0 where that is outside it. out: the colour camera's size.
        void unrectify(std::uint16_t* out) const;

        // the canvas's margin around the rectified image [pixels]
        std::uint32_t margin() const { return margin_; }

    private:
        bool interpolate_ = true;
        bool drop_flying_ = true;
        bool drop_sparse_ = true;
        std::size_t flying_ = 0, sparse_ = 0;
        std::vector<std::uint32_t> flying_list_;
        std::uint32_t depth_width_ = 0, depth_height_ = 0;
        std::uint32_t width_ = 0, height_ = 0;            // the colour camera's
        std::uint32_t margin_ = 0;                        // the canvas's, on every side
        std::uint32_t canvas_width_ = 0, canvas_height_ = 0;
        std::vector<std::uint16_t> canvas_;               // the last frame, rectified with the margin
        float fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0;         // the rectified colour camera (P)
        float t_[3] = {0, 0, 0};                          // depth -> colour translation [mm]
        std::vector<float> ray_;                          // per depth pixel: R ray (3 floats); NaN: no ray
        std::vector<float> half_x_, half_y_;              // per depth pixel: half its angular size (z = 1)
        std::vector<std::int32_t> outside_;               // rectified pixels whose ray is outside the colour image
        std::vector<std::int32_t> from_rectified_;        // per colour pixel: its canvas pixel (-1: none)
        std::vector<float> pu_, pv_, ps_, pz_;            // per depth pixel of the frame: canvas position, footprint
                                                          // scale z / Z, Z (0: none)
    };
}  // namespace xblick::k4a_node

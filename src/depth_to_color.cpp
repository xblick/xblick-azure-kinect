#include "depth_to_color.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace durchblick::k4a_node
{
    namespace
    {
        // a footprint wider than this (pixels, half) is cut: only a depth pixel very near the colour camera, or a
        // broken calibration, would reach it, and one such pixel must not cost a frame its time
        constexpr float kMaxHalf = 8.0f;
        // added to each side of a footprint [pixels]: the depth pixels' grid and the colour pixels' do not line up
        // (the cameras' rotation, their distortions), and footprints that only touch miss a pixel centre now and then
        // (holes, about 1.6 % of a surface's pixels without it). Overlaps are resolved by the z-buffer.
        constexpr float kOverlap = 0.25f;
    }  // namespace

    DepthToColor::DepthToColor(const k4a_calibration_t& calibration, const durchblick::CameraInfo& color,
                               const RectifyMap& map, bool interpolate, bool drop_flying, bool drop_sparse)
        : interpolate_(interpolate), drop_flying_(drop_flying), drop_sparse_(drop_sparse)
    {
        const k4a_calibration_camera_t& depth_camera = calibration.depth_camera_calibration;
        depth_width_ = std::uint32_t(depth_camera.resolution_width);
        depth_height_ = std::uint32_t(depth_camera.resolution_height);
        width_ = color.width;
        height_ = color.height;
        if (depth_width_ == 0 or depth_height_ == 0 or width_ == 0 or height_ == 0)
        {
            throw std::invalid_argument("depth_to_color: the depth or the colour camera is off");
        }
        if (map.width != width_ or map.height != height_ or map.nearest.size() != std::size_t(width_) * height_)
        {
            throw std::invalid_argument("depth_to_color: the rectify map is not the colour camera's");
        }
        if (std::uint32_t(calibration.color_camera_calibration.resolution_width) != width_ or
            std::uint32_t(calibration.color_camera_calibration.resolution_height) != height_)
        {
            throw std::invalid_argument("depth_to_color: the calibration's colour resolution is not the CameraInfo's");
        }
        const durchblick::CameraInfo rect = rectified_info(color);
        fx_ = float(rect.p[0]);
        cx_ = float(rect.p[2]);
        fy_ = float(rect.p[5]);
        cy_ = float(rect.p[6]);

        // the extrinsics map depth to colour: p_colour = R p_depth + t (R row-major, t in mm)
        const k4a_calibration_extrinsics_t& e =
            calibration.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR];
        for (int i = 0; i < 3; ++i)
        {
            t_[i] = e.translation[i];
        }

        // the depth pixels' rays (z = 1) in the depth camera, from the SDK's lens model
        const std::size_t n = std::size_t(depth_width_) * depth_height_;
        constexpr float nan = std::numeric_limits<float>::quiet_NaN();
        std::vector<float> rx(n, nan), ry(n, nan);
        for (std::uint32_t v = 0; v < depth_height_; ++v)
        {
            for (std::uint32_t u = 0; u < depth_width_; ++u)
            {
                const k4a_float2_t pixel{{float(u), float(v)}};
                k4a_float3_t p;
                int valid = 0;
                if (k4a_calibration_2d_to_3d(&calibration, &pixel, 1000.0f, K4A_CALIBRATION_TYPE_DEPTH,
                                             K4A_CALIBRATION_TYPE_DEPTH, &p, &valid) == K4A_RESULT_SUCCEEDED and
                    valid)
                {
                    const std::size_t i = std::size_t(v) * depth_width_ + u;
                    rx[i] = p.xyz.x / 1000.0f;
                    ry[i] = p.xyz.y / 1000.0f;
                }
            }
        }
        // turned into the colour camera, and each pixel's half size from its neighbours' rays (one-sided at an edge
        // or next to a pixel without a ray; none at all: the focal length's)
        const float fallback_x = 0.5f / depth_camera.intrinsics.parameters.param.fx;
        const float fallback_y = 0.5f / depth_camera.intrinsics.parameters.param.fy;
        ray_.assign(3 * n, nan);
        half_x_.assign(n, fallback_x);
        half_y_.assign(n, fallback_y);
        const auto half = [&](const std::vector<float>& r, std::size_t i, std::size_t before, std::size_t after,
                              bool has_before, bool has_after, float fallback) {
            const bool b = has_before and not std::isnan(r[before]);
            const bool a = has_after and not std::isnan(r[after]);
            if (a and b)
            {
                return std::abs(r[after] - r[before]) / 4.0f;
            }
            if (a)
            {
                return std::abs(r[after] - r[i]) / 2.0f;
            }
            if (b)
            {
                return std::abs(r[i] - r[before]) / 2.0f;
            }
            return fallback;
        };
        for (std::uint32_t v = 0; v < depth_height_; ++v)
        {
            for (std::uint32_t u = 0; u < depth_width_; ++u)
            {
                const std::size_t i = std::size_t(v) * depth_width_ + u;
                if (std::isnan(rx[i]))
                {
                    continue;
                }
                const float x = rx[i], y = ry[i];
                for (int k = 0; k < 3; ++k)
                {
                    ray_[3 * i + k] = e.rotation[3 * k] * x + e.rotation[3 * k + 1] * y + e.rotation[3 * k + 2];
                }
                half_x_[i] = half(rx, i, i - 1, i + 1, u > 0, u + 1 < depth_width_, fallback_x);
                half_y_[i] = half(ry, i, i - depth_width_, i + depth_width_, v > 0, v + 1 < depth_height_, fallback_y);
            }
        }

        // the rectified pixels outside the colour image (black in color_rect)
        for (std::size_t i = 0; i < map.nearest.size(); ++i)
        {
            if (map.nearest[i] < 0)
            {
                outside_.push_back(std::int32_t(i));
            }
        }
        // each colour pixel's rectified position: its ray (the SDK's lens model) through the rectified camera; the
        // canvas's margin is as wide as the positions outside the rectified image need (cut at kMaxMargin)
        constexpr long kMaxMargin = 256;
        std::vector<long> ru(std::size_t(width_) * height_, 0), rv(ru.size(), 0);
        std::vector<bool> has(ru.size(), false);
        long overflow = 0;
        for (std::uint32_t y = 0; y < height_; ++y)
        {
            for (std::uint32_t x = 0; x < width_; ++x)
            {
                const k4a_float2_t pixel{{float(x), float(y)}};
                k4a_float3_t p;
                int valid = 0;
                if (k4a_calibration_2d_to_3d(&calibration, &pixel, 1000.0f, K4A_CALIBRATION_TYPE_COLOR,
                                             K4A_CALIBRATION_TYPE_COLOR, &p, &valid) != K4A_RESULT_SUCCEEDED or
                    not valid)
                {
                    continue;
                }
                const std::size_t i = std::size_t(y) * width_ + x;
                ru[i] = std::lround(fx_ * p.xyz.x / p.xyz.z + cx_);
                rv[i] = std::lround(fy_ * p.xyz.y / p.xyz.z + cy_);
                has[i] = true;
                overflow = std::max({overflow, -ru[i], -rv[i], ru[i] - long(width_) + 1, rv[i] - long(height_) + 1});
            }
        }
        margin_ = std::uint32_t(std::clamp(overflow, 0L, kMaxMargin));
        canvas_width_ = width_ + 2 * margin_;
        canvas_height_ = height_ + 2 * margin_;
        canvas_.assign(std::size_t(canvas_width_) * canvas_height_, 0);
        from_rectified_.assign(ru.size(), -1);
        for (std::size_t i = 0; i < ru.size(); ++i)
        {
            const long cu = ru[i] + long(margin_), cv = rv[i] + long(margin_);
            if (has[i] and cu >= 0 and cv >= 0 and cu < long(canvas_width_) and cv < long(canvas_height_))
            {
                from_rectified_[i] = std::int32_t(cv * long(canvas_width_) + cu);
            }
        }
    }

    void DepthToColor::rectified(const std::uint16_t* depth, std::size_t depth_stride, std::uint16_t* out)
    {
        // on the canvas: the rectified camera's pixel (x, y) is the canvas's (x + margin, y + margin)
        const int w = int(canvas_width_), h = int(canvas_height_);
        const float m = float(margin_);
        const std::size_t n = std::size_t(depth_width_) * depth_height_;
        pu_.resize(n);
        pv_.resize(n);
        ps_.resize(n);
        pz_.assign(n, 0.0f);
        // 1: every depth pixel's position on the canvas and its Z
        for (std::uint32_t v = 0; v < depth_height_; ++v)
        {
            const auto* row = reinterpret_cast<const std::uint16_t*>(reinterpret_cast<const std::uint8_t*>(depth) +
                                                                     std::size_t(v) * depth_stride);
            for (std::uint32_t u = 0; u < depth_width_; ++u)
            {
                const std::uint16_t z = row[u];
                const std::size_t i = std::size_t(v) * depth_width_ + u;
                const float* a = &ray_[3 * i];
                if (z == 0 or std::isnan(a[0]))
                {
                    continue;
                }
                const float X = float(z) * a[0] + t_[0];
                const float Y = float(z) * a[1] + t_[1];
                const float Z = float(z) * a[2] + t_[2];
                if (not(Z > 0.0f))
                {
                    continue;
                }
                const float inv = 1.0f / Z;
                pu_[i] = fx_ * X * inv + cx_ + m;
                pv_[i] = fy_ * Y * inv + cy_ + m;
                ps_[i] = float(z) * inv;  // the footprint's scale: the depth pixel's size at z, seen from Z away
                pz_[i] = Z;
            }
        }
        // flying pixels: in the gap between a nearer neighbour on one side and a farther one on the other, along an
        // axis at distance 1 or 2 (found on the projected depths, then left out together)
        flying_list_.clear();
        if (drop_flying_)
        {
            const auto in_gap = [&](std::size_t i, std::size_t a, std::size_t b) {
                const float z = pz_[i], za = pz_[a], zb = pz_[b];
                if (za == 0.0f or zb == 0.0f)
                {
                    return false;
                }
                const float t = std::max(30.0f, 0.03f * z);
                return (za < z - t and zb > z + t) or (zb < z - t and za > z + t);
            };
            for (std::uint32_t v = 0; v < depth_height_; ++v)
            {
                for (std::uint32_t u = 0; u < depth_width_; ++u)
                {
                    const std::size_t i = std::size_t(v) * depth_width_ + u;
                    if (pz_[i] == 0.0f)
                    {
                        continue;
                    }
                    bool gap = false;
                    for (std::uint32_t k = 1; k <= 2 and not gap; ++k)
                    {
                        const std::size_t row = std::size_t(k) * depth_width_;
                        gap = (u >= k and u + k < depth_width_ and in_gap(i, i - k, i + k)) or
                              (v >= k and v + k < depth_height_ and in_gap(i, i - row, i + row));
                    }
                    if (gap)
                    {
                        flying_list_.push_back(std::uint32_t(i));
                    }
                }
            }
            for (const std::uint32_t i : flying_list_)
            {
                pz_[i] = 0.0f;
            }
        }
        flying_ = flying_list_.size();
        // sparse pixels: part of no complete 2x2 square of depth pixels on one surface (as the SDK draws them)
        sparse_ = 0;
        if (drop_sparse_)
        {
            const auto near = [&](std::size_t i, std::size_t k) {
                return pz_[k] > 0.0f and std::abs(pz_[k] - pz_[i]) <= std::max(30.0f, 0.03f * pz_[i]);
            };
            flying_list_.clear();  // reused for the sparse ones
            const std::size_t row = depth_width_;
            for (std::uint32_t v = 0; v < depth_height_; ++v)
            {
                for (std::uint32_t u = 0; u < depth_width_; ++u)
                {
                    const std::size_t i = std::size_t(v) * row + u;
                    if (pz_[i] == 0.0f)
                    {
                        continue;
                    }
                    bool square = false;
                    for (int dv = -1; dv <= 0 and not square; ++dv)
                    {
                        for (int du = -1; du <= 0 and not square; ++du)
                        {
                            const long u0 = long(u) + du, v0 = long(v) + dv;
                            if (u0 < 0 or v0 < 0 or u0 + 1 >= long(depth_width_) or v0 + 1 >= long(depth_height_))
                            {
                                continue;
                            }
                            const std::size_t a = std::size_t(v0) * row + std::size_t(u0);
                            square = near(i, a) and near(i, a + 1) and near(i, a + row) and near(i, a + row + 1);
                        }
                    }
                    if (not square)
                    {
                        flying_list_.push_back(std::uint32_t(i));
                    }
                }
            }
            for (const std::uint32_t i : flying_list_)
            {
                pz_[i] = 0.0f;
            }
            sparse_ = flying_list_.size();
        }
        // 2: each footprint, with the surface's slope from the neighbours on the same surface
        std::fill(canvas_.begin(), canvas_.end(), std::uint16_t(0));
        const auto same_surface = [&](std::size_t i, std::size_t k) {
            return pz_[k] > 0.0f and std::abs(pz_[k] - pz_[i]) <= std::max(30.0f, 0.03f * pz_[i]);
        };
        // the step to the neighbours along one axis of the depth image: central, one-sided, or none (no slope)
        // (lo, hi: the depths of the neighbours taken, which the footprint's interpolated depths stay within)
        const auto step = [&](std::size_t i, bool has_before, std::size_t before, bool has_after, std::size_t after,
                              float& du, float& dv, float& dz, float& lo, float& hi) {
            const bool b = has_before and same_surface(i, before);
            const bool a = has_after and same_surface(i, after);
            const std::size_t from = b ? before : i, to = a ? after : i;
            if (from == to)
            {
                return false;
            }
            du = pu_[to] - pu_[from];
            dv = pv_[to] - pv_[from];
            dz = pz_[to] - pz_[from];
            lo = std::min({lo, pz_[from], pz_[to]});
            hi = std::max({hi, pz_[from], pz_[to]});
            return true;
        };
        for (std::uint32_t v = 0; v < depth_height_; ++v)
        {
            for (std::uint32_t u = 0; u < depth_width_; ++u)
            {
                const std::size_t i = std::size_t(v) * depth_width_ + u;
                const float Z = pz_[i];
                if (Z == 0.0f)
                {
                    continue;
                }
                const float uc = pu_[i], vc = pv_[i];
                // the slope dZ/dx, dZ/dy on the canvas: the two steps' (du, dv) -> dz, solved; along an axis without
                // a neighbour on the surface no change of depth
                float dux = 1, dvx = 0, dzx = 0, duy = 0, dvy = 1, dzy = 0;
                float gx = 0, gy = 0;
                // interpolated only between the pixel's own depth and its neighbours', never beyond them: near an edge
                // a slope would otherwise carry the footprint out into the gap (points hovering in the air)
                float lo = Z, hi = Z;
                if (interpolate_)
                {
                    step(i, u > 0, i - 1, u + 1 < depth_width_, i + 1, dux, dvx, dzx, lo, hi);
                    step(i, v > 0, i - depth_width_, v + 1 < depth_height_, i + depth_width_, duy, dvy, dzy, lo, hi);
                }
                const float det = dux * dvy - dvx * duy;
                if (interpolate_ and std::abs(det) > 1e-6f)
                {
                    gx = (dzx * dvy - dvx * dzy) / det;
                    gy = (dux * dzy - dzx * duy) / det;
                }
                // the footprint: the pixel's angular size at its depth, seen from Z away
                const float scale = ps_[i];
                const float hx = std::min(kMaxHalf, fx_ * half_x_[i] * scale + kOverlap);
                const float hy = std::min(kMaxHalf, fy_ * half_y_[i] * scale + kOverlap);
                // the pixel centres (integers) within it; at least the nearest one
                int x0 = int(std::ceil(uc - hx)), x1 = int(std::floor(uc + hx));
                int y0 = int(std::ceil(vc - hy)), y1 = int(std::floor(vc + hy));
                if (x0 > x1)
                {
                    x0 = x1 = int(std::lround(uc));
                }
                if (y0 > y1)
                {
                    y0 = y1 = int(std::lround(vc));
                }
                x0 = std::max(x0, 0);
                y0 = std::max(y0, 0);
                x1 = std::min(x1, w - 1);
                y1 = std::min(y1, h - 1);
                for (int y = y0; y <= y1; ++y)
                {
                    std::uint16_t* o = canvas_.data() + std::size_t(y) * canvas_width_;
                    const float zy = Z + gy * (float(y) - vc);
                    for (int x = x0; x <= x1; ++x)
                    {
                        const long rounded = std::lround(std::clamp(zy + gx * (float(x) - uc), lo, hi));
                        const std::uint16_t zc = std::uint16_t(std::clamp(rounded, 1L, 65535L));
                        if (o[x] == 0 or zc < o[x])
                        {
                            o[x] = zc;
                        }
                    }
                }
            }
        }
        // the rectified image: the canvas's inside, 0 where its ray is outside the colour image
        for (std::uint32_t y = 0; y < height_; ++y)
        {
            const std::uint16_t* row = canvas_.data() + std::size_t(y + margin_) * canvas_width_ + margin_;
            std::copy(row, row + width_, out + std::size_t(y) * width_);
        }
        for (const std::int32_t i : outside_)
        {
            out[i] = 0;
        }
    }

    void DepthToColor::unrectify(std::uint16_t* out) const
    {
        for (std::size_t i = 0; i < from_rectified_.size(); ++i)
        {
            const std::int32_t at = from_rectified_[i];
            out[i] = at < 0 ? std::uint16_t(0) : canvas_[std::size_t(at)];
        }
    }
}  // namespace durchblick::k4a_node

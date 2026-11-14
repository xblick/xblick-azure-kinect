// k4a_depth_to_color_check: a running camera node's depth_to_color (the CPU's, depth_to_color.hpp) against the SDK's
// transformation of the same depth frame, live.
//
//   k4a_depth_to_color_check [--base cam/azurekinect] [--seconds 10] [--depth-engine-gpu 0] [--probe-max-depth]
//
// Reads <base>/depth and <base>/depth_to_color from shared memory (on the camera's PC, through its Zenoh router),
// pairs them by stamp, runs the SDK's transformation (its depth engine, in an EGL context as the node's) on the depth
// frame with the node's calibration (<base>/k4a_calibration) and compares pixel by pixel, as the offline test does:
// where both have a depth, the share within 5 mm; the pixels only one of them has. One line per frame, a summary at
// the end. Exit status 0 when every frame agrees (99 % within 5 mm, under 2 % covered by only one). It also computes
// the CPU's depth_to_color itself from the same frames, with and without interpolation, against the same reference,
// and counts stray points (between surfaces, isolated, far outliers, beyond 5 m) in each and in the depth image.
// --probe-max-depth: instead, with the node's calibration, the largest constant depth the SDK's transformation shows
// and its threshold for neighbours on one surface (a checkerboard of Z -+ k): about 4.5 % of Z, no distance cut.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include <k4a/k4a.hpp>

#include "args.hpp"
#include "xblick/camera.hpp"
#include "egl_glx.hpp"
#include "depth_to_color.hpp"
#include "node.hpp"
#include "rectify.hpp"

namespace
{
    struct Comparison
    {
        std::size_t both = 0, agree = 0, only_sdk = 0, only_cpu = 0, over20 = 0;
        double share() const { return both ? 100.0 * double(agree) / double(both) : 0.0; }
    };

    // b against a (the reference), pixel by pixel; b's rows `bstep` pixels apart
    Comparison compare(const std::uint16_t* a, const std::uint16_t* b, std::uint32_t bstep, int w, int h)
    {
        Comparison c;
        for (int y = 0; y < h; ++y)
        {
            for (int x = 0; x < w; ++x)
            {
                const std::uint16_t p = a[std::size_t(y) * w + x], q = b[std::size_t(y) * bstep + x];
                if (p and q)
                {
                    ++c.both;
                    const int e = std::abs(int(p) - int(q));
                    c.agree += e <= 5;
                    c.over20 += e > 20;
                }
                else
                {
                    c.only_sdk += p and not q;
                    c.only_cpu += q and not p;
                }
            }
        }
        return c;
    }

    // Stray points in a depth image: "between", a depth in the gap of a near and a far surface (its 5x5 neighbourhood
    // spans more than 100 mm, and it is more than 30 mm from both the nearest and the farthest), which a point cloud
    // shows hovering in the air at an object's edge; "isolated", no neighbour (3x3) with a depth within 30 mm.
    struct Stray
    {
        double between = 0, isolated = 0;
        double far = 0;      // clearly farther than its surroundings: over max(300 mm, 15 %) beyond the 5x5 median
        double beyond5 = 0;  // depths over 5 m
        double deepest = 0;  // the largest depth [mm] (summed over frames: a mean)
    };

    Stray stray(const std::uint16_t* image, std::uint32_t step, int w, int h)
    {
        Stray s;
        for (int y = 2; y < h - 2; ++y)
        {
            for (int x = 2; x < w - 2; ++x)
            {
                const int z = image[std::size_t(y) * step + x];
                if (z == 0)
                {
                    continue;
                }
                int lo = z, hi = z;
                bool close = false, any = false;
                int around[24];
                int count = 0;
                for (int dy = -2; dy <= 2; ++dy)
                {
                    for (int dx = -2; dx <= 2; ++dx)
                    {
                        const int n = image[std::size_t(y + dy) * step + x + dx];
                        if (n == 0 or (dx == 0 and dy == 0))
                        {
                            continue;
                        }
                        lo = std::min(lo, n);
                        hi = std::max(hi, n);
                        around[count++] = n;
                        if (std::abs(dx) <= 1 and std::abs(dy) <= 1)
                        {
                            any = true;
                            close = close or std::abs(n - z) <= 30;
                        }
                    }
                }
                s.between += hi - lo > 100 and z - lo > 30 and hi - z > 30;
                s.isolated += any and not close;
                if (count >= 8)
                {
                    std::nth_element(around, around + count / 2, around + count);
                    const int median = around[count / 2];
                    s.far += z - median > std::max(300.0, 0.15 * median);
                }
                s.beyond5 += z > 5000;
                s.deepest = std::max(s.deepest, double(z));
            }
        }
        return s;
    }
}  // namespace

int main(int argc, char** argv)
{
    using namespace xblick;
    using namespace xblick::k4a_node;
    try
    {
        Args args(argc, argv);
        const std::string base = args.get("base", "cam/azurekinect");
        const double seconds = std::stod(args.get("seconds", "10"));
        use_egl_for_depth_engine(std::stoi(args.get("depth-engine-gpu", "0")));
        zenoh::Session session = open_session({});
        const CalibrationRecord record = query_calibration(session, base);
        const k4a_calibration_t calibration = record.calibration();
        k4a_transformation_t handle = k4a_transformation_create(&calibration);
        if (handle == nullptr)
        {
            throw std::runtime_error("the SDK's transformation could not be created (depth engine on LD_LIBRARY_PATH?)");
        }
        k4a::transformation sdk(handle);
        if (args.flag("probe-max-depth"))
        {
            // the largest constant depth the SDK's transformation still shows, with this camera's calibration
            const int pw = calibration.depth_camera_calibration.resolution_width;
            const int ph = calibration.depth_camera_calibration.resolution_height;
            const int cw = calibration.color_camera_calibration.resolution_width;
            const int ch = calibration.color_camera_calibration.resolution_height;
            k4a::image probe = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16, pw, ph, pw * 2);
            k4a::image out = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16, cw, ch, cw * 2);
            auto* p = reinterpret_cast<std::uint16_t*>(probe.get_buffer());
            int kept = 0, dropped = 0;
            for (int z = 2000; z <= 16000; z += 100)
            {
                std::fill(p, p + std::size_t(pw) * ph, std::uint16_t(z));
                sdk.depth_image_to_color_camera(probe, &out);
                const auto* o = reinterpret_cast<const std::uint16_t*>(out.get_buffer());
                std::size_t n = 0;
                for (int k = 0; k < cw * ch; ++k)
                {
                    n += o[k] != 0;
                }
                if (n > 0)
                {
                    kept = z;
                }
                else if (dropped == 0)
                {
                    dropped = z;
                }
            }
            std::printf("the SDK's transformation with this calibration: constant depth kept up to %d mm, first "
                        "dropped %d mm\n", kept, dropped);
            // its threshold for neighbours on one surface: a checkerboard of Z - k and Z + k, the largest k it keeps
            for (const int z : {700, 1000, 1500, 2000, 3000, 4000, 6000, 9000})
            {
                int largest = -1;
                for (int k = 1; k <= 1000; k += (k < 50 ? 1 : 10))
                {
                    for (int y = 0; y < ph; ++y)
                    {
                        for (int x = 0; x < pw; ++x)
                        {
                            p[std::size_t(y) * pw + x] = std::uint16_t(z + (((x + y) & 1) ? k : -k));
                        }
                    }
                    sdk.depth_image_to_color_camera(probe, &out);
                    const auto* o = reinterpret_cast<const std::uint16_t*>(out.get_buffer());
                    std::size_t n = 0;
                    for (int q = 0; q < cw * ch; ++q)
                    {
                        n += o[q] != 0;
                    }
                    if (n > std::size_t(cw) * ch / 10)
                    {
                        largest = k;
                    }
                    else
                    {
                        break;
                    }
                }
                std::printf("  depth %5d mm: neighbours 2k = %d mm apart still kept (%.2f %%)\n", z, 2 * largest,
                            100.0 * 2 * largest / z);
            }
            return 0;
        }
        const int dw = calibration.depth_camera_calibration.resolution_width;
        const int dh = calibration.depth_camera_calibration.resolution_height;
        const int w = calibration.color_camera_calibration.resolution_width;
        const int h = calibration.color_camera_calibration.resolution_height;
        ImageSubscriber depth(session, base + "/depth", {.transport = Transport::Shm, .keep_recent = 60});
        ImageSubscriber d2c(session, base + "/depth_to_color", {.transport = Transport::Shm});
        k4a::image depth_image = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16, dw, dh, dw * 2);
        k4a::image reference = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16, w, h, w * 2);
        // the CPU's depth_to_color here as well, both ways, on the same frames as the node and the SDK
        const CameraInfo color = record.color_info();
        const RectifyMap map = rectify_map(color);
        // the node's defaults (interpolation, flying pixels left out), without interpolation, keeping flying pixels
        DepthToColor interpolated(calibration, color, map, true, true), constant(calibration, color, map, false, true),
            keeping(calibration, color, map, true, false);
        std::vector<std::uint16_t> rect(std::size_t(w) * h), local_on(rect.size()), local_off(rect.size()),
            local_keep(rect.size());
        double sum_keep = 0, flying = 0;
        double sum_on = 0, sum_off = 0, node_vs_on = 0;
        Stray stray_sdk, stray_on, stray_off, stray_keep, stray_depth;  // summed over the frames
        double far_sdk_none = 0, far_sdk_nearer = 0, far_sdk_same = 0;
        const auto add = [](Stray& sum, const Stray& one) {
            sum.between += one.between;
            sum.isolated += one.isolated;
            sum.far += one.far;
            sum.beyond5 += one.beyond5;
            sum.deepest += one.deepest;
        };

        int frames = 0, good = 0, unpaired = 0;
        double sum_agree = 0, sum_only = 0;
        const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
        while (std::chrono::steady_clock::now() < end)
        {
            const auto frame = d2c.latest(true, std::chrono::milliseconds(200));
            if (not frame)
            {
                continue;
            }
            if (frame->width != std::uint32_t(w) or frame->height != std::uint32_t(h) or frame->encoding != "16UC1")
            {
                throw std::runtime_error("depth_to_color is not 16UC1 of the colour camera's size");
            }
            const std::int64_t stamp = frame->header.stamp.to_ns();
            const auto source = depth.at(stamp, std::chrono::nanoseconds(1000));
            if (not source)
            {
                ++unpaired;
                continue;
            }
            const std::uint32_t step = source->step ? source->step : source->width * 2;
            auto* d = reinterpret_cast<std::uint16_t*>(depth_image.get_buffer());
            for (int y = 0; y < dh; ++y)
            {
                std::copy_n(reinterpret_cast<const std::uint16_t*>(source->data() + std::size_t(y) * step), dw,
                            d + std::size_t(y) * dw);
            }
            sdk.depth_image_to_color_camera(depth_image, &reference);
            const auto* r = reinterpret_cast<const std::uint16_t*>(reference.get_buffer());
            const auto* c = reinterpret_cast<const std::uint16_t*>(frame->data());
            const std::uint32_t cstep = frame->step ? frame->step / 2 : std::uint32_t(w);
            // the same depth frame on the CPU here, with and without interpolation
            interpolated.rectified(d, std::size_t(dw) * 2, rect.data());
            interpolated.unrectify(local_on.data());
            flying += double(interpolated.flying());
            constant.rectified(d, std::size_t(dw) * 2, rect.data());
            constant.unrectify(local_off.data());
            keeping.rectified(d, std::size_t(dw) * 2, rect.data());
            keeping.unrectify(local_keep.data());
            sum_keep += compare(r, local_keep.data(), std::uint32_t(w), w, h).share();
            const Comparison mine_on = compare(r, local_on.data(), std::uint32_t(w), w, h);
            const Comparison mine_off = compare(r, local_off.data(), std::uint32_t(w), w, h);
            const Comparison node_cmp = compare(r, c, cstep, w, h);
            const std::size_t both = node_cmp.both, agree = node_cmp.agree, only_sdk = node_cmp.only_sdk,
                              only_cpu = node_cmp.only_cpu;
            sum_on += mine_on.share();
            sum_off += mine_off.share();
            node_vs_on += compare(local_on.data(), c, cstep, w, h).share();
            {
                // where this image has a depth beyond 5 m: what the SDK has there
                for (int k = 0; k < w * h; ++k)
                {
                    if (local_on[std::size_t(k)] > 5000)
                    {
                        const int a = r[k], b = local_on[std::size_t(k)];
                        far_sdk_none += a == 0;
                        far_sdk_nearer += a != 0 and a < b - std::max(300, b / 20);
                        far_sdk_same += a != 0 and std::abs(a - b) <= std::max(300, b / 20);
                    }
                }
            }
            add(stray_sdk, stray(r, std::uint32_t(w), w, h));
            add(stray_on, stray(local_on.data(), std::uint32_t(w), w, h));
            add(stray_off, stray(local_off.data(), std::uint32_t(w), w, h));
            add(stray_keep, stray(local_keep.data(), std::uint32_t(w), w, h));
            add(stray_depth, stray(d, std::uint32_t(dw), dw, dh));
            if (frames % 30 == 0)
            {
                std::printf("  within 5 mm of the SDK, this frame: node %.2f %%, here with interpolation %.2f %%, without "
                            "%.2f %%; over 20 mm: %zu, %zu, %zu\n",
                            node_cmp.share(), mine_on.share(), mine_off.share(), node_cmp.over20, mine_on.over20,
                            mine_off.over20);
            }
            const double share = both ? 100.0 * double(agree) / double(both) : 0.0;
            const double only = both ? 100.0 * double(only_sdk + only_cpu) / double(both) : 100.0;
            ++frames;
            sum_agree += share;
            sum_only += only;
            good += share >= 99.0 and only < 2.0;
            std::printf("frame %d: %zu pixels with both depths, %.2f %% within 5 mm; only the SDK's %zu, only the "
                        "node's %zu (%.2f %%)\n",
                        frames, both, share, only_sdk, only_cpu, only);
            std::fflush(stdout);
        }
        std::printf("%d frames compared (%d without their depth frame): %d agree; mean %.2f %% within 5 mm, %.2f %% "
                    "covered by only one\n",
                    frames, unpaired, good, frames ? sum_agree / frames : 0.0, frames ? sum_only / frames : 0.0);
        std::printf("the same frames here, within 5 mm of the SDK: defaults (interpolation, flying pixels left out) "
                    "%.2f %%, without interpolation %.2f %%, keeping flying pixels %.2f %%; the node's against the "
                    "defaults here %.2f %%; %.0f depth pixels left out as flying a frame\n",
                    frames ? sum_on / frames : 0.0, frames ? sum_off / frames : 0.0,
                    frames ? sum_keep / frames : 0.0, frames ? node_vs_on / frames : 0.0,
                    frames ? flying / frames : 0.0);
        if (frames)
        {
            const auto line = [&](const char* what, const Stray& t) {
                std::printf("  %-38s %9.0f between surfaces, %8.0f isolated, %6.0f far outliers, %7.0f beyond 5 m a "
                            "frame; deepest %.0f mm\n",
                            what, t.between / frames, t.isolated / frames, t.far / frames, t.beyond5 / frames,
                            t.deepest / frames);
            };
            std::printf("stray points (depth in the gap between a near and a far surface; no neighbour within 30 mm):\n");
            line("the SDK's transformation", stray_sdk);
            line("CPU, defaults (flying pixels left out)", stray_on);
            line("CPU, without interpolation", stray_off);
            line("CPU, keeping flying pixels", stray_keep);
            line("(the depth image itself, 640x576)", stray_depth);
            std::printf("the defaults' pixels beyond 5 m, a frame: the SDK has no depth there %.0f, a nearer one %.0f, "
                        "about the same %.0f\n",
                        far_sdk_none / frames, far_sdk_nearer / frames, far_sdk_same / frames);
        }
        return frames > 0 and good == frames ? 0 : 1;
    }
    catch (const std::exception& e)
    {
        std::fprintf(stderr, "k4a_depth_to_color_check: %s\n", e.what());
        return 2;
    }
}

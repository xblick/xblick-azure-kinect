// k4a_depth_to_color_mirror: a running camera node's depth frames, turned into depth_to_color and depth_to_color_rect
// another way, published below another base: for looking at two ways side by side on the same frames (one camera can
// be opened by one program only).
//
//   k4a_depth_to_color_mirror [--base robot/azurekinect] [--output robot/azurekinect_sdk]
//                             [--variant sdk|cpu|cpu-no-interpolation|cpu-keep-flying] [--depth-engine-gpu 0]
//
// Reads <base>/depth from shared memory (on the camera's PC, through its Zenoh router) with the node's calibration
// (<base>/k4a_calibration) and publishes <output>/depth_to_color and <output>/depth_to_color_rect with the same
// CameraInfo, frame, stamp and seq as the node's own (so that viewers and nodes pair them with <base>/color): sdk the
// SDK's transformation (the node's --sdk-depth-to-color; depth_to_color_rect its nearest-pixel remap), cpu the CPU's
// with the node's defaults, cpu-no-interpolation and cpu-keep-flying with one of them off (depth_to_color.hpp). The
// newest depth frame each time: a slower variant skips frames (the status line says how many). Until SIGINT.
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <string>
#include <vector>

#include <k4a/k4a.hpp>

#include "args.hpp"
#include "depth_to_color.hpp"
#include "durchblick/camera.hpp"
#include "egl_glx.hpp"
#include "node.hpp"
#include "rectify.hpp"

namespace
{
    std::atomic<bool> stop{false};
}  // namespace

int main(int argc, char** argv)
{
    using namespace durchblick;
    using namespace durchblick::k4a_node;
    std::signal(SIGINT, [](int) { stop = true; });
    std::signal(SIGTERM, [](int) { stop = true; });
    try
    {
        Args args(argc, argv);
        const std::string base = args.get("base", "robot/azurekinect");
        const std::string output = check_name(args.get("output", "robot/azurekinect_sdk"));
        const std::string variant = args.get("variant", "sdk");
        if (variant != "sdk" and variant != "cpu" and variant != "cpu-no-interpolation" and variant != "cpu-keep-flying")
        {
            throw std::invalid_argument("--variant " + variant + ": sdk, cpu, cpu-no-interpolation or cpu-keep-flying");
        }
        use_egl_for_depth_engine(std::stoi(args.get("depth-engine-gpu", "0")));
        zenoh::Session session = open_session({});
        const CalibrationRecord record = query_calibration(session, base);
        const k4a_calibration_t calibration = record.calibration();
        const CameraInfo color = record.color_info();
        const RectifyMap map = rectify_map(color);
        const int dw = calibration.depth_camera_calibration.resolution_width;
        const int dh = calibration.depth_camera_calibration.resolution_height;
        const std::uint32_t w = color.width, h = color.height;

        std::optional<k4a::transformation> sdk;
        std::optional<DepthToColor> cpu;
        if (variant == "sdk")
        {
            k4a_transformation_t handle = k4a_transformation_create(&calibration);
            if (handle == nullptr)
            {
                throw std::runtime_error("the SDK's transformation could not be created (depth engine on "
                                         "LD_LIBRARY_PATH?)");
            }
            sdk.emplace(handle);
        }
        else
        {
            cpu.emplace(calibration, color, map, variant != "cpu-no-interpolation", variant != "cpu-keep-flying");
        }
        const auto publisher = [&](const char* name, const CameraInfo& info) {
            return CameraPublisher(session, output + "/" + name,
                                   PublisherOptions{.encoding = "16UC1", .frame_id = record.color_frame_id,
                                                    .source = "mirror", .camera_info = info, .codec = kRvl});
        };
        CameraPublisher raw_out = publisher(kDepthToColor, color);
        CameraPublisher rect_out = publisher(kDepthToColorRect, rectified_info(color));
        ImageSubscriber depth(session, base + "/" + kDepth, {.transport = Transport::Shm});
        std::fprintf(stderr, "k4a_depth_to_color_mirror: %s/depth -> %s/{depth_to_color,depth_to_color_rect} (%s)\n",
                     base.c_str(), output.c_str(), variant.c_str());

        k4a::image depth_image = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16, dw, dh, dw * 2);
        k4a::image in_color = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16, int(w), int(h), int(w) * 2);
        std::vector<std::uint16_t> rect(std::size_t(w) * h), raw(rect.size());
        std::uint64_t frames = 0, reported_frames = 0, reported_skipped = 0;
        double ms = 0;
        auto reported = std::chrono::steady_clock::now();
        while (not stop)
        {
            const auto frame = depth.latest(true, std::chrono::milliseconds(200));
            if (not frame)
            {
                continue;
            }
            const auto start = std::chrono::steady_clock::now();
            const std::uint32_t step = frame->step ? frame->step : frame->width * 2;
            auto* d = reinterpret_cast<std::uint16_t*>(depth_image.get_buffer());
            for (int y = 0; y < dh; ++y)
            {
                std::copy_n(reinterpret_cast<const std::uint16_t*>(frame->data() + std::size_t(y) * step), dw,
                            d + std::size_t(y) * dw);
            }
            const std::uint16_t* raw_pixels = raw.data();
            if (sdk)
            {
                sdk->depth_image_to_color_camera(depth_image, &in_color);
                raw_pixels = reinterpret_cast<const std::uint16_t*>(in_color.get_buffer());
                remap_depth16(map, raw_pixels, rect.data());
            }
            else
            {
                cpu->rectified(d, std::size_t(dw) * 2, rect.data());
                cpu->unrectify(raw.data());
            }
            // the node's frame: its capture stamp and seq, in the colour camera's frame
            const Origin origin{Header{frame->header.stamp, record.color_frame_id}, frame->seq.value_or(0), "mirror"};
            raw_out.publish(HostImage{reinterpret_cast<const std::uint8_t*>(raw_pixels), h, w, 0}, origin);
            rect_out.publish(HostImage{reinterpret_cast<const std::uint8_t*>(rect.data()), h, w, 0}, origin);
            ms += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
            ++frames;
            const auto now = std::chrono::steady_clock::now();
            if (now - reported >= std::chrono::seconds(5))
            {
                const double seconds = std::chrono::duration<double>(now - reported).count();
                const std::uint64_t skipped = depth.overwritten();
                std::fprintf(stderr, "%.1f frames/s, %.1f ms a frame, %llu depth frames skipped\n",
                             double(frames - reported_frames) / seconds,
                             frames > reported_frames ? ms / double(frames - reported_frames) : 0.0,
                             static_cast<unsigned long long>(skipped - reported_skipped));
                reported = now;
                reported_frames = frames;
                reported_skipped = skipped;
                ms = 0;
            }
        }
        return 0;
    }
    catch (const std::exception& e)
    {
        std::fprintf(stderr, "k4a_depth_to_color_mirror: %s\n", e.what());
        return 2;
    }
}

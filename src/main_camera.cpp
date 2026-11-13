// xblick-azure-kinect: an Azure Kinect (or one of its recordings) as Durchblick streams (node.hpp).
//
//   xblick-azure-kinect [--base robot/azurekinect] [--serial 000123456789 | --index 0]
//                           [--params-file params.yaml ...] [--param NAME:=VALUE ...] [--dump-params FILE]
//                           [--params-read-only]
//                           [--fps 30] [--depth-mode NFOV_UNBINNED] [--color-resolution 720P] [--brightness 128]
//                           [--cuda-device 0|none] [--depth-engine-gpu 0] [--no-depth-to-color]
//                           [--sdk-depth-to-color] [--no-depth-to-color-interpolation] [--keep-flying-pixels]
//                           [--keep-sparse-pixels] [--max-depth-mm 0] [--sync-publish]
//                           [--source robot]
//                           [--depth-frame-id AzureKinectDepthCamera] [--color-frame-id AzureKinectCamera]
//                           [--imu-frame-id AzureKinectImu] [--no-imu]
//                           [--color-calibration FILE] [--depth-calibration FILE]   (also with --playback)
//   xblick-azure-kinect --playback recording.mkv [--loop] [--no-realtime] [--source recording]
//
// Zenoh: the PC's router (127.0.0.1:7447), or --listen/--connect ENDPOINT..., or --zenoh-config FILE
// (xblick::open_session). Every option is also a read-only parameter of the node /<base> (xblick/options.hpp),
// beside the device's settings below; --help lists them.
// Parameters (settings.hpp, ROS 2's model; at run time through <base>/set_parameters etc., Durchblick's
// xblick/parameters.hpp and docs/parameters.md):
// --params-file a ROS params YAML (<node>: ros__parameters: ..., the node /<base>), --param NAME:=VALUE (later wins);
// --fps, --depth-mode, --color-resolution, --brightness and --clock-offset mean|min are short for fps, depth.mode,
// color.resolution, color.brightness and clock.offset_estimator.
// --color-calibration/--depth-calibration: ROS calibration files that replace the device's intrinsics
// (calibration.hpp: CalibrationRecord::color_file).
// The depth engine's OpenGL context is created with EGL on CUDA device --depth-engine-gpu (default: --cuda-device,
// else 0); no X server is needed (egl_glx.cpp). --cuda-device (colour decoded with nvJPEG, CUDA rings) defaults to 0
// in builds with XBLICK_K4A_CUDA: cheaper in every case measured with k4a_node_bench (README); none: off.
// depth_to_color and depth_to_color_rect are computed on the CPU (depth_to_color.hpp); --sdk-depth-to-color: by the
// SDK's transformation in the depth engine (OpenGL) instead, which waits for the GPU where other programs keep it busy.
// --no-depth-to-color-interpolation: each depth pixel's footprint with its own depth throughout, not following the
// surface's slope (depth_to_color.hpp). --keep-flying-pixels: depth_to_color keeps the depth pixels in the gap between
// two surfaces (by default left out, as the SDK leaves most of them out); --keep-sparse-pixels: also those in no
// complete 2x2 square on one surface (by default left out, as the SDK draws). --max-depth-mm: depths beyond it are
// made 0 in every depth stream (default 0: none cut).
// The streams are published from threads of their own (NodeOptions::async_publish), so that a publish that waits for
// the GPU does not hold up the next capture; --sync-publish: in the capture loop, one after the other.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "xblick/camera.hpp"
#include "xblick/options.hpp"
#include "egl_glx.hpp"
#include "node.hpp"
#include "xblick/parameters.hpp"
#include "settings.hpp"
#include "source.hpp"

namespace
{
    std::atomic<bool> stop{false};
}  // namespace

int main(int argc, char** argv)
{
    using namespace xblick;
    using namespace xblick::k4a_node;
    std::signal(SIGINT, [](int) { stop = true; });
    std::signal(SIGTERM, [](int) { stop = true; });
    // the device settings' short options: --fps 15 is --param fps:=15 (settings.hpp declares the parameters)
    const std::vector<std::pair<const char*, const char*>> shorts{
        {"fps", "fps"}, {"depth-mode", "depth.mode"}, {"color-resolution", "color.resolution"},
        {"brightness", "color.brightness"}, {"clock-offset", "clock.offset_estimator"}};
    Options options("xblick-azure-kinect", "an Azure Kinect, or one of its recordings, as Durchblick streams");
    options.add("base", "robot/azurekinect", {.help = "the streams' base <namespace>/<camera>; the node /<base>"});
    options.add("serial", "", {.help = "the device's serial number (default: the device of --index)"});
    options.add("index", std::int64_t{0}, {.help = "the device's index, without --serial"});
    for (const auto& [option, parameter] : shorts)
    {
        options.add_optional(option, parameter_type::kString,
                             {.help = std::string("short for --param ") + parameter + ":=VALUE", .metavar = "VALUE"});
    }
#ifdef XBLICK_K4A_CUDA
    options.add("cuda-device", "0", {.help = "colour decoded with nvJPEG and the CUDA rings on this GPU; none: off"});
#else
    options.add("cuda-device", "none", {.help = "colour decoded with nvJPEG and the CUDA rings on this GPU; none: off"});
#endif
    options.add_optional("depth-engine-gpu", parameter_type::kInteger,
                         {.help = "the GPU of the depth engine's EGL context (default: --cuda-device, else 0)"});
    options.add_flag("no-depth-to-color", {.help = "no depth_to_color and depth_to_color_rect"});
    options.add_flag("sdk-depth-to-color", {.help = "depth_to_color by the SDK's transformation (OpenGL)"});
    options.add_flag("no-depth-to-color-interpolation", {.help = "each depth pixel's footprint with its own depth"});
    options.add_flag("keep-flying-pixels", {.help = "depth_to_color keeps the pixels between two surfaces"});
    options.add_flag("keep-sparse-pixels", {.help = "depth_to_color keeps pixels in no complete 2x2 square"});
    options.add("max-depth-mm", std::int64_t{0}, {.help = "depths beyond are made 0 in every depth stream; 0: none cut",
                                                  .parameter = {.unit = "mm", .integer_range = IntegerRange{0, 20000, 1}}});
    options.add_flag("sync-publish", {.help = "publish in the capture loop, one stream after the other"});
    options.add_optional("source", parameter_type::kString,
                         {.help = "the source in the info record (default robot, with --playback recording)"});
    options.add("depth-frame-id", "AzureKinectDepthCamera", {.help = "the depth camera's frame"});
    options.add("color-frame-id", "AzureKinectCamera", {.help = "the colour camera's frame"});
    options.add("imu-frame-id", NodeOptions{}.imu_frame_id, {.help = "the IMU's frame"});
    options.add_flag("no-imu", {.help = "without the IMU"});
    options.add("color-calibration", "", {.help = "a ROS calibration file replacing the colour intrinsics",
                                          .metavar = "FILE"});
    options.add("depth-calibration", "", {.help = "a ROS calibration file replacing the depth intrinsics",
                                          .metavar = "FILE"});
    options.add_optional("playback", parameter_type::kString, {.help = "a recording (.mkv) instead of the device",
                                                               .metavar = "FILE"});
    options.add_flag("loop", {.help = "with --playback: from the start again at its end"});
    options.add_flag("no-realtime", {.help = "with --playback: as fast as it reads"});
    options.add("status-period", 5.0, {.help = "seconds between the status lines", .parameter = {.unit = "s"}});
    options.add_session_options();
    options.add_parameter_options();
    options.parse(argc, argv);
    try
    {
        // the parameters' services need the session: it opens before they are declared
        std::unique_ptr<CaptureSource> source;  // before the parameters: their writers use its colour controls
        zenoh::Session session = open_session(options.session_options());
        const auto [node_namespace, node_name] = node_of(options.text("base"));
        auto node_parameters = std::make_unique<Parameters>(session, node_name, node_namespace);
        node_parameters->configure(options.parameter_startup());
        std::vector<std::string> leave_out{"base"};  // the node's name; the short options are their parameters'
        for (const auto& [option, name] : shorts)
        {
            leave_out.emplace_back(option);
            if (options.has_value(option))
            {
                const std::string value = options.text(option);
                if (value == "auto")
                {
                    throw std::invalid_argument(std::string("--") + option +
                                                " auto: give the value, or --param color.auto_brightness:=true where "
                                                "the device has that mode");
                }
                node_parameters->add_override(std::string(name) + ":=" + value, std::string("--") + option);
            }
        }
        // every other option a read-only parameter (a value given with --param or a params file is the one used)
        options.declare(*node_parameters, {}, leave_out);
        const std::string depth_frame = options.text("depth-frame-id");
        const std::string color_frame = options.text("color-frame-id");
        NodeOptions node_options;
        node_options.base = options.text("base");
        node_options.depth_to_color = not options.flag("no-depth-to-color");
        node_options.sdk_depth_to_color = options.flag("sdk-depth-to-color");
        node_options.async_publish = not options.flag("sync-publish");
        node_options.depth_to_color_interpolation = not options.flag("no-depth-to-color-interpolation");
        node_options.depth_to_color_drop_flying = not options.flag("keep-flying-pixels");
        node_options.depth_to_color_drop_sparse = not options.flag("keep-sparse-pixels");
        node_options.max_depth_mm = static_cast<std::uint32_t>(options.integer("max-depth-mm"));
        node_options.imu_frame_id = options.text("imu-frame-id");
        node_options.cuda_device = cuda_device_from_name(options.text("cuda-device"));
        const double status_period = options.number("status-period");
        use_egl_for_depth_engine(options.has_value("depth-engine-gpu")
                                     ? static_cast<int>(options.integer("depth-engine-gpu"))
                                     : (node_options.cuda_device ? *node_options.cuda_device : 0));
        CameraParameters parameters(std::move(node_parameters));

        node_options.clock = declare_clock_parameters(parameters);
        if (options.has_value("playback"))
        {
            node_options.source = options.has_value("source") ? options.text("source") : "recording";
            source = open_playback({.path = options.text("playback"), .loop = options.flag("loop"),
                                    .realtime = not options.flag("no-realtime"), .depth_frame_id = depth_frame,
                                    .color_frame_id = color_frame});
            declare_recording_parameters(parameters, source->record());
            parameters.check_overrides();
        }
        else
        {
            node_options.source = options.has_value("source") ? options.text("source") : "robot";
            DeviceOptions d;
            d.serial = options.text("serial");
            d.index = static_cast<std::uint32_t>(options.integer("index"));
            declare_start_parameters(parameters, d);
            d.depth_frame_id = depth_frame;
            d.color_frame_id = color_frame;
            d.imu = not options.flag("no-imu");
            source = open_device(d);
            declare_color_controls(parameters, *source->color_controls());
            parameters.check_overrides();
            parameters.apply_all();  // every control: the device keeps the last run's otherwise
        }
        parameters.started();  // changes from now on are counted in the diagnostics
        const CalibrationRecord record = with_calibration_files(source->record(), options.text("color-calibration"),
                                                                options.text("depth-calibration"));

        Node node(session, node_options, record);
        std::cout << "xblick-azure-kinect: " << describe(*source) << " -> " << node_options.base
                  << "/{color,color_rect,depth,ir,depth_to_color,depth_to_color_rect,imu,tf_static,diagnostics}" << std::endl;
        std::cout << "ready" << std::endl;

        auto last = std::chrono::steady_clock::now();
        auto last_report = last;
        NodeStats before = node.stats();
        std::uint64_t dropped_before = 0;
        while (not stop)
        {
            if (auto capture = source->next(std::chrono::milliseconds(1000)))
            {
                node.publish(*capture, source->live());
            }
            node.publish_imu(source->imu_samples());
            const auto now = std::chrono::steady_clock::now();
            if (now - last_report >= std::chrono::seconds(1))
            {
                node.report(*source, parameters.diagnostics());  // <base>/diagnostics
                last_report = now;
            }
            const double span = std::chrono::duration<double>(now - last).count();
            if (span >= status_period)
            {
                const NodeStats s = node.stats();
                std::printf("[k4a] %s\n",
                            describe(s, before, source->dropped() - dropped_before, source->restarts(), span).c_str());
                std::fflush(stdout);
                before = s;
                dropped_before = source->dropped();
                last = now;
            }
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "xblick-azure-kinect: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}

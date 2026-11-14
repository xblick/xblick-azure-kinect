// An Azure Kinect's captures as Durchblick streams below one base name (e.g. cam/azurekinect):
//
//   <base>/color           rgb8, the colour camera; its JPEG (the camera's MJPEG) as image_raw/compressed as it is,
//                          decoded (CPU, or GPU into the CUDA ring with --cuda-device) only for pixel subscribers
//   <base>/color_rect      rgb8, the colour image rectified (rectify.hpp): without lens distortion, in the camera of
//                          the colour CameraInfo's P, as ROS's image_proc
//   <base>/depth           16UC1 [mm], the depth camera's own geometry (as the body tracker needs it)
//   <base>/ir              mono16, the same camera and exposure as depth
//   <base>/depth_to_color  16UC1 [mm], depth seen from the colour camera, for RGB-D: on the CPU (depth_to_color.hpp),
//                          or with sdk_depth_to_color the SDK's transformation (its depth engine, OpenGL)
//   <base>/depth_to_color_rect  16UC1 [mm], depth seen from color_rect's pinhole camera (0 outside the colour image):
//                          the depth of color_rect's pixels; on the CPU projected into it directly, with the SDK's
//                          transformation depth_to_color rectified (nearest pixel)
//   <base>/k4a_calibration a Zenoh queryable: the calibration record (calibration.hpp) that rebuilds the SDK's
//                          calibration in another process (body tracking, point clouds)
//   <base>/tf_static       the colour camera's and the IMU's poses in the depth camera's frame (tf2_msgs/TFMessage)
//   <base>/imu             sensor_msgs/Imu, one per sample (about 1.6 kHz), in the accelerometer's frame
//   <base>/diagnostics     diagnostic_msgs/DiagnosticArray from report(): rates, drops, temperature, firmware
//
// Each stream carries its CameraInfo (rational_polynomial). Every representation is produced only while somebody
// subscribes to it, so an unwatched stream costs nothing but the capture. Stamps: the image's device timestamp in
// wall-clock time (clock.hpp); depth and IR of one capture carry the same stamp, which pairs them.
#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include <k4a/k4a.hpp>
#include <zenoh.hxx>

#include <vector>

#include "calibration.hpp"
#include "clock.hpp"
#include "xblick/msg.hpp"
#include "source.hpp"

namespace xblick::k4a_node
{
    struct NodeOptions
    {
        std::string base = "cam/azurekinect";
        std::string source = "cam";      // in every frame's attachment
        std::optional<int> cuda_device;  // decode colour with nvJPEG and offer CUDA rings on this device
        bool depth_to_color = true;      // offer <base>/depth_to_color and depth_to_color_rect
        // computed by the SDK's transformation (its depth engine, OpenGL) instead of on the CPU (depth_to_color.hpp)
        bool sdk_depth_to_color = false;
        // the CPU's: the depth follows the surface's slope across a depth pixel's footprint (depth_to_color.hpp)
        bool depth_to_color_interpolation = true;
        // the CPU's: flying pixels (a depth in the gap between two surfaces) left out (depth_to_color.hpp)
        bool depth_to_color_drop_flying = true;
        // the CPU's: depth pixels in no complete 2x2 square on one surface left out, as the SDK draws (depth_to_color.hpp)
        bool depth_to_color_drop_sparse = true;
        // depths beyond this [mm] are made 0 (no depth) in every depth stream (depth, depth_to_color, the point clouds
        // made from them); 0: none cut (the camera's own range, NFOV up to about 3.9 m, ambiguous depths beyond it)
        std::uint32_t max_depth_mm = 0;
        // publish from threads of their own (colour; depth and IR; depth_to_color_rect; depth_to_color), so that a
        // publish that waits (a CUDA ring's upload on a busy GPU) does not hold up the next capture; a lane still busy
        // skips its older frame (NodeStats::skipped). Off: publish() publishes before it returns (tests, benchmarks).
        bool async_publish = false;
        std::uint32_t slots = 32;        // ring slots per stream (a reader has slots - 1 frame periods)
        std::string imu_frame_id = "AzureKinectImu";  // the accelerometer's frame (tf_static places it)
        ClockOptions clock;                           // how device time becomes wall time (clock.hpp)
    };

    // Topic names below the base.
    inline constexpr const char* kColor = "color";
    inline constexpr const char* kColorRect = "color_rect";
    inline constexpr const char* kDepth = "depth";
    inline constexpr const char* kIr = "ir";
    inline constexpr const char* kDepthToColor = "depth_to_color";
    inline constexpr const char* kDepthToColorRect = "depth_to_color_rect";
    inline constexpr const char* kCalibration = "k4a_calibration";

    struct NodeStats
    {
        std::uint64_t captures = 0;
        std::uint64_t color_jpeg_only = 0;  // colour sent as the camera's JPEG only
        std::uint64_t color_decoded = 0;    // colour decoded (for pixel subscribers of color or color_rect)
        std::uint64_t color_rect = 0;       // rectified colour frames
        std::uint64_t huffman_added = 0;    // JPEGs that came without Huffman tables
        std::uint64_t depth = 0, ir = 0, depth_to_color = 0, depth_to_color_rect = 0;
        double color_ms = 0, depth_ms = 0, depth_to_color_ms = 0;  // time spent, summed (color_ms with rectifying)
        double color_rect_ms = 0;                                   // rectifying and its publishing
        double depth_to_color_rect_ms = 0;                          // the same, of depth_to_color_rect
        std::uint64_t imu = 0;  // IMU samples published
        std::uint64_t skipped = 0;  // frames a lane skipped because it was still publishing (async_publish)
    };

    // "30.0 captures/s, dropped 0, restarts 0; colour ...": the rates between two stats() taken `seconds` apart, with
    // the source's dropped captures and restarts in that time
    std::string describe(const NodeStats& now, const NodeStats& before, std::uint64_t dropped, std::uint64_t restarts,
                         double seconds);

    // NodeOptions::cuda_device from "0", "1", ... or "none" (off). Throws for anything else, and for a device in a
    // build without XBLICK_K4A_CUDA.
    std::optional<int> cuda_device_from_name(const std::string& name);

    // The calibration record of the node publishing below `base` (its <base>/k4a_calibration). Throws if no node
    // or several answer within the timeout.
    CalibrationRecord query_calibration(zenoh::Session& session, const std::string& base,
                                        std::chrono::milliseconds timeout = std::chrono::milliseconds(2000));

    class Node
    {
    public:
        Node(zenoh::Session& session, const NodeOptions& options, const CalibrationRecord& record);
        ~Node();
        Node(const Node&) = delete;
        Node& operator=(const Node&) = delete;

        // Publishes what is subscribed of one capture. live: stamped through the device clock (system timestamps
        // of the host); a recording is stamped as if it was taken now (its device clock, offset at its first frame).
        // With async_publish it hands the work to the lanes and returns; a lane's failure is thrown by the next call.
        void publish(const k4a::capture& capture, bool live);
        // Publishes IMU samples (CaptureSource::imu_samples()) while somebody subscribes to <base>/imu, stamped through
        // the device clock that the images set (none before the first image).
        void publish_imu(const std::vector<k4a_imu_sample_t>& samples);
        // Publishes how the node is doing since the last report on <base>/diagnostics, and returns it (for a log or
        // another monitor). About once a second. `extra`: further values (the parameters in effect).
        // Exposure timing over the period, per capture with colour and depth: "colour - depth stamp ms" (median, min,
        // max of the device timestamps' difference), "colour exposure ms", "depth exposure ms" (medians) and
        // "predicted if start-aligned ms" = (colour - depth exposure) / 2: whether the difference follows the
        // exposures (centres aligned: 0; starts aligned: the prediction).
        DiagnosticStatus report(const CaptureSource& source,
                                const std::vector<std::pair<std::string, std::string>>& extra = {});

        NodeStats stats() const;  // a copy: with async_publish the lanes count while it is read
        const DeviceClock& clock() const;

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };
}  // namespace xblick::k4a_node

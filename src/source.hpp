// Where captures come from: an Azure Kinect (DeviceSource) or one of its recordings (PlaybackSource, k4arecorder's MKV
// files). Both give the colour frames as MJPEG and the depth and IR images of one exposure together, with the raw
// calibration of the device that took them.
#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <k4a/k4a.hpp>

#include "calibration.hpp"
#include "settings.hpp"

namespace durchblick::k4a_node
{
    struct DeviceOptions
    {
        std::string serial;  // empty: the device with index `index`
        std::uint32_t index = 0;
        int fps = 30;
        k4a_depth_mode_t depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
        k4a_color_resolution_t color_resolution = K4A_COLOR_RESOLUTION_720P;
        // the colour controls are set after the start (settings.hpp: declare_color_controls); these only at the start
        std::int32_t depth_delay_off_color_usec = 0;  // depth's exposure centre after colour's (SDK: within one period)
        // captures only with colour and depth (default: when both run); false: also captures with one of them
        std::optional<bool> synchronized_images_only;
        bool disable_streaming_indicator = false;  // the white LED in front
        std::string depth_frame_id;
        std::string color_frame_id;
        bool imu = true;  // start the IMU (samples through CaptureSource::imu_samples())
    };

    class CaptureSource
    {
    public:
        virtual ~CaptureSource() = default;
        // The newest capture, waiting at most `timeout`; empty if none came. Older ones that are already queued
        // are dropped (counted in dropped()): every capture published is the most recent.
        virtual std::optional<k4a::capture> next(std::chrono::milliseconds timeout) = 0;
        virtual const CalibrationRecord& record() const = 0;
        // True for a camera: captures are stamped with its clock as they come (a recording replays its own).
        virtual bool live() const = 0;
        virtual std::uint64_t dropped() const = 0;
        virtual std::uint64_t restarts() const { return 0; }
        // The IMU samples that came since the last call (a recording's up to its last capture); none if the source
        // has no IMU (a recording without an IMU track, a device started without it).
        virtual std::vector<k4a_imu_sample_t> imu_samples() = 0;
        // What the diagnostics tell about the source (firmware versions, the recording's file), as key and value.
        virtual std::vector<std::pair<std::string, std::string>> info() const = 0;
        // The colour controls of a device (settings.hpp); null for a recording.
        virtual ColorControls* color_controls() { return nullptr; }
    };

    // Starts the cameras: colour as MJPEG (passed on as it is, decoded only for pixel subscribers), depth and IR in
    // one capture with colour (synchronized_images_only). A failed get_capture() restarts the cameras once (the
    // device sometimes stops delivering); a second failure in a row is raised.
    std::unique_ptr<CaptureSource> open_device(const DeviceOptions& options);

    struct PlaybackOptions
    {
        std::string path;
        bool loop = false;
        bool realtime = true;  // pace the captures by their device timestamps; false: as fast as they are read
        std::string depth_frame_id;
        std::string color_frame_id;
    };

    // A recording with MJPEG colour, depth and IR tracks (k4arecorder's default with colour on).
    std::unique_ptr<CaptureSource> open_playback(const PlaybackOptions& options);

    // "device 000123456789, NFOV_UNBINNED, colour 720P, 30 fps" (or "recording ...")
    std::string describe(const CaptureSource& source);
}  // namespace durchblick::k4a_node

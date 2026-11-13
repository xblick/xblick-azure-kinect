#include "source.hpp"

#include <iostream>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

#include <k4arecord/playback.h>

namespace xblick::k4a_node
{
    namespace
    {
        std::string raw_calibration(const std::vector<std::uint8_t>& buffer)
        {
            // the SDK's buffer ends with the terminating zero
            std::size_t n = buffer.size();
            while (n > 0 and buffer[n - 1] == 0)
            {
                --n;
            }
            return std::string(buffer.begin(), buffer.begin() + static_cast<std::ptrdiff_t>(n));
        }

        class DeviceSource : public CaptureSource
        {
        public:
            explicit DeviceSource(const DeviceOptions& o)
            {
                const std::uint32_t count = k4a::device::get_installed_count();
                if (count == 0)
                {
                    throw std::runtime_error("no Azure Kinect is connected (k4a_device_get_installed_count is 0)");
                }
                device_ = open(o, count);
                config_ = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
                config_.camera_fps = fps_from_value(o.fps);
                config_.color_format = K4A_IMAGE_FORMAT_COLOR_MJPG;
                config_.color_resolution = o.color_resolution;
                config_.depth_mode = o.depth_mode;
                const bool both = o.color_resolution != K4A_COLOR_RESOLUTION_OFF and o.depth_mode != K4A_DEPTH_MODE_OFF;
                config_.synchronized_images_only = o.synchronized_images_only.value_or(both);
                if (config_.synchronized_images_only and not both)
                {
                    throw std::invalid_argument("synchronized_images_only needs colour and depth");
                }
                const std::int32_t period_us = 1000000 / o.fps;
                if (o.depth_delay_off_color_usec < -period_us or o.depth_delay_off_color_usec > period_us)
                {
                    throw std::invalid_argument("depth_delay_off_color_usec " +
                                                std::to_string(o.depth_delay_off_color_usec) + " outside one period [" +
                                                std::to_string(-period_us) + ", " + std::to_string(period_us) + "]");
                }
                config_.depth_delay_off_color_usec = o.depth_delay_off_color_usec;
                config_.disable_streaming_indicator = o.disable_streaming_indicator;
                record_.raw = raw_calibration(device_.get_raw_calibration());
                record_.depth_mode = o.depth_mode;
                record_.color_resolution = o.color_resolution;
                record_.fps = o.fps;
                record_.serial = device_.get_serialnum();
                record_.depth_frame_id = o.depth_frame_id;
                record_.color_frame_id = o.color_frame_id;
                device_.start_cameras(&config_);
                controls_ = device_color_controls(device_);
                // the SDK starts the IMU only with the cameras running
                imu_ = o.imu;
                if (imu_)
                {
                    device_.start_imu();
                }
                const k4a_hardware_version_t v = device_.get_version();
                const auto version = [](const k4a_version_t& x) {
                    return std::to_string(x.major) + "." + std::to_string(x.minor) + "." + std::to_string(x.iteration);
                };
                // the serial is the status' hardware_id (Node::report)
                info_ = {{"firmware rgb", version(v.rgb)},
                         {"firmware depth", version(v.depth)},
                         {"firmware depth sensor", version(v.depth_sensor)},
                         {"firmware audio", version(v.audio)}};
            }

            ~DeviceSource() override
            {
                if (imu_)
                {
                    device_.stop_imu();
                }
                device_.stop_cameras();
                device_.close();
            }

            std::optional<k4a::capture> next(std::chrono::milliseconds timeout) override
            {
                k4a::capture capture;
                try
                {
                    if (not device_.get_capture(&capture, timeout))
                    {
                        return std::nullopt;
                    }
                    failures_ = 0;
                }
                catch (const k4a::error& e)
                {
                    if (++failures_ > 1)
                    {
                        throw std::runtime_error(std::string("get_capture failed again after a restart: ") + e.what());
                    }
                    std::cerr << "xblick-azure-kinect: get_capture failed (" << e.what()
                              << "), restarting the cameras" << std::endl;
                    if (imu_)
                    {
                        device_.stop_imu();
                    }
                    device_.stop_cameras();
                    device_.start_cameras(&config_);
                    if (imu_)
                    {
                        device_.start_imu();
                    }
                    ++restarts_;
                    return std::nullopt;
                }
                // drain: a capture that waited in the queue is older than the one behind it
                k4a::capture newer;
                while (device_.get_capture(&newer, std::chrono::milliseconds(0)))
                {
                    capture = std::move(newer);
                    ++dropped_;
                }
                return capture;
            }

            const CalibrationRecord& record() const override { return record_; }
            bool live() const override { return true; }
            std::uint64_t dropped() const override { return dropped_; }
            std::uint64_t restarts() const override { return restarts_; }

            std::vector<k4a_imu_sample_t> imu_samples() override
            {
                std::vector<k4a_imu_sample_t> samples;
                k4a_imu_sample_t sample;
                while (imu_ and device_.get_imu_sample(&sample, std::chrono::milliseconds(0)))
                {
                    samples.push_back(sample);
                }
                return samples;
            }

            std::vector<std::pair<std::string, std::string>> info() const override { return info_; }
            ColorControls* color_controls() override { return controls_.get(); }

        private:
            static k4a::device open(const DeviceOptions& o, std::uint32_t count)
            {
                if (o.serial.empty())
                {
                    if (o.index >= count)
                    {
                        throw std::runtime_error("device index " + std::to_string(o.index) + ": " +
                                                 std::to_string(count) + " Azure Kinect(s) connected");
                    }
                    return k4a::device::open(o.index);
                }
                std::string serials;
                for (std::uint32_t i = 0; i < count; ++i)
                {
                    k4a::device d = k4a::device::open(i);
                    const std::string serial = d.get_serialnum();
                    if (serial == o.serial)
                    {
                        return d;
                    }
                    serials += (serials.empty() ? "" : ", ") + serial;
                }
                throw std::runtime_error("no Azure Kinect with serial " + o.serial + " (connected: " + serials + ")");
            }

            k4a::device device_;
            std::unique_ptr<ColorControls> controls_;  // on device_, which outlives it (declared after)
            k4a_device_configuration_t config_{};
            bool imu_ = false;
            std::vector<std::pair<std::string, std::string>> info_;
            CalibrationRecord record_;
            int failures_ = 0;
            std::uint64_t restarts_ = 0;
            std::uint64_t dropped_ = 0;
        };

        class PlaybackSource : public CaptureSource
        {
        public:
            explicit PlaybackSource(const PlaybackOptions& o) : options_(o)
            {
                if (k4a_playback_open(o.path.c_str(), &playback_) != K4A_RESULT_SUCCEEDED)
                {
                    throw std::runtime_error("cannot open the recording " + o.path);
                }
                k4a_record_configuration_t c;
                if (k4a_playback_get_record_configuration(playback_, &c) != K4A_RESULT_SUCCEEDED)
                {
                    throw std::runtime_error(o.path + ": no record configuration");
                }
                if (not c.color_track_enabled or not c.depth_track_enabled or not c.ir_track_enabled)
                {
                    throw std::runtime_error(o.path + ": needs colour, depth and IR tracks");
                }
                imu_ = c.imu_track_enabled;
                if (c.color_format != K4A_IMAGE_FORMAT_COLOR_MJPG)
                {
                    throw std::runtime_error(o.path + ": the colour track is not MJPEG (k4arecorder's default, "
                                                      "--color-mode)");
                }
                std::size_t size = 0;
                if (k4a_playback_get_raw_calibration(playback_, nullptr, &size) != K4A_BUFFER_RESULT_TOO_SMALL)
                {
                    throw std::runtime_error(o.path + ": no calibration in the recording");
                }
                std::vector<std::uint8_t> raw(size);
                if (k4a_playback_get_raw_calibration(playback_, raw.data(), &size) != K4A_BUFFER_RESULT_SUCCEEDED)
                {
                    throw std::runtime_error(o.path + ": cannot read its calibration");
                }
                record_.raw = raw_calibration(raw);
                record_.depth_mode = c.depth_mode;
                record_.color_resolution = c.color_resolution;
                record_.fps = fps_value(c.camera_fps);
                char serial[64] = {};
                std::size_t serial_size = sizeof(serial);
                if (k4a_playback_get_tag(playback_, "K4A_DEVICE_SERIAL_NUMBER", serial, &serial_size) ==
                    K4A_BUFFER_RESULT_SUCCEEDED)
                {
                    record_.serial = serial;
                }
                record_.depth_frame_id = o.depth_frame_id;
                record_.color_frame_id = o.color_frame_id;
            }

            ~PlaybackSource() override { k4a_playback_close(playback_); }

            std::optional<k4a::capture> next(std::chrono::milliseconds) override
            {
                k4a_capture_t handle = nullptr;
                k4a_stream_result_t result = k4a_playback_get_next_capture(playback_, &handle);
                if (result == K4A_STREAM_RESULT_EOF and options_.loop)
                {
                    if (k4a_playback_seek_timestamp(playback_, 0, K4A_PLAYBACK_SEEK_BEGIN) != K4A_RESULT_SUCCEEDED)
                    {
                        throw std::runtime_error(options_.path + ": cannot seek to the beginning");
                    }
                    first_device_.reset();
                    pending_imu_.reset();
                    result = k4a_playback_get_next_capture(playback_, &handle);
                }
                if (result == K4A_STREAM_RESULT_EOF)
                {
                    throw std::runtime_error(options_.path + ": end of the recording");
                }
                if (result != K4A_STREAM_RESULT_SUCCEEDED)
                {
                    throw std::runtime_error(options_.path + ": cannot read the next capture");
                }
                k4a::capture capture(handle);
                if (const k4a::image depth = capture.get_depth_image())
                {
                    last_device_ = depth.get_device_timestamp();
                }
                if (options_.realtime)
                {
                    pace(capture);
                }
                return capture;
            }

            const CalibrationRecord& record() const override { return record_; }
            bool live() const override { return false; }
            std::uint64_t dropped() const override { return 0; }

            // the recording's samples up to its last capture's device time, as the device gives them along
            std::vector<k4a_imu_sample_t> imu_samples() override
            {
                std::vector<k4a_imu_sample_t> samples;
                while (imu_ and last_device_)
                {
                    if (not pending_imu_)
                    {
                        k4a_imu_sample_t sample;
                        const k4a_stream_result_t result = k4a_playback_get_next_imu_sample(playback_, &sample);
                        if (result == K4A_STREAM_RESULT_EOF)
                        {
                            break;  // the rest comes after a loop's seek
                        }
                        if (result != K4A_STREAM_RESULT_SUCCEEDED)
                        {
                            throw std::runtime_error(options_.path + ": cannot read the next IMU sample");
                        }
                        pending_imu_ = sample;
                    }
                    if (std::chrono::microseconds(pending_imu_->acc_timestamp_usec) > *last_device_)
                    {
                        break;
                    }
                    samples.push_back(*std::exchange(pending_imu_, std::nullopt));
                }
                return samples;
            }

            std::vector<std::pair<std::string, std::string>> info() const override
            {
                return {{"recording", options_.path}};
            }

        private:
            // waits until the capture's device time has passed since the first one, in wall time
            void pace(const k4a::capture& capture)
            {
                const k4a::image depth = capture.get_depth_image();
                if (not depth)
                {
                    return;  // a capture without depth (the recording's start): no pace
                }
                const std::chrono::microseconds device = depth.get_device_timestamp();
                const auto now = std::chrono::steady_clock::now();
                if (not first_device_)
                {
                    first_device_ = device;
                    first_wall_ = now;
                    return;
                }
                std::this_thread::sleep_until(first_wall_ + (device - *first_device_));
            }

            PlaybackOptions options_;
            k4a_playback_t playback_ = nullptr;
            CalibrationRecord record_;
            std::optional<std::chrono::microseconds> first_device_;
            std::chrono::steady_clock::time_point first_wall_;
            bool imu_ = false;
            std::optional<std::chrono::microseconds> last_device_;  // of the last capture with depth
            std::optional<k4a_imu_sample_t> pending_imu_;  // read, but after the last capture
        };
    }  // namespace

    std::unique_ptr<CaptureSource> open_device(const DeviceOptions& options)
    {
        return std::make_unique<DeviceSource>(options);
    }

    std::unique_ptr<CaptureSource> open_playback(const PlaybackOptions& options)
    {
        return std::make_unique<PlaybackSource>(options);
    }

    std::string describe(const CaptureSource& source)
    {
        const CalibrationRecord& r = source.record();
        return std::string(source.live() ? "device " : "recording ") + r.serial + ", " + depth_mode_name(r.depth_mode) +
               ", colour " + color_resolution_name(r.color_resolution) + ", " + std::to_string(r.fps) + " fps";
    }
}  // namespace xblick::k4a_node

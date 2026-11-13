#include "settings.hpp"

#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <tuple>
#include <utility>

#include "calibration.hpp"
#include "source.hpp"

namespace durchblick::k4a_node
{
    namespace
    {
        class DeviceColorControls : public ColorControls
        {
        public:
            explicit DeviceColorControls(k4a::device& device) : device_(device) {}

            ControlCapabilities capabilities(k4a_color_control_command_t command) override
            {
                ControlCapabilities c;
                if (k4a_device_get_color_control_capabilities(device_.handle(), command, &c.supports_auto, &c.min,
                                                              &c.max, &c.step, &c.default_value,
                                                              &c.default_mode) != K4A_RESULT_SUCCEEDED)
                {
                    throw std::runtime_error("k4a_device_get_color_control_capabilities failed for control " +
                                             std::to_string(int(command)));
                }
                return c;
            }

            ControlState get(k4a_color_control_command_t command) override
            {
                ControlState s;
                device_.get_color_control(command, &s.mode, &s.value);
                return s;
            }

            void set(k4a_color_control_command_t command, const ControlState& state) override
            {
                device_.set_color_control(command, state.mode, state.value);
            }

        private:
            k4a::device& device_;
        };

        constexpr std::int64_t kBrightness = 128;  // the node's start value

        // A read-only parameter's options (applies at the next start).
        ParameterOptions read_only(std::string description, std::string unit = "",
                                   std::vector<ParameterValue> choices = {})
        {
            return {.description = std::move(description), .unit = std::move(unit), .choices = std::move(choices),
                    .read_only = true};
        }

        std::vector<ParameterValue> depth_mode_choices()
        {
            std::vector<ParameterValue> choices;
            for (const auto m : {K4A_DEPTH_MODE_OFF, K4A_DEPTH_MODE_NFOV_2X2BINNED, K4A_DEPTH_MODE_NFOV_UNBINNED,
                                 K4A_DEPTH_MODE_WFOV_2X2BINNED, K4A_DEPTH_MODE_WFOV_UNBINNED, K4A_DEPTH_MODE_PASSIVE_IR})
            {
                choices.push_back(ParameterValue::of(depth_mode_name(m)));
            }
            return choices;
        }

        std::vector<ParameterValue> color_resolution_choices()
        {
            std::vector<ParameterValue> choices;
            for (const auto r : {K4A_COLOR_RESOLUTION_OFF, K4A_COLOR_RESOLUTION_720P, K4A_COLOR_RESOLUTION_1080P,
                                 K4A_COLOR_RESOLUTION_1440P, K4A_COLOR_RESOLUTION_1536P, K4A_COLOR_RESOLUTION_2160P,
                                 K4A_COLOR_RESOLUTION_3072P})
            {
                choices.push_back(ParameterValue::of(color_resolution_name(r)));
            }
            return choices;
        }

        const ParameterOptions& fps_options()
        {
            static const ParameterOptions o = read_only("frame rate of both cameras", "Hz",
                                                        {ParameterValue::of(5), ParameterValue::of(15),
                                                         ParameterValue::of(30)});
            return o;
        }

        const ParameterOptions& depth_mode_options()
        {
            static const ParameterOptions o = read_only("depth camera mode", "", depth_mode_choices());
            return o;
        }

        const ParameterOptions& color_resolution_options()
        {
            static const ParameterOptions o = read_only("colour resolution (MJPEG)", "", color_resolution_choices());
            return o;
        }

        bool ros_name(const std::string& part)
        {
            const auto letter = [](char c) { return (c >= 'a' and c <= 'z') or (c >= 'A' and c <= 'Z') or c == '_'; };
            bool ok = not part.empty() and letter(part[0]);
            for (const char c : part)
            {
                ok = ok and (letter(c) or (c >= '0' and c <= '9'));
            }
            return ok;
        }
    }  // namespace

    const std::vector<ControlSpec>& color_controls()
    {
        static const std::vector<ControlSpec> controls = {
            {"color.exposure_time_us", "color.auto_exposure", K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE, "us",
             "colour exposure time (the camera rounds to its supported steps, at most the frame period)"},
            {"color.white_balance_k", "color.auto_white_balance", K4A_COLOR_CONTROL_WHITEBALANCE, "K",
             "colour white balance"},
            {"color.brightness", "color.auto_brightness", K4A_COLOR_CONTROL_BRIGHTNESS, "", "colour brightness"},
            {"color.contrast", "color.auto_contrast", K4A_COLOR_CONTROL_CONTRAST, "", "colour contrast"},
            {"color.saturation", "color.auto_saturation", K4A_COLOR_CONTROL_SATURATION, "", "colour saturation"},
            {"color.sharpness", "color.auto_sharpness", K4A_COLOR_CONTROL_SHARPNESS, "", "colour sharpness"},
            {"color.backlight_compensation", "color.auto_backlight_compensation",
             K4A_COLOR_CONTROL_BACKLIGHT_COMPENSATION, "", "backlight compensation"},
            {"color.gain", "color.auto_gain", K4A_COLOR_CONTROL_GAIN, "", "colour gain"},
            {"color.powerline_frequency", "color.auto_powerline_frequency", K4A_COLOR_CONTROL_POWERLINE_FREQUENCY, "",
             "mains frequency: 1 50 Hz, 2 60 Hz"},
        };
        return controls;
    }

    std::unique_ptr<ColorControls> device_color_controls(k4a::device& device)
    {
        return std::make_unique<DeviceColorControls>(device);
    }

    std::pair<std::string, std::string> node_of(const std::string& base)
    {
        std::size_t begin = 0;
        while (begin <= base.size())
        {
            const std::size_t end = std::min(base.find('/', begin), base.size());
            if (not ros_name(base.substr(begin, end - begin)))
            {
                throw std::invalid_argument("base '" + base + "': each part a ROS name (a letter or _, then letters, "
                                            "digits, _), for the node's name /" + base);
            }
            begin = end + 1;
        }
        const std::size_t slash = base.rfind('/');
        return slash == std::string::npos ? std::pair{std::string(), base}
                                          : std::pair{base.substr(0, slash), base.substr(slash + 1)};
    }

    ParameterValue CameraParameters::declare(const std::string& name, const ParameterValue& default_value,
                                             const ParameterOptions& options, Parameters::Writer write,
                                             Parameters::Reader read)
    {
        Parameters::Writer counted;
        if (write)
        {
            counted = [this, name, write = std::move(write)](const ParameterValue& v) {
                std::string reason;
                try
                {
                    reason = write(v);
                }
                catch (const std::exception& error)
                {
                    reason = error.what();  // e.g. the SDK refused
                }
                if (reason.empty())
                {
                    std::lock_guard lock(mutex_);
                    if (started_)
                    {
                        ++changes_;
                        last_change_ = name + " " + parameter_text(v);
                        std::cout << "durchblick-azure-kinect: parameter " << last_change_ << std::endl;
                    }
                }
                return reason;
            };
        }
        const ParameterValue value = parameters_->declare(name, default_value, options, counted, std::move(read));
        std::lock_guard lock(mutex_);
        units_.emplace_back(name, options.unit);
        return value;
    }

    void CameraParameters::started()
    {
        std::lock_guard lock(mutex_);
        started_ = true;
    }

    std::vector<std::pair<std::string, std::string>> CameraParameters::diagnostics() const
    {
        std::vector<std::pair<std::string, std::string>> units;
        std::uint64_t changes = 0;
        std::string last;
        {
            std::lock_guard lock(mutex_);
            units = units_;
            changes = changes_;
            last = last_change_;
        }
        std::vector<std::pair<std::string, std::string>> out;
        for (const auto& [name, unit] : units)
        {
            out.push_back({name, parameter_text(parameters_->get(name)) + (unit.empty() ? "" : " " + unit)});
        }
        out.push_back({"parameter changes", std::to_string(changes)});
        if (not last.empty())
        {
            out.push_back({"last parameter change", last});
        }
        return out;
    }

    ClockOptions declare_clock_parameters(CameraParameters& parameters)
    {
        ClockOptions clock;
        clock.estimator = clock_estimator_from_name(
            parameters
                .declare("clock.offset_estimator", ParameterValue::of("mean"),
                         read_only("device -> wall time: mean (low-pass, as Microsoft's ROS driver) or min (the "
                                   "smallest arrival delay over clock.window_s)",
                                   "", {ParameterValue::of("mean"), ParameterValue::of("min")}))
                .string_value);
        ParameterOptions window = read_only("min: the window of device time the smallest delay is taken from", "s");
        window.floating_range = FloatingPointRange{0.1, 60.0, 0.0};
        clock.window = std::chrono::nanoseconds(
            std::int64_t(parameters.declare("clock.window_s", ParameterValue::of(2.0), window).double_value * 1e9));
        ParameterOptions fixed = read_only("the delay from exposure centre to arrival not covered by the offset, "
                                           "subtracted (docs/hardware_checks.md check 1; 0: not measured)",
                                           "us");
        fixed.integer_range = IntegerRange{0, 1000000, 1};
        clock.fixed_delay = std::chrono::microseconds(
            parameters.declare("clock.fixed_delay_us", ParameterValue::of(0), fixed).integer_value);
        return clock;
    }

    void declare_start_parameters(CameraParameters& parameters, DeviceOptions& device)
    {
        device.fps = int(parameters.declare("fps", ParameterValue::of(30), fps_options()).integer_value);
        device.depth_mode = depth_mode_from_name(
            parameters.declare("depth.mode", ParameterValue::of("NFOV_UNBINNED"), depth_mode_options()).string_value);
        device.color_resolution = color_resolution_from_name(
            parameters.declare("color.resolution", ParameterValue::of("720P"), color_resolution_options()).string_value);
        ParameterOptions delay = read_only("depth's exposure centre after colour's (SDK: within one frame period)",
                                           "us");
        const std::int64_t period = 1000000 / device.fps;
        delay.integer_range = IntegerRange{-period, period, 1};
        device.depth_delay_off_color_usec =
            std::int32_t(parameters.declare("depth.delay_off_color_us", ParameterValue::of(0), delay).integer_value);
        // true (the default) means "when both run": refused only if given true without both
        const bool both = device.color_resolution != K4A_COLOR_RESOLUTION_OFF and device.depth_mode != K4A_DEPTH_MODE_OFF;
        const bool sync = parameters
                              .declare("synchronized_images_only", ParameterValue::of(true),
                                       read_only("captures only with colour and depth (needs both)"))
                              .bool_value;
        if (parameters.overridden("synchronized_images_only"))
        {
            device.synchronized_images_only = sync;
        }
        else if (not both)
        {
            device.synchronized_images_only = false;
        }
        device.disable_streaming_indicator =
            parameters.declare("disable_streaming_indicator", ParameterValue::of(false),
                               read_only("the white LED in front off"))
                .bool_value;
    }

    void declare_recording_parameters(CameraParameters& parameters, const CalibrationRecord& record)
    {
        const std::vector<std::tuple<const char*, ParameterValue, ParameterOptions>> recorded = {
            {"fps", ParameterValue::of(record.fps), fps_options()},
            {"depth.mode", ParameterValue::of(depth_mode_name(record.depth_mode)), depth_mode_options()},
            {"color.resolution", ParameterValue::of(color_resolution_name(record.color_resolution)),
             color_resolution_options()},
        };
        for (auto [name, value, options] : recorded)
        {
            options.description += " (as the recording was made)";
            if (parameters.overridden(name))
            {
                throw std::invalid_argument(std::string(name) + ": a recording plays as it was made");
            }
            parameters.declare(name, value, options);
        }
    }

    void declare_color_controls(CameraParameters& parameters, ColorControls& controls)
    {
        for (const ControlSpec& spec : color_controls())
        {
            const ControlCapabilities c = controls.capabilities(spec.command);
            const k4a_color_control_command_t command = spec.command;
            ParameterOptions value{.description = spec.description, .unit = spec.unit};
            // ROS's step means min + n * step; the SDK's exposure step does not (its default 15625 is off its grid
            // from 500 in steps of 100: the camera rounds to the exposures it supports). Such a step is not kept.
            const std::int32_t step = std::max(c.step, 1);
            const bool on_grid = (c.default_value - c.min) % step == 0;
            value.integer_range = IntegerRange{c.min, c.max, std::uint64_t(on_grid ? step : 1)};
            if (not on_grid)
            {
                value.description += " (the SDK's step " + std::to_string(step) + " is not a grid: the camera rounds)";
            }
            const std::int64_t default_value = spec.command == K4A_COLOR_CONTROL_BRIGHTNESS
                                                   ? std::clamp<std::int64_t>(kBrightness, c.min, c.max)
                                                   : std::int64_t(c.default_value);
            // the value parameter: setting it makes the control manual
            parameters.declare(
                spec.value_name, ParameterValue::of(default_value), value,
                [&controls, command](const ParameterValue& v) {
                    controls.set(command, {K4A_COLOR_CONTROL_MODE_MANUAL, std::int32_t(v.integer_value)});
                    return std::string();
                },
                [&controls, command]() { return ParameterValue::of(std::int64_t(controls.get(command).value)); });
            if (not c.supports_auto)
            {
                continue;
            }
            // its default: the SDK's, unless a value was given for the control (then manual, as asked)
            const bool automatic = c.default_mode == K4A_COLOR_CONTROL_MODE_AUTO and
                                   not parameters.overridden(spec.value_name);
            const std::int32_t auto_value = c.default_value;
            parameters.declare(
                spec.auto_name, ParameterValue::of(automatic),
                {.description = std::string("automatic ") + spec.description + " (false: " + spec.value_name +
                                " in effect)"},
                [&controls, command, auto_value](const ParameterValue& v) {
                    // auto: the SDK ignores the value; manual: the value in use now is kept
                    controls.set(command, v.bool_value ? ControlState{K4A_COLOR_CONTROL_MODE_AUTO, auto_value}
                                                       : ControlState{K4A_COLOR_CONTROL_MODE_MANUAL,
                                                                      controls.get(command).value});
                    return std::string();
                },
                [&controls, command]() {
                    return ParameterValue::of(controls.get(command).mode == K4A_COLOR_CONTROL_MODE_AUTO);
                });
        }
    }
}  // namespace durchblick::k4a_node

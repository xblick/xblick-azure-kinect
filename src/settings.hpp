// The Azure Kinect's parameters (Durchblick's xblick::Parameters, ROS 2's model; the core's docs/parameters.md):
//
//   runtime    color.exposure_time_us, color.auto_exposure, color.white_balance_k, color.auto_white_balance,
//              color.brightness, color.contrast, color.saturation, color.sharpness, color.backlight_compensation,
//              color.gain, color.powerline_frequency (1: 50 Hz, 2: 60 Hz): the SDK's colour controls, with the ranges
//              and defaults the device tells (k4a_device_get_color_control_capabilities); an auto_* parameter only
//              for a control with an automatic mode. Setting a control's value sets it manual (its auto_* false).
//   read-only  fps, depth.mode, color.resolution, depth.delay_off_color_us, synchronized_images_only,
//              disable_streaming_indicator (k4a_device_configuration_t), clock.*: set at the start only
//
// The device keeps its colour controls until it is power-cycled (SDK), so the node sets every one at the start: as
// given, else to the SDK's default; color.brightness to 128. A recording has the
// read-only parameters it was made with and no controls.
#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <k4a/k4a.hpp>

#include "clock.hpp"
#include "xblick/parameters.hpp"

namespace xblick::k4a_node
{
    struct DeviceOptions;
    struct CalibrationRecord;

    struct ControlCapabilities
    {
        bool supports_auto = false;
        std::int32_t min = 0, max = 0, step = 1, default_value = 0;
        k4a_color_control_mode_t default_mode = K4A_COLOR_CONTROL_MODE_MANUAL;
    };

    struct ControlState
    {
        k4a_color_control_mode_t mode = K4A_COLOR_CONTROL_MODE_MANUAL;
        std::int32_t value = 0;
    };

    // What a device offers (device_color_controls); a fake in the tests.
    class ColorControls
    {
    public:
        virtual ~ColorControls() = default;
        virtual ControlCapabilities capabilities(k4a_color_control_command_t command) = 0;
        virtual ControlState get(k4a_color_control_command_t command) = 0;
        virtual void set(k4a_color_control_command_t command, const ControlState& state) = 0;
    };
    // The controls of an open device (k4a's device functions are safe to call beside get_capture).
    std::unique_ptr<ColorControls> device_color_controls(k4a::device& device);

    // A colour control: its value parameter (and auto_* parameter, if the device has an automatic mode).
    struct ControlSpec
    {
        const char* value_name;  // e.g. color.exposure_time_us
        const char* auto_name;   // e.g. color.auto_exposure; nullptr: none
        k4a_color_control_command_t command;
        const char* unit;
        const char* description;
    };
    const std::vector<ControlSpec>& color_controls();

    // The node of a base: cam/azurekinect -> namespace cam, name azurekinect (its services below the base, as
    // <base>/set_parameters); throws unless each part is a ROS name.
    std::pair<std::string, std::string> node_of(const std::string& base);

    // The camera's parameters as the diagnostics show them: xblick::Parameters' declare, recording each one's unit,
    // and counting the changes once the camera runs (started()); a writer's exception (the SDK refused) becomes the
    // change's reason, as a writer's text does.
    class CameraParameters
    {
    public:
        // owns the node's parameters: their services (Zenoh's threads) end first when it goes, before the counters and
        // before whatever the writers use (declared before it)
        explicit CameraParameters(std::unique_ptr<Parameters> parameters) : parameters_(std::move(parameters)) {}
        Parameters& parameters() { return *parameters_; }

        ParameterValue declare(const std::string& name, const ParameterValue& default_value,
                               const ParameterOptions& options = {}, Parameters::Writer write = {},
                               Parameters::Reader read = {});
        bool overridden(const std::string& name) const { return parameters_->overridden(name); }
        void check_overrides() { parameters_->check_overrides(); }
        // Every writable parameter's writer with its start value, in the order declared: a control's value before its
        // auto_* mode (declare_color_controls declares them so), as the device needs them (Durchblick >= 0.3.32).
        void apply_all() { parameters_->apply_all(); }
        // From now on a write is a change: counted and logged (apply_all's at the start are not).
        void started();
        // name -> value (with its unit), "parameter changes", "last parameter change"
        std::vector<std::pair<std::string, std::string>> diagnostics() const;

    private:
        std::vector<std::pair<std::string, std::string>> units_;  // name, unit, in declaration order
        mutable std::mutex mutex_;
        bool started_ = false;
        std::uint64_t changes_ = 0;
        std::string last_change_;
        std::unique_ptr<Parameters> parameters_;  // last: destroyed first
    };

    // The clock's read-only parameters (clock.hpp): clock.offset_estimator (mean|min), clock.window_s (min),
    // clock.fixed_delay_us (subtracted, 0 until measured).
    ClockOptions declare_clock_parameters(CameraParameters& parameters);
    // The read-only parameters of a device, with the values given (else the defaults), written into `device`.
    void declare_start_parameters(CameraParameters& parameters, DeviceOptions& device);
    // A recording's read-only parameters as it was made.
    void declare_recording_parameters(CameraParameters& parameters, const CalibrationRecord& record);
    // The colour controls with the device's ranges; apply_all() then sets them on the device.
    void declare_color_controls(CameraParameters& parameters, ColorControls& controls);
}  // namespace xblick::k4a_node

// The camera's clock in wall-clock time. An Azure Kinect stamps each image with its own clock (device timestamp, µs,
// the centre of the exposure) and the host's monotonic clock on arrival (system timestamp, ns); Durchblick's stamps are
// wall-clock nanoseconds: device time + offset - fixed_delay.
//
// Two estimators of the offset (ClockOptions::estimator):
//   mean  (default) as Microsoft's ROS driver (k4a_ros_device.cpp, updateTimestampOffset): the
//         offset is (arrival - device) low-pass filtered, replaced when it moves by more than `snap`
//         (start, a jump). A stamp is then the exposure centre placed at the mean arrival: late by the mean delay,
//         load included; an arrival later than `snap` moves its own image's stamp and snaps back with the next.
//   min   the offset is the minimum of (arrival - device) over the last `window` of device time. Arrival delay is a
//         fixed part (readout, transfer) plus non-negative jitter (USB, scheduling, load), so the minimum is the fixed
//         part and the load's tail does not move the stamps. The window follows the drift of the device's crystal
//         against the host clock: crystals differ by tens of ppm, so a 2 s window lags behind the drift by at most
//         about 0.1 ms; a line fit of the lower envelope would remove that, not worth its complexity at this size.
//         The window restarts when the device time jumps (backwards, or a gap over 1 s: a restart, a reconnect, a
//         new mode).
// Both work in the host's monotonic clock, and add the monotonic -> wall difference read at each image: a step of the
// host clock (chrony) moves the stamps with it at once, without restarting the window.
// fixed_delay: the measured delay from exposure centre to the earliest arrival (docs/hardware_checks.md, check 1),
// subtracted in both; 0 until measured.
#pragma once

#include <chrono>
#include <cstdint>
#include <deque>
#include <optional>
#include <string>
#include <vector>

namespace xblick::k4a_node
{
    enum class ClockEstimator
    {
        kMean,
        kMin,
    };
    ClockEstimator clock_estimator_from_name(const std::string& name);  // "mean", "min"
    std::string clock_estimator_name(ClockEstimator estimator);

    struct ClockOptions
    {
        ClockEstimator estimator = ClockEstimator::kMean;
        std::chrono::nanoseconds window = std::chrono::seconds(2);       // min
        std::chrono::nanoseconds snap = std::chrono::milliseconds(10);   // mean
        double alpha = 0.1;                                              // mean
        std::chrono::nanoseconds fixed_delay{0};
    };

    class DeviceClock
    {
    public:
        explicit DeviceClock(ClockOptions options = {});
        // The mean estimator with these (the tests of before).
        DeviceClock(std::chrono::nanoseconds snap, double alpha);

        // One image: its device timestamp and its system timestamp (host monotonic clock), with the host's wall and
        // monotonic clocks now (passed in for tests; update() reads them).
        void update(std::chrono::microseconds device, std::chrono::nanoseconds system,
                    std::chrono::nanoseconds wall_now, std::chrono::nanoseconds monotonic_now);
        void update(std::chrono::microseconds device, std::chrono::nanoseconds system);

        // A device timestamp in wall-clock nanoseconds since the epoch. Throws before the first update().
        std::int64_t to_wall_ns(std::chrono::microseconds device) const;

        // Device to wall clock [ns], fixed delay not subtracted; empty before the first update().
        std::optional<std::chrono::nanoseconds> offset() const { return offset_; }
        std::uint64_t snaps() const { return snaps_; }    // mean: the offset replaced
        std::uint64_t resets() const { return resets_; }  // min: the window restarted
        const ClockOptions& options() const { return options_; }
        // (arrival - device - offset) of every update since the last call [us]: the arrival delay's jitter as each
        // estimator sees it (min: >= 0, the delay above the fixed part)
        std::vector<double> take_residuals_us();

    private:
        ClockOptions options_;
        std::optional<std::chrono::nanoseconds> offset_;
        std::uint64_t snaps_ = 0, resets_ = 0;
        // min: (device time, arrival - device) of the window, and the indices' minima (a monotonic deque)
        std::deque<std::pair<std::chrono::microseconds, std::chrono::nanoseconds>> window_, minima_;
        std::optional<std::chrono::microseconds> last_device_;
        std::vector<double> residuals_us_;
    };
}  // namespace xblick::k4a_node

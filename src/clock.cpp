#include "clock.hpp"

#include <cmath>
#include <stdexcept>

namespace durchblick::k4a_node
{
    ClockEstimator clock_estimator_from_name(const std::string& name)
    {
        if (name == "mean")
        {
            return ClockEstimator::kMean;
        }
        if (name == "min")
        {
            return ClockEstimator::kMin;
        }
        throw std::invalid_argument("clock offset estimator '" + name + "': mean or min");
    }

    std::string clock_estimator_name(ClockEstimator estimator)
    {
        return estimator == ClockEstimator::kMin ? "min" : "mean";
    }

    DeviceClock::DeviceClock(ClockOptions options) : options_(options)
    {
        if (options_.alpha <= 0.0 or options_.alpha > 1.0)
        {
            throw std::invalid_argument("DeviceClock: alpha must be in (0, 1]");
        }
        if (options_.window <= std::chrono::nanoseconds(0))
        {
            throw std::invalid_argument("DeviceClock: the window must be positive");
        }
    }

    DeviceClock::DeviceClock(std::chrono::nanoseconds snap, double alpha)
        : DeviceClock(ClockOptions{.estimator = ClockEstimator::kMean, .snap = snap, .alpha = alpha})
    {
    }

    void DeviceClock::update(std::chrono::microseconds device, std::chrono::nanoseconds system,
                             std::chrono::nanoseconds wall_now, std::chrono::nanoseconds monotonic_now)
    {
        const std::chrono::nanoseconds to_wall = wall_now - monotonic_now;  // monotonic -> wall, now
        const std::chrono::nanoseconds delay = system - device;             // device -> monotonic, this image
        if (options_.estimator == ClockEstimator::kMean)
        {
            const std::chrono::nanoseconds measured = delay + to_wall;
            if (offset_)
            {
                residuals_us_.push_back(double((measured - *offset_).count()) / 1e3);
            }
            if (not offset_ or std::chrono::abs(*offset_ - measured) > options_.snap)
            {
                offset_ = measured;
                ++snaps_;
                return;
            }
            const auto correction =
                static_cast<std::int64_t>(std::floor(options_.alpha * double((measured - *offset_).count())));
            *offset_ += std::chrono::nanoseconds(correction);
            return;
        }
        // min: restart on a jump of the device time (backwards, or a gap of more than a second)
        if (last_device_ and (device < *last_device_ or device - *last_device_ > std::chrono::seconds(1)))
        {
            window_.clear();
            minima_.clear();
            ++resets_;
        }
        last_device_ = device;
        window_.push_back({device, delay});
        while (not minima_.empty() and minima_.back().second >= delay)
        {
            minima_.pop_back();
        }
        minima_.push_back({device, delay});
        while (device - window_.front().first > options_.window)
        {
            window_.pop_front();
        }
        while (minima_.front().first < window_.front().first)
        {
            minima_.pop_front();
        }
        const std::chrono::nanoseconds fixed = minima_.front().second;
        offset_ = fixed + to_wall;
        residuals_us_.push_back(double((delay - fixed).count()) / 1e3);
    }

    void DeviceClock::update(std::chrono::microseconds device, std::chrono::nanoseconds system)
    {
        // wall clock first, then monotonic: their difference is what matters, both read back to back
        const auto wall = std::chrono::system_clock::now().time_since_epoch();
        const auto monotonic = std::chrono::steady_clock::now().time_since_epoch();
        update(device, system, std::chrono::duration_cast<std::chrono::nanoseconds>(wall),
               std::chrono::duration_cast<std::chrono::nanoseconds>(monotonic));
    }

    std::int64_t DeviceClock::to_wall_ns(std::chrono::microseconds device) const
    {
        if (not offset_)
        {
            throw std::logic_error("DeviceClock: no image yet");
        }
        return (std::chrono::duration_cast<std::chrono::nanoseconds>(device) + *offset_ - options_.fixed_delay).count();
    }

    std::vector<double> DeviceClock::take_residuals_us()
    {
        std::vector<double> out;
        out.swap(residuals_us_);
        return out;
    }
}  // namespace durchblick::k4a_node

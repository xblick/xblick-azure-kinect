// Tests without a camera: the rcl_interfaces CDR of Durchblick's core (byte-identical to rosbags), the camera's
// parameters (xblick::Parameters) and their services over Zenoh, declared on fake colour controls, and the clock's estimators on synthetic
// arrival delays (fixed part, load, outliers, drift, a jump).
#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <tuple>
#include <cstdio>
#include <fstream>
#include <map>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "clock.hpp"
#include "xblick/camera.hpp"
#include "xblick/parameters.hpp"
#include "settings.hpp"
#include "source.hpp"

namespace
{
    using namespace xblick;
    using namespace xblick::k4a_node;
    using namespace std::chrono_literals;
    namespace type = xblick::parameter_type;

    int failures = 0;

    void expect(bool condition, const std::string& what)
    {
        if (not condition)
        {
            std::fprintf(stderr, "FAILED: %s\n", what.c_str());
            ++failures;
        }
    }

    template <typename F>
    std::string error_of(F&& f)
    {
        try
        {
            f();
        }
        catch (const std::exception& e)
        {
            return e.what();
        }
        return "";
    }

    std::string hex(const std::vector<std::uint8_t>& bytes)
    {
        static const char* digits = "0123456789abcdef";
        std::string out;
        for (const std::uint8_t b : bytes)
        {
            out += digits[b >> 4];
            out += digits[b & 15];
        }
        return out;
    }

    std::vector<std::uint8_t> unhex(const std::string& text)
    {
        std::vector<std::uint8_t> out;
        for (std::size_t i = 0; i + 1 < text.size(); i += 2)
        {
            out.push_back(std::uint8_t(std::stoi(text.substr(i, 2), nullptr, 16)));
        }
        return out;
    }

    // rosbags 0.10, ROS 2 Lyrical definitions (the service requests and responses registered as messages of their
    // fields: CDR has no type names). "event": rosbags' bytes for another node name of the same length, with the
    // name's bytes replaced by /camera/azurekinect (the CDR string's length and alignment stay)
    const std::map<std::string, std::string> kRosbags = {
        {"set_request", "000100000400000011000000636f6c6f722e6272696768746e65737300020000000000008c0000000000000000000000000000000100000000000000000000000000000000000000000000000000000014000000636f6c6f722e6175746f5f6578706f73757265000101000000000000000000000000000000000000010000000000000000000000000000000000000000000000000000000b00000064657074682e6d6f6465000400000000000000000000000000000000000000000e00000057464f565f554e42494e4e454400000000000000000000000000000000000000000000000f000000636c6f636b2e77696e646f775f730003000000000000000000000000000000000000044001000000000000000000000000000000000000000000000000000000"},
        {"describe_response", "000100000200000017000000636f6c6f722e6578706f737572655f74696d655f75730002090000006578706f737572650000000009000000756e69743a207573000000000000000001000000f401000000000000d20802000000000064000000000000000f000000636c6f636b2e77696e646f775f73000302000000770000000100000000010000010000009a9999999999b93f0000000000004e40000000000000000000000000"},
        {"event", "0001000065d5eb6aadbc4725140000002f63616d6572612f617a7572656b696e6563740000000000010000000b000000636f6c6f722e6761696e00020000000000000000fdffffffffffffff00000000000000000100000000000000000000000000000000000000000000000000000000000000"},
        {"set_response", "00010000020000000100000001000000000000000a000000726561642d6f6e6c7900"},
        {"list_request", "000100000100000006000000636f6c6f720000000100000000000000"},
    };

    void test_cdr()
    {
        const ParametersRequest set{{{"color.brightness", ParameterValue::of(140)},
                                             {"color.auto_exposure", ParameterValue::of(true)},
                                             {"depth.mode", ParameterValue::of("WFOV_UNBINNED")},
                                             {"clock.window_s", ParameterValue::of(2.5)}}};
        expect(hex(set.serialize()) == kRosbags.at("set_request"), "SetParameters request as rosbags writes it");
        expect(ParametersRequest::deserialize(unhex(kRosbags.at("set_request"))).parameters == set.parameters,
               "SetParameters request read back");

        ParameterDescriptor exposure, window;
        exposure.name = "color.exposure_time_us";
        exposure.type = type::kInteger;
        exposure.description = "exposure";
        exposure.additional_constraints = "unit: us";
        exposure.integer_range = {{500, 133330, 100}};
        window.name = "clock.window_s";
        window.type = type::kDouble;
        window.description = "w";
        window.read_only = true;
        window.floating_point_range = {{0.1, 60.0, 0.0}};
        const DescribeParametersResponse describe{{exposure, window}};
        expect(hex(describe.serialize()) == kRosbags.at("describe_response"), "DescribeParameters response as rosbags");
        expect(DescribeParametersResponse::deserialize(unhex(kRosbags.at("describe_response"))).descriptors ==
                   describe.descriptors,
               "DescribeParameters response read back");

        ParameterEvent event;
        event.stamp = {1793840485, 625458349};
        event.node = "/camera/azurekinect";
        event.changed_parameters = {{"color.gain", ParameterValue::of(-3)}};
        expect(hex(event.serialize()) == kRosbags.at("event"), "ParameterEvent as rosbags writes it");
        const ParameterEvent back = ParameterEvent::deserialize(unhex(kRosbags.at("event")));
        expect(back.node == event.node and back.changed_parameters == event.changed_parameters and
                   back.stamp.sec == event.stamp.sec and back.stamp.nanosec == event.stamp.nanosec,
               "ParameterEvent read back");

        const SetParametersResponse results{{{true, ""}, {false, "read-only"}}};
        expect(hex(results.serialize()) == kRosbags.at("set_response"), "SetParameters response as rosbags");
        const ListParametersRequest list{{"color"}, 1};
        expect(hex(list.serialize()) == kRosbags.at("list_request"), "ListParameters request as rosbags");
    }

    // a device's colour controls as the SDK answers them (ranges of an Azure Kinect, 1.4 firmware)
    class FakeControls : public ColorControls
    {
    public:
        ControlCapabilities capabilities(k4a_color_control_command_t command) override
        {
            switch (command)
            {
            case K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE:
                return {true, 500, 133330, 100, 15625, K4A_COLOR_CONTROL_MODE_AUTO};
            case K4A_COLOR_CONTROL_WHITEBALANCE:
                return {true, 2500, 12500, 10, 4500, K4A_COLOR_CONTROL_MODE_AUTO};
            case K4A_COLOR_CONTROL_POWERLINE_FREQUENCY:
                return {false, 1, 2, 1, 2, K4A_COLOR_CONTROL_MODE_MANUAL};
            default:
                return {false, 0, 255, 1, 128, K4A_COLOR_CONTROL_MODE_MANUAL};
            }
        }
        ControlState get(k4a_color_control_command_t command) override
        {
            return states.count(command) ? states[command] : ControlState{K4A_COLOR_CONTROL_MODE_MANUAL, -1};
        }
        void set(k4a_color_control_command_t command, const ControlState& state) override
        {
            if (fail)
            {
                throw std::runtime_error("set_color_control failed");
            }
            states[command] = state;
            ++sets;
        }

        std::map<k4a_color_control_command_t, ControlState> states;
        int sets = 0;
        bool fail = false;
    };

    void test_overrides()
    {
        const std::string path = "/tmp/xblick_k4a_test_params.yaml";
        {
            std::ofstream f(path);
            f << "/robot/azurekinect:\n  ros__parameters:\n    fps: 15\n    depth:\n      mode: WFOV_2X2BINNED\n"
                 "    color.exposure_time_us: 10000\n    color.white_balance_k: 4000\n"
                 "/other/node:\n  ros__parameters:\n    fps: 5\n";
        }
        FakeControls controls;  // before the parameters, whose writers use it
        CameraParameters parameters(std::make_unique<Parameters>(nullptr, "azurekinect", "robot"));
        parameters.parameters().configure({.params_files = {path}});
        parameters.parameters().add_override("color.white_balance_k:=4510");  // over the file's: wins
        DeviceOptions d;
        declare_clock_parameters(parameters);
        declare_start_parameters(parameters, d);
        expect(d.fps == 15 and d.depth_mode == K4A_DEPTH_MODE_WFOV_2X2BINNED, "the YAML's nested and dotted names");
        declare_color_controls(parameters, controls);
        expect(error_of([&] { parameters.check_overrides(); }).empty(), "the values given fit");
        parameters.apply_all();
        expect(controls.states[K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE].mode == K4A_COLOR_CONTROL_MODE_MANUAL and
                   controls.states[K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE].value == 10000,
               "an exposure time given: manual at that value (its auto_exposure default false then)");
        expect(controls.states[K4A_COLOR_CONTROL_WHITEBALANCE].value == 4510, "name:=value wins over the file");
        expect(controls.states[K4A_COLOR_CONTROL_BRIGHTNESS].value == 128, "brightness starts at 128");
        expect(controls.states[K4A_COLOR_CONTROL_POWERLINE_FREQUENCY].value == 2, "a control not given: its default");
        expect(not parameters.parameters().has("color.auto_powerline_frequency"), "no auto_* for a control without it");

        // fail fast: a value out of range, a wrong type, a step missed, a choice not offered (each at its declare), a
        // typo (once all are declared)
        for (const auto& [assignment, part] :
             {std::pair{"color.gain:=300", "color.gain"}, std::pair{"fps:=fast", "fps"},
              std::pair{"color.white_balance_k:=4505", "color.white_balance_k"}, std::pair{"depth.mode:=NFOV", "depth.mode"},
              std::pair{"color.brightnes:=10", "color.brightnes"}})
        {
            FakeControls c2;
            CameraParameters wrong(std::make_unique<Parameters>(nullptr, "azurekinect", "robot"));
            wrong.parameters().add_override(assignment);
            const std::string error = error_of([&] {
                DeviceOptions d2;
                declare_clock_parameters(wrong);
                declare_start_parameters(wrong, d2);
                declare_color_controls(wrong, c2);
                wrong.check_overrides();
            });
            expect(error.find(part) != std::string::npos,
                   std::string(assignment) + " refused, naming " + part + " (" + error + ")");
        }
        expect(not error_of([] { Parameters(nullptr, "a", "").add_override("bad"); }).empty(),
               "an assignment without := is refused");
        expect(error_of([] { node_of("robot/azure-kinect"); }).find("ROS name") != std::string::npos,
               "a base that is no ROS name is refused");
        expect(node_of("robot/azurekinect") == std::pair<std::string, std::string>{"robot", "azurekinect"},
               "the base's node: namespace and name, its services below the base");
    }

    void test_services(zenoh::Session& node_session, zenoh::Session& client_session)
    {
        FakeControls controls;  // before the parameters, whose writers use it
        CameraParameters parameters(std::make_unique<Parameters>(node_session, "k4a_params", "t"));
        DeviceOptions d;
        declare_clock_parameters(parameters);
        declare_start_parameters(parameters, d);
        declare_color_controls(parameters, controls);
        parameters.check_overrides();
        parameters.apply_all();
        parameters.started();
        expect(controls.states[K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE].mode == K4A_COLOR_CONTROL_MODE_AUTO and
                   controls.states[K4A_COLOR_CONTROL_WHITEBALANCE].mode == K4A_COLOR_CONTROL_MODE_AUTO,
               "the start: a control whose default is automatic is automatic (its value written before its auto_*)");

        std::vector<ParameterEvent> events;
        std::mutex mutex;
        auto subscriber = client_session.declare_subscriber(
            zenoh::KeyExpr("parameter_events"),  // every node's, as ROS's /parameter_events
            [&](const zenoh::Sample& s) {
                std::lock_guard lock(mutex);
                events.push_back(ParameterEvent::deserialize(s.get_payload().as_vector()));
            },
            zenoh::closures::none);
        std::this_thread::sleep_for(300ms);

        ParameterClient client(client_session, "t/k4a_params", 5000);
        const auto names = client.list().names;
        expect(names.size() == 20, "list: 9 start-only and clock, 9 controls, 2 auto_* (" + std::to_string(names.size()) + ")");
        const auto colour = client.list({{"color"}, 1});
        expect(std::count(colour.names.begin(), colour.names.end(), "color.exposure_time_us") == 1 and
                   std::count(colour.names.begin(), colour.names.end(), "fps") == 0,
               "list with a prefix");
        const auto described = client.describe({"color.exposure_time_us", "fps", "nope"}).descriptors;
        expect(described.size() == 3 and described[0].integer_range.size() == 1 and
                   described[0].integer_range[0].to_value == 133330 and described[0].integer_range[0].step == 1 and
                   described[0].additional_constraints == "unit: us" and not described[0].read_only,
               "describe: the device's range, step and unit");
        expect(described[1].read_only and described[1].additional_constraints == "unit: Hz; choices: 5, 15, 30",
               "describe: a start-only parameter is read-only, its choices in additional_constraints");
        expect(described[2].type == type::kNotSet, "describe: an undeclared name is NOT_SET");
        expect(client.types({"color.auto_exposure", "depth.mode"}).types == std::vector<std::uint8_t>{type::kBool, type::kString},
               "get_parameter_types");

        // set: one applied, one read-only, one out of range, one undeclared; the event names the changes
        const auto results = client.set({{"color.exposure_time_us", ParameterValue::of(20000)},
                                         {"fps", ParameterValue::of(5)},
                                         {"color.brightness", ParameterValue::of(999)},
                                         {"color.nothing", ParameterValue::of(1)}})
                                 .results;
        expect(results.size() == 4 and results[0].successful, "set: a runtime parameter");
        expect(not results[1].successful and results[1].reason == "read-only: applies at the next start",
               "set: a start-only parameter needs a restart");
        expect(not results[2].successful and results[2].reason.find("outside") != std::string::npos, "set: out of range");
        expect(not results[3].successful and results[3].reason.find("no parameter") != std::string::npos,
               "set: undeclared (" + results[3].reason + ")");
        expect(controls.states[K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE].value == 20000 and
                   controls.states[K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE].mode == K4A_COLOR_CONTROL_MODE_MANUAL,
               "the device has it, manual");
        const auto values = client.get({"color.exposure_time_us", "color.auto_exposure"}).values;
        expect(values[0] == ParameterValue::of(20000) and values[1] == ParameterValue::of(false),
               "get: the value set, auto_exposure false now");

        // auto again: the event lists auto_exposure
        expect(client.set({{"color.auto_exposure", ParameterValue::of(true)}}).results[0].successful, "auto exposure on");
        expect(controls.states[K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE].mode == K4A_COLOR_CONTROL_MODE_AUTO, "device auto");

        // atomically: nothing is applied when one fails
        const int before = controls.sets;
        const auto atomic = client.set_atomically({{"color.gain", ParameterValue::of(10)},
                                                   {"color.contrast", ParameterValue::of(-1)}})
                                .result;
        expect(not atomic.successful and controls.sets == before, "set_parameters_atomically: all or none");
        controls.fail = true;
        const auto refused = client.set({{"color.gain", ParameterValue::of(10)}}).results[0];
        expect(not refused.successful and refused.reason == "set_color_control failed", "the device's error as reason");
        controls.fail = false;

        const auto deadline = std::chrono::steady_clock::now() + 3s;
        while (std::chrono::steady_clock::now() < deadline)
        {
            {
                std::lock_guard lock(mutex);
                if (events.size() >= 2)
                {
                    break;
                }
            }
            std::this_thread::sleep_for(20ms);
        }
        std::lock_guard lock(mutex);
        expect(events.size() == 2, "two events: the two sets that changed values (" + std::to_string(events.size()) + ")");
        if (events.size() == 2)
        {
            bool time = false, automatic = false;
            for (const Parameter& p : events[0].changed_parameters)
            {
                time = time or (p.name == "color.exposure_time_us" and p.value == ParameterValue::of(20000));
                automatic = automatic or (p.name == "color.auto_exposure" and p.value == ParameterValue::of(false));
            }
            std::string seen = events[0].node + ":";
            for (const Parameter& p : events[0].changed_parameters)
            {
                seen += " " + p.name + "=" + parameter_text(p.value);
            }
            expect(events[0].node == "/t/k4a_params" and time and automatic,
                   "the event: the value and, changed with it, auto_exposure (" + seen + ")");
        }
        const auto diagnostics = parameters.diagnostics();
        expect(std::find(diagnostics.begin(), diagnostics.end(),
                         std::pair<std::string, std::string>{"color.auto_exposure", "true"}) != diagnostics.end() and
                   std::find(diagnostics.begin(), diagnostics.end(),
                             std::pair<std::string, std::string>{"parameter changes", "2"}) != diagnostics.end(),
               "the parameters in the diagnostics");
    }

    // arrival = exposure centre (device time on the host clock) + fixed delay + load; the stamp's error is its
    // difference from exposure centre + fixed delay (what a min estimator can reach without a measured fixed delay)
    struct ClockRun
    {
        double mean_error_ms = 0, p95_error_ms = 0, max_error_ms = 0;
        std::uint64_t snaps = 0, resets = 0;
    };

    ClockRun run_clock(ClockEstimator estimator, bool drift, bool jump, std::uint32_t seed)
    {
        DeviceClock clock(ClockOptions{.estimator = estimator});
        std::mt19937 random(seed);
        std::exponential_distribution<double> load(1.0 / 3.0);  // ms, mean 3
        std::uniform_real_distribution<double> uniform(0.0, 1.0);
        const double fixed_ms = 20.0, period_ms = 1000.0 / 30.0;
        const std::int64_t wall_minus_monotonic = 1'700'000'000'000'000'000;
        // the exposure centre on the host's monotonic clock; the device clock counts from `base`, 50 ppm fast with
        // drift, and starts again at 100 ms at the jump (a restart)
        double true_monotonic_ms = 6000.0, base_ms = 5000.0;
        std::vector<double> errors;
        for (int i = 0; i < 30 * 60; ++i)  // 60 s
        {
            true_monotonic_ms += period_ms;
            if (jump and i == 30 * 30)
            {
                base_ms = true_monotonic_ms - 100.0;
            }
            const double device_ms = (true_monotonic_ms - base_ms) * (drift ? 1.0 + 50e-6 : 1.0);
            double delay_ms = fixed_ms + load(random);
            if (uniform(random) < 0.02)
            {
                delay_ms += 30.0;  // an outlier: a held-up USB transfer
            }
            const auto device = std::chrono::microseconds(std::int64_t(device_ms * 1e3));
            const auto system = std::chrono::nanoseconds(std::int64_t((true_monotonic_ms + delay_ms) * 1e6));
            const auto monotonic_now = system + 1ms;
            clock.update(device, system, monotonic_now + std::chrono::nanoseconds(wall_minus_monotonic), monotonic_now);
            if (i > 30 * 3)
            {
                const double stamp_ms = double(clock.to_wall_ns(device) - wall_minus_monotonic) / 1e6;
                errors.push_back(stamp_ms - (true_monotonic_ms + fixed_ms));
            }
        }
        ClockRun run;
        std::vector<double> absolute;
        for (const double e : errors)
        {
            run.mean_error_ms += e / double(errors.size());
            absolute.push_back(std::abs(e));
        }
        std::sort(absolute.begin(), absolute.end());
        run.p95_error_ms = absolute[std::size_t(0.95 * double(absolute.size() - 1))];
        run.max_error_ms = absolute.back();
        run.snaps = clock.snaps();
        run.resets = clock.resets();
        return run;
    }

    void test_clock_estimators()
    {
        for (const auto& [name, drift, jump] : {std::tuple{"load", false, false}, std::tuple{"load+drift", true, false},
                                                std::tuple{"load+drift+jump", true, true}})
        {
            const ClockRun mean = run_clock(ClockEstimator::kMean, drift, jump, 1);
            const ClockRun min = run_clock(ClockEstimator::kMin, drift, jump, 1);
            std::fprintf(stderr,
                         "clock %-16s mean: bias %+.2f ms, p95 %.2f, max %.2f, snaps %llu | min: bias %+.2f ms, p95 %.2f, "
                         "max %.2f, resets %llu\n",
                         name, mean.mean_error_ms, mean.p95_error_ms, mean.max_error_ms,
                         static_cast<unsigned long long>(mean.snaps), min.mean_error_ms, min.p95_error_ms,
                         min.max_error_ms, static_cast<unsigned long long>(min.resets));
            expect(std::abs(min.mean_error_ms) < 0.3 and min.p95_error_ms < 0.5,
                   std::string(name) + ": min keeps the stamps at the fixed delay (within 0.5 ms)");
            expect(mean.mean_error_ms > 2.0, std::string(name) + ": mean carries the load's mean (about 3.6 ms)");
            if (jump)
            {
                expect(min.resets == 1, "min restarts its window once at the jump");
            }
        }
        DeviceClock fixed(ClockOptions{.estimator = ClockEstimator::kMin, .fixed_delay = 20ms});
        fixed.update(1000us, 25ms, 1ms + 100s, 1ms);
        // offset = (system - device) + (wall - monotonic) = 24 ms + 100 s; the stamp: device + offset - 20 ms
        expect(fixed.to_wall_ns(1000us) == std::chrono::nanoseconds(1ms + 24ms + 100s - 20ms).count(),
               "the fixed delay is subtracted");
    }
}  // namespace

int main()
{
    try
    {
        test_cdr();
        test_overrides();
        test_clock_estimators();
        std::random_device random;
        const int port = 20000 + int(random() % 20000);
        zenoh::Session node_session = open_session({.listen = {"127.0.0.1:" + std::to_string(port)}});
        zenoh::Session client = open_session({.connect = {"127.0.0.1:" + std::to_string(port)}});
        test_services(node_session, client);
    }
    catch (const std::exception& e)
    {
        std::fprintf(stderr, "FAILED with an exception: %s\n", e.what());
        return 1;
    }
    if (failures)
    {
        std::fprintf(stderr, "%d failed\n", failures);
        return 1;
    }
    std::fprintf(stderr, "all passed\n");
    return 0;
}

// k4a_node_bench: the camera node's cost per capture with subscribers on all four streams (color, depth, ir,
// depth_to_color), by transport: none, shared memory, CUDA, both. Captures from JPEG frames of the colour resolutions
// (FRAME_DIR/<720P|2160P|3072P>.jpg) and a depth/IR pattern, at 30 per second; the depth engine
// computes depth_to_color on the GPU in its EGL context. Subscribers are processes of their own (CUDA IPC handles do
// not open in the process that made them) that take every frame as a view into the ring.
//
//   k4a_node_bench FRAME_DIR CALIBRATION_JSON [captures] [cuda device|none]
//
// none: the node without --cuda-device (colour decoded on the CPU, no CUDA rings; shm subscribers only); the depth
// engine still runs on CUDA device 0.
//
// CALIBRATION_JSON: the SDK's test calibration (tests/data/k4a_test_calibration.json), made a retail device's
// (Brown-Conrady). Prints per scenario the time Node::publish took (median, p90) and its parts (mean), and the frames
// each subscriber got.
#include <signal.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <optional>
#include <random>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <nlohmann/json.hpp>

#include "xblick/camera.hpp"
#include "egl_glx.hpp"
#include "node.hpp"

extern char** environ;

namespace
{
    using namespace xblick;
    using namespace xblick::k4a_node;
    using namespace std::chrono_literals;

    std::atomic<bool> stop{false}, restart{false};

    std::vector<std::uint8_t> read_bytes(const std::string& path)
    {
        std::ifstream in(path, std::ios::binary);
        if (not in)
        {
            throw std::runtime_error("cannot read " + path);
        }
        return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
    }

    // the test calibration (Rational 6KT, an early internal device's) as a retail device's: Brown-Conrady
    std::string brown_conrady(const std::string& raw)
    {
        nlohmann::json j = nlohmann::json::parse(raw);
        for (auto& camera : j.at("CalibrationInformation").at("Cameras"))
        {
            camera.at("Intrinsics").at("ModelType") = "CALIBRATION_LensDistortionModelBrownConrady";
            camera.at("Intrinsics").at("ModelParameters").at(10) = 0.0;
            camera.at("Intrinsics").at("ModelParameters").at(11) = 0.0;
        }
        return j.dump();
    }

    // a subscriber process: takes every frame of KEY over TRANSPORT until SIGTERM, then prints how many it got since
    // the last SIGUSR1 (the start of the measured captures)
    int subscribe(const std::string& endpoint, const std::string& key, const std::string& transport)
    {
        std::signal(SIGTERM, [](int) { stop = true; });
        std::signal(SIGUSR1, [](int) { restart = true; });
        zenoh::Session session = open_session({.connect = {endpoint}});
        ImageSubscriber sub(session, key,
                            {.transport = transport == "cuda" ? Transport::Cuda : Transport::Shm, .copy = false});
        std::uint64_t frames = 0;
        while (not stop)
        {
            if (auto frame = sub.latest(true, 100ms))
            {
                ++frames;
            }
            if (restart.exchange(false))
            {
                frames = 0;
            }
        }
        std::printf("%s", (std::to_string(frames) + "\n").c_str());
        return 0;
    }

    struct Resolution
    {
        const char* name;
        k4a_color_resolution_t mode;
        int width, height;
    };

    k4a::capture make_capture(const std::vector<std::uint8_t>& jpeg, const Resolution& r, int seq)
    {
        k4a::capture capture = k4a::capture::create();
        k4a::image depth = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16, 640, 576, 640 * 2);
        k4a::image ir = k4a::image::create(K4A_IMAGE_FORMAT_IR16, 640, 576, 640 * 2);
        auto* d = reinterpret_cast<std::uint16_t*>(depth.get_buffer());
        auto* i = reinterpret_cast<std::uint16_t*>(ir.get_buffer());
        for (int y = 0; y < 576; ++y)
        {
            for (int x = 0; x < 640; ++x)
            {
                d[y * 640 + x] = std::uint16_t(1500 + (x + y + seq) % 1000);
                i[y * 640 + x] = std::uint16_t(1000 + x * y % 777);
            }
        }
        k4a::image color = k4a::image::create_from_buffer(K4A_IMAGE_FORMAT_COLOR_MJPG, r.width, r.height, 0,
                                                          const_cast<std::uint8_t*>(jpeg.data()), jpeg.size(),
                                                          nullptr, nullptr);
        const auto device = std::chrono::microseconds(1000000 + seq * 33333);
        const auto system = std::chrono::steady_clock::now().time_since_epoch();
        for (k4a::image* image : {&depth, &ir, &color})
        {
            image->set_timestamp(device);
            k4a_image_set_system_timestamp_nsec(
                image->handle(),
                std::uint64_t(std::chrono::duration_cast<std::chrono::nanoseconds>(system).count()));
        }
        capture.set_depth_image(depth);
        capture.set_ir_image(ir);
        capture.set_color_image(color);
        return capture;
    }

    pid_t spawn(const std::vector<std::string>& args, int out_fd)
    {
        posix_spawn_file_actions_t actions;
        posix_spawn_file_actions_init(&actions);
        posix_spawn_file_actions_adddup2(&actions, out_fd, STDOUT_FILENO);
        std::vector<char*> argv;
        for (const std::string& a : args)
        {
            argv.push_back(const_cast<char*>(a.c_str()));
        }
        argv.push_back(nullptr);
        pid_t pid = 0;
        if (posix_spawn(&pid, "/proc/self/exe", &actions, nullptr, argv.data(), environ) != 0)
        {
            throw std::runtime_error("cannot start a subscriber process");
        }
        posix_spawn_file_actions_destroy(&actions);
        return pid;
    }

    double quantile(std::vector<double> v, double q)
    {
        std::sort(v.begin(), v.end());
        return v[std::min(v.size() - 1, std::size_t(q * double(v.size())))];
    }
}  // namespace

int main(int argc, char** argv)
{
    if (argc == 5 and std::string(argv[1]) == "--subscribe")
    {
        return subscribe(argv[2], argv[3], argv[4]);
    }
    if (argc < 3)
    {
        std::fprintf(stderr, "usage: k4a_node_bench FRAME_DIR CALIBRATION_JSON [captures] [cuda device|none]\n");
        return 2;
    }
    try
    {
        const std::string frame_dir = argv[1];
        const std::vector<std::uint8_t> calibration = read_bytes(argv[2]);
        const std::string raw = brown_conrady(std::string(calibration.begin(), calibration.end()));
        const int captures = argc > 3 ? std::stoi(argv[3]) : 150;
        const std::optional<int> device =
            argc > 4 and std::string(argv[4]) == "none" ? std::nullopt : std::optional<int>(argc > 4 ? std::stoi(argv[4]) : 0);
        use_egl_for_depth_engine(device.value_or(0));

        std::random_device random;
        const std::string endpoint = "127.0.0.1:" + std::to_string(20000 + int(random() % 20000));
        zenoh::Session session = open_session({.listen = {endpoint}});

        const std::vector<Resolution> resolutions = {{"720P", K4A_COLOR_RESOLUTION_720P, 1280, 720},
                                                     {"2160P", K4A_COLOR_RESOLUTION_2160P, 3840, 2160},
                                                     {"3072P", K4A_COLOR_RESOLUTION_3072P, 4096, 3072}};
        std::vector<std::pair<const char*, std::vector<std::string>>> scenarios = {{"none", {}}, {"shm", {"shm"}}};
        if (device)
        {
            scenarios.push_back({"cuda", {"cuda"}});
            scenarios.push_back({"shm+cuda", {"shm", "cuda"}});
        }
        const std::vector<const char*> streams = {kColor, kDepth, kIr, kDepthToColor};

        std::printf("%-6s %-9s %8s %8s %8s %8s %8s  %s\n", "colour", "subscr.", "median", "p90", "colour", "depth+ir",
                    "d2c", "frames got (color depth ir d2c, per transport)");
        for (const Resolution& r : resolutions)
        {
            const std::vector<std::uint8_t> jpeg = read_bytes(frame_dir + "/" + r.name + ".jpg");
            CalibrationRecord record{.raw = raw, .depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED,
                                     .color_resolution = r.mode, .fps = 30, .serial = "000000000000",
                                     .depth_frame_id = "AzureKinectDepthCamera",
                                     .color_frame_id = "AzureKinectCamera"};
            const std::string base = std::string("bench/r") + r.name;  // ROS name tokens start with a letter
            Node node(session, NodeOptions{.base = base, .source = "bench", .cuda_device = device}, record);
            for (const auto& [scenario, transports] : scenarios)
            {
                std::vector<std::pair<pid_t, int>> children;  // pid, read end of its stdout
                for (const std::string& transport : transports)
                {
                    for (const char* stream : streams)
                    {
                        int fds[2];
                        if (pipe(fds) != 0)
                        {
                            throw std::runtime_error("pipe");
                        }
                        const pid_t pid = spawn({"k4a_node_bench", "--subscribe", endpoint,
                                                 base + "/" + stream, transport},
                                                fds[1]);
                        close(fds[1]);
                        children.emplace_back(pid, fds[0]);
                    }
                }
                // subscribers matched, rings made, decoders warm
                const auto warm_until = std::chrono::steady_clock::now() + 3s;
                int seq = 0;
                while (std::chrono::steady_clock::now() < warm_until)
                {
                    node.publish(make_capture(jpeg, r, seq++), true);
                    std::this_thread::sleep_for(33ms);
                }
                for (const auto& [pid, fd] : children)
                {
                    kill(pid, SIGUSR1);
                }
                std::this_thread::sleep_for(50ms);  // no capture in between: the counters start at 0
                const NodeStats before = node.stats();
                std::vector<double> times;
                auto next = std::chrono::steady_clock::now();
                for (int i = 0; i < captures; ++i)
                {
                    const k4a::capture capture = make_capture(jpeg, r, seq++);
                    const auto t = std::chrono::steady_clock::now();
                    node.publish(capture, true);
                    times.push_back(std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t)
                                        .count());
                    next += 33333us;
                    std::this_thread::sleep_until(next);
                }
                const NodeStats& after = node.stats();
                const double n = captures;
                std::this_thread::sleep_for(300ms);
                std::string got;
                for (const auto& [pid, fd] : children)
                {
                    kill(pid, SIGTERM);
                    int status = 0;
                    waitpid(pid, &status, 0);
                    char buffer[64] = {};
                    const ssize_t k = read(fd, buffer, sizeof(buffer) - 1);
                    close(fd);
                    got += (k > 0 ? std::string(buffer, std::size_t(k) - 1) : std::string("?")) + " ";
                }
                std::printf("%-6s %-9s %6.2f ms %6.2f ms %6.2f ms %6.2f ms %6.2f ms  %s\n", r.name, scenario,
                            quantile(times, 0.5), quantile(times, 0.9), (after.color_ms - before.color_ms) / n,
                            (after.depth_ms - before.depth_ms) / n,
                            (after.depth_to_color_ms - before.depth_to_color_ms) / n, got.c_str());
                std::fflush(stdout);
            }
        }
        return 0;
    }
    catch (const std::exception& e)
    {
        std::fprintf(stderr, "k4a_node_bench: %s\n", e.what());
        return 1;
    }
}

// --ros without a camera: xblick-azure-kinect plays the recording that the offline test writes (K4A_TEST_RECORDING),
// and, where it is built, xblick-k4abt tracks its streams. The test is the Zenoh peer both programs connect to (peers
// do not forward for each other), first as a plain session, then as a ROS node (Durchblick's SessionOptions::ros).
// - Without --ros: our keys, and no key, token or attachment of rmw_zenoh.
// - With --ros: what ROS 2 sees, read through Durchblick's ROS reader (Transport::Ros, ros::Subscriber): the images
//   with their camera_info, /tf_static, the IMU, the diagnostics; every ROS data key with rmw_zenoh's attachment; the
//   tokens of the programs' nodes, of their parameters' nodes and of their publishers; no ROS form of RVL depth or of
//   the body tracking's JSON.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <map>
#include <mutex>
#include <optional>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <sys/wait.h>
#include <unistd.h>

#include "xblick/camera.hpp"
#include "xblick/protocol.hpp"
#include "xblick/ros.hpp"

namespace
{
    using namespace xblick;
    using namespace std::chrono_literals;

    int failures = 0;

    void expect(bool condition, const std::string& what)
    {
        std::fprintf(stderr, "%s: %s\n", condition ? "ok" : "FAILED", what.c_str());
        failures += condition ? 0 : 1;
    }

    bool wait_for(const std::function<bool()>& done, std::chrono::milliseconds timeout)
    {
        const auto end = std::chrono::steady_clock::now() + timeout;
        while (not done())
        {
            if (std::chrono::steady_clock::now() > end)
            {
                return false;
            }
            std::this_thread::sleep_for(50ms);
        }
        return true;
    }

    std::string read_text(const std::string& path)
    {
        std::ifstream in(path);
        std::stringstream s;
        s << in.rdbuf();
        return s.str();
    }

    // A program of this test, its stdout and stderr in <log>; ended with SIGTERM (then SIGKILL) when it goes.
    class Program
    {
    public:
        Program(std::vector<std::string> argv, std::string log) : log_(std::move(log))
        {
            pid_ = ::fork();
            if (pid_ == 0)
            {
                const int fd = ::open(log_.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
                ::dup2(fd, 1);
                ::dup2(fd, 2);
                std::vector<char*> args;
                for (std::string& a : argv)
                {
                    args.push_back(a.data());
                }
                args.push_back(nullptr);
                ::execv(args[0], args.data());
                std::_Exit(127);
            }
            if (pid_ < 0)
            {
                throw std::runtime_error("fork failed");
            }
        }
        Program(const Program&) = delete;
        Program& operator=(const Program&) = delete;
        ~Program() { stop(); }

        // its exit status (stopped by SIGTERM counts as 0); waits up to 20 s, then SIGKILL
        int stop()
        {
            if (pid_ <= 0)
            {
                return status_;
            }
            ::kill(pid_, SIGTERM);
            int status = 0;
            for (int i = 0; i < 400 and ::waitpid(pid_, &status, WNOHANG) == 0; ++i)
            {
                std::this_thread::sleep_for(50ms);
                if (i == 399)
                {
                    ::kill(pid_, SIGKILL);
                    ::waitpid(pid_, &status, 0);
                }
            }
            pid_ = -1;
            status_ = WIFEXITED(status) ? WEXITSTATUS(status) : (WIFSIGNALED(status) ? 128 + WTERMSIG(status) : -1);
            return status_;
        }

        // its exit status once it ended by itself within timeout, else empty
        std::optional<int> wait(std::chrono::milliseconds timeout)
        {
            int status = 0;
            const auto end = std::chrono::steady_clock::now() + timeout;
            while (::waitpid(pid_, &status, WNOHANG) == 0)
            {
                if (std::chrono::steady_clock::now() > end)
                {
                    return std::nullopt;
                }
                std::this_thread::sleep_for(50ms);
            }
            pid_ = -1;
            status_ = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
            return status_;
        }

        std::string output() const { return read_text(log_); }

    private:
        std::string log_;
        pid_t pid_ = -1;
        int status_ = 0;
    };

    // Every key the session sees (data and liveliness), with the attachment of the last sample on each data key.
    struct Observer
    {
        Observer(zenoh::Session& session, const std::string& data, const std::string& tokens)
        {
            data_.emplace(session.declare_subscriber(
                zenoh::KeyExpr(data),
                [this](const zenoh::Sample& sample) {
                    const auto a = sample.get_attachment();
                    std::lock_guard lock(mutex);
                    attachments[std::string(sample.get_keyexpr().as_string_view())] =
                        a ? a->get().as_vector() : std::vector<std::uint8_t>{};
                },
                zenoh::closures::none));
            zenoh::Session::LivelinessSubscriberOptions options;
            options.history = true;
            tokens_.emplace(session.liveliness_declare_subscriber(
                zenoh::KeyExpr(tokens),
                [this](const zenoh::Sample& sample) {
                    if (sample.get_kind() == Z_SAMPLE_KIND_PUT)
                    {
                        std::lock_guard lock(mutex);
                        token_keys.insert(std::string(sample.get_keyexpr().as_string_view()));
                    }
                },
                zenoh::closures::none, std::move(options)));
        }

        std::mutex mutex;
        std::map<std::string, std::vector<std::uint8_t>> attachments;
        std::set<std::string> token_keys;

        std::map<std::string, std::vector<std::uint8_t>> data()
        {
            std::lock_guard lock(mutex);
            return attachments;
        }
        std::set<std::string> tokens()
        {
            std::lock_guard lock(mutex);
            return token_keys;
        }

    private:
        std::optional<zenoh::Subscriber<void>> data_, tokens_;
    };

    const std::string kBase = "t/k4a_ros";
    constexpr int kDomain = 95;  // ROS_DOMAIN_ID of nothing else here

    std::vector<std::string> camera_argv(int port, bool ros)
    {
        std::vector<std::string> argv{K4A_CAMERA_PROGRAM, "--playback", K4A_TEST_RECORDING, "--loop",
                                      "--base", kBase, "--connect", "127.0.0.1:" + std::to_string(port)};
        if (ros)
        {
            argv.insert(argv.end(), {"--ros", "--ros-domain", std::to_string(kDomain)});
        }
        return argv;
    }

#ifdef K4A_BODY_TRACKING_PROGRAM
    std::vector<std::string> tracking_argv(int port, bool ros)
    {
        // on the CPU, a pair a second: the test looks at what goes to ROS, not at the bodies
        std::vector<std::string> argv{K4A_BODY_TRACKING_PROGRAM, "--base", kBase, "--model", K4A_BODY_TRACKING_MODEL,
                                      "--processing", "cpu", "--max-rate", "1",
                                      "--connect", "127.0.0.1:" + std::to_string(port)};
        if (ros)
        {
            argv.insert(argv.end(), {"--ros", "--ros-domain", std::to_string(kDomain)});
        }
        return argv;
    }
#endif

    int free_port()
    {
        std::random_device random;
        return 20000 + int(random() % 20000);
    }

    // ------------------------------------------------------------------------------------------- without --ros

    void test_without_ros()
    {
        const int port = free_port();
        zenoh::Session session = open_session({.listen = {"127.0.0.1:" + std::to_string(port)}, .declare = false});
        Observer observer(session, "**", "@ros2_lv/**");
        Program camera(camera_argv(port, false), std::string(K4A_TEST_TMP) + "/test_ros_camera_plain.log");
#ifdef K4A_BODY_TRACKING_PROGRAM
        Program tracking(tracking_argv(port, false), std::string(K4A_TEST_TMP) + "/test_ros_k4abt_plain.log");
#endif
        const bool frames = wait_for([&] { return observer.data().contains(kBase + "/depth/image_raw"); }, 20s);
        expect(frames, "without --ros: the recording's frames on our keys");
#ifdef K4A_BODY_TRACKING_PROGRAM
        const bool tracked = wait_for([&] { return observer.data().contains(kBase + "/bodies"); }, 60s);
        expect(tracked, "without --ros: the body tracking's results on <base>/bodies");
#endif
        std::this_thread::sleep_for(2s);
        std::vector<std::string> ros_keys;
        for (const auto& [key, _] : observer.data())
        {
            if (key.find("::dds_::") != std::string::npos or key.rfind(std::to_string(kDomain) + "/", 0) == 0)
            {
                ros_keys.push_back(key);
            }
        }
        expect(ros_keys.empty(), "without --ros: no data key of rmw_zenoh (" + std::to_string(ros_keys.size()) + ")");
        expect(observer.tokens().empty(),
               "without --ros: no liveliness token of rmw_zenoh (" + std::to_string(observer.tokens().size()) + ")");
        expect(camera.stop() == 0, "without --ros: the camera program ends cleanly");
#ifdef K4A_BODY_TRACKING_PROGRAM
        expect(tracking.stop() == 0, "without --ros: xblick-k4abt ends cleanly");
#endif
    }

    // ---------------------------------------------------------------------------------------------- with --ros

    std::string ros_key(const std::string& key, const std::string& type)
    {
        return ros::data_key(kDomain, ros::topic_of(key), type, ros::hash_of(type));
    }

    void test_with_ros()
    {
        const int port = free_port();
        zenoh::Session session = open_session({.listen = {"127.0.0.1:" + std::to_string(port)}, .declare = false,
                                               .ros = ros::Options{.domain = kDomain, .node = "k4a_ros_test"}});
        const std::shared_ptr<ros::Ros> part = ros::of(session);
        Observer observer(session, std::to_string(kDomain) + "/**", "@ros2_lv/" + std::to_string(kDomain) + "/**");

        // the readers of ROS's topics, there before the programs start (the C++ API reads ROS volatile only: the
        // /tf_static reader takes the transforms as they are published at the camera's start)
        std::mutex mutex;
        std::optional<TFMessage> tf_static;
        std::optional<Imu> imu;
        std::vector<DiagnosticStatus> statuses;
        ros::Subscriber tf_reader(
            session, *part, part->node(), "/tf_static", std::string(protocol::kTFMessageType.name),
            [&](std::vector<std::uint8_t>&& payload, const ros::Attachment&) {
                std::lock_guard lock(mutex);
                tf_static = TFMessage::deserialize(payload);
            });
        ros::Subscriber imu_reader(session, *part, part->node(), ros::topic_of(kBase + "/imu"),
                                   std::string(protocol::kImuType.name),
                                   [&](std::vector<std::uint8_t>&& payload, const ros::Attachment&) {
                                       std::lock_guard lock(mutex);
                                       imu = Imu::deserialize(payload);
                                   });
        ros::Subscriber diagnostics_reader(
            session, *part, part->node(), ros::topic_of(kBase + "/diagnostics"),
            std::string(protocol::kDiagnosticArrayType.name),
            [&](std::vector<std::uint8_t>&& payload, const ros::Attachment&) {
                std::lock_guard lock(mutex);
                for (const DiagnosticStatus& status : DiagnosticArray::deserialize(payload).status)
                {
                    statuses.push_back(status);
                }
            });
        const SubscriberOptions through_ros{.transport = Transport::Ros, .camera_info = true};
        std::map<std::string, std::unique_ptr<ImageSubscriber>> images;
        for (const char* stream : {"color", "color_rect", "depth", "ir", "depth_to_color", "depth_to_color_rect"})
        {
            images[stream] = std::make_unique<ImageSubscriber>(session, kBase + "/" + stream, through_ros);
        }

        Program camera(camera_argv(port, true), std::string(K4A_TEST_TMP) + "/test_ros_camera.log");
#ifdef K4A_BODY_TRACKING_PROGRAM
        Program tracking(tracking_argv(port, true), std::string(K4A_TEST_TMP) + "/test_ros_k4abt.log");
#endif
        // each stream through ROS: pixels as published, its camera_info with it
        const std::map<std::string, std::tuple<std::string, std::uint32_t, std::uint32_t>> expected{
            {"color", {"rgb8", 1280, 720}},        {"color_rect", {"rgb8", 1280, 720}},
            {"depth", {"16UC1", 640, 576}},        {"ir", {"mono16", 640, 576}},
            {"depth_to_color", {"16UC1", 1280, 720}}, {"depth_to_color_rect", {"16UC1", 1280, 720}}};
        for (const auto& [stream, shape] : expected)
        {
            const auto& [encoding, width, height] = shape;
            std::optional<Frame> frame = images[stream]->latest(true, 20000ms);
            // the camera_info of a frame comes on its own topic: the next frame waits for it
            wait_for([&] { return images[stream]->camera_info().has_value(); }, 2s);
            const std::optional<CameraInfo> info = images[stream]->camera_info();
            if (frame and info)
            {
                std::fprintf(stderr, "%s: %s %ux%u, camera_info %ux%u in %s, frame %s\n", stream.c_str(),
                             frame->encoding.c_str(), frame->width, frame->height, info->width, info->height,
                             info->header.frame_id.c_str(), frame->header.frame_id.c_str());
            }
            expect(frame and frame->encoding == encoding and frame->width == width and frame->height == height and
                       frame->data() != nullptr and info and info->width == width and
                       info->height == height and info->header.frame_id == frame->header.frame_id,
                   "--ros: /" + kBase + "/" + stream + "/image_raw (" + encoding + ") and its camera_info through ROS");
            expect(frame and not frame->seq, "--ros: a ROS sample carries no attachment of ours (" + stream + ")");
        }
        const bool came = wait_for(
            [&] {
                std::lock_guard lock(mutex);
                return tf_static and imu and
                       std::any_of(statuses.begin(), statuses.end(),
                                   [](const DiagnosticStatus& s) { return s.name == "azure_kinect"; });
            },
            10s);
        {
            std::lock_guard lock(mutex);
            const auto has = [&](const std::string& parent, const std::string& child) {
                return tf_static and std::any_of(tf_static->transforms.begin(), tf_static->transforms.end(),
                                                 [&](const TransformStamped& t) {
                                                     return t.header.frame_id == parent and t.child_frame_id == child;
                                                 });
            };
            expect(came, "--ros: /tf_static, the IMU and the diagnostics through ROS");
            expect(has("AzureKinectDepthCamera", "AzureKinectCamera") and has("AzureKinectDepthCamera", "AzureKinectImu"),
                   "--ros: /tf_static has the colour camera and the IMU in the depth camera's frame");
            expect(imu and imu->header.frame_id == "AzureKinectImu", "--ros: /" + kBase + "/imu in the IMU's frame");
        }

        // rmw_zenoh's keys with its attachment: every topic above, the compressed colour, nothing for RVL
        std::vector<std::pair<std::string, std::string>> topics{
            {kBase + "/imu", "sensor_msgs/msg/Imu"},
            {kBase + "/diagnostics", "diagnostic_msgs/msg/DiagnosticArray"},
            {"tf_static", "tf2_msgs/msg/TFMessage"},
            {kBase + "/color/image_raw/compressed", "sensor_msgs/msg/CompressedImage"},
            {kBase + "/color_rect/image_raw/compressed", "sensor_msgs/msg/CompressedImage"}};
        for (const auto& [stream, _] : expected)
        {
            topics.emplace_back(kBase + "/" + stream + "/image_raw", "sensor_msgs/msg/Image");
            topics.emplace_back(kBase + "/" + stream + "/camera_info", "sensor_msgs/msg/CameraInfo");
        }
        wait_for([&] { return observer.data().size() >= topics.size(); }, 5s);
        const auto data = observer.data();
        for (const auto& [key, type] : topics)
        {
            const auto it = data.find(ros_key(key, type));
            bool attached = false;
            if (it != data.end())
            {
                try
                {
                    const ros::Attachment a = ros::decode_attachment(it->second);
                    const std::int64_t now = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                                 std::chrono::system_clock::now().time_since_epoch())
                                                 .count();
                    attached = a.seq >= 1 and std::abs(now - a.source_ns) < 60'000'000'000;
                }
                catch (const ros::RosError&)
                {
                }
            }
            expect(attached, "--ros: " + ros_key(key, type) + " with rmw_zenoh's attachment");
        }
        std::vector<std::string> unexpected;
        for (const auto& [key, _] : data)
        {
            if (key.find("bodies") != std::string::npos)
            {
                unexpected.push_back(key);
            }
        }
        for (const char* stream : {"depth", "ir", "depth_to_color", "depth_to_color_rect"})
        {
            const std::string rvl = ros_key(kBase + "/" + stream + "/image_raw/compressed", "sensor_msgs/msg/CompressedImage");
            if (data.contains(rvl))
            {
                unexpected.push_back(rvl);
            }
        }
        expect(unexpected.empty(), "--ros: no ROS form of RVL depth or of the bodies (" +
                                       (unexpected.empty() ? std::string() : unexpected.front()) + ")");

        // the tokens: the programs' nodes, their parameters' nodes, the publishers (in the program's node)
        std::set<std::string> nodes, publishers;
        for (const std::string& key : observer.tokens())
        {
            const ros::Token token = ros::Token::parse(key);
            if (token.kind == ros::kNode)
            {
                nodes.insert(token.fully_qualified_node());
            }
            else if (token.kind == ros::kPublisher and token.node == "xblick_azure_kinect")
            {
                publishers.insert(*token.topic + " " + *token.type_name +
                                  (token.qos and token.qos->durability == ros::kTransientLocal ? " transient" : ""));
            }
        }
        expect(nodes.contains("/xblick_azure_kinect") and nodes.contains("/" + kBase),
               "--ros: the ROS nodes /xblick_azure_kinect and /" + kBase + " (its parameters)");
        expect(publishers.contains("/" + kBase + "/color/image_raw sensor_msgs/msg/Image") and
                   publishers.contains("/" + kBase + "/depth/image_raw sensor_msgs/msg/Image") and
                   publishers.contains("/tf_static tf2_msgs/msg/TFMessage transient"),
               "--ros: publisher tokens of /xblick_azure_kinect, /tf_static transient local (" +
                   std::to_string(publishers.size()) + " publishers)");
#ifdef K4A_BODY_TRACKING_PROGRAM
        const bool tracking_reported = wait_for(
            [&] {
                std::lock_guard lock(mutex);
                return std::any_of(statuses.begin(), statuses.end(),
                                   [](const DiagnosticStatus& s) { return s.name == "body_tracking"; });
            },
            60s);
        expect(tracking_reported, "--ros: xblick-k4abt's diagnostics through ROS");
        std::set<std::string> nodes_now;
        for (const std::string& key : observer.tokens())
        {
            const ros::Token token = ros::Token::parse(key);
            if (token.kind == ros::kNode)
            {
                nodes_now.insert(token.fully_qualified_node());
            }
        }
        expect(nodes_now.contains("/xblick_k4abt") and nodes_now.contains("/" + kBase + "/k4abt"),
               "--ros: the ROS nodes /xblick_k4abt and /" + kBase + "/k4abt (its parameters)");
        expect(tracking.output().find("xblick: " + kBase + "/bodies: not offered to ROS") != std::string::npos,
               "--ros: xblick-k4abt says that <base>/bodies is not offered to ROS");
        expect(tracking.stop() == 0, "--ros: xblick-k4abt ends cleanly");
#endif
        expect(camera.stop() == 0, "--ros: the camera program ends cleanly");
    }

    // A ROS option without --ros stops the program with the reason.
    void test_refused()
    {
        Program camera({K4A_CAMERA_PROGRAM, "--playback", K4A_TEST_RECORDING, "--ros-domain", "3"},
                       std::string(K4A_TEST_TMP) + "/test_ros_refused.log");
        const std::optional<int> status = camera.wait(10s);
        expect(status and *status != 0 and camera.output().find("need --ros") != std::string::npos,
               "--ros-domain without --ros: the program stops and says why");
    }
}  // namespace

int main()
{
    try
    {
        test_refused();
        test_without_ros();
        test_with_ros();
    }
    catch (const std::exception& e)
    {
        std::fprintf(stderr, "FAILED with an exception: %s\n", e.what());
        return 1;
    }
    std::fprintf(stderr, failures ? "%d failures\n" : "ok\n", failures);
    return failures ? 1 : 0;
}

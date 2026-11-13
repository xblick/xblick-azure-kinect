#include "node.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <exception>
#include <functional>
#include <mutex>
#include <span>
#include <thread>
#include <stdexcept>
#include <vector>

#include "depth_to_color.hpp"
#include "durchblick/camera.hpp"
#include "jpeg.hpp"
#include "rectify.hpp"

#ifdef DURCHBLICK_K4A_CUDA
#include "nvjpeg_decoder.hpp"
#endif

namespace durchblick::k4a_node
{
    namespace
    {
        double ms_since(std::chrono::steady_clock::time_point start)
        {
            return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
        }

        std::int64_t wall_ns()
        {
            return std::chrono::duration_cast<std::chrono::nanoseconds>(
                       std::chrono::system_clock::now().time_since_epoch())
                .count();
        }

        // The pose of a sensor of the device in the depth camera's frame, from the factory calibration: the SDK's
        // extrinsics map depth to the sensor (p_sensor = R p_depth + t, mm); tf2 wants the inverse (p_depth =
        // R^T p_sensor - R^T t), in metres.
        TransformStamped pose_in_depth(const k4a_calibration_t& calibration, k4a_calibration_type_t sensor,
                                       const std::string& depth_frame, const std::string& sensor_frame)
        {
            const k4a_calibration_extrinsics_t& e = calibration.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][sensor];
            std::array<double, 9> rt{};
            Vector3 translation;
            double* t[3] = {&translation.x, &translation.y, &translation.z};
            for (int i = 0; i < 3; ++i)
            {
                for (int j = 0; j < 3; ++j)
                {
                    rt[3 * i + j] = e.rotation[3 * j + i];
                    *t[i] -= e.rotation[3 * j + i] * e.translation[j] / 1000.0;
                }
            }
            TransformStamped pose;
            pose.header = {Time::from_ns(wall_ns()), depth_frame};
            pose.child_frame_id = sensor_frame;
            pose.transform = Transform::from_matrix(rt, translation);
            return pose;
        }

        HostImage host_image(const k4a::image& image)
        {
            return HostImage{image.get_buffer(), static_cast<std::uint32_t>(image.get_height_pixels()),
                             static_cast<std::uint32_t>(image.get_width_pixels()),
                             static_cast<std::uint32_t>(image.get_stride_bytes())};
        }
    }  // namespace

    // One thread that publishes, with a mailbox of one job: a job posted while the previous one waits replaces it
    // (that frame is skipped for this lane and counted), so that a lane that falls behind publishes the newest frame
    // and never queues up latency. An exception ends the lane's work and is handed to on_error.
    class Lane
    {
    public:
        Lane(std::function<void()> on_skip, std::function<void(std::exception_ptr)> on_error)
            : on_skip_(std::move(on_skip)), on_error_(std::move(on_error)), thread_([this] { run(); })
        {
        }

        ~Lane()
        {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                stop_ = true;
            }
            wake_.notify_all();
            thread_.join();
        }

        Lane(const Lane&) = delete;
        Lane& operator=(const Lane&) = delete;

        void post(std::function<void()> job)
        {
            bool replaced = false;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                replaced = static_cast<bool>(job_);
                job_ = std::move(job);
            }
            wake_.notify_one();
            if (replaced)
            {
                on_skip_();
            }
        }

    private:
        void run()
        {
            for (;;)
            {
                std::function<void()> job;
                {
                    std::unique_lock<std::mutex> lock(mutex_);
                    wake_.wait(lock, [this] { return stop_ or static_cast<bool>(job_); });
                    if (stop_)
                    {
                        return;
                    }
                    job = std::move(job_);
                    job_ = nullptr;
                }
                try
                {
                    job();
                }
                catch (...)
                {
                    on_error_(std::current_exception());
                }
            }
        }

        std::function<void()> on_skip_;
        std::function<void(std::exception_ptr)> on_error_;
        std::mutex mutex_;
        std::condition_variable wake_;
        std::function<void()> job_;
        bool stop_ = false;
        std::thread thread_;  // last: started when the rest is ready
    };

    struct Node::Impl
    {
        NodeOptions o;
        CalibrationRecord record;
        k4a_calibration_t calibration{};
        std::unique_ptr<CameraPublisher> color, color_rect, depth, ir, depth_to_color, depth_to_color_rect;
        RectifyMap rect_map;
        std::vector<std::uint8_t> rect_pixels;  // the CPU's rectified frame
        std::vector<std::uint16_t> rect_depth;  // depth_to_color rectified
        std::optional<zenoh::Queryable<void>> calibration_queryable;
        std::unique_ptr<StaticTransformPublisher> tf_static;  // depth camera -> colour camera, IMU
        std::unique_ptr<ImuPublisher> imu;
        std::unique_ptr<DiagnosticsPublisher> diagnostics;
        std::array<float, 9> gyro_to_accel{};
        std::optional<float> temperature;  // of the last capture, deg C
        // since the last report, per capture with colour and depth [us]: colour - depth device timestamps, exposures
        std::vector<double> stamp_differences, color_exposures, depth_exposures;
        std::chrono::steady_clock::time_point started, reported;
        NodeStats reported_stats;
        std::uint64_t reported_dropped = 0, reported_restarts = 0;
        k4a::transformation transformation;  // sdk_depth_to_color
        k4a::image depth_in_color;
        std::unique_ptr<DepthToColor> cpu_depth_to_color;  // else
        CpuJpegDecoder cpu_jpeg;
#ifdef DURCHBLICK_K4A_CUDA
        std::unique_ptr<NvJpegDecoder> nv_jpeg;
        std::unique_ptr<GpuRectifier> gpu_rect;
#endif
        DeviceClock clock;
        std::optional<std::chrono::microseconds> last_recorded;  // a recording's latest device time
        mutable std::mutex stats_mutex;  // stats: the lanes count, the capture thread reads
        NodeStats stats;
        std::uint32_t color_width = 0, color_height = 0;
        std::mutex error_mutex;
        std::exception_ptr error;  // a lane's, rethrown by the next publish()
        // async_publish: colour; depth and IR; depth_to_color(_rect) computed, the rectified one published; the
        // CPU's raw depth_to_color published. Last members: destroyed (joined) first, before what their jobs use.
        std::unique_ptr<Lane> color_lane, depth_lane, to_color_lane, to_color_raw_lane;

        void count(std::uint64_t NodeStats::* field, std::uint64_t n = 1)
        {
            std::lock_guard<std::mutex> lock(stats_mutex);
            stats.*field += n;
        }

        void add_ms(double NodeStats::* field, double ms)
        {
            std::lock_guard<std::mutex> lock(stats_mutex);
            stats.*field += ms;
        }

        // the job on its lane, or here and now without async_publish
        void run(const std::unique_ptr<Lane>& lane, std::function<void()> job)
        {
            if (lane)
            {
                lane->post(std::move(job));
            }
            else
            {
                job();
            }
        }

        void start_lanes()
        {
            const auto skip = [this] { count(&NodeStats::skipped); };
            const auto fail = [this](std::exception_ptr e) {
                std::lock_guard<std::mutex> lock(error_mutex);
                if (not error)
                {
                    error = e;
                }
            };
            color_lane = std::make_unique<Lane>(skip, fail);
            depth_lane = std::make_unique<Lane>(skip, fail);
            to_color_lane = std::make_unique<Lane>(skip, fail);
            to_color_raw_lane = std::make_unique<Lane>(skip, fail);
        }

        void rethrow_lane_error()
        {
            std::lock_guard<std::mutex> lock(error_mutex);
            if (error)
            {
                std::rethrow_exception(error);
            }
        }

        std::int64_t stamp(const k4a::image& image, bool live)
        {
            const std::chrono::microseconds device = image.get_device_timestamp();
            if (live)
            {
                clock.update(device, image.get_system_timestamp());
            }
            else if (not clock.offset() or (last_recorded and device < *last_recorded))
            {
                // a recording: its device clock, placed so that its first image was taken now; again when its device
                // time goes back (it started again: --loop), so that the stamps go on increasing
                const auto wall = std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::system_clock::now().time_since_epoch());
                // (offset = wall - device: this image's stamp is now)
                clock.update(device, std::chrono::nanoseconds(0), wall, std::chrono::nanoseconds(0));
            }
            if (not live)
            {
                last_recorded = device;
            }
            return clock.to_wall_ns(device);
        }

        // In the capture thread: whether anybody wants the colour, and its stamp (the device clock takes its images
        // in order); the work on the colour lane.
        void publish_color(const k4a::image& image, bool live)
        {
            if (not image)  // also: the colour camera is off (no publishers)
            {
                return;
            }
            const bool raw = color->subscribed();
            const bool rect = color_rect->subscribed();
            if (not raw and not rect)
            {
                return;
            }
            if (image.get_format() != K4A_IMAGE_FORMAT_COLOR_MJPG)
            {
                throw std::runtime_error("the colour image is not MJPEG");
            }
            const std::int64_t ns = stamp(image, live);
            run(color_lane, [this, image, ns, raw, rect] { color_work(image, ns, raw, rect); });
        }

        void color_work(const k4a::image& image, std::int64_t ns, bool raw, bool rect)
        {
            const auto start = std::chrono::steady_clock::now();
            std::span<const std::uint8_t> jpeg(image.get_buffer(), image.get_size());
            std::vector<std::uint8_t> completed;
            if (not has_huffman_tables(jpeg))
            {
                completed = with_huffman_tables(jpeg);
                jpeg = completed;
                count(&NodeStats::huffman_added);
            }
            const bool pixels = raw and color->needs_pixels();
            if (raw and not pixels)
            {
                color->publish_jpeg(jpeg, ns);
            }
            if (not pixels and not rect)
            {
                count(&NodeStats::color_jpeg_only);
                add_ms(&NodeStats::color_ms, ms_since(start));
                return;
            }
            // decoded once, for the pixels of color and for color_rect
#ifdef DURCHBLICK_K4A_CUDA
            if (nv_jpeg)
            {
                const std::uint64_t ptr = nv_jpeg->decode(jpeg, color_width, color_height);
                if (pixels)
                {
                    color->publish(DeviceImage{ptr, color_height, color_width, nv_jpeg->stream()}, ns, jpeg);
                }
                if (rect)
                {
                    const auto t = std::chrono::steady_clock::now();
                    const std::uint64_t out = gpu_rect->rectify(ptr, nv_jpeg->stream());
                    count(&NodeStats::color_rect);
                    color_rect->publish(DeviceImage{out, color_height, color_width, nv_jpeg->stream()}, ns);
                    add_ms(&NodeStats::color_rect_ms, ms_since(t));
                }
            }
            else
#endif
            {
                const std::uint8_t* decoded = cpu_jpeg.decode(jpeg, color_width, color_height);
                if (pixels)
                {
                    color->publish(HostImage{decoded, color_height, color_width, 0}, ns, jpeg);
                }
                if (rect)
                {
                    const auto t = std::chrono::steady_clock::now();
                    remap_rgb8(rect_map, decoded, rect_pixels.data());
                    count(&NodeStats::color_rect);
                    color_rect->publish(HostImage{rect_pixels.data(), color_height, color_width, 0}, ns);
                    add_ms(&NodeStats::color_rect_ms, ms_since(t));
                }
            }
            count(&NodeStats::color_decoded);
            add_ms(&NodeStats::color_ms, ms_since(start));
        }

        // In the capture thread: the stamp and who wants what; depth and IR on the depth lane, depth_to_color on its
        // own (the counts come before each publish, so that a reader that has a frame also sees it counted).
        void publish_depth(const k4a::image& depth_image, const k4a::image& ir_image, bool live)
        {
            if (not depth_image)
            {
                return;
            }
            const std::int64_t ns = stamp(depth_image, live);
            if (o.max_depth_mm != 0)
            {
                // beyond the cut no depth, for every stream made from this image (in place: the capture's own image)
                auto* d = reinterpret_cast<std::uint16_t*>(const_cast<std::uint8_t*>(depth_image.get_buffer()));
                const std::size_t stride = std::size_t(depth_image.get_stride_bytes()) / 2;
                const int w = depth_image.get_width_pixels(), h = depth_image.get_height_pixels();
                for (int y = 0; y < h; ++y)
                {
                    std::uint16_t* row = d + std::size_t(y) * stride;
                    for (int x = 0; x < w; ++x)
                    {
                        row[x] = row[x] > o.max_depth_mm ? std::uint16_t(0) : row[x];
                    }
                }
            }
            const bool to_depth = depth->subscribed();
            const bool to_ir = ir_image and ir->subscribed();
            if (to_depth or to_ir)
            {
                run(depth_lane, [this, depth_image, ir_image, ns, to_depth, to_ir] {
                    const auto start = std::chrono::steady_clock::now();
                    if (to_depth)
                    {
                        count(&NodeStats::depth);
                        depth->publish(host_image(depth_image), ns);
                    }
                    if (to_ir)
                    {
                        // the same exposure: the depth image's stamp, which pairs them for a consumer
                        count(&NodeStats::ir);
                        ir->publish(host_image(ir_image), ns);
                    }
                    add_ms(&NodeStats::depth_ms, ms_since(start));
                });
            }
            const bool to_color = depth_to_color and depth_to_color->subscribed();
            const bool to_color_rect = depth_to_color_rect and depth_to_color_rect->subscribed();
            if (to_color or to_color_rect)
            {
                run(to_color_lane,
                    [this, depth_image, ns, to_color, to_color_rect] { to_color_work(depth_image, ns, to_color,
                                                                                      to_color_rect); });
            }
        }

        void to_color_work(const k4a::image& depth_image, std::int64_t ns, bool to_color, bool to_color_rect)
        {
            const auto publish_rect = [&] {
                const auto r = std::chrono::steady_clock::now();
                count(&NodeStats::depth_to_color_rect);
                depth_to_color_rect->publish(
                    HostImage{reinterpret_cast<const std::uint8_t*>(rect_depth.data()), color_height, color_width, 0},
                    ns);
                add_ms(&NodeStats::depth_to_color_rect_ms, ms_since(r));
            };
            if (cpu_depth_to_color)
            {
                // projected into the rectified camera; depth_to_color read from it, published on a lane of its own
                const auto t = std::chrono::steady_clock::now();
                cpu_depth_to_color->rectified(reinterpret_cast<const std::uint16_t*>(depth_image.get_buffer()),
                                              std::size_t(depth_image.get_stride_bytes()), rect_depth.data());
                if (to_color)
                {
                    auto raw = std::make_shared<std::vector<std::uint16_t>>(rect_depth.size());
                    cpu_depth_to_color->unrectify(raw->data());
                    const double compute_ms = ms_since(t);
                    run(to_color_raw_lane, [this, raw, ns, compute_ms] {
                        const auto p = std::chrono::steady_clock::now();
                        count(&NodeStats::depth_to_color);
                        depth_to_color->publish(
                            HostImage{reinterpret_cast<const std::uint8_t*>(raw->data()), color_height, color_width, 0},
                            ns);
                        add_ms(&NodeStats::depth_to_color_ms, compute_ms + ms_since(p));
                    });
                }
                if (to_color_rect)
                {
                    publish_rect();
                }
                return;
            }
            const auto t = std::chrono::steady_clock::now();
            transformation.depth_image_to_color_camera(depth_image, &depth_in_color);
            if (to_color)
            {
                count(&NodeStats::depth_to_color);
                depth_to_color->publish(host_image(depth_in_color), ns);
            }
            add_ms(&NodeStats::depth_to_color_ms, ms_since(t));
            if (to_color_rect)
            {
                remap_depth16(rect_map, reinterpret_cast<const std::uint16_t*>(depth_in_color.get_buffer()),
                              rect_depth.data());
                publish_rect();
            }
        }
    };

    Node::Node(zenoh::Session& session, const NodeOptions& options, const CalibrationRecord& record)
        : impl_(std::make_unique<Impl>())
    {
        Impl& m = *impl_;
        m.o = options;
        m.clock = DeviceClock(options.clock);
        m.record = record;
        m.calibration = record.calibration();
        const std::string base = check_name(options.base);
        if (record.color_resolution != K4A_COLOR_RESOLUTION_OFF)
        {
            const CameraInfo info = record.color_info();
            m.color_width = info.width;
            m.color_height = info.height;
            m.color = std::make_unique<CameraPublisher>(
                session, base + "/" + kColor,
                PublisherOptions{.encoding = "rgb8", .frame_id = record.color_frame_id, .source = options.source,
                                 .camera_info = info, .codec = kJpeg, .slots = options.slots,
                                 .cuda_device = options.cuda_device});
#ifndef DURCHBLICK_K4A_CUDA
            if (options.cuda_device)
            {
                throw std::invalid_argument("--cuda-device: built without CUDA (DURCHBLICK_K4A_CUDA)");
            }
#endif
            m.rect_map = rectify_map(info);
            m.color_rect = std::make_unique<CameraPublisher>(
                session, base + "/" + kColorRect,
                PublisherOptions{.encoding = "rgb8", .frame_id = record.color_frame_id, .source = options.source,
                                 .camera_info = rectified_info(info), .codec = kJpeg, .slots = options.slots,
                                 .cuda_device = options.cuda_device});
#ifdef DURCHBLICK_K4A_CUDA
            if (options.cuda_device)
            {
                m.nv_jpeg = std::make_unique<NvJpegDecoder>(*options.cuda_device);
                m.gpu_rect = std::make_unique<GpuRectifier>(*options.cuda_device, m.rect_map);
            }
            else
#endif
            {
                m.rect_pixels.resize(std::size_t(m.color_width) * m.color_height * 3);
            }
        }
        if (record.depth_mode != K4A_DEPTH_MODE_OFF)
        {
            const CameraInfo info = record.depth_info();
            const auto publisher = [&](const char* name, const char* encoding, const CameraInfo& camera) {
                return std::make_unique<CameraPublisher>(
                    session, base + "/" + name,
                    PublisherOptions{.encoding = encoding, .frame_id = camera.header.frame_id,
                                     .source = options.source, .camera_info = camera, .codec = kRvl,
                                     .slots = options.slots, .cuda_device = options.cuda_device});
            };
            m.depth = publisher(kDepth, "16UC1", info);
            if (record.depth_mode != K4A_DEPTH_MODE_PASSIVE_IR)
            {
                m.ir = publisher(kIr, "mono16", info);
            }
            if (options.depth_to_color and record.color_resolution != K4A_COLOR_RESOLUTION_OFF)
            {
                if (options.sdk_depth_to_color)
                {
                    // k4a::transformation's constructor swallows a failure (a null handle, found only per frame
                    // later): create it here, where the cause is known
                    k4a_transformation_t handle = k4a_transformation_create(&m.calibration);
                    if (handle == nullptr)
                    {
                        throw std::runtime_error(
                            "the SDK's transformation could not be created (depth_to_color): the depth engine plugin "
                            "libdepthengine.so.2.0 must be on LD_LIBRARY_PATH, "
                            "or pass --no-depth-to-color, or leave out --sdk-depth-to-color");
                    }
                    m.transformation = k4a::transformation(handle);
                    m.depth_in_color = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16, int(m.color_width),
                                                          int(m.color_height), int(m.color_width) * 2);
                }
                else
                {
                    m.cpu_depth_to_color =
                        std::make_unique<DepthToColor>(m.calibration, record.color_info(), m.rect_map,
                                                       options.depth_to_color_interpolation,
                                                       options.depth_to_color_drop_flying,
                                                       options.depth_to_color_drop_sparse);
                }
                m.depth_to_color = publisher(kDepthToColor, "16UC1", record.color_info());
                m.depth_to_color_rect = publisher(kDepthToColorRect, "16UC1", rectified_info(record.color_info()));
                m.rect_depth.resize(std::size_t(m.color_width) * m.color_height);
            }
        }
        if (record.depth_mode != K4A_DEPTH_MODE_OFF)
        {
            // the colour camera and the IMU in the depth camera's frame
            TFMessage transforms;
            if (record.color_resolution != K4A_COLOR_RESOLUTION_OFF)
            {
                transforms.transforms.push_back(pose_in_depth(m.calibration, K4A_CALIBRATION_TYPE_COLOR,
                                                              record.depth_frame_id, record.color_frame_id));
            }
            transforms.transforms.push_back(pose_in_depth(m.calibration, K4A_CALIBRATION_TYPE_ACCEL,
                                                          record.depth_frame_id, options.imu_frame_id));
            m.tf_static = std::make_unique<StaticTransformPublisher>(session, base, transforms);
        }
        // the gyroscope's axes turned into the accelerometer's, whose frame the samples are published in
        const k4a_calibration_extrinsics_t& g =
            m.calibration.extrinsics[K4A_CALIBRATION_TYPE_GYRO][K4A_CALIBRATION_TYPE_ACCEL];
        std::copy(std::begin(g.rotation), std::end(g.rotation), m.gyro_to_accel.begin());
        m.imu = std::make_unique<ImuPublisher>(session, base);
        m.diagnostics = std::make_unique<DiagnosticsPublisher>(session, base);
        m.started = std::chrono::steady_clock::now();
        m.reported = m.started;
        if (options.async_publish)
        {
            m.start_lanes();
        }
        const std::string key = base + "/" + kCalibration;
        const std::string answer = record.to_json();
        m.calibration_queryable = session.declare_queryable(
            zenoh::KeyExpr(key), [key, answer](const zenoh::Query& query) { query.reply(zenoh::KeyExpr(key), answer); },
            zenoh::closures::none);
    }

    Node::~Node() = default;

    void Node::publish(const k4a::capture& capture, bool live)
    {
        Impl& m = *impl_;
        m.rethrow_lane_error();
        m.count(&NodeStats::captures);
        if (const float t = capture.get_temperature_c(); not std::isnan(t))
        {
            m.temperature = t;
        }
        {
            const k4a::image color = capture.get_color_image(), depth = capture.get_depth_image();
            if (color and depth)
            {
                m.stamp_differences.push_back(
                    double((color.get_device_timestamp() - depth.get_device_timestamp()).count()));
                m.color_exposures.push_back(double(color.get_exposure().count()));
                m.depth_exposures.push_back(double(depth.get_exposure().count()));
            }
        }
        if (m.color)
        {
            m.publish_color(capture.get_color_image(), live);
        }
        if (m.depth)
        {
            m.publish_depth(capture.get_depth_image(), m.ir ? capture.get_ir_image() : k4a::image(), live);
        }
    }

    void Node::publish_imu(const std::vector<k4a_imu_sample_t>& samples)
    {
        Impl& m = *impl_;
        if (samples.empty() or not m.imu->subscribed() or not m.clock.offset())
        {
            return;  // nobody wants them, or no image has set the device clock yet
        }
        const std::array<float, 9>& r = m.gyro_to_accel;
        for (const k4a_imu_sample_t& s : samples)
        {
            const float* g = s.gyro_sample.v;
            Imu imu;
            imu.header = {Time::from_ns(m.clock.to_wall_ns(std::chrono::microseconds(s.acc_timestamp_usec))),
                          m.o.imu_frame_id};
            imu.orientation_covariance[0] = -1.0;  // no orientation
            imu.angular_velocity = {r[0] * g[0] + r[1] * g[1] + r[2] * g[2], r[3] * g[0] + r[4] * g[1] + r[5] * g[2],
                                    r[6] * g[0] + r[7] * g[1] + r[8] * g[2]};
            imu.linear_acceleration = {s.acc_sample.xyz.x, s.acc_sample.xyz.y, s.acc_sample.xyz.z};
            m.imu->publish(imu);
            ++m.stats.imu;
        }
    }

    DiagnosticStatus Node::report(const CaptureSource& source,
                                  const std::vector<std::pair<std::string, std::string>>& extra)
    {
        Impl& m = *impl_;
        const auto now = std::chrono::steady_clock::now();
        const double seconds = std::max(1e-3, std::chrono::duration<double>(now - m.reported).count());
        NodeStats a;
        {
            std::lock_guard<std::mutex> lock(m.stats_mutex);
            a = m.stats;
        }
        const NodeStats& b = m.reported_stats;
        const std::uint64_t dropped = source.dropped() - m.reported_dropped;
        const std::uint64_t restarts = source.restarts() - m.reported_restarts;
        const double rate = double(a.captures - b.captures) / seconds;
        const auto per = [](double ms, std::uint64_t n) { return n ? ms / double(n) : 0.0; };
        const auto number = [](double v) {
            char text[32];
            std::snprintf(text, sizeof(text), "%.2f", v);
            return std::string(text);
        };
        DiagnosticStatus status;
        status.name = "azure_kinect";
        status.hardware_id = m.record.serial;
        const std::uint64_t colors = (a.color_decoded + a.color_jpeg_only) - (b.color_decoded + b.color_jpeg_only);
        status.values = {{"captures/s", number(rate)},
                         {"fps", std::to_string(m.record.fps)},
                         {"dropped", std::to_string(dropped)},
                         {"restarts", std::to_string(restarts)},
                         {"colour/s", number(double(colors) / seconds)},
                         {"colour decoded/s", number(double(a.color_decoded - b.color_decoded) / seconds)},
                         {"colour ms", number(per(a.color_ms - b.color_ms, colors))},
                         {"color_rect/s", number(double(a.color_rect - b.color_rect) / seconds)},
                         {"color_rect ms", number(per(a.color_rect_ms - b.color_rect_ms, a.color_rect - b.color_rect))},
                         {"depth/s", number(double(a.depth - b.depth) / seconds)},
                         {"ir/s", number(double(a.ir - b.ir) / seconds)},
                         {"depth ms", number(per(a.depth_ms - b.depth_ms, a.depth - b.depth))},
                         {"depth_to_color/s", number(double(a.depth_to_color - b.depth_to_color) / seconds)},
                         {"depth_to_color ms",
                          number(per(a.depth_to_color_ms - b.depth_to_color_ms, a.depth_to_color - b.depth_to_color))},
                         {"depth_to_color_rect/s",
                          number(double(a.depth_to_color_rect - b.depth_to_color_rect) / seconds)},
                         {"depth_to_color_rect ms",
                          number(per(a.depth_to_color_rect_ms - b.depth_to_color_rect_ms,
                                     a.depth_to_color_rect - b.depth_to_color_rect))},
                         {"huffman tables added", std::to_string(a.huffman_added - b.huffman_added)},
                         {"skipped by a busy lane", std::to_string(a.skipped - b.skipped)},
                         {"imu/s", number(double(a.imu - b.imu) / seconds)}};
        if (m.temperature)
        {
            status.values.push_back({"temperature", number(*m.temperature)});
        }
        if (not m.stamp_differences.empty())
        {
            // does colour - depth follow the exposures? centres aligned (the SDK's default): 0; starts aligned:
            // (colour - depth exposure) / 2
            const auto median = [](std::vector<double> v) {
                std::nth_element(v.begin(), v.begin() + std::ptrdiff_t(v.size() / 2), v.end());
                return v[v.size() / 2];
            };
            const auto [lo, hi] = std::minmax_element(m.stamp_differences.begin(), m.stamp_differences.end());
            const double color_exposure = median(m.color_exposures);
            double depth_exposure = median(m.depth_exposures);
            // the SDK may give 0 for depth: then Microsoft's hardware specification's exposure of the mode, marked
            // "(spec)" (learn.microsoft.com/azure/kinect-dk/hardware-specification: 12.8 ms, WFOV unbinned 20.3 ms)
            std::string depth_source;
            if (depth_exposure <= 0)
            {
                depth_exposure = m.record.depth_mode == K4A_DEPTH_MODE_WFOV_UNBINNED ? 20300.0
                                 : m.record.depth_mode == K4A_DEPTH_MODE_PASSIVE_IR ? 0.0
                                                                                     : 12800.0;
                depth_source = " (spec)";
            }
            status.values.push_back({"colour - depth stamp ms", number(median(m.stamp_differences) / 1e3)});
            status.values.push_back({"colour - depth stamp min ms", number(*lo / 1e3)});
            status.values.push_back({"colour - depth stamp max ms", number(*hi / 1e3)});
            status.values.push_back({"colour exposure ms", number(color_exposure / 1e3)});
            status.values.push_back({"depth exposure ms", depth_exposure > 0 ? number(depth_exposure / 1e3) + depth_source
                                                                             : "unknown"});
            status.values.push_back({"predicted if start-aligned ms",
                                     depth_exposure > 0 ? number((color_exposure - depth_exposure) / 2e3) + depth_source
                                                        : "unknown"});
            m.stamp_differences.clear();
            m.color_exposures.clear();
            m.depth_exposures.clear();
        }
        {
            // the clock (clock.hpp): its estimator, offset, and the arrival delay's jitter as it sees it
            std::vector<double> residuals = m.clock.take_residuals_us();
            status.values.push_back({"clock estimator", clock_estimator_name(m.clock.options().estimator)});
            if (const auto offset = m.clock.offset())
            {
                char text[40];
                std::snprintf(text, sizeof(text), "%.6f", double(offset->count()) / 1e9);
                status.values.push_back({"clock offset s", text});
            }
            if (not residuals.empty())
            {
                std::sort(residuals.begin(), residuals.end());
                const auto at = [&](double q) { return residuals[std::size_t(q * double(residuals.size() - 1))]; };
                status.values.push_back({"arrival delay p50 ms", number(at(0.5) / 1e3)});
                status.values.push_back({"arrival delay p95 ms", number(at(0.95) / 1e3)});
                status.values.push_back({"arrival delay max ms", number(residuals.back() / 1e3)});
            }
            status.values.push_back({"clock snaps", std::to_string(m.clock.snaps())});
            status.values.push_back({"clock resets", std::to_string(m.clock.resets())});
        }
        for (const auto& [key, value] : source.info())
        {
            status.values.push_back({key, value});
        }
        for (const auto& [key, value] : extra)
        {
            status.values.push_back({key, value});
        }
        // a recording plays at its own rate (or as fast as it can): only a device is held to its frame rate
        if (a.captures == b.captures)
        {
            status.level = DiagnosticStatus::kError;
            status.message = "no captures";
        }
        else if (dropped or restarts or a.skipped != b.skipped or (source.live() and rate < 0.9 * m.record.fps))
        {
            status.level = DiagnosticStatus::kWarn;
            status.message = number(rate) + " captures/s, " + std::to_string(dropped) + " dropped, " +
                             std::to_string(restarts) + " restarts";
            if (a.skipped != b.skipped)
            {
                status.message += ", " + std::to_string(a.skipped - b.skipped) + " skipped by a busy lane";
            }
        }
        else
        {
            status.message = number(rate) + " captures/s";
        }
        DiagnosticArray diagnostics;
        diagnostics.header.stamp = Time::from_ns(wall_ns());
        diagnostics.status = {status};
        m.diagnostics->publish(diagnostics);
        m.reported = now;
        m.reported_stats = a;
        m.reported_dropped = source.dropped();
        m.reported_restarts = source.restarts();
        return status;
    }

    CalibrationRecord query_calibration(zenoh::Session& session, const std::string& base,
                                        std::chrono::milliseconds timeout)
    {
        // asked again until an answer comes: right after a session connects, the node's queryable may not be known
        // to it yet, and a query then ends at once without answers
        const std::string key = check_name(base) + "/" + kCalibration;
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        std::vector<std::string> records;
        while (records.empty() and std::chrono::steady_clock::now() < deadline)
        {
            zenoh::Session::GetOptions options;
            options.timeout_ms = 500;
            auto replies = session.get(zenoh::KeyExpr(key), "", zenoh::channels::FifoChannel(16), std::move(options));
            for (auto r = replies.recv(); std::holds_alternative<zenoh::Reply>(r); r = replies.recv())
            {
                const auto& reply = std::get<zenoh::Reply>(r);
                if (reply.is_ok())
                {
                    records.push_back(reply.get_ok().get_payload().as_string());
                }
            }
            if (records.empty())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
        if (records.size() != 1)
        {
            throw std::runtime_error(key + ": " + std::to_string(records.size()) +
                                     " answers (one Azure Kinect node expected) within " +
                                     std::to_string(timeout.count()) + " ms");
        }
        return CalibrationRecord::from_json(records[0]);
    }

    NodeStats Node::stats() const
    {
        std::lock_guard<std::mutex> lock(impl_->stats_mutex);
        return impl_->stats;
    }
    const DeviceClock& Node::clock() const { return impl_->clock; }

    std::string describe(const NodeStats& now, const NodeStats& before, std::uint64_t dropped, std::uint64_t restarts,
                         double seconds)
    {
        const auto per = [](double ms, std::uint64_t n) { return n ? ms / double(n) : 0.0; };
        const std::uint64_t colors = (now.color_decoded + now.color_jpeg_only) -
                                     (before.color_decoded + before.color_jpeg_only);
        char text[448];
        std::snprintf(text, sizeof(text),
                      "%.1f captures/s, dropped %llu, restarts %llu; colour %llu (decoded %llu) at %.2f ms, color_rect "
                      "%llu at %.2f ms, depth %llu ir %llu at %.2f ms, depth_to_color %llu at %.2f ms, "
                      "depth_to_color_rect %llu at %.2f ms; skipped by a busy lane %llu",
                      double(now.captures - before.captures) / seconds, static_cast<unsigned long long>(dropped),
                      static_cast<unsigned long long>(restarts), static_cast<unsigned long long>(colors),
                      static_cast<unsigned long long>(now.color_decoded - before.color_decoded),
                      per(now.color_ms - before.color_ms, colors),
                      static_cast<unsigned long long>(now.color_rect - before.color_rect),
                      per(now.color_rect_ms - before.color_rect_ms, now.color_rect - before.color_rect),
                      static_cast<unsigned long long>(now.depth - before.depth),
                      static_cast<unsigned long long>(now.ir - before.ir),
                      per(now.depth_ms - before.depth_ms, now.depth - before.depth),
                      static_cast<unsigned long long>(now.depth_to_color - before.depth_to_color),
                      per(now.depth_to_color_ms - before.depth_to_color_ms,
                          now.depth_to_color - before.depth_to_color),
                      static_cast<unsigned long long>(now.depth_to_color_rect - before.depth_to_color_rect),
                      per(now.depth_to_color_rect_ms - before.depth_to_color_rect_ms,
                          now.depth_to_color_rect - before.depth_to_color_rect),
                      static_cast<unsigned long long>(now.skipped - before.skipped));
        return text;
    }

    std::optional<int> cuda_device_from_name(const std::string& name)
    {
        if (name == "none")
        {
            return std::nullopt;
        }
        if (name.empty() or name.find_first_not_of("0123456789") != std::string::npos)
        {
            throw std::invalid_argument("CUDA device '" + name + "': a CUDA device ordinal or none");
        }
#ifdef DURCHBLICK_K4A_CUDA
        return std::stoi(name);
#else
        throw std::invalid_argument("CUDA device " + name + ": built without CUDA (DURCHBLICK_K4A_CUDA)");
#endif
    }
}  // namespace durchblick::k4a_node

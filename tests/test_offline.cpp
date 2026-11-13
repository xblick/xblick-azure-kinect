// Tests without a camera: the device clock, the calibration (from the SDK's own test calibration,
// tests/data/k4a_test_calibration.json, src/csharp/Tests/UnitTests/TestCalibration.json of Azure-Kinect-Sensor-SDK
// v1.4.2, MIT licence, its UTF-8 byte order mark removed as a device's raw calibration has none), the MJPEG handling,
// and the node end to end: captures built in memory (as the SDK builds them from a device) go out over Zenoh and
// come back through Durchblick's subscribers.
// Playback: a recording written by the SDK from such captures (no device) is read back.
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <random>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <csignal>
#include <sys/wait.h>
#include <unistd.h>

#include <jpeglib.h>
#include <nlohmann/json.hpp>

#ifdef DURCHBLICK_K4A_CUDA
#include <cuda_runtime.h>

#include "nvjpeg_decoder.hpp"
#endif

#include "calibration.hpp"
#include "clock.hpp"
#include "depth_to_color.hpp"
#include "durchblick/camera.hpp"
#include "durchblick/points.hpp"
#include "egl_glx.hpp"
#include "jpeg.hpp"
#include "node.hpp"
#include "rectify.hpp"
#include "source.hpp"

#include <k4arecord/record.h>

namespace
{
    using namespace durchblick;
    using namespace durchblick::k4a_node;
    using namespace std::chrono_literals;

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
    bool throws(F&& f)
    {
        try
        {
            f();
        }
        catch (const std::exception&)
        {
            return true;
        }
        return false;
    }

    std::string read_file(const char* path)
    {
        std::ifstream in(path, std::ios::binary);
        std::stringstream s;
        s << in.rdbuf();
        if (not in)
        {
            throw std::runtime_error(std::string("cannot read ") + path);
        }
        return s.str();
    }

    // the test calibration as a retail device's: Brown-Conrady (it is Rational 6KT, an early internal device's)
    // with its centre of distortion at the principal point (codx, cody: parameters 10 and 11, k4atypes.h's order)
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

    std::vector<std::uint8_t> encode_jpeg(const std::vector<std::uint8_t>& rgb, int width, int height)
    {
        jpeg_compress_struct cinfo{};
        jpeg_error_mgr err{};
        cinfo.err = jpeg_std_error(&err);
        jpeg_create_compress(&cinfo);
        unsigned char* out = nullptr;
        unsigned long size = 0;
        jpeg_mem_dest(&cinfo, &out, &size);
        cinfo.image_width = width;
        cinfo.image_height = height;
        cinfo.input_components = 3;
        cinfo.in_color_space = JCS_RGB;
        jpeg_set_defaults(&cinfo);
        jpeg_set_quality(&cinfo, 90, TRUE);
        jpeg_start_compress(&cinfo, TRUE);
        while (cinfo.next_scanline < cinfo.image_height)
        {
            JSAMPROW row = const_cast<std::uint8_t*>(rgb.data()) + std::size_t(cinfo.next_scanline) * width * 3;
            jpeg_write_scanlines(&cinfo, &row, 1);
        }
        jpeg_finish_compress(&cinfo);
        std::vector<std::uint8_t> jpeg(out, out + size);
        jpeg_destroy_compress(&cinfo);
        std::free(out);
        return jpeg;
    }

    // the JPEG without its DHT segments, as some UVC cameras send MJPEG
    std::vector<std::uint8_t> strip_huffman_tables(const std::vector<std::uint8_t>& jpeg)
    {
        std::vector<std::uint8_t> out(jpeg.begin(), jpeg.begin() + 2);
        std::size_t i = 2;
        while (i + 3 < jpeg.size() and jpeg[i + 1] != 0xDA)
        {
            const std::size_t length = (std::size_t(jpeg[i + 2]) << 8) | jpeg[i + 3];
            if (jpeg[i + 1] != 0xC4)
            {
                out.insert(out.end(), jpeg.begin() + i, jpeg.begin() + i + 2 + length);
            }
            i += 2 + length;
        }
        out.insert(out.end(), jpeg.begin() + i, jpeg.end());
        return out;
    }

    std::vector<std::uint8_t> pattern_rgb(int width, int height)
    {
        std::vector<std::uint8_t> rgb(std::size_t(width) * height * 3);
        for (int y = 0; y < height; ++y)
        {
            for (int x = 0; x < width; ++x)
            {
                std::uint8_t* p = &rgb[(std::size_t(y) * width + x) * 3];
                p[0] = std::uint8_t(x / 5);
                p[1] = std::uint8_t(y / 3);
                p[2] = std::uint8_t(128 + 100 * std::sin(x * 0.01));
            }
        }
        return rgb;
    }

    void test_clock()
    {
        DeviceClock clock;
        expect(throws([&] { clock.to_wall_ns(1s); }), "the clock refuses a stamp before its first image");
        // device 1 s, arrived at monotonic 101 s, wall = monotonic + 1000 s: offset 100 s + 1000 s
        clock.update(1s, 101s, 2000s, 1000s);
        expect(clock.offset() == std::chrono::nanoseconds(1100s), "first image sets the offset");
        expect(clock.to_wall_ns(2s) == std::chrono::nanoseconds(1102s).count(), "device time to wall time");
        clock.update(2s, 102s + 5ms, 2000s, 1000s);  // 5 ms more transfer: filtered, not taken
        expect(clock.offset() == std::chrono::nanoseconds(1100s + 500us), "a small difference moves it by alpha");
        clock.update(3s, 103s + 50ms, 2000s, 1000s);  // 50 ms: beyond the snap threshold
        expect(clock.offset() == std::chrono::nanoseconds(1100s + 50ms) and clock.snaps() == 2,
               "a large difference replaces it");
    }

    CalibrationRecord test_calibration()
    {
        const std::string raw = read_file(K4A_TEST_CALIBRATION);
        CalibrationRecord record{.raw = raw, .depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED,
                                 .color_resolution = K4A_COLOR_RESOLUTION_720P, .fps = 30, .serial = "000000000000",
                                 .depth_frame_id = "AzureKinectDepthCamera", .color_frame_id = "AzureKinectCamera"};
        const k4a_calibration_t six_kt = record.calibration();
        expect(six_kt.depth_camera_calibration.resolution_width == 640 and
                   six_kt.depth_camera_calibration.resolution_height == 576 and
                   six_kt.color_camera_calibration.resolution_width == 1280,
               "the calibration for NFOV unbinned and 720p");
        expect(throws([&] { camera_info(six_kt.color_camera_calibration, "c"); }),
               "Rational 6KT is refused (no ROS model)");

        record.raw = brown_conrady(raw);
        const k4a_calibration_t c = record.calibration();
        const auto& p = c.color_camera_calibration.intrinsics.parameters.param;
        const CameraInfo color = camera_info(c.color_camera_calibration, record.color_frame_id);
        expect(color.width == 1280 and color.height == 720 and color.distortion_model == "rational_polynomial" and
                   color.header.frame_id == "AzureKinectCamera",
               "colour CameraInfo: size, model, frame");
        expect(color.k[0] == double(p.fx) and color.k[2] == double(p.cx) and color.k[4] == double(p.fy) and
                   color.k[5] == double(p.cy) and color.p[0] == double(p.fx) and color.p[3] == 0.0,
               "colour CameraInfo: K and P");
        expect(color.d == std::vector<double>{p.k1, p.k2, p.p1, p.p2, p.k3, p.k4, p.k5, p.k6},
               "colour CameraInfo: D in OpenCV's order k1 k2 p1 p2 k3 k4 k5 k6");
        const CameraInfo depth = camera_info(c.depth_camera_calibration, record.depth_frame_id);
        expect(depth.width == 640 and depth.height == 576, "depth CameraInfo: NFOV unbinned size");

        const CalibrationRecord back = CalibrationRecord::from_json(record.to_json());
        const k4a_calibration_t again = back.calibration();
        expect(std::memcmp(&again, &c, sizeof(c)) == 0 and back.serial == record.serial and back.fps == 30,
               "the calibration record rebuilds the identical calibration");
        expect(throws([] { CalibrationRecord::from_json(R"({"v": 1})"); }), "another record version is refused");
        expect(throws([] { depth_mode_from_name("NFOV"); }) and throws([] { fps_from_value(25); }),
               "unknown modes are refused");
        return record;
    }

    // ROS calibration files instead of the device's intrinsics: K and D into the SDK's calibration, the files'
    // CameraInfo (with their own P) for the streams, carried by the record to other processes; files that do not fit
    // are refused. Returns the record with both files (test_depth_to_color runs on it as well).
    CalibrationRecord test_calibration_files(const CalibrationRecord& record)
    {
        const auto dir = std::filesystem::temp_directory_path() /
                         ("durchblick_k4a_calibration_" + std::to_string(::getpid()));
        std::filesystem::create_directories(dir);
        const auto file = [&](const std::string& name, const CameraInfo& info) {
            const std::string path = (dir / name).string();
            info.write_file(path, name);
            return path;
        };
        CameraInfo color = record.color_info();
        color.k[0] *= 1.02;
        color.k[4] *= 1.02;
        color.k[2] += 3.0;
        color.d[0] += 0.01;
        color.p = {0.9 * color.k[0], 0, color.k[2], 0, 0, 0.9 * color.k[4], color.k[5], 0, 0, 0, 1, 0};
        CameraInfo depth = record.depth_info();
        depth.k[0] *= 0.98;
        depth.k[2] -= 2.0;
        depth.distortion_model = "plumb_bob";
        depth.d.resize(5);
        const CalibrationRecord with =
            with_calibration_files(record, file("color.yaml", color), file("depth.yaml", depth));

        const k4a_calibration_t c = with.calibration();
        const auto& pc = c.color_camera_calibration.intrinsics.parameters.param;
        const auto& pd = c.depth_camera_calibration.intrinsics.parameters.param;
        expect(pc.fx == float(color.k[0]) and pc.cx == float(color.k[2]) and pc.k1 == float(color.d[0]) and
                   pc.k6 == float(color.d[7]) and pd.fx == float(depth.k[0]) and pd.cx == float(depth.k[2]) and
                   pd.k4 == 0.0f and pd.k6 == 0.0f,
               "calibration files: K and D in the SDK's calibration (plumb_bob: k4..k6 = 0)");
        const k4a_calibration_t device = record.calibration();
        expect(std::memcmp(&c.extrinsics, &device.extrinsics, sizeof(c.extrinsics)) == 0,
               "calibration files: the extrinsics stay the device's");
        CameraInfo color_info = with.color_info();
        expect(color_info.header.frame_id == "AzureKinectCamera" and color_info.p == color.p and
                   color_info.k == color.k and with.depth_info().distortion_model == "plumb_bob",
               "calibration files: the streams' CameraInfo is the files' (their own P), in the record's frames");
        const CalibrationRecord back = CalibrationRecord::from_json(with.to_json());
        const k4a_calibration_t back_calibration = back.calibration();
        expect(back.color_file == with.color_file and back.depth_file == with.depth_file and
                   std::memcmp(&back_calibration, &c, sizeof(c)) == 0,
               "calibration files: the record carries them to other processes");

        CameraInfo bigger = color;
        bigger.width = 1920;
        bigger.height = 1080;
        CameraInfo turned = color;
        turned.r = {0, -1, 0, 1, 0, 0, 0, 0, 1};
        CameraInfo fisheye = color;
        fisheye.distortion_model = "equidistant";
        fisheye.d.resize(4);
        CalibrationRecord no_color = record;
        no_color.color_resolution = K4A_COLOR_RESOLUTION_OFF;
        expect(throws([&] { with_calibration_files(record, file("bigger.yaml", bigger), ""); }) and
                   throws([&] { with_calibration_files(record, file("turned.yaml", turned), ""); }) and
                   throws([&] { with_calibration_files(record, file("fisheye.yaml", fisheye), ""); }) and
                   throws([&] { with_calibration_files(no_color, file("color.yaml", color), ""); }) and
                   throws([&] { with_calibration_files(record, (dir / "missing.yaml").string(), ""); }),
               "calibration files: another size, R, model, a camera that is off or no file are refused");
        std::filesystem::remove_all(dir);
        return with;
    }

    void test_jpeg()
    {
        const int w = 320, h = 240;
        const std::vector<std::uint8_t> rgb = pattern_rgb(w, h);
        const std::vector<std::uint8_t> jpeg = encode_jpeg(rgb, w, h);
        const std::vector<std::uint8_t> bare = strip_huffman_tables(jpeg);
        expect(has_huffman_tables(jpeg) and not has_huffman_tables(bare) and bare.size() < jpeg.size(),
               "a JPEG with and one without Huffman tables");
        expect(with_huffman_tables(jpeg) == jpeg, "a JPEG with tables stays unchanged");
        const std::vector<std::uint8_t> completed = with_huffman_tables(bare);
        expect(has_huffman_tables(completed), "the standard tables are added");
        CpuJpegDecoder decoder;
        const std::vector<std::uint8_t> a(decoder.decode(jpeg, w, h), decoder.decode(jpeg, w, h) + rgb.size());
        const std::vector<std::uint8_t> b(decoder.decode(completed, w, h),
                                          decoder.decode(completed, w, h) + rgb.size());
        // libjpeg writes the standard tables by default: the completed file decodes to exactly the same pixels
        expect(a == b, "the completed JPEG decodes like the original");
        double error = 0;
        for (std::size_t i = 0; i < rgb.size(); ++i)
        {
            error += std::abs(int(a[i]) - int(rgb[i]));
        }
        expect(error / double(rgb.size()) < 3.0, "the decoded frame is the pattern");
        expect(throws([&] { decoder.decode(jpeg, w + 1, h); }), "a frame of the wrong size is refused");
        expect(throws([] { with_huffman_tables(std::vector<std::uint8_t>{1, 2, 3, 4}); }), "no JPEG is refused");
    }

    std::uint16_t depth_value(int x, int y, int seq) { return std::uint16_t(500 + x + 3 * y + seq); }

    k4a::capture make_capture(const std::vector<std::uint8_t>& jpeg, int seq, std::chrono::microseconds device,
                              std::chrono::nanoseconds system)
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
                d[y * 640 + x] = depth_value(x, y, seq);
                i[y * 640 + x] = std::uint16_t(1000 + x * y % 777);
            }
        }
        // the frame as the camera's JPEG of its own size (the buffer stays the caller's)
        k4a::image color = k4a::image::create_from_buffer(K4A_IMAGE_FORMAT_COLOR_MJPG, 1280, 720, 0,
                                                          const_cast<std::uint8_t*>(jpeg.data()), jpeg.size(), nullptr,
                                                          nullptr);
        for (k4a::image* image : {&depth, &ir, &color})
        {
            image->set_timestamp(device);
            k4a_image_set_system_timestamp_nsec(image->handle(), static_cast<std::uint64_t>(system.count()));
        }
        capture.set_depth_image(depth);
        capture.set_ir_image(ir);
        capture.set_color_image(color);
        return capture;
    }

    void test_node(const CalibrationRecord& record)
    {
        std::random_device random;
        const int port = 20000 + int(random() % 20000);
        zenoh::Session node_session = open_session({.listen = {"127.0.0.1:" + std::to_string(port)}});
        zenoh::Session client = open_session({.connect = {"127.0.0.1:" + std::to_string(port)}});
        const std::string base = "t/k4a";
        Node node(node_session, NodeOptions{.base = base, .source = "test", .depth_to_color = false}, record);

        const CalibrationRecord queried = query_calibration(client, base);
        expect(queried.to_json() == record.to_json(), "<base>/k4a_calibration answers the record");

        // <base>/tf_static: the colour camera's pose in the depth camera's frame; it maps a point as the SDK does
        std::optional<TFMessage> tf;
        for (int attempt = 0; attempt < 50 and not tf; ++attempt)
        {
            zenoh::Session::GetOptions options;
            options.timeout_ms = 500;
            auto replies = client.get(zenoh::KeyExpr(base + "/tf_static"), "", zenoh::channels::FifoChannel(4),
                                      std::move(options));
            for (auto r = replies.recv(); std::holds_alternative<zenoh::Reply>(r); r = replies.recv())
            {
                const auto& reply = std::get<zenoh::Reply>(r);
                if (reply.is_ok())
                {
                    expect(reply.get_ok().get_encoding().as_string() == "application/cdr;tf2_msgs/msg/TFMessage",
                           "tf_static's encoding");
                    tf = TFMessage::deserialize(reply.get_ok().get_payload().as_vector());
                }
            }
        }
        expect(tf and tf->transforms.size() == 2, "<base>/tf_static answers the colour camera's and the IMU's poses");
        if (tf and tf->transforms.size() == 2)
        {
            const TransformStamped& imu_pose = tf->transforms[1];
            expect(imu_pose.header.frame_id == record.depth_frame_id and imu_pose.child_frame_id == "AzureKinectImu",
                   "depth camera -> IMU");
            const TransformStamped& t = tf->transforms[0];
            expect(t.header.frame_id == record.depth_frame_id and t.child_frame_id == record.color_frame_id,
                   "depth camera -> colour camera");
            const k4a_calibration_t calibration = record.calibration();
            const k4a_float3_t p_color{{120.0f, -80.0f, 1500.0f}};  // mm
            k4a_float3_t sdk{};
            k4a_calibration_3d_to_3d(&calibration, &p_color, K4A_CALIBRATION_TYPE_COLOR, K4A_CALIBRATION_TYPE_DEPTH,
                                     &sdk);
            const Quaternion& q = t.transform.rotation;
            const double c[3] = {p_color.xyz.x / 1000.0, p_color.xyz.y / 1000.0, p_color.xyz.z / 1000.0};
            // p_depth = q c q* + translation
            const double x = q.x, y = q.y, z = q.z, w = q.w;
            const double r[9] = {1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w),
                                 2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
                                 2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)};
            const double tr[3] = {t.transform.translation.x, t.transform.translation.y, t.transform.translation.z};
            double worst = 0.0;
            for (int i = 0; i < 3; ++i)
            {
                const double mm = 1000.0 * (r[3 * i] * c[0] + r[3 * i + 1] * c[1] + r[3 * i + 2] * c[2] + tr[i]);
                worst = std::max(worst, std::abs(mm - double(sdk.v[i])));
            }
            expect(worst < 0.01, "tf_static maps colour to depth as k4a_calibration_3d_to_3d (" +
                                     std::to_string(worst) + " mm)");
        }

        // the camera's JPEG (here without Huffman tables, as some UVC cameras send them)
        const std::vector<std::uint8_t> rgb = pattern_rgb(1280, 720);
        const std::vector<std::uint8_t> camera_jpeg = strip_huffman_tables(encode_jpeg(rgb, 1280, 720));
        const std::vector<std::uint8_t> sent_jpeg = with_huffman_tables(camera_jpeg);

        ImageSubscriber depth(client, base + "/depth", {.transport = Transport::Raw, .camera_info = true});
        ImageSubscriber ir(client, base + "/ir", {.transport = Transport::Shm});
        ImageSubscriber jpeg(client, base + "/color", {.transport = Transport::Compressed});

        // only the JPEG is wanted at first: no decoding
        std::optional<Frame> d, i, c;
        const auto t0 = std::chrono::steady_clock::now();
        for (int seq = 0; std::chrono::steady_clock::now() - t0 < 10s and not(d and i and c); ++seq)
        {
            node.publish(make_capture(camera_jpeg, seq % 50, std::chrono::microseconds(1000000 + seq * 33333),
                                      std::chrono::nanoseconds(5000000000LL + seq * 33333000LL)),
                         true);
            if (not d) d = depth.latest(true, 30ms);
            if (not i) i = ir.latest(true, 1ms);
            if (not c) c = jpeg.latest(true, 1ms);
        }
        expect(d and i and c, "depth, IR and colour arrive");
        if (not(d and i and c))
        {
            return;
        }
        expect(d->encoding == "16UC1" and d->width == 640 and d->height == 576 and
                   d->header.frame_id == "AzureKinectDepthCamera",
               "depth frame");
        const auto* dp = reinterpret_cast<const std::uint16_t*>(d->data());
        const int seq_d = int(dp[0]) - 500;
        bool exact = true;
        for (int y = 0; y < 576 and exact; y += 7)
        {
            for (int x = 0; x < 640; x += 5)
            {
                exact = exact and dp[y * 640 + x] == depth_value(x, y, seq_d);
            }
        }
        expect(exact, "depth arrives exactly");
        expect(i->encoding == "mono16" and i->width == 640, "IR frame (shared memory)");
        expect(node.stats().color_decoded == 0 and node.stats().color_jpeg_only > 0,
               "without pixel subscribers the colour is not decoded");
        expect(node.stats().huffman_added > 0, "the missing Huffman tables were added");
        const auto info = depth.camera_info();
        expect(info and info->distortion_model == "rational_polynomial" and info->width == 640,
               "depth CameraInfo arrives");
        // the stamp is the device time through the clock: offset = system - device (+ wall - monotonic)
        expect(node.clock().offset().has_value(), "the clock runs on the images");

        // a pixel subscriber: now the JPEG is decoded too, and the JPEG still goes out as the camera's
        ImageSubscriber pixels(client, base + "/color", {.transport = Transport::Raw});
        std::optional<Frame> p;
        const auto t1 = std::chrono::steady_clock::now();
        for (int seq = 0; std::chrono::steady_clock::now() - t1 < 10s and not p; ++seq)
        {
            node.publish(make_capture(camera_jpeg, seq % 50, std::chrono::microseconds(3000000 + seq * 33333),
                                      std::chrono::nanoseconds(7000000000LL + seq * 33333000LL)),
                         true);
            p = pixels.latest(true, 30ms);
        }
        expect(p.has_value() and node.stats().color_decoded > 0, "the colour is decoded for a pixel subscriber");
        if (p)
        {
            double error = 0;
            for (std::size_t k = 0; k < rgb.size(); k += 97)
            {
                error += std::abs(int(p->data()[k]) - int(rgb[k]));
            }
            expect(p->encoding == "rgb8" and p->width == 1280 and error / double(rgb.size() / 97) < 3.0,
                   "the decoded colour frame is the pattern");
        }

        // the compressed messages carry the camera's bytes (with the tables added), not a JPEG of the publisher
        std::vector<std::vector<std::uint8_t>> received;
        std::mutex mutex;
        auto raw_sub = client.declare_subscriber(
            zenoh::KeyExpr(base + "/color/image_raw/compressed"),
            [&](const zenoh::Sample& sample) {
                std::lock_guard lock(mutex);
                received.push_back(CompressedImage::deserialize(sample.get_payload().as_vector()).data);
            },
            zenoh::closures::none);
        const auto t2 = std::chrono::steady_clock::now();
        for (int seq = 0; std::chrono::steady_clock::now() - t2 < 5s; ++seq)
        {
            node.publish(make_capture(camera_jpeg, 0, std::chrono::microseconds(5000000 + seq * 33333),
                                      std::chrono::nanoseconds(9000000000LL + seq * 33333000LL)),
                         true);
            std::this_thread::sleep_for(10ms);
            std::lock_guard lock(mutex);
            if (not received.empty())
            {
                break;
            }
        }
        std::lock_guard lock(mutex);
        expect(not received.empty() and received.back() == sent_jpeg, "the camera's JPEG goes out byte for byte");
    }
    // publishes captures until `sub` has a frame (subscriptions take a moment to reach the publisher)
    std::optional<Frame> first_frame(Node& node, ImageSubscriber& sub, const std::vector<std::uint8_t>& jpeg)
    {
        const auto start = std::chrono::steady_clock::now();
        for (int seq = 0; std::chrono::steady_clock::now() - start < 10s; ++seq)
        {
            node.publish(make_capture(jpeg, 0, std::chrono::microseconds(1000000 + seq * 33333),
                                      std::chrono::nanoseconds(5000000000LL + seq * 33333000LL)),
                         true);
            if (auto frame = sub.latest(true, 30ms))
            {
                return frame;
            }
        }
        return std::nullopt;
    }

    // Depth seen from the colour camera: on the CPU (depth_to_color.hpp), or with sdk the SDK's transformation (needs
    // its depth engine plugin on LD_LIBRARY_PATH). The synthetic depth is a slanted plane 0.5 to 2.8 m away: most of
    // it is in the colour image.
    // <base>/depth_to_color_rect: remap_depth16 takes the nearest pixel of the rectify map, never a blend
    void test_depth_to_color_rect(const CalibrationRecord& record, zenoh::Session& node_session, zenoh::Session& client,
                                  const std::string& base)
    {
        const CameraInfo raw = record.color_info();
        const RectifyMap map = rectify_map(raw);
        std::vector<std::uint16_t> src(std::size_t(raw.width) * raw.height), dst(src.size());
        for (std::size_t i = 0; i < src.size(); ++i)
        {
            src[i] = std::uint16_t(1 + i % 60000);
        }
        remap_depth16(map, src.data(), dst.data());
        std::size_t outside = 0, wrong = 0;
        for (std::size_t i = 0; i < dst.size(); ++i)
        {
            if (not(map.x[i] >= 0.0f and map.y[i] >= 0.0f and map.x[i] <= float(raw.width - 1) and
                    map.y[i] <= float(raw.height - 1)))
            {
                ++outside;
                wrong += dst[i] != 0 ? 1 : 0;
                continue;
            }
            const std::size_t at = std::size_t(std::lround(map.y[i])) * raw.width + std::size_t(std::lround(map.x[i]));
            wrong += dst[i] != src[at] ? 1 : 0;
        }
        expect(wrong == 0, "remap_depth16 takes the nearest pixel, 0 outside");
        std::fprintf(stderr, "depth_to_color_rect: %zu of %zu pixels outside the camera's image\n", outside, dst.size());

        Node node(node_session, NodeOptions{.base = base, .source = "test", .depth_to_color = true}, record);
        ImageSubscriber sub(client, base + "/depth_to_color_rect", {.transport = Transport::Raw, .camera_info = true});
        const std::vector<std::uint8_t> jpeg = encode_jpeg(pattern_rgb(1280, 720), 1280, 720);
        const auto frame = first_frame(node, sub, jpeg);
        expect(frame.has_value(), "depth_to_color_rect arrives");
        if (not frame)
        {
            return;
        }
        expect(frame->encoding == "16UC1" and frame->width == raw.width and frame->height == raw.height and
                   frame->header.frame_id == raw.header.frame_id and node.stats().depth_to_color_rect > 0,
               "depth_to_color_rect has the colour camera's size and frame");
        std::optional<CameraInfo> info = sub.camera_info();
        for (int attempt = 0; attempt < 100 and not info; ++attempt)  // it may come after the frame
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            info = sub.camera_info();
        }
        CameraInfo expected = rectified_info(raw);
        expected.header = info ? info->header : expected.header;
        expect(info and *info == expected, "depth_to_color_rect carries color_rect's CameraInfo, without distortion");
        const auto* d = reinterpret_cast<const std::uint16_t*>(frame->data());
        std::size_t valid = 0;
        for (std::size_t k = 0; k < dst.size(); ++k)
        {
            valid += d[k] != 0 ? 1 : 0;
        }
        expect(valid > dst.size() / 4, "depth_to_color_rect covers the image");
    }

    void test_depth_to_color(const CalibrationRecord& record, zenoh::Session& node_session, zenoh::Session& client,
                             const std::string& base, bool sdk = false, bool async = false)
    {
        Node node(node_session,
                  NodeOptions{.base = base, .source = "test", .depth_to_color = true, .sdk_depth_to_color = sdk,
                              .async_publish = async},
                  record);
        ImageSubscriber sub(client, base + "/depth_to_color", {.transport = Transport::Raw, .camera_info = true});
        const std::vector<std::uint8_t> jpeg = encode_jpeg(pattern_rgb(1280, 720), 1280, 720);
        const auto frame = first_frame(node, sub, jpeg);
        expect(frame.has_value(), "depth_to_color arrives");
        if (not frame)
        {
            return;
        }
        expect(frame->encoding == "16UC1" and frame->width == 1280 and frame->height == 720 and
                   frame->header.frame_id == "AzureKinectCamera",
               "depth_to_color has the colour camera's size and frame");
        std::optional<CameraInfo> info = sub.camera_info();
        for (int attempt = 0; attempt < 100 and not info; ++attempt)  // it may come after the frame
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            info = sub.camera_info();
        }
        CameraInfo expected = record.color_info();
        if (info)
        {
            expected.header = info->header;
        }
        if (info != expected)
        {
            std::fprintf(stderr, "depth_to_color CameraInfo: %s, fx %g (expected %g), P[0] %g (expected %g)\n",
                         info ? "received" : "none", info ? info->k[0] : 0.0, expected.k[0], info ? info->p[0] : 0.0,
                         expected.p[0]);
        }
        expect(info == expected, "depth_to_color carries the colour camera's CameraInfo");
        const auto* d = reinterpret_cast<const std::uint16_t*>(frame->data());
        std::size_t valid = 0;
        std::uint16_t lo = 65535, hi = 0;
        for (std::size_t k = 0; k < std::size_t(1280) * 720; ++k)
        {
            if (d[k] != 0)
            {
                ++valid;
                lo = std::min(lo, d[k]);
                hi = std::max(hi, d[k]);
            }
        }
        std::fprintf(stderr, "depth_to_color: %.1f %% of the pixels have depth, %u to %u mm\n",
                     100.0 * double(valid) / (1280.0 * 720.0), unsigned(lo), unsigned(hi));
        expect(valid > std::size_t(1280) * 720 / 4 and lo >= 500 and hi <= 500 + 639 + 3 * 575 + 49,
               "depth_to_color covers the image with the plane's depths");

        // the depth engine's result against the SDK's CPU projection: depth pixels on a grid, moved into the colour
        // camera (their depth there is the z in the colour camera's frame) and projected into its image
        const k4a_calibration_t calibration = record.calibration();
        int checked = 0, agree = 0;
        double worst = 0;
        for (int y = 8; y < 576; y += 16)
        {
            for (int x = 8; x < 640; x += 16)
            {
                const float depth = depth_value(x, y, 0);
                k4a_float3_t p;
                const k4a_float2_t pixel{{float(x), float(y)}};
                k4a_float2_t uv;
                int ok_3d = 0, ok_2d = 0;
                k4a_calibration_2d_to_3d(&calibration, &pixel, depth,
                                         K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR, &p, &ok_3d);
                k4a_calibration_3d_to_2d(&calibration, &p, K4A_CALIBRATION_TYPE_COLOR, K4A_CALIBRATION_TYPE_COLOR,
                                         &uv, &ok_2d);
                const int u = int(std::lround(uv.xy.x)), v = int(std::lround(uv.xy.y));
                if (not ok_3d or not ok_2d or u < 1 or v < 1 or u >= 1279 or v >= 719)
                {
                    continue;
                }
                ++checked;
                const double error = std::abs(double(d[std::size_t(v) * 1280 + u]) - double(p.xyz.z));
                worst = std::max(worst, error);
                agree += error <= 5.0;
            }
        }
        std::fprintf(stderr, "depth_to_color against the CPU projection: %d of %d grid points within 5 mm, worst "
                     "%.1f mm\n", agree, checked, worst);
        expect(checked > 500 and agree >= checked * 98 / 100, "depth_to_color agrees with the CPU projection");
    }

    // The CPU's depth_to_color (depth_to_color.hpp) against the SDK's transformation, both from the same depth image:
    // the slanted plane, and the plane with a box 0.6 m in front of it (an occluding edge: the nearer depth must win).
    // Where both have a depth they agree within 5 mm (the CPU's footprints and the SDK's triangles differ only at
    // edges, by a pixel); each covers about as many pixels as the other. Also its time per frame.
    // Pixels of a depth image in the gap between a near and a far surface: its 5x5 neighbourhood spans more than 100 mm
    // and it is more than 30 mm from both ends (a point cloud shows them hovering at an object's edge).
    std::size_t between_surfaces(const std::uint16_t* image, int w, int h)
    {
        std::size_t n = 0;
        for (int y = 2; y < h - 2; ++y)
        {
            for (int x = 2; x < w - 2; ++x)
            {
                const int z = image[std::size_t(y) * w + x];
                if (z == 0)
                {
                    continue;
                }
                int lo = z, hi = z;
                for (int dy = -2; dy <= 2; ++dy)
                {
                    for (int dx = -2; dx <= 2; ++dx)
                    {
                        const int v = image[std::size_t(y + dy) * w + x + dx];
                        if (v != 0)
                        {
                            lo = std::min(lo, v);
                            hi = std::max(hi, v);
                        }
                    }
                }
                n += hi - lo > 100 and z - lo > 30 and hi - z > 30;
            }
        }
        return n;
    }

    // Flying pixels: the box with a ramp of two pixels at its left and right edges (depths between the box and the
    // plane, as the depth engine gives them). The CPU's depth_to_color leaves them out: no more pixels in the gap than the SDK's,
    // and fewer than when it keeps them.
    void test_flying_pixels(const CalibrationRecord& record)
    {
        const k4a_calibration_t calibration = record.calibration();
        const CameraInfo color = record.color_info();
        const RectifyMap map = rectify_map(color);
        DepthToColor drop(calibration, color, map, true, true, false), keep(calibration, color, map, true, false, false);
        k4a_transformation_t handle = k4a_transformation_create(&calibration);
        expect(handle != nullptr, "the SDK's transformation for the flying pixels");
        if (handle == nullptr)
        {
            return;
        }
        k4a::transformation sdk(handle);
        const int w = int(color.width), h = int(color.height);
        k4a::image depth = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16, 640, 576, 640 * 2);
        auto* d = reinterpret_cast<std::uint16_t*>(depth.get_buffer());
        for (int y = 0; y < 576; ++y)
        {
            for (int x = 0; x < 640; ++x)
            {
                const bool rows = y >= 180 and y < 380;
                // a ramp at both edges: the colour camera, 3 cm beside the depth camera, sees the box shifted
                // over one of them (parallax), the other one in the open
                const int offset = not rows                ? 0
                                   : x >= 220 and x < 420 ? 600
                                   : x == 219 or x == 420 ? 400
                                   : x == 218 or x == 421 ? 200
                                                          : 0;
                d[y * 640 + x] = std::uint16_t(depth_value(x, y, 0) - offset);
            }
        }
        k4a::image reference = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16, w, h, w * 2);
        sdk.depth_image_to_color_camera(depth, &reference);
        std::vector<std::uint16_t> rect(std::size_t(w) * h), dropped(rect.size()), kept(rect.size());
        drop.rectified(d, 640 * 2, rect.data());
        drop.unrectify(dropped.data());
        keep.rectified(d, 640 * 2, rect.data());
        keep.unrectify(kept.data());
        const std::size_t in_sdk =
            between_surfaces(reinterpret_cast<const std::uint16_t*>(reference.get_buffer()), w, h);
        const std::size_t in_dropped = between_surfaces(dropped.data(), w, h);
        const std::size_t in_kept = between_surfaces(kept.data(), w, h);
        std::fprintf(stderr,
                     "flying pixels: %zu pixels in the gap with the SDK, %zu on the CPU leaving them out (%zu depth "
                     "pixels left out), %zu keeping them\n",
                     in_sdk, in_dropped, drop.flying(), in_kept);
        expect(drop.flying() > 0 and in_dropped <= in_sdk and in_dropped < in_kept,
               "depth_to_color on the CPU leaves the flying pixels out");
    }

    // Sparse pixels: the plane with scattered single pixels at 9 m (as the ambiguous depths beyond the camera's range
    // come). The SDK leaves them out (no complete 2x2 square), the CPU's depth_to_color too; with keep sparse not.
    void test_sparse_pixels(const CalibrationRecord& record)
    {
        const k4a_calibration_t calibration = record.calibration();
        const CameraInfo color = record.color_info();
        const RectifyMap map = rectify_map(color);
        DepthToColor drop(calibration, color, map), keep(calibration, color, map, true, true, false);
        k4a_transformation_t handle = k4a_transformation_create(&calibration);
        expect(handle != nullptr, "the SDK's transformation for the sparse pixels");
        if (handle == nullptr)
        {
            return;
        }
        k4a::transformation sdk(handle);
        const int w = int(color.width), h = int(color.height);
        k4a::image depth = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16, 640, 576, 640 * 2);
        auto* d = reinterpret_cast<std::uint16_t*>(depth.get_buffer());
        for (int y = 0; y < 576; ++y)
        {
            for (int x = 0; x < 640; ++x)
            {
                const bool scattered = x % 7 == 3 and y % 7 == 3;
                d[y * 640 + x] = scattered ? std::uint16_t(9000) : depth_value(x, y, 0);
            }
        }
        k4a::image reference = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16, w, h, w * 2);
        sdk.depth_image_to_color_camera(depth, &reference);
        std::vector<std::uint16_t> rect(std::size_t(w) * h), dropped(rect.size()), kept(rect.size());
        drop.rectified(d, 640 * 2, rect.data());
        drop.unrectify(dropped.data());
        keep.rectified(d, 640 * 2, rect.data());
        keep.unrectify(kept.data());
        const auto far = [&](const std::uint16_t* image) {
            std::size_t n = 0;
            for (std::size_t k = 0; k < std::size_t(w) * h; ++k)
            {
                n += image[k] > 5000;
            }
            return n;
        };
        const std::size_t in_sdk = far(reinterpret_cast<const std::uint16_t*>(reference.get_buffer()));
        const std::size_t in_dropped = far(dropped.data()), in_kept = far(kept.data());
        std::fprintf(stderr,
                     "sparse pixels: %zu pixels beyond 5 m with the SDK, %zu on the CPU leaving them out (%zu depth "
                     "pixels left out), %zu keeping them\n",
                     in_sdk, in_dropped, drop.sparse(), in_kept);
        expect(drop.sparse() > 0 and in_dropped <= in_sdk and in_dropped < in_kept,
               "depth_to_color on the CPU leaves the sparse pixels out, as the SDK");
    }

    void test_cpu_depth_to_color(const CalibrationRecord& record, bool interpolate = true)
    {
        const k4a_calibration_t calibration = record.calibration();
        const CameraInfo color = record.color_info();
        const RectifyMap map = rectify_map(color);
        DepthToColor cpu(calibration, color, map, interpolate);
        k4a_transformation_t handle = k4a_transformation_create(&calibration);
        expect(handle != nullptr, "the SDK's transformation for the comparison (depth engine on LD_LIBRARY_PATH)");
        if (handle == nullptr)
        {
            return;
        }
        k4a::transformation sdk(handle);
        const int w = int(color.width), h = int(color.height);
        for (const bool box : {false, true})
        {
            k4a::image depth = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16, 640, 576, 640 * 2);
            auto* d = reinterpret_cast<std::uint16_t*>(depth.get_buffer());
            for (int y = 0; y < 576; ++y)
            {
                for (int x = 0; x < 640; ++x)
                {
                    const bool in_box = box and x >= 220 and x < 420 and y >= 180 and y < 380;
                    d[y * 640 + x] = std::uint16_t(depth_value(x, y, 0) - (in_box ? 600 : 0));
                }
            }
            k4a::image reference = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16, w, h, w * 2);
            sdk.depth_image_to_color_camera(depth, &reference);
            const auto* r = reinterpret_cast<const std::uint16_t*>(reference.get_buffer());
            std::vector<std::uint16_t> rect(std::size_t(w) * h), raw(rect.size());
            const auto start = std::chrono::steady_clock::now();
            constexpr int kRuns = 20;
            for (int run = 0; run < kRuns; ++run)
            {
                cpu.rectified(d, 640 * 2, rect.data());
                cpu.unrectify(raw.data());
            }
            const double ms =
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count() / kRuns;
            std::size_t both = 0, agree = 0, only_sdk = 0, only_cpu = 0;
            double worst = 0;
            for (std::size_t k = 0; k < raw.size(); ++k)
            {
                if (r[k] and raw[k])
                {
                    ++both;
                    const double e = std::abs(double(r[k]) - double(raw[k]));
                    agree += e <= 5.0;
                    worst = std::max(worst, e);
                }
                else
                {
                    only_sdk += r[k] and not raw[k];
                    only_cpu += raw[k] and not r[k];
                }
            }
            const std::string what = std::string(box ? "depth_to_color on the CPU, box before the plane"
                                                     : "depth_to_color on the CPU") +
                                     (interpolate ? "" : " (no interpolation)");
            std::fprintf(stderr,
                         "%s: %zu pixels with both depths, %.2f %% within 5 mm (worst %.0f mm); only the SDK's %zu, "
                         "only the CPU's %zu; %.2f ms a frame (rectified and raw)\n",
                         what.c_str(), both, 100.0 * double(agree) / double(std::max<std::size_t>(both, 1)), worst,
                         only_sdk, only_cpu, ms);
            expect(both > raw.size() / 4 and agree >= both * 99 / 100, what + " agrees with the SDK's transformation");
            expect(only_sdk + only_cpu <= both / 50, what + " covers the pixels the SDK's covers");
        }
    }

#ifdef DURCHBLICK_K4A_CUDA
    // The colour decoded on the GPU (nvJPEG): the same picture as the CPU's decoding (rounding apart), in the decoder
    // and through the node's CUDA ring to another process (tests/cuda_reader.cpp).
    void compare(const std::uint8_t* got, const std::uint8_t* reference, std::size_t n, const std::string& what)
    {
        int worst = 0;
        double mean = 0;
        for (std::size_t k = 0; k < n; ++k)
        {
            const int e = std::abs(int(got[k]) - int(reference[k]));
            worst = std::max(worst, e);
            mean += e;
        }
        std::fprintf(stderr, "%s against libjpeg: mean %.3f, largest %d\n", what.c_str(), mean / double(n), worst);
        expect(mean / double(n) < 1.0 and worst <= 8, what + " decodes like libjpeg (rounding only)");
    }

    // <base>/color_rect against the SDK's own projection: a rectified pixel (u, v) shows the ray of P's camera through
    // it, which k4a_calibration_3d_to_2d puts at a point of the colour image; the rectified pixel is the colour
    // image there (bilinear), black where that point is outside. On the CPU, or on the GPU (NPP) with cuda_device.
    void test_color_rect(const CalibrationRecord& record, zenoh::Session& node_session, zenoh::Session& client,
                         const std::string& base, std::optional<int> cuda_device)
    {
        Node node(node_session,
                  NodeOptions{.base = base, .source = "test", .cuda_device = cuda_device, .depth_to_color = false},
                  record);
        ImageSubscriber rect(client, base + "/color_rect", {.transport = Transport::Raw, .camera_info = true});
        ImageSubscriber color(client, base + "/color", {.transport = Transport::Raw});
        const std::vector<std::uint8_t> jpeg = encode_jpeg(pattern_rgb(1280, 720), 1280, 720);
        std::optional<Frame> r, c;
        const auto start = std::chrono::steady_clock::now();
        for (int seq = 0; std::chrono::steady_clock::now() - start < 10s and not(r and c); ++seq)
        {
            node.publish(make_capture(jpeg, 0, std::chrono::microseconds(1000000 + seq * 33333),
                                      std::chrono::nanoseconds(5000000000LL + seq * 33333000LL)),
                         true);
            std::this_thread::sleep_for(30ms);
            r = r ? r : rect.latest(true, 0ms);
            c = c ? c : color.latest(true, 0ms);
        }
        const std::string what = "color_rect (" + base + "): ";
        expect(r and c and r->encoding == "rgb8" and r->width == 1280 and r->height == 720 and
                   r->header.frame_id == "AzureKinectCamera" and node.stats().color_rect > 0,
               (what + "arrives").c_str());
        if (not(r and c))
        {
            return;
        }
        const CameraInfo raw = record.color_info();
        std::optional<CameraInfo> info = rect.camera_info();
        for (int attempt = 0; attempt < 100 and not info; ++attempt)
        {
            std::this_thread::sleep_for(20ms);
            info = rect.camera_info();
        }
        expect(info and info->p == raw.p and info->k == std::array<double, 9>{raw.p[0], 0, raw.p[2], 0, raw.p[5],
                                                                              raw.p[6], 0, 0, 1} and
                   info->d == std::vector<double>(5, 0.0) and info->distortion_model == "plumb_bob",
               (what + "its CameraInfo: K from P, no distortion").c_str());

        const k4a_calibration_t calibration = record.calibration();
        const std::uint8_t* src = c->data();
        const std::uint8_t* out = r->data();
        int checked = 0, agree = 0, outside = 0, black = 0;
        int worst = 0;
        for (int v = 2; v < 720; v += 6)
        {
            for (int u = 2; u < 1280; u += 6)
            {
                const k4a_float3_t ray{{float(1000.0 * (u - raw.p[2]) / raw.p[0]),
                                        float(1000.0 * (v - raw.p[6]) / raw.p[5]), 1000.0f}};
                k4a_float2_t uv;
                int valid = 0;
                k4a_calibration_3d_to_2d(&calibration, &ray, K4A_CALIBRATION_TYPE_COLOR, K4A_CALIBRATION_TYPE_COLOR,
                                         &uv, &valid);
                const float x = uv.xy.x, y = uv.xy.y;
                const std::uint8_t* got = out + 3 * (std::size_t(v) * 1280 + u);
                if (valid and (x < -1.5f or y < -1.5f or x > 1280.5f or y > 720.5f))
                {
                    ++outside;
                    black += got[0] == 0 and got[1] == 0 and got[2] == 0;
                    continue;
                }
                if (not valid or x < 1.0f or y < 1.0f or x > 1277.0f or y > 717.0f)
                {
                    continue;  // at the border the two sides may differ by the interpolation's edge rule
                }
                const int x0 = int(x), y0 = int(y);
                const float fx = x - float(x0), fy = y - float(y0);
                int error = 0;
                for (int ch = 0; ch < 3; ++ch)
                {
                    const auto at = [&](int xx, int yy) { return float(src[3 * (std::size_t(yy) * 1280 + xx) + ch]); };
                    const float top = at(x0, y0) + fx * (at(x0 + 1, y0) - at(x0, y0));
                    const float bottom = at(x0, y0 + 1) + fx * (at(x0 + 1, y0 + 1) - at(x0, y0 + 1));
                    error = std::max(error, int(std::lround(std::abs(top + fy * (bottom - top) - float(got[ch])))));
                }
                ++checked;
                agree += error <= 2;
                worst = std::max(worst, error);
            }
        }
        // the cost per frame once the rings exist: 60 more frames, read by the subscriber
        const NodeStats before = node.stats();
        for (int seq = 0; seq < 60; ++seq)
        {
            node.publish(make_capture(jpeg, 0, std::chrono::microseconds(9000000 + seq * 33333),
                                      std::chrono::nanoseconds(9000000000LL + seq * 33333000LL)),
                         true);
            rect.latest(true, 30ms);
        }
        const NodeStats after = node.stats();
        std::fprintf(stderr,
                     "%s%d of %d grid pixels within 2 of the SDK's point (worst %d), %d of %d outside black; "
                     "%.2f ms per frame to rectify and publish (%llu frames), %.2f ms for the whole colour\n",
                     what.c_str(), agree, checked, worst, black, outside,
                     (after.color_rect_ms - before.color_rect_ms) / double(after.color_rect - before.color_rect),
                     static_cast<unsigned long long>(after.color_rect - before.color_rect),
                     (after.color_ms - before.color_ms) / double(after.color_decoded - before.color_decoded));
        expect(checked > 15000 and agree >= checked * 995 / 1000,
               (what + "the colour image at the SDK's point of every rectified pixel").c_str());
        expect(black == outside, (what + "black where the point is outside the colour image").c_str());
    }

    void test_cuda_color(const CalibrationRecord& record, zenoh::Session& node_session, int port)
    {
        const std::vector<std::uint8_t> rgb = pattern_rgb(1280, 720);
        const std::vector<std::uint8_t> jpeg = with_huffman_tables(strip_huffman_tables(encode_jpeg(rgb, 1280, 720)));
        CpuJpegDecoder cpu;
        const std::uint8_t* decoded = cpu.decode(jpeg, 1280, 720);
        const std::vector<std::uint8_t> reference(decoded, decoded + rgb.size());

        NvJpegDecoder gpu(0);
        const std::uint64_t ptr = gpu.decode(jpeg, 1280, 720);
        gpu.synchronize();
        std::vector<std::uint8_t> host(rgb.size());
        cudaMemcpy(host.data(), reinterpret_cast<void*>(ptr), host.size(), cudaMemcpyDeviceToHost);
        compare(host.data(), reference.data(), host.size(), "nvJPEG");

        Node node(node_session,
                  NodeOptions{.base = "t/k4a_cuda", .source = "test", .cuda_device = 0, .depth_to_color = false},
                  record);
        const std::string out = std::string(K4A_TEST_TMP) + "/cuda_frame.bin";
        std::remove(out.c_str());
        const std::string command = std::string(K4A_CUDA_READER) + " 127.0.0.1:" + std::to_string(port) +
                                    " t/k4a_cuda/color " + out + " &";
        expect(std::system(command.c_str()) == 0, "the reader starts");
        std::vector<std::uint8_t> received;
        const auto start = std::chrono::steady_clock::now();
        for (int seq = 0; std::chrono::steady_clock::now() - start < 20s; ++seq)
        {
            node.publish(make_capture(strip_huffman_tables(jpeg), seq % 50,
                                      std::chrono::microseconds(1000000 + seq * 33333),
                                      std::chrono::nanoseconds(5000000000LL + seq * 33333000LL)),
                         true);
            std::this_thread::sleep_for(30ms);
            std::ifstream in(out, std::ios::binary | std::ios::ate);
            if (in and in.tellg() == std::streamoff(rgb.size()))
            {
                in.seekg(0);
                received.resize(rgb.size());
                in.read(reinterpret_cast<char*>(received.data()), std::streamsize(received.size()));
                break;
            }
        }
        expect(received.size() == rgb.size(), "a colour frame arrives over the CUDA ring in another process");
        if (received.size() == rgb.size())
        {
            compare(received.data(), reference.data(), received.size(), "nvJPEG through the CUDA ring");
            expect(node.stats().color_decoded > 0, "the node decoded for the CUDA subscriber");
        }
    }
#endif

#ifdef K4A_POINTS_PROGRAM
    // Durchblick's point cloud node (durchblick-points, a process of its own) on this node's depth and colour, against
    // the SDK: every point as k4a_calibration_2d_to_3d puts it, its colour from the colour pixel
    // k4a_calibration_3d_to_2d projects it to. The test calibration's lens model is the SDK's own (Brown-Conrady),
    // the node's the rational_polynomial of the published camera_info: the same function, so this checks the
    // camera_info, the frames and the node's arithmetic together.
    void test_point_cloud(const CalibrationRecord& record, zenoh::Session& node_session, int port)
    {
        std::optional<int> cuda_device;
#ifdef DURCHBLICK_K4A_CUDA
        cuda_device = 0;  // the node takes depth and colour from the CUDA rings, as on the robot
#endif
        Node node(node_session, NodeOptions{.base = "t/k4a_pc", .source = "test", .cuda_device = cuda_device,
                                            .depth_to_color = false},
                  record);
        const pid_t child = ::fork();
        if (child == 0)
        {
            const std::string connect = "127.0.0.1:" + std::to_string(port);
            ::execl(K4A_POINTS_PROGRAM, K4A_POINTS_PROGRAM, "--connect", connect.c_str(), "--depth", "t/k4a_pc/depth",
                    "--color", "t/k4a_pc/color", "--cuda-device", "0", static_cast<char*>(nullptr));
            std::_Exit(127);
        }
        // read in the session the program connects to: Zenoh peers do not forward for each other
        PointCloudSubscriber clouds(node_session, "t/k4a_pc/points", {.transport = Transport::Shm});
        ImageSubscriber colors(node_session, "t/k4a_pc/color", {.transport = Transport::Raw});
        const std::vector<std::uint8_t> jpeg = encode_jpeg(pattern_rgb(1280, 720), 1280, 720);
        std::optional<Cloud> cloud;
        std::optional<Frame> color;
        const auto start = std::chrono::steady_clock::now();
        for (int seq = 0; std::chrono::steady_clock::now() - start < 30s and not(cloud and color); ++seq)
        {
            node.publish(make_capture(jpeg, seq % 50, std::chrono::microseconds(1000000 + seq * 33333),
                                      std::chrono::nanoseconds(5000000000LL + seq * 33333000LL)),
                         true);
            if (not cloud) cloud = clouds.latest(true, 30ms);
            if (not color) color = colors.latest(true, 1ms);
        }
        ::kill(child, SIGTERM);
        int status = 0;
        ::waitpid(child, &status, 0);
        expect(cloud and color, "a point cloud of the node's frames arrives");
        if (not(cloud and color))
        {
            return;
        }
        expect(cloud->header.frame_id == record.depth_frame_id and cloud->width == 640 and cloud->height == 576 and
                   cloud->point_step == 16 and cloud->field("rgb") != nullptr,
               "the cloud is organized in the depth camera's frame, with colour");
        const auto* points = reinterpret_cast<const float*>(cloud->data());
        const int seq = int(std::lround(points[2] * 1000.0f)) - 500;  // z of pixel (0, 0): depth_value(0, 0, seq)
        const k4a_calibration_t calibration = record.calibration();
        const auto* rgb = color->data();
        std::size_t sdk_valid = 0, missing = 0, extra = 0, coloured = 0, colour_agrees = 0;
        double worst = 0.0;
        for (int y = 0; y < 576; ++y)
        {
            for (int x = 0; x < 640; ++x)
            {
                const float* p = points + 4 * (static_cast<std::size_t>(y) * 640 + x);
                const k4a_float2_t pixel{{float(x), float(y)}};
                k4a_float3_t sdk{};
                int valid = 0;
                k4a_calibration_2d_to_3d(&calibration, &pixel, float(depth_value(x, y, seq)),
                                         K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_DEPTH, &sdk, &valid);
                const bool ours = std::isfinite(p[0]);
                if (not valid)
                {
                    extra += ours ? 1 : 0;  // the SDK refuses pixels outside the lens's calibrated radius
                    continue;
                }
                ++sdk_valid;
                if (not ours)
                {
                    ++missing;
                    continue;
                }
                for (int i = 0; i < 3; ++i)
                {
                    worst = std::max(worst, std::abs(1000.0 * p[i] - double(sdk.v[i])));
                }
                k4a_float2_t uv{};
                int in_color = 0;
                k4a_calibration_3d_to_2d(&calibration, &sdk, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR,
                                         &uv, &in_color);
                const int u = int(std::lround(uv.xy.x)), v = int(std::lround(uv.xy.y));
                if (in_color and u >= 0 and v >= 0 and u < 1280 and v < 720)
                {
                    ++coloured;
                    std::uint32_t packed;
                    std::memcpy(&packed, &p[3], 4);
                    const std::uint8_t* c = rgb + 3 * (static_cast<std::size_t>(v) * 1280 + u);
                    colour_agrees += packed == ((std::uint32_t(c[0]) << 16) | (std::uint32_t(c[1]) << 8) | c[2]);
                }
            }
        }
        std::fprintf(stderr,
                     "point cloud: %zu points the SDK has, %zu of them missing, %zu more than the SDK's; worst %.4f "
                     "mm; colour as the SDK projects it for %zu of %zu\n",
                     sdk_valid, missing, extra, worst, colour_agrees, coloured);
        expect(sdk_valid > 300000 and missing == 0, "a point wherever the SDK has one");
        expect(worst < 0.1, "the points are where the SDK puts them (" + std::to_string(worst) + " mm)");
        expect(coloured > 100000 and double(colour_agrees) > 0.99 * double(coloured),
               "the colour of the pixel the SDK projects a point to");
    }
#endif

    // A recording as k4arecorder writes one (MJPEG colour, depth, IR, the device's calibration as attachment), written
    // by the SDK from the captures above without a device, and read back through open_playback. Returns its path.
    constexpr int kImuPerCapture = 16;

    k4a_imu_sample_t imu_sample(int seq, int k)
    {
        k4a_imu_sample_t s{};
        s.temperature = 30.0f;
        s.acc_timestamp_usec = 100000 + seq * 33333 + 1 + k * 2000;
        s.gyro_timestamp_usec = s.acc_timestamp_usec;
        s.acc_sample.xyz = {0.1f * float(k), -9.81f, 0.01f * float(seq)};
        s.gyro_sample.xyz = {0.5f, -0.25f, 0.125f * float(k)};
        return s;
    }

    std::string test_playback(const CalibrationRecord& record)
    {
        const std::string path = std::string(K4A_TEST_TMP) + "/test_recording.mkv";
        k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
        config.color_format = K4A_IMAGE_FORMAT_COLOR_MJPG;
        config.color_resolution = record.color_resolution;
        config.depth_mode = record.depth_mode;
        config.camera_fps = K4A_FRAMES_PER_SECOND_30;
        k4a_record_t recording = nullptr;
        if (k4a_record_create(path.c_str(), nullptr, config, &recording) != K4A_RESULT_SUCCEEDED)
        {
            throw std::runtime_error("cannot create " + path);
        }
        // what k4arecorder stores from the device (the SDK's tag and attachment names)
        if (k4a_record_add_tag(recording, "K4A_DEVICE_SERIAL_NUMBER", record.serial.c_str()) != K4A_RESULT_SUCCEEDED or
            k4a_record_add_tag(recording, "K4A_CALIBRATION_FILE", "calibration.json") != K4A_RESULT_SUCCEEDED or
            k4a_record_add_attachment(recording, "calibration.json",
                                      reinterpret_cast<const std::uint8_t*>(record.raw.data()),
                                      record.raw.size()) != K4A_RESULT_SUCCEEDED or
            k4a_record_add_imu_track(recording) != K4A_RESULT_SUCCEEDED or
            k4a_record_write_header(recording) != K4A_RESULT_SUCCEEDED)
        {
            throw std::runtime_error("cannot write the header of " + path);
        }
        const std::vector<std::uint8_t> jpeg = encode_jpeg(pattern_rgb(1280, 720), 1280, 720);
        constexpr int kCaptures = 10;
        for (int seq = 0; seq < kCaptures; ++seq)
        {
            const k4a::capture capture = make_capture(jpeg, seq, std::chrono::microseconds(100000 + seq * 33333), 0ns);
            if (k4a_record_write_capture(recording, capture.handle()) != K4A_RESULT_SUCCEEDED)
            {
                throw std::runtime_error("cannot write a capture to " + path);
            }
            // the IMU between this capture and the next: kImuPerCapture samples
            for (int k = 0; k < kImuPerCapture; ++k)
            {
                if (k4a_record_write_imu_sample(recording, imu_sample(seq, k)) != K4A_RESULT_SUCCEEDED)
                {
                    throw std::runtime_error("cannot write an IMU sample to " + path);
                }
            }
        }
        k4a_record_flush(recording);
        k4a_record_close(recording);

        std::unique_ptr<CaptureSource> source =
            open_playback({.path = path, .loop = false, .realtime = false, .depth_frame_id = record.depth_frame_id,
                           .color_frame_id = record.color_frame_id});
        const CalibrationRecord& r = source->record();
        expect(r.raw == record.raw and r.depth_mode == record.depth_mode and
                   r.color_resolution == record.color_resolution and r.fps == 30 and r.serial == record.serial and
                   r.depth_frame_id == record.depth_frame_id,
               "playback: the recording's calibration, modes and serial");
        int read = 0;
        bool same = true;
        while (true)
        {
            std::optional<k4a::capture> capture;
            try
            {
                capture = source->next(0ms);
            }
            catch (const std::runtime_error&)
            {
                break;  // the end of the recording
            }
            const k4a::image depth = capture->get_depth_image();
            const k4a::image color = capture->get_color_image();
            same = same and depth and color and color.get_size() == jpeg.size() and
                   reinterpret_cast<const std::uint16_t*>(depth.get_buffer())[3 * 640 + 2] == depth_value(2, 3, read);
            ++read;
        }
        expect(read == kCaptures and same, "playback: every capture, depth and JPEG as written, then the end");
        std::unique_ptr<CaptureSource> looping =
            open_playback({.path = path, .loop = true, .realtime = false, .depth_frame_id = record.depth_frame_id,
                           .color_frame_id = record.color_frame_id});
        int looped = 0;
        for (; looped < 2 * kCaptures + 1; ++looped)
        {
            looping->next(0ms);
        }
        expect(looped == 2 * kCaptures + 1, "playback: --loop starts again at the end");

        // the node stamps a looping recording so that its stamps go on increasing over the loop
        std::random_device random;
        const int port = 20000 + int(random() % 20000);
        zenoh::Session node_session = open_session({.listen = {"127.0.0.1:" + std::to_string(port)}});
        zenoh::Session client = open_session({.connect = {"127.0.0.1:" + std::to_string(port)}});
        const std::string base = "t/k4a_playback";
        std::unique_ptr<CaptureSource> source2 =
            open_playback({.path = path, .loop = true, .realtime = true, .depth_frame_id = record.depth_frame_id,
                           .color_frame_id = record.color_frame_id});
        Node node(node_session, NodeOptions{.base = base, .source = "test", .depth_to_color = false},
                  source2->record());
        ImageSubscriber depth(client, base + "/depth", {.transport = Transport::Raw});
        std::mutex imu_mutex;
        std::vector<std::pair<std::string, Imu>> imu;  // encoding, sample
        std::vector<DiagnosticArray> diagnostics;
        auto imu_subscriber = client.declare_subscriber(
            zenoh::KeyExpr(base + "/imu"),
            [&](const zenoh::Sample& sample) {
                std::lock_guard lock(imu_mutex);
                imu.emplace_back(sample.get_encoding().as_string(), Imu::deserialize(sample.get_payload().as_vector()));
            },
            zenoh::closures::none);
        auto diagnostics_subscriber = client.declare_subscriber(
            zenoh::KeyExpr(base + "/diagnostics"),
            [&](const zenoh::Sample& sample) {
                std::lock_guard lock(imu_mutex);
                diagnostics.push_back(DiagnosticArray::deserialize(sample.get_payload().as_vector()));
            },
            zenoh::closures::none);
        std::this_thread::sleep_for(300ms);  // the subscribers reach the node's session
        std::vector<std::int64_t> stamps;
        std::int64_t worst_age_ns = 0;  // received - stamped, the largest by magnitude
        for (int i = 0; i < 3 * kCaptures; ++i)
        {
            node.publish(*source2->next(0ms), false);
            node.publish_imu(source2->imu_samples());
            if (std::optional<Frame> frame = depth.latest(true, 100ms))
            {
                stamps.push_back(frame->stamp_ns());
                const std::int64_t age = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                             std::chrono::system_clock::now().time_since_epoch())
                                             .count() -
                                         frame->stamp_ns();
                worst_age_ns = std::abs(age) > std::abs(worst_age_ns) ? age : worst_age_ns;
            }
        }
        // the recording starts at device time 100 ms: stamped as taken when published, not 100 ms later (the age is
        // the frame's way to the subscriber, and how late a capture is published here: latest() waits up to 100 ms)
        expect(worst_age_ns >= 0 and worst_age_ns < 200000000,
               "playback: a frame is stamped as taken when it is published (received " +
                   std::to_string(double(worst_age_ns) / 1e6) + " ms after its stamp at most)");
        expect(stamps.size() > std::size_t(kCaptures + 1) and
                   std::adjacent_find(stamps.begin(), stamps.end(), std::greater_equal<>()) == stamps.end(),
               "playback: the stamps of a looping recording go on increasing (" + std::to_string(stamps.size()) +
                   " frames)");

        // <base>/imu: the recording's samples, the gyroscope turned into the accelerometer's axes
        const DiagnosticStatus reported = node.report(*source2);
        std::this_thread::sleep_for(300ms);
        std::lock_guard lock(imu_mutex);
        // a capture brings the samples up to its time; those after a loop's last capture are skipped by the seek
        expect(imu.size() >= std::size_t(3 * (kCaptures - 1) * kImuPerCapture),
               "playback: the recording's IMU samples on <base>/imu (" + std::to_string(imu.size()) + ")");
        if (not imu.empty())
        {
            const auto& [encoding, first] = imu.front();
            const k4a_calibration_t calibration = source2->record().calibration();
            const float* r = calibration.extrinsics[K4A_CALIBRATION_TYPE_GYRO][K4A_CALIBRATION_TYPE_ACCEL].rotation;
            const k4a_imu_sample_t written = imu_sample(0, 0);
            const float* g = written.gyro_sample.v;
            const double gx = r[0] * g[0] + r[1] * g[1] + r[2] * g[2], gz = r[6] * g[0] + r[7] * g[1] + r[8] * g[2];
            expect(encoding == "application/cdr;sensor_msgs/msg/Imu" and first.header.frame_id == "AzureKinectImu" and
                       first.orientation_covariance[0] == -1.0 and
                       std::abs(first.linear_acceleration.y - -9.81) < 1e-5 and
                       std::abs(first.angular_velocity.x - gx) < 1e-6 and
                       std::abs(first.angular_velocity.z - gz) < 1e-6,
                   "playback: an IMU sample as written, in the accelerometer's frame without orientation");
            bool increasing = true;
            for (std::size_t i = 1; i < imu.size(); ++i)
            {
                increasing = increasing and imu[i].second.header.stamp.to_ns() > imu[i - 1].second.header.stamp.to_ns();
            }
            expect(increasing, "playback: the IMU stamps go on increasing, also over a loop");
        }
        // <base>/diagnostics: what report() returned
        expect(diagnostics.size() == 1 and diagnostics[0].status.size() == 1 and diagnostics[0].status[0] == reported,
               "playback: report() publishes its status on <base>/diagnostics");
        const auto value = [&](const std::string& key) {
            for (const KeyValue& kv : reported.values)
            {
                if (kv.key == key)
                {
                    return kv.value;
                }
            }
            return std::string("(missing)");
        };
        expect(reported.name == "azure_kinect" and reported.hardware_id == record.serial and
                   reported.level == DiagnosticStatus::kOk and std::stod(value("captures/s")) > 0 and
                   std::stod(value("imu/s")) > 0 and value("recording") == path,
               "playback: the diagnostics name the camera and its rates (" + reported.message + ")");
        return path;
    }
}  // namespace

int main()
{
    try
    {
        test_clock();
        const CalibrationRecord record = test_calibration();
        const CalibrationRecord files = test_calibration_files(record);
        test_jpeg();
        test_node(record);
        test_playback(record);
        std::random_device random;
        const int port = 20000 + int(random() % 20000);
        zenoh::Session node_session = open_session({.listen = {"127.0.0.1:" + std::to_string(port)}});
        zenoh::Session client = open_session({.connect = {"127.0.0.1:" + std::to_string(port)}});
        // the SDK's transformation runs in its depth engine, whose OpenGL context comes from EGL here (no X server)
        use_egl_for_depth_engine(0);
        test_depth_to_color(record, node_session, client, "t/k4a_d2c");
        test_depth_to_color(record, node_session, client, "t/k4a_d2c_sdk", true);  // the SDK's transformation
        test_depth_to_color(record, node_session, client, "t/k4a_d2c_async", false, true);  // published from lanes
        test_depth_to_color(files, node_session, client, "t/k4a_d2c_files");  // the files' intrinsics
        test_depth_to_color_rect(record, node_session, client, "t/k4a_d2c_rect");
        test_depth_to_color_rect(files, node_session, client, "t/k4a_d2c_rect_files");  // P != K
        test_cpu_depth_to_color(record);
        test_cpu_depth_to_color(files);  // P != K
        test_cpu_depth_to_color(record, false);
        test_cpu_depth_to_color(files, false);
        test_flying_pixels(record);
        test_flying_pixels(files);
        test_sparse_pixels(record);
        test_sparse_pixels(files);
        test_color_rect(record, node_session, client, "t/k4a_rect", std::nullopt);
        test_color_rect(files, node_session, client, "t/k4a_rect_files", std::nullopt);  // P != K: a zoomed-out view
#ifdef DURCHBLICK_K4A_CUDA
        test_cuda_color(record, node_session, port);
        test_color_rect(files, node_session, client, "t/k4a_rect_cuda", 0);
#endif
#ifdef K4A_POINTS_PROGRAM
        test_point_cloud(record, node_session, port);
#endif
    }
    catch (const std::exception& e)
    {
        std::fprintf(stderr, "FAILED with an exception: %s\n", e.what());
        return 1;
    }
    std::fprintf(stderr, failures ? "%d failures\n" : "ok\n", failures);
    return failures ? 1 : 0;
}

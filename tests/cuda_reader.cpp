// Part of test_offline's CUDA test: a Durchblick subscriber in a process of its own (CUDA IPC handles cannot be opened
// by the process that made them) that takes one frame of <base> over the CUDA ring and writes its pixels to a file.
//
//   test_k4a_cuda_reader ENDPOINT BASE OUT_FILE
#include <chrono>
#include <cstdio>
#include <fstream>
#include <vector>

#include <cuda.h>

#include "xblick/camera.hpp"

int main(int argc, char** argv)
{
    using namespace xblick;
    if (argc != 4)
    {
        std::fprintf(stderr, "usage: test_k4a_cuda_reader ENDPOINT BASE OUT_FILE\n");
        return 2;
    }
    try
    {
        zenoh::Session session = open_session({.connect = {argv[1]}});
        ImageSubscriber sub(session, argv[2], {.transport = Transport::Cuda});
        const auto start = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - start < std::chrono::seconds(20))
        {
            if (auto frame = sub.latest(true, std::chrono::milliseconds(100)))
            {
                // the ring's memory lives in the device's primary context (Durchblick's CUDA ring)
                CUdevice device;
                CUcontext context;
                if (cuDeviceGet(&device, frame->device) != CUDA_SUCCESS or
                    cuDevicePrimaryCtxRetain(&context, device) != CUDA_SUCCESS or
                    cuCtxSetCurrent(context) != CUDA_SUCCESS)
                {
                    std::fprintf(stderr, "no CUDA context for device %d\n", frame->device);
                    return 1;
                }
                std::vector<std::uint8_t> host(std::size_t(frame->step) * frame->height);
                if (cuMemcpyDtoH_v2(host.data(), frame->device_ptr, host.size()) != CUDA_SUCCESS)
                {
                    std::fprintf(stderr, "cuMemcpyDtoH failed\n");
                    return 1;
                }
                std::ofstream(argv[3], std::ios::binary).write(reinterpret_cast<const char*>(host.data()),
                                                                std::streamsize(host.size()));
                std::printf("%s %u %u\n", frame->encoding.c_str(), frame->width, frame->height);
                return 0;
            }
        }
        std::fprintf(stderr, "no frame within 20 s\n");
        return 1;
    }
    catch (const std::exception& e)
    {
        std::fprintf(stderr, "test_k4a_cuda_reader: %s\n", e.what());
        return 1;
    }
}

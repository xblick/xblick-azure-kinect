// The colour camera's MJPEG frames. They go out as they are (Durchblick's image_raw/compressed) and are decoded only
// for subscribers to the pixels: on the CPU (libjpeg-turbo) or on the GPU (nvjpeg_decoder.hpp).
//
// UVC cameras may send MJPEG frames without Huffman tables (JPEG's DHT segment) and rely on the standard tables of
// the JPEG specification (Annex K). libjpeg-turbo, which the SDK decodes with, falls back to them; nvJPEG and other
// decoders need them in the file. with_huffman_tables() adds them to a frame that lacks them.
#pragma once

#include <cstdint>
#include <memory>
#include <span>
#include <vector>

namespace durchblick::k4a_node
{
    // True if the JPEG has a DHT segment before its first scan.
    bool has_huffman_tables(std::span<const std::uint8_t> jpeg);
    // The JPEG with the standard Huffman tables inserted before its first scan if it has none (else unchanged).
    // Throws for data that is no JPEG.
    std::vector<std::uint8_t> with_huffman_tables(std::span<const std::uint8_t> jpeg);

    // Decodes JPEG frames to rgb8 rows (width * 3 bytes, no padding) in host memory; reuses its state.
    class CpuJpegDecoder
    {
    public:
        CpuJpegDecoder();
        ~CpuJpegDecoder();
        CpuJpegDecoder(const CpuJpegDecoder&) = delete;
        CpuJpegDecoder& operator=(const CpuJpegDecoder&) = delete;

        // Decodes into the internal buffer (valid until the next decode); checks the expected size.
        const std::uint8_t* decode(std::span<const std::uint8_t> jpeg, std::uint32_t width, std::uint32_t height);

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };
}  // namespace durchblick::k4a_node

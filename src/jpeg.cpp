#include "jpeg.hpp"

#include <csetjmp>
#include <cstdio>
#include <stdexcept>
#include <string>

#include <jpeglib.h>

namespace xblick::k4a_node
{
    namespace
    {
        constexpr std::uint8_t kMarker = 0xFF, kSoi = 0xD8, kSos = 0xDA, kDht = 0xC4, kEoi = 0xD9;

        // Markers without a length field: SOI, EOI, RSTn, TEM.
        bool standalone(std::uint8_t marker)
        {
            return marker == kSoi or marker == kEoi or (marker >= 0xD0 and marker <= 0xD7) or marker == 0x01;
        }

        // Offset of the SOS marker and whether a DHT segment comes before it.
        std::pair<std::size_t, bool> scan_header(std::span<const std::uint8_t> jpeg)
        {
            if (jpeg.size() < 4 or jpeg[0] != kMarker or jpeg[1] != kSoi)
            {
                throw std::invalid_argument("not a JPEG file (no start-of-image marker)");
            }
            bool dht = false;
            std::size_t i = 2;
            while (i + 1 < jpeg.size())
            {
                if (jpeg[i] != kMarker)
                {
                    throw std::invalid_argument("JPEG header: no marker at byte " + std::to_string(i));
                }
                const std::uint8_t marker = jpeg[i + 1];
                if (marker == kMarker)  // fill byte
                {
                    ++i;
                    continue;
                }
                if (marker == kSos)
                {
                    return {i, dht};
                }
                if (standalone(marker))
                {
                    i += 2;
                    continue;
                }
                if (i + 3 >= jpeg.size())
                {
                    break;
                }
                dht = dht or marker == kDht;
                i += 2 + ((std::size_t(jpeg[i + 2]) << 8) | jpeg[i + 3]);
            }
            throw std::invalid_argument("JPEG header: no start-of-scan marker");
        }

        // The DHT segment with the four standard tables, as libjpeg's jpeg_set_defaults() sets them.
        std::vector<std::uint8_t> standard_dht()
        {
            jpeg_compress_struct cinfo{};
            jpeg_error_mgr err{};
            cinfo.err = jpeg_std_error(&err);
            jpeg_create_compress(&cinfo);
            cinfo.in_color_space = JCS_YCbCr;
            cinfo.input_components = 3;
            jpeg_set_defaults(&cinfo);
            std::vector<std::uint8_t> body;
            const auto add = [&body](JHUFF_TBL* table, std::uint8_t cls_id) {
                body.push_back(cls_id);  // class (0 DC, 1 AC) << 4 | table id
                int count = 0;
                for (int k = 1; k <= 16; ++k)
                {
                    body.push_back(table->bits[k]);
                    count += table->bits[k];
                }
                body.insert(body.end(), table->huffval, table->huffval + count);
            };
            add(cinfo.dc_huff_tbl_ptrs[0], 0x00);
            add(cinfo.ac_huff_tbl_ptrs[0], 0x10);
            add(cinfo.dc_huff_tbl_ptrs[1], 0x01);
            add(cinfo.ac_huff_tbl_ptrs[1], 0x11);
            jpeg_destroy_compress(&cinfo);
            const std::size_t length = body.size() + 2;
            std::vector<std::uint8_t> segment = {kMarker, kDht, std::uint8_t(length >> 8), std::uint8_t(length & 0xFF)};
            segment.insert(segment.end(), body.begin(), body.end());
            return segment;
        }

        struct ErrorManager
        {
            jpeg_error_mgr pub;
            std::jmp_buf jump;
            char message[JMSG_LENGTH_MAX];
        };

        void on_error(j_common_ptr info)
        {
            auto* e = reinterpret_cast<ErrorManager*>(info->err);
            (*info->err->format_message)(info, e->message);
            std::longjmp(e->jump, 1);
        }
    }  // namespace

    bool has_huffman_tables(std::span<const std::uint8_t> jpeg) { return scan_header(jpeg).second; }

    std::vector<std::uint8_t> with_huffman_tables(std::span<const std::uint8_t> jpeg)
    {
        const auto [sos, dht] = scan_header(jpeg);
        if (dht)
        {
            return {jpeg.begin(), jpeg.end()};
        }
        static const std::vector<std::uint8_t> segment = standard_dht();
        std::vector<std::uint8_t> out;
        out.reserve(jpeg.size() + segment.size());
        out.insert(out.end(), jpeg.begin(), jpeg.begin() + static_cast<std::ptrdiff_t>(sos));
        out.insert(out.end(), segment.begin(), segment.end());
        out.insert(out.end(), jpeg.begin() + static_cast<std::ptrdiff_t>(sos), jpeg.end());
        return out;
    }

    struct CpuJpegDecoder::Impl
    {
        jpeg_decompress_struct cinfo{};
        ErrorManager err{};
        std::vector<std::uint8_t> pixels;
    };

    CpuJpegDecoder::CpuJpegDecoder() : impl_(std::make_unique<Impl>())
    {
        impl_->cinfo.err = jpeg_std_error(&impl_->err.pub);
        impl_->err.pub.error_exit = on_error;
        jpeg_create_decompress(&impl_->cinfo);
    }

    CpuJpegDecoder::~CpuJpegDecoder() { jpeg_destroy_decompress(&impl_->cinfo); }

    const std::uint8_t* CpuJpegDecoder::decode(std::span<const std::uint8_t> jpeg, std::uint32_t width,
                                               std::uint32_t height)
    {
        Impl& m = *impl_;
        if (setjmp(m.err.jump))
        {
            jpeg_abort_decompress(&m.cinfo);
            throw std::runtime_error(std::string("JPEG decoding failed: ") + m.err.message);
        }
        jpeg_mem_src(&m.cinfo, jpeg.data(), static_cast<unsigned long>(jpeg.size()));
        jpeg_read_header(&m.cinfo, TRUE);
        m.cinfo.out_color_space = JCS_RGB;
        m.cinfo.dct_method = JDCT_ISLOW;
        jpeg_start_decompress(&m.cinfo);
        if (m.cinfo.output_width != width or m.cinfo.output_height != height or m.cinfo.output_components != 3)
        {
            const std::string got = std::to_string(m.cinfo.output_width) + "x" + std::to_string(m.cinfo.output_height);
            jpeg_abort_decompress(&m.cinfo);
            throw std::runtime_error("JPEG frame of " + got + ", the colour mode gives " + std::to_string(width) +
                                     "x" + std::to_string(height));
        }
        const std::size_t row = std::size_t(width) * 3;
        m.pixels.resize(row * height);
        while (m.cinfo.output_scanline < m.cinfo.output_height)
        {
            JSAMPROW rows[8];
            const JDIMENSION y = m.cinfo.output_scanline;
            JDIMENSION n = 0;
            for (; n < 8 and y + n < height; ++n)
            {
                rows[n] = m.pixels.data() + std::size_t(y + n) * row;
            }
            jpeg_read_scanlines(&m.cinfo, rows, n);
        }
        jpeg_finish_decompress(&m.cinfo);
        return m.pixels.data();
    }
}  // namespace xblick::k4a_node

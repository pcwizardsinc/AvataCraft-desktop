#include "DjiMetaTrack.h"

#include <algorithm>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
}

namespace camera {
namespace {

/// The sample-entry code of a stream, as the four characters DJI writes
/// ('djmd', 'dbgi', 'camd'). Empty when the stream has none.
std::string codecTag(const AVStream* st) {
    const std::uint32_t tag = st->codecpar->codec_tag;
    if (tag == 0)
        return {};
    std::string s(4, '\0');
    for (int i = 0; i < 4; ++i)
        s[static_cast<std::size_t>(i)] = static_cast<char>((tag >> (8 * i)) & 0xFF);
    return s;
}

struct FormatCloser {
    AVFormatContext* fmt = nullptr;
    ~FormatCloser() {
        if (fmt)
            avformat_close_input(&fmt);
    }
};

} // namespace

std::vector<std::vector<std::uint8_t>> ReadDjiMetaSamples(const std::string& path, int maxSamples) {
    std::vector<std::vector<std::uint8_t>> out;
    if (path.empty() || maxSamples <= 0)
        return out;

    FormatCloser in;
    if (avformat_open_input(&in.fmt, path.c_str(), nullptr, nullptr) < 0)
        return out;
    if (avformat_find_stream_info(in.fmt, nullptr) < 0)
        return out;

    // Which streams are DJI's metadata? Match the sample entry rather than the
    // handler name: 'dbgi' and 'camd' share the same 'meta' handler and are
    // not what we are after.
    std::vector<int> wanted;
    for (unsigned i = 0; i < in.fmt->nb_streams; ++i) {
        if (codecTag(in.fmt->streams[i]) == "djmd")
            wanted.push_back(static_cast<int>(i));
    }
    if (wanted.empty())
        return out;

    AVPacket* pkt = av_packet_alloc();
    if (!pkt)
        return out;

    int taken = 0;
    while (taken < maxSamples && av_read_frame(in.fmt, pkt) >= 0) {
        const bool mine =
            std::find(wanted.begin(), wanted.end(), pkt->stream_index) != wanted.end();
        if (mine && pkt->size > 0) {
            out.emplace_back(pkt->data, pkt->data + pkt->size);
            ++taken;
        }
        av_packet_unref(pkt);
    }
    av_packet_free(&pkt);
    return out;
}

} // namespace camera

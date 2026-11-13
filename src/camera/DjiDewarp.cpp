#include "DjiDewarp.h"

#include "DjiProto.h"

#include <cmath>

namespace camera {
namespace {

using proto::Field;
using proto::Scanner;
using proto::Wire;

/// A repeated float field: DJI writes these either packed (one
/// length-delimited run of fixed32s) or one field per value.
void appendFloats(const Field& f, std::vector<float>& out) {
    if (f.wire == Wire::LengthDelimited) {
        const std::size_t n = f.data.size() / 4;
        for (std::size_t i = 0; i < n; ++i) {
            std::uint32_t bits = 0;
            for (int b = 0; b < 4; ++b)
                bits |= static_cast<std::uint32_t>(f.data[i * 4 + b]) << (8 * b);
            float v = 0.0f;
            std::memcpy(&v, &bits, sizeof v);
            out.push_back(v);
        }
        return;
    }
    if (const auto v = f.asFloat())
        out.push_back(*v);
}

/// Is this a credible lens record rather than a message that happens to carry
/// the same field numbers? Cheap sanity, not a checksum: the focal length and
/// principal point must sit inside a frame of a believable size, and the
/// principal point must be somewhere near the middle of it.
bool plausible(const DewarpParams& d) {
    if (!d.hasCore())
        return false;
    const float w = *d.width, h = *d.height;
    if (w < 64.0f || h < 64.0f || w > 16384.0f || h > 16384.0f)
        return false;
    if (!std::isfinite(*d.fx) || !std::isfinite(*d.fy) || *d.fx <= 0.0f || *d.fy <= 0.0f)
        return false;
    // A fisheye's focal length in pixels is a fraction of the frame, never a
    // multiple of it.
    if (*d.fx > w * 4.0f || *d.fy > h * 4.0f)
        return false;
    // The principal point is near the centre; allow a generous quarter-frame.
    if (std::fabs(*d.cx - w * 0.5f) > w * 0.25f || std::fabs(*d.cy - h * 0.5f) > h * 0.25f)
        return false;
    return true;
}

void collect(std::span<const std::uint8_t> message, int depth, int maxDepth,
             std::vector<DewarpParams>& out) {
    if (depth > maxDepth || message.empty())
        return;

    // Does this message itself parse as a lens record?
    const DewarpParams here = DecodeDewarp(message);
    if (plausible(here))
        out.push_back(here);

    // Walk into nested messages regardless: a container can hold several.
    Scanner s(message);
    Field f;
    while (s.next(f)) {
        if (!f.isMessage() || f.data.empty())
            continue;
        // Only descend into something that scans cleanly as a message, so we
        // do not recurse into strings or packed arrays.
        Scanner probe(f.data);
        Field g;
        if (probe.next(g))
            collect(f.data, depth + 1, maxDepth, out);
    }
}

} // namespace

DewarpParams DecodeDewarp(std::span<const std::uint8_t> message) noexcept {
    DewarpParams d;
    Scanner s(message);
    Field f;
    while (s.next(f)) {
        switch (f.number) {
        case 1: d.fx = f.asFloat(); break;
        case 2: d.fy = f.asFloat(); break;
        case 3: d.cx = f.asFloat(); break;
        case 4: d.cy = f.asFloat(); break;
        case 5: d.k[0] = f.asFloat(); break;
        case 6: d.k[1] = f.asFloat(); break;
        case 7: d.k[2] = f.asFloat(); break;
        case 8: d.k[3] = f.asFloat(); break;
        case 9: d.xi = f.asFloat(); break;
        case 10: d.width = f.asFloat(); break;
        case 11: d.height = f.asFloat(); break;
        case 12: d.yaw = f.asFloat(); break;
        case 13: d.pitch = f.asFloat(); break;
        case 14: d.roll = f.asFloat(); break;
        case 15: d.k[4] = f.asFloat(); break;
        case 16: d.k[5] = f.asFloat(); break;
        case 17: d.k[6] = f.asFloat(); break;
        case 18: d.k[7] = f.asFloat(); break;
        case 19: d.k[8] = f.asFloat(); break;
        case 20: appendFloats(f, d.p); break;
        default: break; // a field we do not need, or a newer firmware's
        }
    }
    return d;
}

std::vector<DewarpParams> FindDewarpRecords(std::span<const std::uint8_t> message,
                                            int maxDepth) noexcept {
    std::vector<DewarpParams> out;
    try {
        collect(message, 0, maxDepth, out);
    } catch (...) {
        // Decoding is non-throwing by construction; this is belt and braces
        // so a malformed file can never take the app down.
    }
    return out;
}

} // namespace camera

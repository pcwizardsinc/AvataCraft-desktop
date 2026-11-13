#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

// DJI's per-lens calibration record, as written in the `djmd` metadata tracks
// of an .OSV.
//
// The model is Kannala-Brandt (the same one OpenOSV's KannalaBrandt5 reads
// these records into):
//
//     r(theta) = f * theta * (1 + k1 theta^2 + k2 theta^4 + ... )
//
// It is NOT orbis-reframe's unified (MEI) model, even though the record has an
// `xi` field: DJI writes xi = 0 because the model has no such term, and the
// radial polynomial multiplies the angle, not a normalised radius. The two
// families cannot be mapped field for field; DjiLensFit.h converts one into
// the other. (An earlier version of this comment claimed the families were the
// same; acting on that rendered a black preview.) DJI also calibrates in its
// own frame (3840x3840 on the Avata 360) rather than the stream's.
//
// Field numbers are DJI's and are not guessed: they match OpenOSV's decoder,
// which was derived from real files.

namespace camera {

struct DewarpParams {
    // Intrinsics, in the calibration frame named by width/height below.
    std::optional<float> fx, fy;   ///< 1, 2  focal lengths, px
    std::optional<float> cx, cy;   ///< 3, 4  principal point, px
    std::optional<float> xi;       ///< 9     unified-model term (DJI writes 0)
    std::array<std::optional<float>, 9> k{}; ///< 5-8, 15-19  radial polynomial
    std::vector<float> p;          ///< 20    tangential distortion (p1, p2)

    // The frame the intrinsics are expressed in. DJI writes these as floats
    // in some firmwares and varints in others, hence the loose read.
    std::optional<float> width;    ///< 10
    std::optional<float> height;   ///< 11

    // The lens's pose relative to the camera body, in degrees.
    std::optional<float> yaw, pitch, roll; ///< 12, 13, 14

    /// A record is usable only with intrinsics and a frame to read them in.
    /// DJI writes placeholder records with everything absent for the slots a
    /// given camera does not fill, and those must be skipped, not used.
    [[nodiscard]] bool hasCore() const noexcept {
        return fx && fy && cx && cy && width && height && *width > 0.0f && *height > 0.0f;
    }
};

/// Decode one DewarpParams message body.
[[nodiscard]] DewarpParams DecodeDewarp(std::span<const std::uint8_t> message) noexcept;

/// Every usable DewarpParams found anywhere in `message`, depth-first.
///
/// DJI nests these differently between models and firmwares (and the Avata
/// 360 leaves its primary slots empty, writing the real pair further in), so
/// rather than hard-code one path we walk the tree and keep what parses as a
/// plausible lens. Records are returned in the order encountered, which for
/// every file seen so far is slave lens then master lens.
[[nodiscard]] std::vector<DewarpParams> FindDewarpRecords(std::span<const std::uint8_t> message,
                                                          int maxDepth = 6) noexcept;

} // namespace camera

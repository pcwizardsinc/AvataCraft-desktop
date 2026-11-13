#pragma once

#include <array>
#include <cstddef>

// DJI's lens records are Kannala-Brandt; orbis-reframe's stitcher speaks the
// unified (MEI) model. These two are not the same family, so the numbers
// cannot be copied across: they must be converted. This file does that.
//
// Kannala-Brandt (what DJI writes, and what OpenOSV's KannalaBrandt5 uses):
//
//     r(theta) = f * theta * (1 + k1 theta^2 + k2 theta^4 + ... + k5 theta^10)
//
// the polynomial multiplies the OFF-AXIS ANGLE, and the base projection is
// equidistant (r ~ f * theta). DJI's `xi` field is 0 because the model has no
// such term; it is not a MEI lens with xi = 0.
//
// MEI (orbis-reframe, insv::LensCalibExt, FisheyeProjection::meiRadial):
//
//     rho = sin(theta) / (cos(theta) + xi)
//     r(theta) = f * rho * (1 + K1 rho^2 + K2 rho^4 + K3 rho^6 + K4 rho^8)
//
// the polynomial multiplies the NORMALISED RADIUS, and the base projection
// depends on xi (xi = 1 is stereographic, rho = tan(theta / 2)).
//
// Feeding KB's k1..k4 into MEI's K1..K4 with xi = 0 (a pinhole) is what put
// the preview black: a pinhole cannot even represent a 190 deg lens, and the
// coefficients mean different things.
//
// The conversion: fix xi = 1, so rho = tan(theta / 2) and the two base
// projections share the same slope at the axis when f_mei = 2 f_kb; then
// choose K1..K4 by linear least squares so the MEI curve matches the KB curve
// out to the usable rim. Both curves are smooth and monotonic over a fisheye's
// field, so four terms fit to a small fraction of a pixel (the test checks).

namespace camera {

/// Kannala-Brandt radius (same units as f) at off-axis angle theta (rad).
/// `kb` holds k1..k5; OpenOSV's model uses five of DJI's nine terms and the
/// rest are zero on every clip seen.
[[nodiscard]] double KannalaBrandtRadius(double f, const std::array<double, 5>& kb, double theta) noexcept;

/// A MEI lens, in the same units f was given in.
struct MeiLens {
    double xi = 1.0;
    double f = 0.0;
    std::array<double, 4> k{0.0, 0.0, 0.0, 0.0};
    /// Largest |r_mei - r_kb| over the fitted range, in the units of f.
    double maxErr = 0.0;
};

/// MEI radius (same units as lens.f) at off-axis angle theta (rad).
[[nodiscard]] double MeiRadius(const MeiLens& lens, double theta) noexcept;

/// Convert a Kannala-Brandt lens into the MEI model, matching the radial
/// curve over [0, thetaMaxRad]. Never throws; a degenerate input (f <= 0,
/// non-finite terms) yields the plain stereographic lens with all K = 0.
[[nodiscard]] MeiLens FitMeiToKannalaBrandt(double fKb, const std::array<double, 5>& kb,
                                            double thetaMaxRad) noexcept;

} // namespace camera

#include "DjiLensFit.h"

#include <algorithm>
#include <cmath>

namespace camera {

double KannalaBrandtRadius(double f, const std::array<double, 5>& kb, double theta) noexcept
{
    const double t2 = theta * theta;
    const double poly = 1.0 + t2 * (kb[0] + t2 * (kb[1] + t2 * (kb[2] + t2 * (kb[3] + t2 * kb[4]))));
    return f * theta * poly;
}

double MeiRadius(const MeiLens& lens, double theta) noexcept
{
    const double st = std::sin(theta), ct = std::cos(theta);
    const double denom = ct + lens.xi;
    if (denom <= 1e-12)
        return 0.0;
    const double rho = st / denom;
    const double r2 = rho * rho, r4 = r2 * r2;
    const double poly = 1.0 + lens.k[0] * r2 + lens.k[1] * r4 + lens.k[2] * r2 * r4 + lens.k[3] * r4 * r4;
    return lens.f * rho * poly;
}

namespace {

constexpr int N = 5; // unknowns: a focal scale and K1..K4

/// Solve the NxN system A x = b in place by Gaussian elimination with partial
/// pivoting. Returns false if the matrix is singular to working precision.
bool solveN(double a[N][N], double b[N], double x[N]) noexcept
{
    int order[N] = {0, 1, 2, 3, 4};
    for (int col = 0; col < N; ++col) {
        int best = col;
        for (int r = col + 1; r < N; ++r)
            if (std::fabs(a[order[r]][col]) > std::fabs(a[order[best]][col]))
                best = r;
        std::swap(order[col], order[best]);
        const int p = order[col];
        if (std::fabs(a[p][col]) < 1e-300)
            return false;
        for (int r = col + 1; r < N; ++r) {
            const int q = order[r];
            const double m = a[q][col] / a[p][col];
            for (int c = col; c < N; ++c)
                a[q][c] -= m * a[p][c];
            b[q] -= m * b[p];
        }
    }
    for (int col = N - 1; col >= 0; --col) {
        const int p = order[col];
        double s = b[p];
        for (int c = col + 1; c < N; ++c)
            s -= a[p][c] * x[c];
        x[col] = s / a[p][col];
    }
    return true;
}

} // namespace

MeiLens FitMeiToKannalaBrandt(double fKb, const std::array<double, 5>& kb, double thetaMaxRad) noexcept
{
    MeiLens out;
    out.xi = 1.0;
    // Slope match on the axis: KB gives r ~ f_kb * theta, MEI with xi = 1
    // gives r ~ f_mei * theta / 2.
    out.f = 2.0 * fKb;

    bool sane = std::isfinite(fKb) && fKb > 0.0 && std::isfinite(thetaMaxRad) && thetaMaxRad > 0.1;
    for (const double k : kb)
        sane = sane && std::isfinite(k);
    if (!sane) {
        out.f = std::isfinite(fKb) && fKb > 0.0 ? 2.0 * fKb : 0.0;
        return out;
    }

    // Least squares on y(theta) = r_kb / (f0 * rho) = c0 + sum_j c_j rho^(2j),
    // rho = tan(theta / 2), sampled evenly in theta, with f0 = 2 f_kb. Linear
    // in (c0, c1..c4); afterwards f_mei = f0 c0 and K_j = c_j / c0, so the
    // focal length is free to move a little off the axis slope in exchange
    // for a tighter match at the rim, where the seam is.
    constexpr int kSamples = 512;
    double ata[N][N] = {};
    double atb[N] = {};
    const double t0 = 0.02; // skip the axis itself, where rho -> 0
    for (int s = 0; s < kSamples; ++s) {
        const double theta = t0 + (thetaMaxRad - t0) * (static_cast<double>(s) / (kSamples - 1));
        const double rho = std::tan(theta / 2.0);
        const double y = KannalaBrandtRadius(fKb, kb, theta) / (out.f * rho);
        const double r2 = rho * rho;
        const double basis[N] = {1.0, r2, r2 * r2, r2 * r2 * r2, r2 * r2 * r2 * r2};
        // Weight the outer field: an error there lands on the seam, an error
        // near the axis lands in the middle of a lens where nothing overlaps.
        const double u = theta / thetaMaxRad;
        const double w = 1.0 + 8.0 * u * u * u * u;
        for (int i = 0; i < N; ++i) {
            atb[i] += w * basis[i] * y;
            for (int j = 0; j < N; ++j)
                ata[i][j] += w * basis[i] * basis[j];
        }
    }
    double c[N] = {};
    if (!solveN(ata, atb, c) || !std::isfinite(c[0]) || c[0] <= 0.1 || c[0] >= 10.0)
        return out; // leave K = 0: a stereographic stand-in
    out.f *= c[0];
    for (int i = 0; i < 4; ++i) {
        const double kj = c[i + 1] / c[0];
        out.k[static_cast<std::size_t>(i)] = std::isfinite(kj) ? kj : 0.0;
    }

    double worst = 0.0;
    for (int s = 0; s < kSamples; ++s) {
        const double theta = t0 + (thetaMaxRad - t0) * (static_cast<double>(s) / (kSamples - 1));
        worst = std::max(worst, std::fabs(MeiRadius(out, theta) - KannalaBrandtRadius(fKb, kb, theta)));
    }
    out.maxErr = worst;
    return out;
}

} // namespace camera

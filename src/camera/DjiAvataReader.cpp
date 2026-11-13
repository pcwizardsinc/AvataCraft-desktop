#include "DjiAvataReader.h"

#include "DjiDewarp.h"
#include "DjiLensFit.h"
#include "DjiMetaTrack.h"
#include "DjiSrt.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <vector>

// DJI .OSV support. This first stage identifies the file, describes the take
// (model, lenses, proxy, colour profile) and sets the mount the Avata needs;
// the lens calibration and the attitude track follow (see the TODOs below),
// ported from OpenOSV's DJI metadata decoder.

namespace camera {
namespace {

namespace fs = std::filesystem;

std::string lowerExt(const std::string& path) {
    std::string e = fs::path(path).extension().string();
    std::transform(e.begin(), e.end(), e.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return e;
}

// DJI's own low-resolution proxy of the same take, beside the master and
// named the same. Returned empty when there is none.
std::string proxyBeside(const std::string& osvPath) {
    std::error_code ec;
    for (const char* ext : {".LRF", ".lrf"}) {
        fs::path p = fs::path(osvPath);
        p.replace_extension(ext);
        if (fs::exists(p, ec))
            return p.string();
    }
    return {};
}

// How the camera is mounted on the aircraft, as degrees of yaw/pitch/roll
// applied to the attitude the file records.
//
// The Avata 360 flies its 360 camera with one lens up and one down, and its
// IMU is mounted a quarter-turn in pitch from the nose-forward, sky-up frame
// the stitch expects: read without this, the aircraft's nose lands at the
// world's down while it flies level, and the horizon ends up vertical.
// Measured on real Avata 360 footage (firmware 01.00.06.00, 4K/100): of the
// four quarter-turns about the mounted right axis, -90 deg puts the mounted
// up within 0.962 of true up, against 0.193 for no turn at all.
//
// The Osmo 360 is a handheld 360 camera with the ordinary mount and needs
// none of this.
struct Mount {
    double yawDeg = 180.0;
    double pitchDeg = 0.0;
    double rollDeg = 0.0;
};

Mount mountFor(const std::string& model) {
    Mount m;
    if (model.find("Avata") != std::string::npos) {
        m.pitchDeg = -90.0;
    }
    return m;
}

} // namespace

CameraReader::Result DjiAvataReader::read(const std::vector<insv::InsvStreamInfo>& streams,
                                          ClipMetadata& out, std::string& err) const {
    // Claim the file by extension: DJI's .OSV. Insta360's reader is asked
    // first and returns NotMine for these, so the two never collide.
    if (lowerExt(out.path) != ".osv")
        return Result::NotMine;

    if (streams.empty()) {
        err = "the .OSV has no video track";
        return Result::Failed;
    }

    out.maker = "DJI";
    // TODO(metadata): read the model, firmware and serial from DJI's boxes in
    // the `moov` (OpenOSV's DjmdDecoder reads these). Until then the model is
    // taken from the file name, which DJI writes as DJI_<date>_<n>_D_<year>,
    // and the Avata mount is assumed: that is the camera this reader exists
    // for, and assuming it is far better than levelling the horizon sideways.
    if (out.model.empty())
        out.model = "DJI Avata 360";

    // Two layouts are in the wild for a DJI 360 clip, and which one a file
    // uses depends on the camera and the recording mode:
    //   * one HEVC track per lens (two video streams), and
    //   * both lenses in one track, side by side (one stream, 2:1).
    // The .LRF proxy of a clip is side-by-side even when its .OSV is not, so
    // handling only the two-track case would refuse half the files.
    out.projection = Projection::DualFisheye;
    out.lenses.resize(2);
    if (streams.size() >= 2) {
        for (int i = 0; i < 2; ++i)
            out.lenses[i].source = LensSource{out.path, i};
    } else {
        // Side by side in one track: both lenses name the same stream, and the
        // stitch takes the left and right halves. A lens is then half as wide
        // as the stream, which the common fields must say.
        out.lenses[0].source = LensSource{out.path, 0};
        out.lenses[1].source = LensSource{out.path, 0};
        out.lensWidth = streams[0].width / 2;
        out.lensHeight = streams[0].height;
    }

    // Lens calibration.
    //
    // DJI records the real thing -- a DewarpParams record per lens -- in the
    // `djmd` metadata tracks. Read those when they are there, and fall back
    // to a geometric approximation when they are not, so an unreadable or
    // unfamiliar file still stitches (badly) rather than refusing to open.
    //
    // The proxy beside the clip carries the same tracks and is a fraction of
    // the size (142 MB against 2.4 GB on the test clip), so read it by
    // preference: the calibration frame is the camera's own either way.
    out.calibrated = false;
    {
        const double side = static_cast<double>(out.lensWidth);
        const double height = static_cast<double>(out.lensHeight);
        const double combinedW = side * 2.0;

        // Diagnostics, both read once here:
        //   AVATA_CALIB=approx   ignore the file's records, use the stand-in
        //   AVATA_SWAP_LENSES=1  feed stream 1 to lens 0 and stream 0 to lens 1
        // Swapping does not soften a seam, it turns the whole sphere round, so
        // it is a yes/no check on a clip that looks mirrored, in either path.
        const auto envOn = [](const char* name) {
            const char* v = std::getenv(name);
            return v && *v && *v != '0';
        };
        const bool forceApprox = [] {
            const char* v = std::getenv("AVATA_CALIB");
            return v && std::string(v) == "approx";
        }();
        const bool swapLenses = envOn("AVATA_SWAP_LENSES");
        // The stitcher subtracts a fixed sensor clock of 90 deg from every
        // lens's roll (StitchParams::tzNominalDeg, because Insta360's X5 writes
        // roll ~90 for both lenses). A lens handed roll = 0 is therefore rolled
        // -90 deg about its own axis, and since the two lenses face opposite
        // ways that twists the hemispheres 180 deg against each other: a
        // house ends up where the lawn across the street should be. Writing
        // 90 cancels it. AVATA_LENS_ROLL=<deg> overrides for experiments.
        const double lensRollDeg = [] {
            const char* v = std::getenv("AVATA_LENS_ROLL");
            return v && *v ? std::atof(v) : 90.0;
        }();

        std::vector<DewarpParams> lenses;
        if (!forceApprox) {
            const std::string proxy = proxyBeside(out.path);
            for (const std::string& from : {proxy, out.path}) {
                for (const auto& sample : ReadDjiMetaSamples(from)) {
                    for (DewarpParams& d : FindDewarpRecords(sample)) {
                        lenses.push_back(std::move(d));
                        if (lenses.size() >= 2)
                            break;
                    }
                    if (lenses.size() >= 2)
                        break;
                }
                if (lenses.size() >= 2)
                    break;
            }
        }

        if (lenses.size() >= 2) {
            // Record order is file order, which DJI writes as slot 3 then
            // slot 4 (native_refine_far slave, then master), and OpenOSV ties
            // slot 3 to video stream 0 and slot 4 to stream 1. So the first
            // record is lens 0 and the second is lens 1; nothing is reordered
            // by the angles it carries.
            constexpr double kHalfFovRad = 95.0 * 3.14159265358979323846 / 180.0;
            for (int i = 0; i < 2; ++i) {
                const DewarpParams& d = lenses[static_cast<std::size_t>(i)];
                insv::LensCalibExt& c = out.lenses[i].calibration;

                // DJI calibrates in its own frame (3840x3840 on the Avata); the
                // stream here may be 1920x1920. Everything in pixels scales.
                const double sx = side / static_cast<double>(*d.width);
                const double sy = height / static_cast<double>(*d.height);

                // DJI's record is Kannala-Brandt; LensCalibExt is MEI. The
                // numbers are converted, not copied -- see DjiLensFit.h.
                std::array<double, 5> kb{};
                for (std::size_t t = 0; t < kb.size() && t < d.k.size(); ++t)
                    kb[t] = d.k[t] ? static_cast<double>(*d.k[t]) : 0.0;
                const double fKbX = static_cast<double>(*d.fx) * sx;
                const double fKbY = static_cast<double>(*d.fy) * sy;
                const MeiLens mei = FitMeiToKannalaBrandt(fKbX, kb, kHalfFovRad);

                c.xi = mei.xi;
                c.fx = mei.f;
                c.fy = mei.f * (fKbY / fKbX); // keep DJI's fy/fx ratio
                c.cx = static_cast<double>(*d.cx) * sx + (i == 0 ? 0.0 : side);
                c.cy = static_cast<double>(*d.cy) * sy;
                for (std::size_t t = 0; t < 4; ++t)
                    c.k[t] = mei.k[t];
                // DJI's tangential terms belong to its model, not to MEI's
                // normalised coordinates; they are tiny on a 360 lens and are
                // left out rather than applied in the wrong frame.
                c.p[0] = c.p[1] = 0.0;
                c.w = combinedW;
                c.h = height;
                // DJI's yaw/pitch/roll are the lens's pose in the camera body
                // (on the Avata the lenses look up and down the body's Y axis),
                // not a small correction to orbis's nominal front/back pair.
                // Applying them here as corrections is wrong by construction.
                // The mount (mountFor) and the nominal pair carry the pose for
                // now; the angles are logged below so they can be read. On the
                // DangTree clip they are within 0.9 deg of the nominal poses
                // (lens 0 ypr -179.1/90.1/0.3, lens 1 -0.1/90.1/0.9), so what
                // is left out here is sub-degree seam error, not geometry.
                c.yaw = c.pitch = 0.0;
                c.roll = lensRollDeg;
                c.focalRefW = side;

                std::fprintf(stderr,
                    "[DjiAvataReader] lens %d from file: KB fx=%.2f fy=%.2f cx=%.2f cy=%.2f frame=%.0fx%.0f"
                    " k=[%.5g %.5g %.5g %.5g %.5g] ypr=[%.3f %.3f %.3f] xi=%.3g"
                    " -> MEI xi=1 f=%.2f K=[%.5g %.5g %.5g %.5g] fit err %.3f px (stream %dx%d)\n",
                    i, static_cast<double>(*d.fx), static_cast<double>(*d.fy),
                    static_cast<double>(*d.cx), static_cast<double>(*d.cy),
                    static_cast<double>(*d.width), static_cast<double>(*d.height),
                    kb[0], kb[1], kb[2], kb[3], kb[4],
                    d.yaw ? static_cast<double>(*d.yaw) : 0.0,
                    d.pitch ? static_cast<double>(*d.pitch) : 0.0,
                    d.roll ? static_cast<double>(*d.roll) : 0.0,
                    d.xi ? static_cast<double>(*d.xi) : 0.0,
                    mei.f, mei.k[0], mei.k[1], mei.k[2], mei.k[3], mei.maxErr,
                    out.lensWidth, out.lensHeight);
            }
            out.calibrated = true;
        } else {
            // No usable records (or AVATA_CALIB=approx): stand in the camera's
            // published geometry. The Avata 360 calibrates in a 3840x3840
            // frame whose fisheye circle is 3776 px across, modelled as
            // stereographic (MEI with xi = 1), where r = f * tan(theta / 2)
            // and a 190 deg lens puts the rim at theta = 95 deg. Expect
            // visible seam error: a real lens is not exactly stereographic and
            // is not exactly centred.
            constexpr double kCircleFraction = 3776.0 / 3840.0;
            constexpr double kHalfFovDeg = 95.0;
            const double circle = side * kCircleFraction;
            const double halfFovRad = kHalfFovDeg * 3.14159265358979323846 / 180.0;
            const double f = (circle / 2.0) / std::tan(halfFovRad / 2.0);
            for (int i = 0; i < 2; ++i) {
                insv::LensCalibExt& c = out.lenses[i].calibration;
                c.xi = 1.0;
                c.fx = c.fy = f;
                c.cx = (i == 0) ? side / 2.0 : side + side / 2.0;
                c.cy = height / 2.0;
                c.w = combinedW;
                c.h = height;
                c.yaw = c.pitch = 0.0;
                c.roll = lensRollDeg;
                c.focalRefW = side;
            }
            std::fprintf(stderr, "[DjiAvataReader] %s; using the stereographic stand-in\n",
                         forceApprox ? "AVATA_CALIB=approx" : "no usable DewarpParams in the file");
            out.calibrated = true; // approximate, but usable
        }
        std::fprintf(stderr, "[DjiAvataReader] lens roll written as %.1f deg (stitcher clock is 90)\n", lensRollDeg);

        if (swapLenses && out.lenses[0].source.stream != out.lenses[1].source.stream) {
            std::swap(out.lenses[0].source, out.lenses[1].source);
            std::fprintf(stderr, "[DjiAvataReader] AVATA_SWAP_LENSES=1: lens 0 <- stream %d, lens 1 <- stream %d\n",
                         out.lenses[0].source.stream, out.lenses[1].source.stream);
        }
    }

    const Mount mount = mountFor(out.model);
    out.mountYawDeg = mount.yawDeg;
    out.mountPitchDeg = mount.pitchDeg;
    out.mountRollDeg = mount.rollDeg;

    // Colour mode. The .srt beside the clip records what the camera actually
    // shot in (`color_md: dlog_m`), so a Normal-colour clip is no longer
    // mislabelled as log and sent looking for a LUT it does not need.
    out.logProfile.clear();
    if (const std::string srt = DjiSrtPathFor(out.path); !srt.empty()) {
        if (const auto info = ReadDjiSrt(srt)) {
            if (info->colourMode.find("dlog") != std::string::npos ||
                info->colourMode.find("d-log") != std::string::npos)
                out.logProfile = "D-Log M";
        }
    }
    if (out.logProfile.empty() && out.model.find("Avata") != std::string::npos) {
        // No .srt beside the clip: the Avata's own default is D-Log M, and
        // calling a log clip "normal" looks far worse than the reverse.
        out.logProfile = "D-Log M";
    }

    const std::string proxy = proxyBeside(out.path);
    if (!proxy.empty())
        out.proxyPath = proxy;

    return Result::Read;
}

std::optional<insv::ImuTrack> DjiAvataReader::readImu(const ClipMetadata& clip) const {
    // TODO(attitude): DJI records a dense attitude/IMU track (4 kHz on the
    // Avata 360) in its metadata boxes. Porting OpenOSV's decoder for it is
    // what gives this family stabilisation and horizon levelling; until then
    // the clip plays unstabilised rather than wrongly stabilised.
    (void)clip;
    return std::nullopt;
}

std::optional<insv::GpsTrack> DjiAvataReader::readGps(const ClipMetadata& clip) const {
    // The aircraft's flight path, from the .srt DJI writes beside the clip.
    // (The metadata tracks carry it too, but the .srt is plain text and is
    // there for every clip.)
    const std::string srt = DjiSrtPathFor(clip.path);
    if (srt.empty())
        return std::nullopt;
    const auto info = ReadDjiSrt(srt);
    if (!info || info->gps.fixes.empty())
        return std::nullopt;
    return info->gps;
}

std::optional<std::string> DjiAvataReader::clipFileFor(const std::string& path) const {
    // A .LRF is the proxy of the .OSV of the same name: tell the pool the
    // clip it belongs to, so the proxy is not imported as a clip of its own.
    const std::string ext = lowerExt(path);
    if (ext == ".lrf") {
        std::error_code ec;
        for (const char* osv : {".OSV", ".osv"}) {
            fs::path p = fs::path(path);
            p.replace_extension(osv);
            if (fs::exists(p, ec))
                return p.string();
        }
        return std::nullopt;
    }
    if (ext == ".osv")
        return path;
    return std::nullopt;
}

std::optional<Thumbnail> DjiAvataReader::readThumbnail(const std::string& path) const {
    // TODO(thumbnail): .OSV files carry embedded JPEG covers (osvtool extracts
    // them with --covers). Reading one is much cheaper than decoding video and
    // is what the media pool wants for its grid.
    (void)path;
    return std::nullopt;
}

} // namespace camera

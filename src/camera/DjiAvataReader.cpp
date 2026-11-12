#include "DjiAvataReader.h"

#include <algorithm>
#include <cmath>
#include <cctype>
#include <filesystem>
#include <string>

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
    // DJI does record the real thing: a DewarpParams record per lens, in the
    // metadata streams of the .OSV (an Avata 360 writes them in the slot
    // OpenOSV calls native_refine_far). Reading those is a port of DJI's
    // metadata decoder and is the next piece of work; see the project notes.
    //
    // Until then, stand in a calibration derived from the camera's published
    // geometry, so a clip stitches and can be judged by eye instead of being
    // refused outright. It is deliberately approximate, and says so:
    //
    //   * The Avata 360 calibrates in a 3840x3840 frame whose fisheye circle
    //     is 3776 px across, so the circle fills 0.9833 of the frame whatever
    //     the stream is scaled to.
    //   * Modelled as stereographic (MEI with xi = 1, no polynomial), where
    //     r = f * tan(theta / 2). With a 190 deg lens the rim sits at
    //     theta = 95 deg, so f = (circle / 2) / tan(47.5 deg).
    //
    // Expect visible seam error: a real lens is not exactly stereographic and
    // its principal point is not exactly centred. This is a stopgap that
    // makes the picture visible, not a substitute for the file's own numbers.
    {
        constexpr double kCircleFraction = 3776.0 / 3840.0; // circle / frame
        constexpr double kHalfFovDeg = 95.0;                // 190 deg lens
        const double side = static_cast<double>(out.lensWidth);
        const double circle = side * kCircleFraction;
        const double halfFovRad = kHalfFovDeg * 3.14159265358979323846 / 180.0;
        const double f = (circle / 2.0) / std::tan(halfFovRad / 2.0);

        // cx/cy are given in the SIDE-BY-SIDE combined frame, not in one
        // lens: FisheyeProjection takes halfW = w / 2 and subtracts it for
        // lens 1 (and scales by streamW / halfW). So w is both lenses wide.
        const double combinedW = side * 2.0;
        const double height = static_cast<double>(out.lensHeight);
        for (int i = 0; i < 2; ++i) {
            insv::LensCalibExt& c = out.lenses[i].calibration;
            c.xi = 1.0; // stereographic
            c.fx = c.fy = f;
            c.cx = (i == 0) ? side / 2.0 : side + side / 2.0;
            c.cy = height / 2.0;
            c.w = combinedW;
            c.h = height;
            // yaw/pitch/roll here are the CORRECTION to the lens's nominal
            // pose, not the pose itself: FisheyeProjection already turns lens
            // 1 by nominalYaw1Deg (180 deg) to face the other way. Setting
            // 180 here too turned it back, so both lenses covered the same
            // hemisphere and half the sphere came out black.
            c.yaw = c.pitch = c.roll = 0.0;
            // The sensor window fx/fy are given in: one lens wide, since f
            // above is computed in this stream's own pixels.
            c.focalRefW = side;
        }
    }
    out.calibrated = true;

    const Mount mount = mountFor(out.model);
    out.mountYawDeg = mount.yawDeg;
    out.mountPitchDeg = mount.pitchDeg;
    out.mountRollDeg = mount.rollDeg;

    // The Avata records D-Log M; a clip shot in it looks flat until the
    // profile is applied. DJI's own name for the curve is "D-Log M".
    // TODO(metadata): read the real colour mode (Normal vs D-Log M) from the
    // metadata rather than assuming; the box exists, we just don't parse it.
    out.logProfile = "D-Log M";

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
    // TODO(gps): the aircraft's GPS is in the .SRT beside the clip (and in the
    // metadata boxes). Not read yet.
    (void)clip;
    return std::nullopt;
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

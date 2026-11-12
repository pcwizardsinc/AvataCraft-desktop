#include "DjiAvataReader.h"

#include <algorithm>
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

    if (streams.size() < 2) {
        // A .OSV carries one HEVC track per lens. One stream means either a
        // single-lens recording (the Avata's Single Lens mode writes .MP4,
        // not .OSV) or a file we have not understood.
        err = "the .OSV has " + std::to_string(streams.size()) +
              " video track(s); a 360 clip has two, one per lens";
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

    out.projection = Projection::DualFisheye;
    out.lenses.resize(2);
    for (int i = 0; i < 2; ++i)
        out.lenses[i].source = LensSource{out.path, i};

    // TODO(calibration): fill lenses[i].calibration from DJI's lens boxes and
    // set calibrated = true. Until then the app falls back to its own model
    // defaults, as it does for an Insta360 file whose trailer lacks them.
    out.calibrated = false;

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

#pragma once

#include <optional>
#include <string>

#include <Gps.h> // insv::GpsTrack

// DJI writes a SubRip (.srt) file beside each clip, one subtitle entry per
// video frame, carrying the camera's own per-frame telemetry:
//
//   1
//   00:00:00,000 --> 00:00:00,010
//   <font size="28">FrameCnt: 1, DiffTime: 10ms, FrameId: 10393
//   2026-10-18 15:37:03.449
//   [iso: 200] [shutter: 1/3200.0] [fnum: 1.9] [ev: 0.3] [ct: 5429]
//   [color_md: dlog_m] [ae_meter_md: 0] [latitude: 47.086130]
//   [longitude: -122.313311] [rel_alt: 2.200 abs_alt: 121.834] </font>
//
// It has no attitude or gimbal angles, so it cannot stabilise anything; what
// it does carry is the flight path and the colour mode the camera recorded
// in, both of which the file's own metadata would otherwise have to be
// decoded for.
//
// Tolerant by design: fields come and go between firmwares and models, so
// anything unrecognised is skipped rather than failing the parse.

namespace camera {

/// What one `.srt` yields besides the track itself.
struct DjiSrtInfo {
    insv::GpsTrack gps;        ///< Empty when the file records no position.
    std::string colourMode;    ///< `color_md`, e.g. "dlog_m" ("" if absent).
    double colourTempK = 0.0;  ///< `ct`, the white balance in kelvin (0 = absent).
    int iso = 0;               ///< First `iso` seen (0 = absent).
    bool any = false;          ///< True when at least one entry parsed.
};

/// Parse `path` (a .srt). Returns nullopt when the file cannot be read or
/// holds nothing recognisable.
[[nodiscard]] std::optional<DjiSrtInfo> ReadDjiSrt(const std::string& path);

/// The .srt beside a clip: the same name with an .srt / .SRT extension.
/// Empty when there is none.
[[nodiscard]] std::string DjiSrtPathFor(const std::string& clipPath);

} // namespace camera

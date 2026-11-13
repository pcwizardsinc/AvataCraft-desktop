#pragma once

#include <cstdint>
#include <string>
#include <vector>

// Reads the samples of DJI's metadata tracks out of an .OSV (or its .LRF
// proxy, which carries the same tracks).
//
// An .OSV is an ordinary ISO BMFF file. Beside the two HEVC lens tracks it
// carries several `meta`-handler tracks, told apart by their sample-entry
// four-character code:
//
//     djmd   DJI metadata -- lens calibration and attitude (what we want)
//     dbgi   DJI debug    -- not used
//     camd   camera data
//
// ffprobe on an Avata 360 clip shows streams 2 and 3 as `djmd`. Each sample
// is a protobuf message; see DjiDewarp.h for what we read out of them.

namespace camera {

/// Read up to `maxSamples` samples from the file's `djmd` tracks.
///
/// Samples are returned in file order, which for every clip seen so far means
/// the calibration appears within the first few. Reading is capped because a
/// 100 fps clip writes a sample per frame and we only need the header-ish
/// ones; a cap keeps this to a quick read of the start of the file rather
/// than a full scan of 2.4 GB.
///
/// Returns an empty vector when the file has no such track, cannot be opened,
/// or FFmpeg refuses it. Never throws.
[[nodiscard]] std::vector<std::vector<std::uint8_t>> ReadDjiMetaSamples(const std::string& path,
                                                                        int maxSamples = 64);

} // namespace camera

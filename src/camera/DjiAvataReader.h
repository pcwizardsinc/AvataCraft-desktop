#pragma once

#include "CameraReader.h"

// DJI's .OSV files (the Avata 360 and the Osmo 360): an ISO BMFF container
// whose `moov` carries DJI's own metadata boxes -- the model and firmware, the
// lens calibration, and a dense attitude/IMU track -- alongside two HEVC video
// tracks, one per lens. A `.LRF` of the same name beside it is the camera's
// low-resolution proxy of the same take.
//
// Unlike Insta360's trailer (which sits after the `moov`), DJI's metadata is
// inside it, so the file is a valid MP4 to any player; only the per-lens
// geometry and the attitude need DJI's boxes to read.
//
// What differs between the Avata 360 and the Osmo 360 is the mount: the Avata
// flies its 360 camera with one lens up and one down, so its attitude needs a
// quarter-turn the Osmo does not (see DjiAvataReader.cpp, mountFor()).

namespace camera {

class DjiAvataReader : public CameraReader {
public:
    Result read(const std::vector<insv::InsvStreamInfo>& streams, ClipMetadata& out,
                std::string& err) const override;
    std::optional<insv::ImuTrack> readImu(const ClipMetadata& clip) const override;
    std::optional<insv::GpsTrack> readGps(const ClipMetadata& clip) const override;
    std::optional<Thumbnail> readThumbnail(const std::string& path) const override;
    std::optional<std::string> clipFileFor(const std::string& path) const override;
};

} // namespace camera

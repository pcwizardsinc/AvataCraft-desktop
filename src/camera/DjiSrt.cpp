#include "DjiSrt.h"

#include <cctype>
#include <charconv>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string_view>

namespace camera {
namespace {

namespace fs = std::filesystem;

/// The value of `[key: value]`, or of `key: value` inside a bracket group
/// that holds several (DJI writes `[rel_alt: 2.200 abs_alt: 121.834]`).
/// Returns the text up to the next space or ']'.
std::optional<std::string_view> field(std::string_view line, std::string_view key) {
    std::string needle;
    needle.reserve(key.size() + 2);
    needle += key;
    needle += ':';
    std::size_t at = 0;
    for (;;) {
        at = line.find(needle, at);
        if (at == std::string_view::npos)
            return std::nullopt;
        // Must start a token: preceded by '[' or a space, so `alt:` does not
        // match inside `rel_alt:`.
        const bool starts = at == 0 || line[at - 1] == '[' || line[at - 1] == ' ';
        if (starts)
            break;
        at += needle.size();
    }
    std::size_t v = at + needle.size();
    while (v < line.size() && line[v] == ' ')
        ++v;
    std::size_t end = v;
    while (end < line.size() && line[end] != ' ' && line[end] != ']')
        ++end;
    if (end == v)
        return std::nullopt;
    return line.substr(v, end - v);
}

/// A double from a field, NaN-free: returns nullopt when absent or unparsable.
std::optional<double> number(std::string_view line, std::string_view key) {
    const auto text = field(line, key);
    if (!text)
        return std::nullopt;
    // from_chars for doubles is not in every libstdc++ we build against, and
    // the values are short and plain, so strtod on a bounded copy is fine.
    const std::string copy(*text);
    char* end = nullptr;
    const double v = std::strtod(copy.c_str(), &end);
    if (end == copy.c_str())
        return std::nullopt;
    return v;
}

/// "00:00:01,234" -> 1234 ms. nullopt when it is not a timestamp.
std::optional<int64_t> timecode(std::string_view s) {
    int h = 0, m = 0, sec = 0, ms = 0;
    if (s.size() < 12)
        return std::nullopt;
    const auto num = [&](std::size_t at, std::size_t len, int& out) {
        int v = 0;
        for (std::size_t i = at; i < at + len; ++i) {
            if (i >= s.size() || !std::isdigit(static_cast<unsigned char>(s[i])))
                return false;
            v = v * 10 + (s[i] - '0');
        }
        out = v;
        return true;
    };
    if (!num(0, 2, h) || !num(3, 2, m) || !num(6, 2, sec) || !num(9, 3, ms))
        return std::nullopt;
    return (static_cast<int64_t>(h) * 3600 + m * 60 + sec) * 1000 + ms;
}

} // namespace

std::string DjiSrtPathFor(const std::string& clipPath) {
    std::error_code ec;
    for (const char* ext : {".SRT", ".srt"}) {
        fs::path p(clipPath);
        p.replace_extension(ext);
        if (fs::exists(p, ec))
            return p.string();
    }
    return {};
}

std::optional<DjiSrtInfo> ReadDjiSrt(const std::string& path) {
    std::ifstream in(path);
    if (!in)
        return std::nullopt;

    DjiSrtInfo out;
    std::string line;
    int64_t entryMs = 0;   // start time of the entry being read
    bool haveTime = false;

    while (std::getline(in, line)) {
        // Strip a trailing CR from a file written on Windows.
        if (!line.empty() && line.back() == '\r')
            line.pop_back();
        const std::string_view sv(line);

        // "00:00:00,000 --> 00:00:00,010": the entry's time.
        if (const std::size_t arrow = sv.find(" --> "); arrow != std::string_view::npos) {
            if (const auto t = timecode(sv.substr(0, arrow))) {
                entryMs = *t;
                haveTime = true;
            }
            continue;
        }

        // The telemetry line: the one carrying bracketed fields.
        if (sv.find('[') == std::string_view::npos)
            continue;

        const auto lat = number(sv, "latitude");
        const auto lon = number(sv, "longitude");
        if (lat && lon && haveTime) {
            // DJI writes 0/0 before the aircraft has a fix; those are not
            // positions and would drag a map to the Gulf of Guinea.
            if (*lat != 0.0 || *lon != 0.0) {
                insv::GpsFix fix;
                fix.timeMs = entryMs;
                fix.latitude = *lat;
                fix.longitude = *lon;
                // Absolute altitude is above sea level, which is what a map
                // wants; relative is above the take-off point.
                if (const auto abs = number(sv, "abs_alt"))
                    fix.altitudeM = *abs;
                else if (const auto rel = number(sv, "rel_alt"))
                    fix.altitudeM = *rel;
                // Only keep a fix when it moved or time advanced: DJI repeats
                // the same position for every frame at 100 fps, which would
                // be 8552 identical fixes for one 85 s clip.
                if (out.gps.fixes.empty() || out.gps.fixes.back().latitude != fix.latitude ||
                    out.gps.fixes.back().longitude != fix.longitude ||
                    out.gps.fixes.back().altitudeM != fix.altitudeM)
                    out.gps.fixes.push_back(fix);
            }
        }

        if (out.colourMode.empty())
            if (const auto md = field(sv, "color_md"))
                out.colourMode = std::string(*md);
        if (out.colourTempK == 0.0)
            if (const auto ct = number(sv, "ct"))
                out.colourTempK = *ct;
        if (out.iso == 0)
            if (const auto iso = number(sv, "iso"))
                out.iso = static_cast<int>(*iso);
        out.any = true;
    }

    if (!out.any)
        return std::nullopt;
    return out;
}

} // namespace camera

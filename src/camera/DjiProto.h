#pragma once

#include <cstdint>
#include <cstring>
#include <optional>
#include <span>
#include <string_view>

// Just enough protobuf wire format to read DJI's metadata records.
//
// DJI's .OSV carries its lens calibration and attitude as protobuf messages in
// the `djmd` tracks. We only ever read them, and only a handful of field
// numbers, so this is a scanner rather than a protobuf library: no schema, no
// code generation, no dependency. Unknown fields are skipped by wire type,
// which is exactly what protobuf's format is designed to allow -- so a
// firmware that adds fields keeps working.
//
// Everything here is bounds-checked and non-throwing. A truncated or corrupt
// record ends the scan; it never runs off the buffer.

namespace camera::proto {

enum class Wire : std::uint8_t {
    Varint = 0,          ///< base-128 varint
    Fixed64 = 1,         ///< 8 bytes, little endian
    LengthDelimited = 2, ///< length-prefixed bytes (string, bytes, message)
    StartGroup = 3,      ///< deprecated in protobuf; DJI does not use it
    EndGroup = 4,
    Fixed32 = 5,         ///< 4 bytes, little endian
};

/// One field as it appears on the wire. `data` is a view into the caller's
/// buffer for LengthDelimited; nothing is copied.
struct Field {
    std::uint32_t number = 0;
    Wire wire = Wire::Varint;
    std::uint64_t varint = 0;          ///< Varint / Fixed32 / Fixed64 raw bits
    std::span<const std::uint8_t> data; ///< LengthDelimited payload

    /// The field as a float. DJI writes most numbers as fixed32 floats, but
    /// some (notably DewarpParams.width/height) appear as plain varints in
    /// some firmwares, so both are accepted.
    [[nodiscard]] std::optional<float> asFloat() const noexcept {
        if (wire == Wire::Fixed32) {
            float f = 0.0f;
            const std::uint32_t bits = static_cast<std::uint32_t>(varint);
            std::memcpy(&f, &bits, sizeof f);
            return f;
        }
        if (wire == Wire::Fixed64) {
            double d = 0.0;
            std::memcpy(&d, &varint, sizeof d);
            return static_cast<float>(d);
        }
        if (wire == Wire::Varint)
            return static_cast<float>(varint);
        return std::nullopt;
    }

    /// The field as a double, for the few 64-bit values DJI writes.
    [[nodiscard]] std::optional<double> asDouble() const noexcept {
        if (wire == Wire::Fixed64) {
            double d = 0.0;
            std::memcpy(&d, &varint, sizeof d);
            return d;
        }
        if (const auto f = asFloat())
            return static_cast<double>(*f);
        return std::nullopt;
    }

    [[nodiscard]] bool isMessage() const noexcept { return wire == Wire::LengthDelimited; }
};

/// Walks the fields of one protobuf message.
///
///     Scanner s(bytes);
///     Field f;
///     while (s.next(f)) {
///         if (f.number == 3 && f.isMessage()) { Scanner inner(f.data); ... }
///     }
///
/// next() returns false at the end of the buffer and on any malformed field,
/// so a corrupt record simply stops yielding rather than misreading.
class Scanner {
public:
    explicit Scanner(std::span<const std::uint8_t> bytes) noexcept : m_at(bytes) {}

    bool next(Field& out) noexcept {
        std::uint64_t tag = 0;
        if (!varint(tag))
            return false;
        const std::uint32_t number = static_cast<std::uint32_t>(tag >> 3);
        const auto wire = static_cast<Wire>(tag & 0x7);
        if (number == 0)
            return false; // field 0 is not valid: corrupt

        out = Field{};
        out.number = number;
        out.wire = wire;
        switch (wire) {
        case Wire::Varint:
            return varint(out.varint);
        case Wire::Fixed64:
            return fixed(8, out.varint);
        case Wire::Fixed32:
            return fixed(4, out.varint);
        case Wire::LengthDelimited: {
            std::uint64_t len = 0;
            if (!varint(len) || len > m_at.size() - m_pos)
                return false;
            out.data = m_at.subspan(m_pos, static_cast<std::size_t>(len));
            m_pos += static_cast<std::size_t>(len);
            return true;
        }
        case Wire::StartGroup:
        case Wire::EndGroup:
        default:
            return false; // not used by DJI; refuse rather than guess
        }
    }

private:
    bool varint(std::uint64_t& out) noexcept {
        out = 0;
        for (int shift = 0; shift < 64; shift += 7) {
            if (m_pos >= m_at.size())
                return false;
            const std::uint8_t b = m_at[m_pos++];
            out |= static_cast<std::uint64_t>(b & 0x7F) << shift;
            if ((b & 0x80) == 0)
                return true;
        }
        return false; // more than 10 bytes: malformed
    }

    bool fixed(std::size_t n, std::uint64_t& out) noexcept {
        if (m_at.size() - m_pos < n)
            return false;
        out = 0;
        for (std::size_t i = 0; i < n; ++i)
            out |= static_cast<std::uint64_t>(m_at[m_pos + i]) << (8 * i);
        m_pos += n;
        return true;
    }

    std::span<const std::uint8_t> m_at;
    std::size_t m_pos = 0;
};

} // namespace camera::proto

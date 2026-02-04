#pragma once

#include <datapod/datapod.hpp>
#include <wirebit/wirebit.hpp>

#include <cmath>

#include <nonsens/pods/gnss.hpp>

namespace nonsens::codec::gnss {

    namespace detail {

        struct J1939Id {
            uint8_t priority{0};
            uint8_t dp{0};
            uint8_t pf{0};
            uint8_t ps{0};
            uint8_t sa{0};
        };

        inline bool is_extended(wirebit::can_frame const &frame) {
            return (frame.can_id & wirebit::CAN_EFF_FLAG_V) != 0;
        }

        inline uint32_t raw_id29(wirebit::can_frame const &frame) { return (frame.can_id & wirebit::CAN_EFF_MASK_V); }

        inline J1939Id parse_j1939_id(uint32_t id29) {
            J1939Id out{};
            out.priority = static_cast<uint8_t>((id29 >> 26) & 0x7);
            out.dp = static_cast<uint8_t>((id29 >> 24) & 0x1);
            out.pf = static_cast<uint8_t>((id29 >> 16) & 0xFF);
            out.ps = static_cast<uint8_t>((id29 >> 8) & 0xFF);
            out.sa = static_cast<uint8_t>((id29 >> 0) & 0xFF);
            return out;
        }

        inline uint32_t pgn_from_id(J1939Id const &id) {
            // For PDU2 (PF >= 240) PGN includes PS (group extension). Our GNSS PGNs are 0xFE**.
            return (static_cast<uint32_t>(id.dp) << 16) | (static_cast<uint32_t>(id.pf) << 8) |
                   (static_cast<uint32_t>(id.ps));
        }

        inline uint32_t u32_le(uint8_t const *p) {
            return (static_cast<uint32_t>(p[0]) << 0) | (static_cast<uint32_t>(p[1]) << 8) |
                   (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
        }

        inline uint16_t u16_le(uint8_t const *p) {
            return static_cast<uint16_t>((static_cast<uint16_t>(p[0]) << 0) | (static_cast<uint16_t>(p[1]) << 8));
        }

        inline void update_velocity_from_track_speed(nonsens::pod::Gnss &out) {
            double tr = static_cast<double>(out.track_deg) * (M_PI / 180.0);
            out.velocity_neu[0] = static_cast<double>(out.speed_mps) * std::cos(tr);
            out.velocity_neu[1] = static_cast<double>(out.speed_mps) * std::sin(tr);
            out.velocity_neu[2] = 0.0;
        }

    } // namespace detail

    /// Update a GNSS pod from a single SAE J1939 CAN frame.
    ///
    /// Supported PGNs:
    /// - 0xFEF3 (65267): Vehicle Position
    /// - 0xFEE8 (65256): Vehicle Direction/Speed
    ///
    /// Unsupported frames are ignored.
    inline dp::VoidRes j1939_to_pod(wirebit::can_frame const &frame, nonsens::pod::Gnss &out) {
        if (!detail::is_extended(frame)) {
            return dp::VoidRes::ok();
        }
        if (frame.can_dlc != 8) {
            return dp::VoidRes::ok();
        }

        auto id = detail::parse_j1939_id(detail::raw_id29(frame));
        uint32_t pgn = detail::pgn_from_id(id);

        // Vehicle Position (Latitude/Longitude)
        if (pgn == 0xFEF3) {
            uint32_t lat_raw = detail::u32_le(frame.data + 0);
            uint32_t lon_raw = detail::u32_le(frame.data + 4);

            // Value = (raw * 1e-7) - 210
            double lat = (static_cast<double>(lat_raw) * 1e-7) - 210.0;
            double lon = (static_cast<double>(lon_raw) * 1e-7) - 210.0;

            out.latitude = lat;
            out.longitude = lon;

            // Mark as having a fix (J1939 frame does not carry fix quality here).
            out.status.status = nonsens::pod::Gnss::NavSatStatus::STATUS_FIX;
            out.status.service = nonsens::pod::Gnss::NavSatStatus::SERVICE_GPS;
            if (out.rtk_status == nonsens::pod::Gnss::RtkStatus::NO_FIX) {
                out.rtk_status = nonsens::pod::Gnss::RtkStatus::SINGLE;
            }
            return dp::VoidRes::ok();
        }

        // Vehicle Direction/Speed
        if (pgn == 0xFEE8) {
            uint16_t bearing_raw = detail::u16_le(frame.data + 0);
            uint16_t speed_raw = detail::u16_le(frame.data + 2);

            // Bearing: 1/128 deg
            if (bearing_raw != 0xFFFF) {
                out.track_deg = static_cast<double>(bearing_raw) / 128.0;
            }

            // Speed: 1/256 km/h -> m/s
            if (speed_raw != 0xFFFF) {
                double kmh = static_cast<double>(speed_raw) / 256.0;
                out.speed_mps = kmh / 3.6;
            }

            detail::update_velocity_from_track_speed(out);

            out.status.status = nonsens::pod::Gnss::NavSatStatus::STATUS_FIX;
            out.status.service = nonsens::pod::Gnss::NavSatStatus::SERVICE_GPS;
            if (out.rtk_status == nonsens::pod::Gnss::RtkStatus::NO_FIX) {
                out.rtk_status = nonsens::pod::Gnss::RtkStatus::SINGLE;
            }
            return dp::VoidRes::ok();
        }

        return dp::VoidRes::ok();
    }

} // namespace nonsens::codec::gnss

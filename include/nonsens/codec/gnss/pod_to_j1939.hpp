#pragma once

#include <algorithm>
#include <cmath>

#include <datapod/datapod.hpp>
#include <wirebit/wirebit.hpp>

#include <nonsens/pods/gnss.hpp>

namespace nonsens::codec::gnss {

    namespace detail_j1939 {

        inline double speed_mps_from_pod(nonsens::pod::Gnss const &in) {
            if (in.speed_mps > 0.0) {
                return static_cast<double>(in.speed_mps);
            }
            double vn = static_cast<double>(in.velocity_neu[0]);
            double ve = static_cast<double>(in.velocity_neu[1]);
            return std::sqrt(vn * vn + ve * ve);
        }

        inline double track_deg_from_pod(nonsens::pod::Gnss const &in) {
            if (in.track_deg != 0.0) {
                return static_cast<double>(in.track_deg);
            }
            double vn = static_cast<double>(in.velocity_neu[0]);
            double ve = static_cast<double>(in.velocity_neu[1]);
            if (vn == 0.0 && ve == 0.0) {
                return 0.0;
            }
            double tr = std::atan2(ve, vn) * (180.0 / M_PI);
            if (tr < 0.0)
                tr += 360.0;
            return tr;
        }

    } // namespace detail_j1939

    /// SAE J1939 Parameter Group Numbers for GNSS
    struct J1939Pgn {
        static constexpr uint32_t VEHICLE_POSITION = 0xFEF3;        // 65267 - Vehicle Position
        static constexpr uint32_t VEHICLE_DIRECTION_SPEED = 0xFEE8; // 65256 - Vehicle Direction/Speed
    };

    /// SAE J1939 priority defaults
    inline constexpr uint8_t J1939_DEFAULT_PRIORITY = 6;
    inline constexpr uint8_t J1939_DEFAULT_SOURCE_ADDRESS = 0x80;

    /// Build a 29-bit SAE J1939 CAN ID
    ///
    /// bits 26-28: priority (3 bits)
    /// bit 25: reserved (0)
    /// bit 24: data page (0)
    /// bits 16-23: PF (PDU Format)
    /// bits 8-15: PS (PDU Specific / Group Extension)
    /// bits 0-7: source address
    inline uint32_t j1939_build_can_id(uint32_t pgn, uint8_t sa, uint8_t priority = J1939_DEFAULT_PRIORITY) {
        uint8_t pf = static_cast<uint8_t>((pgn >> 8) & 0xFF);
        uint8_t ps = static_cast<uint8_t>(pgn & 0xFF);
        uint8_t dp = 0; // Data page 0

        return ((static_cast<uint32_t>(priority) & 0x7) << 26) | (static_cast<uint32_t>(dp) << 24) |
               (static_cast<uint32_t>(pf) << 16) | (static_cast<uint32_t>(ps) << 8) |
               (static_cast<uint32_t>(sa) & 0xFF);
    }

    /// Encode SAE J1939 Vehicle Position (PGN 0xFEF3 / 65267)
    ///
    /// Per J1939-71:
    /// Byte 1-4: Latitude  (resolution: 1e-7 deg, offset: -210 deg)
    /// Byte 5-8: Longitude (resolution: 1e-7 deg, offset: -210 deg)
    inline dp::Res<wirebit::can_frame> encode_vehicle_position(double lat_deg, double lon_deg,
                                                               uint8_t sa = J1939_DEFAULT_SOURCE_ADDRESS,
                                                               uint8_t priority = J1939_DEFAULT_PRIORITY) {

        // Apply offset and scale
        // Value = (raw * resolution) + offset
        // raw = (Value - offset) / resolution
        double lat_raw_d = (lat_deg + 210.0) / 1e-7;
        double lon_raw_d = (lon_deg + 210.0) / 1e-7;

        // Clamp to uint32 range
        uint32_t lat_raw = static_cast<uint32_t>(std::max(0.0, std::min(4294967295.0, lat_raw_d)));
        uint32_t lon_raw = static_cast<uint32_t>(std::max(0.0, std::min(4294967295.0, lon_raw_d)));

        // Pack as little-endian
        uint8_t data[8];
        data[0] = lat_raw & 0xFF;
        data[1] = (lat_raw >> 8) & 0xFF;
        data[2] = (lat_raw >> 16) & 0xFF;
        data[3] = (lat_raw >> 24) & 0xFF;
        data[4] = lon_raw & 0xFF;
        data[5] = (lon_raw >> 8) & 0xFF;
        data[6] = (lon_raw >> 16) & 0xFF;
        data[7] = (lon_raw >> 24) & 0xFF;

        uint32_t can_id = j1939_build_can_id(J1939Pgn::VEHICLE_POSITION, sa, priority);
        return dp::Res<wirebit::can_frame>::ok(wirebit::CanEndpoint::make_ext_frame(can_id, data, 8));
    }

    /// Encode SAE J1939 Vehicle Direction/Speed (PGN 0xFEE8 / 65256)
    ///
    /// Byte 1-2: Compass Bearing (resolution: 1/128 deg, 0-360 deg)
    /// Byte 3-4: Navigation-based Vehicle Speed (resolution: 1/256 km/h)
    /// Byte 5-6: Pitch (resolution: 1/128 deg, offset: -200 deg) - 0xFFFF = not available
    /// Byte 7-8: Altitude (resolution: 0.125 m, offset: -2500 m) - 0xFFFF = not available
    inline dp::Res<wirebit::can_frame> encode_vehicle_direction_speed(double cog_deg, double sog_ms,
                                                                      uint8_t sa = J1939_DEFAULT_SOURCE_ADDRESS,
                                                                      uint8_t priority = J1939_DEFAULT_PRIORITY) {

        // Compass Bearing: resolution 1/128 deg
        uint16_t bearing_raw = static_cast<uint16_t>(std::max(0.0, std::min(360.0 * 128.0, cog_deg * 128.0)));

        // Speed: resolution 1/256 km/h, convert m/s to km/h
        double sog_kmh = sog_ms * 3.6;
        uint16_t speed_raw = static_cast<uint16_t>(std::max(0.0, std::min(65535.0, sog_kmh * 256.0)));

        // Pitch and Altitude: not available
        uint16_t pitch_raw = 0xFFFF;
        uint16_t altitude_raw = 0xFFFF;

        // Pack as little-endian
        uint8_t data[8];
        data[0] = bearing_raw & 0xFF;
        data[1] = (bearing_raw >> 8) & 0xFF;
        data[2] = speed_raw & 0xFF;
        data[3] = (speed_raw >> 8) & 0xFF;
        data[4] = pitch_raw & 0xFF;
        data[5] = (pitch_raw >> 8) & 0xFF;
        data[6] = altitude_raw & 0xFF;
        data[7] = (altitude_raw >> 8) & 0xFF;

        uint32_t can_id = j1939_build_can_id(J1939Pgn::VEHICLE_DIRECTION_SPEED, sa, priority);
        return dp::Res<wirebit::can_frame>::ok(wirebit::CanEndpoint::make_ext_frame(can_id, data, 8));
    }

    /// Convert Gnss pod to J1939 CAN frames
    ///
    /// Returns a vector containing up to 2 CAN frames:
    /// - PGN 0xFEF3: Vehicle Position (latitude, longitude)
    /// - PGN 0xFEE8: Vehicle Direction/Speed (course over ground, speed over ground)
    inline dp::Res<dp::Vector<wirebit::can_frame>> pod_to_j1939(nonsens::pod::Gnss const &in,
                                                                uint8_t sa = J1939_DEFAULT_SOURCE_ADDRESS,
                                                                uint8_t priority = J1939_DEFAULT_PRIORITY) {

        dp::Vector<wirebit::can_frame> frames;
        frames.reserve(2);

        // Match common GNSS bridges: if we have a fix, publish both frames.
        if (in.status.status >= in.status.STATUS_FIX) {
            auto pos_result = encode_vehicle_position(in.latitude, in.longitude, sa, priority);
            if (!pos_result.is_ok()) {
                return dp::Res<dp::Vector<wirebit::can_frame>>::err(pos_result.error());
            }
            frames.push_back(pos_result.value());

            double track_deg = detail_j1939::track_deg_from_pod(in);
            double speed_mps = detail_j1939::speed_mps_from_pod(in);
            auto dir_result = encode_vehicle_direction_speed(track_deg, speed_mps, sa, priority);
            if (!dir_result.is_ok()) {
                return dp::Res<dp::Vector<wirebit::can_frame>>::err(dir_result.error());
            }
            frames.push_back(dir_result.value());
        }

        return dp::Res<dp::Vector<wirebit::can_frame>>::ok(std::move(frames));
    }

} // namespace nonsens::codec::gnss

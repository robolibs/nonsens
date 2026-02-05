#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>

#include <datapod/datapod.hpp>
#include <wirebit/wirebit.hpp>

#include <nonsens/pods/gnss.hpp>

namespace nonsens::codec::gnss {

    struct Nmea2000Pgn {
        static constexpr uint32_t POSITION_RAPID = 129025; // 0x1F801 - Position, Rapid Update
        static constexpr uint32_t COG_SOG_RAPID = 129026;  // 0x1F802 - COG & SOG, Rapid Update
        static constexpr uint32_t GNSS_POSITION = 129029;  // 0x1F805 - GNSS Position Data
    };

    inline constexpr uint8_t NMEA2000_DEFAULT_SA = 0x80;
    inline constexpr uint8_t NMEA2000_PRIORITY_POS_RAPID = 2;
    inline constexpr uint8_t NMEA2000_PRIORITY_COG_SOG = 2;
    inline constexpr uint8_t NMEA2000_PRIORITY_GNSS = 3;

    inline uint32_t nmea2000_build_can_id(uint32_t pgn, uint8_t sa, uint8_t priority) {
        uint8_t pf = static_cast<uint8_t>((pgn >> 8) & 0xFF);
        uint8_t ps = static_cast<uint8_t>(pgn & 0xFF);
        uint8_t dp = static_cast<uint8_t>((pgn >> 16) & 0x1);

        // For our GNSS-related PGNs we only need the PDU2/broadcast case (PF >= 240).
        // Keep the PS field as given (group extension) and do not force DA=0xFF here.
        return ((static_cast<uint32_t>(priority) & 0x7) << 26) | (static_cast<uint32_t>(dp) << 24) |
               (static_cast<uint32_t>(pf) << 16) | (static_cast<uint32_t>(ps) << 8) |
               (static_cast<uint32_t>(sa) & 0xFF);
    }

    namespace detail_n2k {

        inline double speed_mps_from_pod(nonsens::pod::Gnss const &in) { return static_cast<double>(in.speed_mps); }

        inline double track_deg_from_pod(nonsens::pod::Gnss const &in) { return static_cast<double>(in.track_deg); }

        inline void pack_i32_le(uint8_t *out, int32_t v) {
            out[0] = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 0) & 0xFF);
            out[1] = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 8) & 0xFF);
            out[2] = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 16) & 0xFF);
            out[3] = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 24) & 0xFF);
        }

        inline void pack_u16_le(uint8_t *out, uint16_t v) {
            out[0] = static_cast<uint8_t>(v & 0xFF);
            out[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
        }

        inline void pack_u32_le(uint8_t *out, uint32_t v) {
            out[0] = static_cast<uint8_t>((v >> 0) & 0xFF);
            out[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
            out[2] = static_cast<uint8_t>((v >> 16) & 0xFF);
            out[3] = static_cast<uint8_t>((v >> 24) & 0xFF);
        }

        inline void pack_i64_le(uint8_t *out, int64_t v) {
            uint64_t u = static_cast<uint64_t>(v);
            out[0] = static_cast<uint8_t>((u >> 0) & 0xFF);
            out[1] = static_cast<uint8_t>((u >> 8) & 0xFF);
            out[2] = static_cast<uint8_t>((u >> 16) & 0xFF);
            out[3] = static_cast<uint8_t>((u >> 24) & 0xFF);
            out[4] = static_cast<uint8_t>((u >> 32) & 0xFF);
            out[5] = static_cast<uint8_t>((u >> 40) & 0xFF);
            out[6] = static_cast<uint8_t>((u >> 48) & 0xFF);
            out[7] = static_cast<uint8_t>((u >> 56) & 0xFF);
        }

        inline uint16_t clamp_u16(double v) {
            if (v < 0.0)
                return 0;
            if (v > 65535.0)
                return 65535;
            return static_cast<uint16_t>(v);
        }

        inline uint16_t days_since_1970_utc() {
            using clock = std::chrono::system_clock;
            auto now = clock::now();
            auto secs = std::chrono::time_point_cast<std::chrono::seconds>(now);
            auto epoch = secs.time_since_epoch();
            auto days = std::chrono::duration_cast<std::chrono::hours>(epoch).count() / 24;
            if (days < 0)
                days = 0;
            if (days > 65535)
                days = 65535;
            return static_cast<uint16_t>(days);
        }

        inline uint32_t time_since_midnight_0_0001s_utc() {
            using clock = std::chrono::system_clock;
            auto now = clock::now();
            auto secs = std::chrono::time_point_cast<std::chrono::seconds>(now);
            auto sub = now - secs;

            std::time_t tt = clock::to_time_t(secs);
            std::tm tm{};
            gmtime_r(&tt, &tm);

            double s = static_cast<double>(tm.tm_hour * 3600 + tm.tm_min * 60 + tm.tm_sec);
            s += std::chrono::duration<double>(sub).count();
            double scaled = s * 10000.0;
            if (scaled < 0.0)
                scaled = 0.0;
            if (scaled > 4294967295.0)
                scaled = 4294967295.0;
            return static_cast<uint32_t>(scaled);
        }

        inline dp::Res<wirebit::can_frame> encode_position_rapid(double lat_deg, double lon_deg, uint8_t sa) {
            // Quantized at 1e-7 degrees by the PGN definition.
            int64_t lat64 = static_cast<int64_t>(std::llround(lat_deg * 1e7));
            int64_t lon64 = static_cast<int64_t>(std::llround(lon_deg * 1e7));
            int32_t lat_i =
                static_cast<int32_t>(std::max<int64_t>(-2147483648LL, std::min<int64_t>(2147483647LL, lat64)));
            int32_t lon_i =
                static_cast<int32_t>(std::max<int64_t>(-2147483648LL, std::min<int64_t>(2147483647LL, lon64)));

            uint8_t data[8];
            pack_i32_le(data + 0, lat_i);
            pack_i32_le(data + 4, lon_i);

            uint32_t can_id = nmea2000_build_can_id(Nmea2000Pgn::POSITION_RAPID, sa, NMEA2000_PRIORITY_POS_RAPID);
            return dp::Res<wirebit::can_frame>::ok(wirebit::CanEndpoint::make_ext_frame(can_id, data, 8));
        }

        inline dp::Res<wirebit::can_frame> encode_cog_sog_rapid(double cog_deg, double sog_ms, uint8_t sid,
                                                                uint8_t sa) {
            // Byte 1: SID
            // Byte 2: COG reference (2 bits) + reserved bits 2-7 set to 1
            uint8_t cog_reference = 0; // 0=True
            uint8_t ref_byte = static_cast<uint8_t>((cog_reference & 0x03) | 0xFC);

            double cog_rad = cog_deg * (M_PI / 180.0);
            uint16_t cog_scaled = clamp_u16(cog_rad / 0.0001);
            uint16_t sog_scaled = clamp_u16(sog_ms / 0.01);

            uint8_t data[8];
            data[0] = sid;
            data[1] = ref_byte;
            pack_u16_le(data + 2, cog_scaled);
            pack_u16_le(data + 4, sog_scaled);
            data[6] = 0xFF;
            data[7] = 0xFF;

            uint32_t can_id = nmea2000_build_can_id(Nmea2000Pgn::COG_SOG_RAPID, sa, NMEA2000_PRIORITY_COG_SOG);
            return dp::Res<wirebit::can_frame>::ok(wirebit::CanEndpoint::make_ext_frame(can_id, data, 8));
        }

        inline dp::Res<dp::Vector<uint8_t>> encode_gnss_position_payload(double lat_deg, double lon_deg,
                                                                         double altitude_m, uint8_t sid) {
            // Mirrors the provided Python layout.
            dp::Vector<uint8_t> payload;
            payload.resize(47);

            uint16_t days = days_since_1970_utc();
            uint32_t time_scaled = time_since_midnight_0_0001s_utc();

            int64_t lat_scaled = static_cast<int64_t>(std::llround(lat_deg * 1e16));
            int64_t lon_scaled = static_cast<int64_t>(std::llround(lon_deg * 1e16));
            int64_t alt_scaled = static_cast<int64_t>(std::llround(altitude_m * 1e6));

            // GNSS fields (packed)
            uint8_t gnss_type = 0; // GPS
            uint8_t method = 1;    // GNSS fix
            uint8_t integrity = 1; // Safe
            uint8_t reserved6 = 0x3F;

            uint8_t byte_gnss_method = static_cast<uint8_t>((gnss_type & 0x0F) | ((method & 0x0F) << 4));
            uint8_t byte_integrity_reserved = static_cast<uint8_t>((integrity & 0x03) | ((reserved6 & 0x3F) << 2));

            uint8_t num_svs = 8;
            uint16_t hdop = 100; // 1.00
            uint16_t pdop = 200; // 2.00
            int32_t geoidal_sep = 0;
            uint8_t ref_stations = 1;

            size_t o = 0;
            payload[o++] = sid;
            pack_u16_le(payload.data() + o, days);
            o += 2;
            pack_u32_le(payload.data() + o, time_scaled);
            o += 4;
            pack_i64_le(payload.data() + o, lat_scaled);
            o += 8;
            pack_i64_le(payload.data() + o, lon_scaled);
            o += 8;
            pack_i64_le(payload.data() + o, alt_scaled);
            o += 8;
            payload[o++] = byte_gnss_method;
            payload[o++] = byte_integrity_reserved;
            payload[o++] = num_svs;
            pack_u16_le(payload.data() + o, hdop);
            o += 2;
            pack_u16_le(payload.data() + o, pdop);
            o += 2;
            // int32 le
            pack_u32_le(payload.data() + o, static_cast<uint32_t>(geoidal_sep));
            o += 4;
            payload[o++] = ref_stations;

            // Reference station data
            uint16_t ref_type_id = 0xFFFF;
            uint16_t ref_age = 0xFFFF;
            pack_u16_le(payload.data() + o, ref_type_id);
            o += 2;
            pack_u16_le(payload.data() + o, ref_age);
            o += 2;

            if (o != payload.size()) {
                return dp::Res<dp::Vector<uint8_t>>::err(
                    dp::Error::invalid_argument("nmea2000 gnss payload size mismatch"));
            }
            return dp::Res<dp::Vector<uint8_t>>::ok(std::move(payload));
        }

        inline dp::Res<dp::Vector<wirebit::can_frame>>
        encode_fast_packet(uint32_t can_id_noeff, dp::Vector<uint8_t> const &data, uint8_t &sequence_counter_io) {
            dp::Vector<wirebit::can_frame> frames;

            uint8_t frame_counter = static_cast<uint8_t>(sequence_counter_io & 0x1F);
            sequence_counter_io = static_cast<uint8_t>((sequence_counter_io + 1) & 0x1F);

            size_t total_len = data.size();
            if (total_len == 0 || total_len > 223) {
                return dp::Res<dp::Vector<wirebit::can_frame>>::err(
                    dp::Error::invalid_argument("nmea2000 fast packet length out of range"));
            }

            // Frame 0: [counter, total_len, data0..data5]
            uint8_t f0[8];
            f0[0] = frame_counter;
            f0[1] = static_cast<uint8_t>(total_len);
            for (int i = 0; i < 6; ++i) {
                f0[2 + i] = (i < static_cast<int>(total_len)) ? data[static_cast<size_t>(i)] : 0xFF;
            }
            frames.push_back(wirebit::CanEndpoint::make_ext_frame(can_id_noeff, f0, 8));

            size_t offset = 6;
            uint8_t frame_num = 1;
            while (offset < total_len) {
                uint8_t fx[8];
                fx[0] = static_cast<uint8_t>(frame_counter | (frame_num << 5));
                size_t remaining = total_len - offset;
                size_t chunk = std::min<size_t>(7, remaining);
                for (size_t i = 0; i < 7; ++i) {
                    fx[1 + i] = (i < chunk) ? data[offset + i] : 0xFF;
                }
                frames.push_back(wirebit::CanEndpoint::make_ext_frame(can_id_noeff, fx, 8));
                offset += chunk;
                ++frame_num;
            }

            return dp::Res<dp::Vector<wirebit::can_frame>>::ok(std::move(frames));
        }

    } // namespace detail_n2k

    /// Convert Gnss pod to NMEA 2000 CAN frames.
    ///
    /// Emits:
    /// - PGN 129025 (Position Rapid Update)
    /// - PGN 129026 (COG & SOG Rapid Update)
    /// - PGN 129029 (GNSS Position Data) as Fast Packet
    inline dp::Res<dp::Vector<wirebit::can_frame>> pod_to_nmea2000(nonsens::pod::Gnss const &in, uint8_t sid,
                                                                   uint8_t &fast_packet_seq_io,
                                                                   bool include_gnss_position,
                                                                   uint8_t sa = NMEA2000_DEFAULT_SA) {
        dp::Vector<wirebit::can_frame> frames;

        if (in.status.status < in.status.STATUS_FIX) {
            return dp::Res<dp::Vector<wirebit::can_frame>>::ok(std::move(frames));
        }

        auto pos = detail_n2k::encode_position_rapid(in.latitude, in.longitude, sa);
        if (!pos.is_ok())
            return dp::Res<dp::Vector<wirebit::can_frame>>::err(pos.error());
        frames.push_back(pos.value());

        double track_deg = detail_n2k::track_deg_from_pod(in);
        double speed_mps = detail_n2k::speed_mps_from_pod(in);
        auto cog = detail_n2k::encode_cog_sog_rapid(track_deg, speed_mps, sid, sa);
        if (!cog.is_ok())
            return dp::Res<dp::Vector<wirebit::can_frame>>::err(cog.error());
        frames.push_back(cog.value());

        if (include_gnss_position) {
            auto payload = detail_n2k::encode_gnss_position_payload(in.latitude, in.longitude, in.altitude, sid);
            if (!payload.is_ok())
                return dp::Res<dp::Vector<wirebit::can_frame>>::err(payload.error());

            uint32_t can_id = nmea2000_build_can_id(Nmea2000Pgn::GNSS_POSITION, sa, NMEA2000_PRIORITY_GNSS);
            auto fp = detail_n2k::encode_fast_packet(can_id, payload.value(), fast_packet_seq_io);
            if (!fp.is_ok())
                return dp::Res<dp::Vector<wirebit::can_frame>>::err(fp.error());

            for (auto const &f : fp.value()) {
                frames.push_back(f);
            }
        }

        return dp::Res<dp::Vector<wirebit::can_frame>>::ok(std::move(frames));
    }

} // namespace nonsens::codec::gnss

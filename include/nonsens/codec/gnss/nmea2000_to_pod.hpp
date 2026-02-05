#pragma once

#include <cmath>
#include <cstdint>
#include <unordered_map>

#include <datapod/datapod.hpp>
#include <wirebit/wirebit.hpp>

#include <nonsens/pods/gnss.hpp>

namespace nonsens::codec::gnss {

    namespace detail_n2k {

        struct Can29Id {
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

        inline Can29Id parse_id29(uint32_t id29) {
            Can29Id out{};
            out.priority = static_cast<uint8_t>((id29 >> 26) & 0x7);
            out.dp = static_cast<uint8_t>((id29 >> 24) & 0x1);
            out.pf = static_cast<uint8_t>((id29 >> 16) & 0xFF);
            out.ps = static_cast<uint8_t>((id29 >> 8) & 0xFF);
            out.sa = static_cast<uint8_t>((id29 >> 0) & 0xFF);
            return out;
        }

        inline uint32_t pgn_from_id(Can29Id const &id) {
            // PDU2: PF >= 240, PGN includes PS.
            // PDU1: PF < 240, PGN does not include destination (PS is DA), but we don't use those PGNs here.
            uint32_t ge = (id.pf >= 240) ? static_cast<uint32_t>(id.ps) : 0U;
            return (static_cast<uint32_t>(id.dp) << 16) | (static_cast<uint32_t>(id.pf) << 8) | ge;
        }

        inline int32_t i32_le(uint8_t const *p) {
            uint32_t u = (static_cast<uint32_t>(p[0]) << 0) | (static_cast<uint32_t>(p[1]) << 8) |
                         (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
            return static_cast<int32_t>(u);
        }

        inline uint16_t u16_le(uint8_t const *p) {
            return static_cast<uint16_t>((static_cast<uint16_t>(p[0]) << 0) | (static_cast<uint16_t>(p[1]) << 8));
        }

        inline int64_t i64_le(uint8_t const *p) {
            uint64_t u = (static_cast<uint64_t>(p[0]) << 0) | (static_cast<uint64_t>(p[1]) << 8) |
                         (static_cast<uint64_t>(p[2]) << 16) | (static_cast<uint64_t>(p[3]) << 24) |
                         (static_cast<uint64_t>(p[4]) << 32) | (static_cast<uint64_t>(p[5]) << 40) |
                         (static_cast<uint64_t>(p[6]) << 48) | (static_cast<uint64_t>(p[7]) << 56);
            return static_cast<int64_t>(u);
        }

        inline void set_fix(nonsens::pod::Gnss &out) {
            out.status.status = nonsens::pod::Gnss::NavSatStatus::STATUS_FIX;
            out.status.service = nonsens::pod::Gnss::NavSatStatus::SERVICE_GPS;
            if (out.rtk_status == nonsens::pod::Gnss::RtkStatus::NO_FIX) {
                out.rtk_status = nonsens::pod::Gnss::RtkStatus::SINGLE;
            }
        }

        struct FastPacketRx {
            uint8_t seq{0};
            uint8_t expected_len{0};
            dp::Vector<uint8_t> buf{};
            size_t received{0};
        };

        inline dp::Res<dp::Vector<uint8_t>> n2k_fast_packet_feed(uint32_t id29, wirebit::can_frame const &frame) {
            static std::unordered_map<uint32_t, FastPacketRx> rx;

            uint8_t h = frame.data[0];
            uint8_t seq = static_cast<uint8_t>(h & 0x1F);
            uint8_t frame_num = static_cast<uint8_t>((h >> 5) & 0x7);

            auto &st = rx[id29];

            if (frame_num == 0) {
                uint8_t total_len = frame.data[1];
                if (total_len == 0) {
                    rx.erase(id29);
                    return dp::Res<dp::Vector<uint8_t>>::err(
                        dp::Error::parse_error("nmea2000 fast packet: invalid length"));
                }
                st.seq = seq;
                st.expected_len = total_len;
                st.buf.clear();
                st.buf.reserve(total_len);

                size_t take = std::min<size_t>(6, total_len);
                for (size_t i = 0; i < take; ++i) {
                    st.buf.push_back(frame.data[2 + i]);
                }
                st.received = st.buf.size();
            } else {
                if (st.expected_len == 0 || st.seq != seq) {
                    return dp::Res<dp::Vector<uint8_t>>::err(
                        dp::Error::timeout("nmea2000 fast packet: no active session"));
                }

                size_t remaining = static_cast<size_t>(st.expected_len) - st.buf.size();
                size_t take = std::min<size_t>(7, remaining);
                for (size_t i = 0; i < take; ++i) {
                    st.buf.push_back(frame.data[1 + i]);
                }
                st.received = st.buf.size();
            }

            if (st.expected_len > 0 && st.buf.size() >= static_cast<size_t>(st.expected_len)) {
                dp::Vector<uint8_t> out = std::move(st.buf);
                rx.erase(id29);
                return dp::Res<dp::Vector<uint8_t>>::ok(std::move(out));
            }
            return dp::Res<dp::Vector<uint8_t>>::err(dp::Error::timeout("nmea2000 fast packet: incomplete"));
        }

    } // namespace detail_n2k

    /// Update a GNSS pod from a single NMEA 2000 CAN frame.
    ///
    /// Supported PGNs:
    /// - 129025 (0x1F801): Position, Rapid Update
    /// - 129026 (0x1F802): COG & SOG, Rapid Update
    /// - 129029 (0x1F805): GNSS Position Data (Fast Packet; partial decode)
    ///
    /// Unsupported frames are ignored.
    inline dp::VoidRes nmea2000_to_pod(wirebit::can_frame const &frame, nonsens::pod::Gnss &out) {
        if (!detail_n2k::is_extended(frame)) {
            return dp::VoidRes::ok();
        }
        if (frame.can_dlc != 8) {
            return dp::VoidRes::ok();
        }

        uint32_t id29 = detail_n2k::raw_id29(frame);
        auto id = detail_n2k::parse_id29(id29);
        uint32_t pgn = detail_n2k::pgn_from_id(id);

        // 129025: Position, Rapid Update
        if (pgn == 129025) {
            int32_t lat_i = detail_n2k::i32_le(frame.data + 0);
            int32_t lon_i = detail_n2k::i32_le(frame.data + 4);
            out.latitude = static_cast<double>(lat_i) * 1e-7;
            out.longitude = static_cast<double>(lon_i) * 1e-7;
            detail_n2k::set_fix(out);
            return dp::VoidRes::ok();
        }

        // 129026: COG & SOG, Rapid Update
        if (pgn == 129026) {
            // Byte 0: SID, Byte 1: ref/reserved, Byte 2-3: COG (0.0001 rad), Byte 4-5: SOG (0.01 m/s)
            uint16_t cog_scaled = detail_n2k::u16_le(frame.data + 2);
            uint16_t sog_scaled = detail_n2k::u16_le(frame.data + 4);

            double cog_rad = static_cast<double>(cog_scaled) * 0.0001;
            out.track_deg = cog_rad * (180.0 / M_PI);
            out.speed_mps = static_cast<double>(sog_scaled) * 0.01;
            detail_n2k::set_fix(out);
            return dp::VoidRes::ok();
        }

        // 129029: GNSS Position Data (Fast Packet)
        if (pgn == 129029) {
            auto res = detail_n2k::n2k_fast_packet_feed(id29, frame);
            if (!res.is_ok()) {
                // Incomplete packet: treat as "no update".
                return dp::VoidRes::ok();
            }

            auto const &payload = res.value();
            if (payload.size() < 43) {
                return dp::VoidRes::ok();
            }

            // Offsets follow the Python layout:
            // sid(1) days(2) time(4) lat(8) lon(8) alt(8) ...
            size_t o = 0;
            (void)payload[o++];
            o += 2;
            o += 4;

            if (o + 24 > payload.size()) {
                return dp::VoidRes::ok();
            }
            int64_t lat_scaled = detail_n2k::i64_le(payload.data() + o);
            o += 8;
            int64_t lon_scaled = detail_n2k::i64_le(payload.data() + o);
            o += 8;
            int64_t alt_scaled = detail_n2k::i64_le(payload.data() + o);
            o += 8;

            out.latitude = static_cast<double>(lat_scaled) * 1e-16;
            out.longitude = static_cast<double>(lon_scaled) * 1e-16;
            out.altitude = static_cast<double>(alt_scaled) * 1e-6;
            detail_n2k::set_fix(out);
            return dp::VoidRes::ok();
        }

        return dp::VoidRes::ok();
    }

} // namespace nonsens::codec::gnss

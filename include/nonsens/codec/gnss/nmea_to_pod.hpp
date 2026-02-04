#pragma once

#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <datapod/datapod.hpp>

#include <nonsens/pods/gnss.hpp>

#include <nonsens/codec/gnss/nmea_common.hpp>

namespace nonsens::codec::gnss {

    namespace detail {

        inline dp::Error parse_error(char const *msg) { return dp::Error::parse_error(dp::String(msg)); }

        inline bool is_hex_digit(char c) {
            return (c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') || (c >= 'a' && c <= 'f');
        }

        inline int hex_value(char c) {
            if (c >= '0' && c <= '9')
                return c - '0';
            if (c >= 'A' && c <= 'F')
                return 10 + (c - 'A');
            if (c >= 'a' && c <= 'f')
                return 10 + (c - 'a');
            return 0;
        }

        inline dp::Res<std::string_view> strip_crlf(std::string_view s) {
            while (!s.empty() && (s.back() == '\r' || s.back() == '\n')) {
                s.remove_suffix(1);
            }
            if (s.empty()) {
                return dp::Res<std::string_view>::err(parse_error("empty nmea line"));
            }
            return dp::Res<std::string_view>::ok(s);
        }

        inline dp::Res<std::pair<std::string_view, uint8_t>> split_payload_and_checksum(std::string_view line) {
            if (line.size() < 4 || line.front() != '$') {
                return dp::Res<std::pair<std::string_view, uint8_t>>::err(parse_error("invalid nmea prefix"));
            }

            auto star = line.find('*');
            if (star == std::string_view::npos || star + 2 >= line.size()) {
                return dp::Res<std::pair<std::string_view, uint8_t>>::err(parse_error("missing nmea checksum"));
            }

            char c1 = line[star + 1];
            char c2 = line[star + 2];
            if (!is_hex_digit(c1) || !is_hex_digit(c2)) {
                return dp::Res<std::pair<std::string_view, uint8_t>>::err(parse_error("invalid nmea checksum hex"));
            }

            uint8_t expected = static_cast<uint8_t>((hex_value(c1) << 4) | hex_value(c2));
            std::string_view payload = line.substr(1, star - 1);
            return dp::Res<std::pair<std::string_view, uint8_t>>::ok({payload, expected});
        }

        inline std::vector<std::string_view> split_csv(std::string_view s) {
            std::vector<std::string_view> out;
            size_t start = 0;
            while (start <= s.size()) {
                size_t comma = s.find(',', start);
                if (comma == std::string_view::npos) {
                    out.push_back(s.substr(start));
                    break;
                }
                out.push_back(s.substr(start, comma - start));
                start = comma + 1;
            }
            return out;
        }

        inline dp::Res<double> parse_double(std::string_view s) {
            if (s.empty()) {
                return dp::Res<double>::err(parse_error("empty number"));
            }
            try {
                size_t idx = 0;
                std::string tmp(s);
                double v = std::stod(tmp, &idx);
                if (idx != tmp.size()) {
                    return dp::Res<double>::err(parse_error("invalid number"));
                }
                return dp::Res<double>::ok(v);
            } catch (...) {
                return dp::Res<double>::err(parse_error("invalid number"));
            }
        }

        inline dp::Res<int> parse_int(std::string_view s) {
            if (s.empty()) {
                return dp::Res<int>::err(parse_error("empty int"));
            }
            try {
                size_t idx = 0;
                std::string tmp(s);
                int v = std::stoi(tmp, &idx);
                if (idx != tmp.size()) {
                    return dp::Res<int>::err(parse_error("invalid int"));
                }
                return dp::Res<int>::ok(v);
            } catch (...) {
                return dp::Res<int>::err(parse_error("invalid int"));
            }
        }

        // NMEA lat/lon format: latitude = ddmm.mmmm + hemisphere, longitude = dddmm.mmmm + hemisphere
        inline dp::Res<double> parse_latlon(std::string_view v, std::string_view hemi, bool is_lat) {
            if (v.empty() || hemi.empty()) {
                return dp::Res<double>::err(parse_error("missing lat/lon"));
            }

            auto d = parse_double(v);
            if (!d.is_ok()) {
                return dp::Res<double>::err(d.error());
            }

            double raw = d.value();

            // ddmm.mmmm / dddmm.mmmm parsing:
            // The minutes are always the last two digits before the decimal point.
            // Therefore degrees = floor(raw / 100) for both latitude and longitude.
            int deg = static_cast<int>(raw / 100.0);
            double min = raw - (static_cast<double>(deg) * 100.0);

            double dec = static_cast<double>(deg) + (min / 60.0);

            char h = hemi.front();
            if (is_lat) {
                if (h == 'S' || h == 's') {
                    dec = -dec;
                } else if (h != 'N' && h != 'n') {
                    return dp::Res<double>::err(parse_error("invalid lat hemisphere"));
                }
            } else {
                if (h == 'W' || h == 'w') {
                    dec = -dec;
                } else if (h != 'E' && h != 'e') {
                    return dp::Res<double>::err(parse_error("invalid lon hemisphere"));
                }
            }
            return dp::Res<double>::ok(dec);
        }

        inline void set_fix_status_from_quality(nonsens::pod::Gnss &out, int quality) {
            if (quality <= 0) {
                out.status.status = nonsens::pod::Gnss::NavSatStatus::STATUS_NO_FIX;
                out.rtk_status = nonsens::pod::Gnss::RtkStatus::NO_FIX;
            } else {
                out.status.status = nonsens::pod::Gnss::NavSatStatus::STATUS_FIX;
                out.status.service = nonsens::pod::Gnss::NavSatStatus::SERVICE_GPS;

                // Common NMEA GGA fix quality mapping:
                // 1 = GPS, 2 = DGPS, 4 = RTK fixed, 5 = RTK float
                switch (quality) {
                case 4:
                    out.rtk_status = nonsens::pod::Gnss::RtkStatus::RTK_FIXED;
                    break;
                case 5:
                    out.rtk_status = nonsens::pod::Gnss::RtkStatus::RTK_FLOAT;
                    break;
                case 2:
                    out.rtk_status = nonsens::pod::Gnss::RtkStatus::DGPS;
                    break;
                default:
                    out.rtk_status = nonsens::pod::Gnss::RtkStatus::SINGLE;
                    break;
                }
            }
        }

        inline void set_fix_status_from_rmc(nonsens::pod::Gnss &out, char rmc_status) {
            if (rmc_status == 'A') {
                out.status.status = nonsens::pod::Gnss::NavSatStatus::STATUS_FIX;
                out.status.service = nonsens::pod::Gnss::NavSatStatus::SERVICE_GPS;
                if (out.rtk_status == nonsens::pod::Gnss::RtkStatus::NO_FIX) {
                    out.rtk_status = nonsens::pod::Gnss::RtkStatus::SINGLE;
                }
            } else {
                out.status.status = nonsens::pod::Gnss::NavSatStatus::STATUS_NO_FIX;
                out.rtk_status = nonsens::pod::Gnss::RtkStatus::NO_FIX;
            }
        }

        inline dp::u8 clamp_u8_from_int(int v) {
            if (v < 0)
                return 0;
            if (v > 255)
                return 255;
            return static_cast<dp::u8>(v);
        }

    } // namespace detail

    /// Update a GNSS pod from a single NMEA0183 sentence.
    ///
    /// Supported sentences:
    /// - GGA, RMC, GNS, GST, GSV, PHTG
    ///
    /// Unknown sentence types are ignored.
    inline dp::VoidRes nmea_to_pod(std::string_view sentence, nonsens::pod::Gnss &out) {
        auto stripped = detail::strip_crlf(sentence);
        if (!stripped.is_ok()) {
            return dp::VoidRes::err(stripped.error());
        }

        auto split = detail::split_payload_and_checksum(stripped.value());
        if (!split.is_ok()) {
            return dp::VoidRes::err(split.error());
        }

        auto payload = split.value().first;
        auto expected = split.value().second;
        auto computed = detail::nmea_xor_checksum(payload);
        if (computed != expected) {
            return dp::VoidRes::err(dp::Error::parse_error("nmea checksum mismatch"));
        }

        auto fields = detail::split_csv(payload);
        if (fields.empty() || fields[0].size() < 3) {
            return dp::VoidRes::err(dp::Error::parse_error("invalid nmea talker/type"));
        }

        std::string_view type = fields[0];
        if (type.size() >= 3) {
            type = type.substr(type.size() - 3);
        }

        if (type == "GGA") {
            // $--GGA,time,lat,NS,lon,EW,quality,numsv,hdop,alt,M,...
            if (fields.size() < 10) {
                return dp::VoidRes::err(dp::Error::parse_error("gga: not enough fields"));
            }

            if (!fields[2].empty() && !fields[3].empty()) {
                auto lat = detail::parse_latlon(fields[2], fields[3], true);
                if (lat.is_ok())
                    out.latitude = lat.value();
            }
            if (!fields[4].empty() && !fields[5].empty()) {
                auto lon = detail::parse_latlon(fields[4], fields[5], false);
                if (lon.is_ok())
                    out.longitude = lon.value();
            }

            auto q = detail::parse_int(fields[6]);
            if (q.is_ok()) {
                detail::set_fix_status_from_quality(out, q.value());
            }

            auto ns = detail::parse_int(fields[7]);
            if (ns.is_ok()) {
                out.num_satellites = detail::clamp_u8_from_int(ns.value());
            }

            auto hd = detail::parse_double(fields[8]);
            if (hd.is_ok()) {
                out.hdop = hd.value();
            }

            auto alt = detail::parse_double(fields[9]);
            if (alt.is_ok()) {
                out.altitude = alt.value();
            }

            return dp::VoidRes::ok();
        }

        if (type == "RMC") {
            // $--RMC,time,status,lat,NS,lon,EW,sog,cog,date,...
            if (fields.size() < 9) {
                return dp::VoidRes::err(dp::Error::parse_error("rmc: not enough fields"));
            }

            if (!fields[2].empty()) {
                detail::set_fix_status_from_rmc(out, fields[2].front());
            }

            if (!fields[3].empty() && !fields[4].empty()) {
                auto lat = detail::parse_latlon(fields[3], fields[4], true);
                if (lat.is_ok())
                    out.latitude = lat.value();
            }
            if (!fields[5].empty() && !fields[6].empty()) {
                auto lon = detail::parse_latlon(fields[5], fields[6], false);
                if (lon.is_ok())
                    out.longitude = lon.value();
            }

            // Speed over ground in knots -> m/s
            if (!fields[7].empty()) {
                auto sog = detail::parse_double(fields[7]);
                if (sog.is_ok()) {
                    out.speed_mps = sog.value() / 1.94384;
                }
            }

            // Course over ground in degrees
            if (!fields[8].empty()) {
                auto cog = detail::parse_double(fields[8]);
                if (cog.is_ok()) {
                    out.track_deg = cog.value();
                }
            }

            // Derive NEU velocity from speed+track.
            double tr = out.track_deg * (M_PI / 180.0);
            out.velocity_neu[0] = out.speed_mps * std::cos(tr);
            out.velocity_neu[1] = out.speed_mps * std::sin(tr);
            out.velocity_neu[2] = 0.0;

            return dp::VoidRes::ok();
        }

        if (type == "GNS") {
            // $--GNS,time,lat,NS,lon,EW,mode,numSV,hdop,alt,sep,...
            if (fields.size() < 11) {
                return dp::VoidRes::err(dp::Error::parse_error("gns: not enough fields"));
            }

            if (!fields[2].empty() && !fields[3].empty()) {
                auto lat = detail::parse_latlon(fields[2], fields[3], true);
                if (lat.is_ok())
                    out.latitude = lat.value();
            }
            if (!fields[4].empty() && !fields[5].empty()) {
                auto lon = detail::parse_latlon(fields[4], fields[5], false);
                if (lon.is_ok())
                    out.longitude = lon.value();
            }

            // Mode (fields[6]) is left as-is; derive fix if mode isn't empty and not 'N'.
            if (!fields[6].empty() && fields[6].find('N') == std::string_view::npos) {
                out.status.status = nonsens::pod::Gnss::NavSatStatus::STATUS_FIX;
                out.status.service = nonsens::pod::Gnss::NavSatStatus::SERVICE_GPS;
                if (out.rtk_status == nonsens::pod::Gnss::RtkStatus::NO_FIX) {
                    out.rtk_status = nonsens::pod::Gnss::RtkStatus::SINGLE;
                }
            }

            auto ns = detail::parse_int(fields[7]);
            if (ns.is_ok()) {
                out.num_satellites = detail::clamp_u8_from_int(ns.value());
            }

            auto hd = detail::parse_double(fields[8]);
            if (hd.is_ok()) {
                out.hdop = hd.value();
            }

            auto alt = detail::parse_double(fields[9]);
            if (alt.is_ok()) {
                out.altitude = alt.value();
            }

            return dp::VoidRes::ok();
        }

        if (type == "GST") {
            // $--GST,time,rms,major,minor,orient,lat_err,lon_err,alt_err
            if (fields.size() < 9) {
                return dp::VoidRes::err(dp::Error::parse_error("gst: not enough fields"));
            }

            auto rms = detail::parse_double(fields[2]);
            if (rms.is_ok()) {
                out.horizontal_accuracy = rms.value();
            }
            auto alt_err = detail::parse_double(fields[8]);
            if (alt_err.is_ok()) {
                out.vertical_accuracy = alt_err.value();
            }
            return dp::VoidRes::ok();
        }

        if (type == "GSV") {
            // $--GSV,numMsgs,msgNum,numSats,...
            if (fields.size() >= 4) {
                auto ns = detail::parse_int(fields[3]);
                if (ns.is_ok()) {
                    out.num_satellites = detail::clamp_u8_from_int(ns.value());
                }
            }
            return dp::VoidRes::ok();
        }

        if (type == "HTG" || type == "PHTG") {
            // $PHTG,date,time,System,Service,AuthResult,Status
            // We only care about the "Status" last field.
            if (fields.size() >= 7) {
                auto st = detail::parse_int(fields[6]);
                if (st.is_ok()) {
                    out.phtg = (st.value() != 0);
                }
            }
            return dp::VoidRes::ok();
        }

        return dp::VoidRes::ok();
    }

} // namespace nonsens::codec::gnss

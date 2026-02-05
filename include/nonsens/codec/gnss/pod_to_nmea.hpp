#pragma once

#include <cmath>
#include <cstdint>
#include <ctime>
#include <string>
#include <string_view>

#include <datapod/datapod.hpp>

#include <nonsens/pods/gnss.hpp>

#include <nonsens/codec/gnss/nmea_common.hpp>

namespace nonsens::codec::gnss {

    namespace detail {

        inline std::string get_utc_time_hhmmss_sss() {
            timespec ts{};
            clock_gettime(CLOCK_REALTIME, &ts);

            std::time_t t = ts.tv_sec;
            std::tm utc_tm;
            gmtime_r(&t, &utc_tm);

            int ms = static_cast<int>(ts.tv_nsec / 1000000);
            char buf[16];
            std::snprintf(buf, sizeof(buf), "%02d%02d%02d.%03d", utc_tm.tm_hour, utc_tm.tm_min, utc_tm.tm_sec, ms);
            return std::string(buf);
        }

        inline std::string get_utc_date_ddmmyy() {
            std::time_t t = std::time(nullptr);
            std::tm utc_tm;
            gmtime_r(&t, &utc_tm);
            char buf[32];
            std::snprintf(buf, sizeof(buf), "%02d%02d%02d", utc_tm.tm_mday, utc_tm.tm_mon + 1, (utc_tm.tm_year % 100));
            return std::string(buf);
        }

        inline std::string get_utc_date_dd_mm_yyyy() {
            std::time_t t = std::time(nullptr);
            std::tm utc_tm;
            gmtime_r(&t, &utc_tm);
            char buf[32];
            std::snprintf(buf, sizeof(buf), "%02d:%02d:%04d", utc_tm.tm_mday, utc_tm.tm_mon + 1, utc_tm.tm_year + 1900);
            return std::string(buf);
        }

        inline std::string get_utc_time_hh_mm_ss_ss() {
            std::time_t t = std::time(nullptr);
            std::tm utc_tm;
            gmtime_r(&t, &utc_tm);
            char buf[16];
            std::snprintf(buf, sizeof(buf), "%02d:%02d:%02d.00", utc_tm.tm_hour, utc_tm.tm_min, utc_tm.tm_sec);
            return std::string(buf);
        }

        inline void format_lat_nmea(dp::f64 lat_deg, char &hemi_out, std::string &value_out) {
            hemi_out = (lat_deg >= 0.0) ? 'N' : 'S';
            double lat_abs = std::abs(static_cast<double>(lat_deg));
            int deg = static_cast<int>(lat_abs);
            double minutes = (lat_abs - deg) * 60.0;
            char buf[32];
            std::snprintf(buf, sizeof(buf), "%02d%011.8f", deg, minutes);
            value_out = std::string(buf);
        }

        inline void format_lon_nmea(dp::f64 lon_deg, char &hemi_out, std::string &value_out) {
            hemi_out = (lon_deg >= 0.0) ? 'E' : 'W';
            double lon_abs = std::abs(static_cast<double>(lon_deg));
            int deg = static_cast<int>(lon_abs);
            double minutes = (lon_abs - deg) * 60.0;
            char buf[32];
            std::snprintf(buf, sizeof(buf), "%03d%011.8f", deg, minutes);
            value_out = std::string(buf);
        }

        inline int gga_fix_quality(nonsens::pod::Gnss const &in) {
            using RS = nonsens::pod::Gnss::RtkStatus;
            switch (in.rtk_status) {
            case RS::RTK_FIXED:
                return 4;
            case RS::RTK_FLOAT:
                return 5;
            case RS::DGPS:
                return 2;
            case RS::SINGLE:
                return 1;
            case RS::NO_FIX:
            default:
                return 0;
            }
        }

        inline double speed_mps_from_pod(nonsens::pod::Gnss const &in) { return static_cast<double>(in.speed_mps); }

        inline double track_deg_from_pod(nonsens::pod::Gnss const &in) { return static_cast<double>(in.track_deg); }

    } // namespace detail

    inline dp::Res<dp::String> pod_to_nmea_rmc(nonsens::pod::Gnss const &in) {
        std::string lat_v, lon_v;
        char lat_h = 'N';
        char lon_h = 'E';
        detail::format_lat_nmea(in.latitude, lat_h, lat_v);
        detail::format_lon_nmea(in.longitude, lon_h, lon_v);

        std::string time_str = detail::get_utc_time_hhmmss_sss();
        std::string date_str = detail::get_utc_date_ddmmyy();

        char status = (in.status.status == nonsens::pod::Gnss::NavSatStatus::STATUS_NO_FIX) ? 'V' : 'A';
        double speed_knots = detail::speed_mps_from_pod(in) * 1.94384;
        double track_deg = detail::track_deg_from_pod(in);

        char body[256];
        std::snprintf(body, sizeof(body), "GPRMC,%s,%c,%s,%c,%s,%c,%.1f,%.1f,%s,,", time_str.c_str(), status,
                      lat_v.c_str(), lat_h, lon_v.c_str(), lon_h, speed_knots, track_deg, date_str.c_str());

        return dp::Res<dp::String>::ok(detail::make_sentence(body));
    }

    inline dp::Res<dp::String> pod_to_nmea_gga(nonsens::pod::Gnss const &in) {
        std::string lat_v, lon_v;
        char lat_h = 'N';
        char lon_h = 'E';
        detail::format_lat_nmea(in.latitude, lat_h, lat_v);
        detail::format_lon_nmea(in.longitude, lon_h, lon_v);

        std::string time_str = detail::get_utc_time_hhmmss_sss();

        int quality = detail::gga_fix_quality(in);
        int sats = static_cast<int>(in.num_satellites);
        if (sats <= 0)
            sats = 8;
        double hdop = (in.hdop > 0.0) ? static_cast<double>(in.hdop) : 0.8;

        char body[256];
        std::snprintf(body, sizeof(body), "GPGGA,%s,%s,%c,%s,%c,%d,%02d,%.1f,%.2f,M,0.0,M,,", time_str.c_str(),
                      lat_v.c_str(), lat_h, lon_v.c_str(), lon_h, quality, sats, hdop,
                      static_cast<double>(in.altitude));

        return dp::Res<dp::String>::ok(detail::make_sentence(body));
    }

    inline dp::Res<dp::String> pod_to_nmea_gns(nonsens::pod::Gnss const &in) {
        std::string lat_v, lon_v;
        char lat_h = 'N';
        char lon_h = 'E';
        detail::format_lat_nmea(in.latitude, lat_h, lat_v);
        detail::format_lon_nmea(in.longitude, lon_h, lon_v);

        std::string time_str = detail::get_utc_time_hhmmss_sss();

        int sats = static_cast<int>(in.num_satellites);
        if (sats <= 0)
            sats = 8;
        double hdop = (in.hdop > 0.0) ? static_cast<double>(in.hdop) : 0.8;

        const char *mode = (in.status.status == nonsens::pod::Gnss::NavSatStatus::STATUS_NO_FIX) ? "NNNNN" : "RRNNN";

        char body[256];
        std::snprintf(body, sizeof(body), "GNGNS,%s,%s,%c,%s,%c,%s,%d,%.1f,%.2f,0.0,,,", time_str.c_str(),
                      lat_v.c_str(), lat_h, lon_v.c_str(), lon_h, mode, sats, hdop, static_cast<double>(in.altitude));

        return dp::Res<dp::String>::ok(detail::make_sentence(body));
    }

    inline dp::Res<dp::String> pod_to_nmea_gst(nonsens::pod::Gnss const &in) {
        std::string time_str = detail::get_utc_time_hhmmss_sss();

        double h = static_cast<double>(in.horizontal_accuracy);
        if (h <= 0.0)
            h = 0.3;
        double v = static_cast<double>(in.vertical_accuracy);
        if (v <= 0.0)
            v = h * 1.5;

        char body[256];
        std::snprintf(body, sizeof(body), "GPGST,%s,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f", time_str.c_str(), h, h, h, 0.0,
                      h, h, v);

        return dp::Res<dp::String>::ok(detail::make_sentence(body));
    }

    inline dp::Res<dp::String> pod_to_nmea_gsv(nonsens::pod::Gnss const &in) {
        int sats = static_cast<int>(in.num_satellites);
        if (sats <= 0)
            sats = 8;

        char body[256];
        std::snprintf(body, sizeof(body), "GPGSV,1,1,%02d,11,45,120,40", sats);
        return dp::Res<dp::String>::ok(detail::make_sentence(body));
    }

    inline dp::Res<dp::String> pod_to_nmea_phtg(nonsens::pod::Gnss const &in) {
        std::string d = detail::get_utc_date_dd_mm_yyyy();
        std::string t = detail::get_utc_time_hh_mm_ss_ss();
        int pth_status = in.phtg ? 1 : 0;

        char body[256];
        std::snprintf(body, sizeof(body), "PHTG,%s,%s,GAL,HAS,%d,0", d.c_str(), t.c_str(), pth_status);
        return dp::Res<dp::String>::ok(detail::make_sentence(body));
    }

    /// Generate the same set of sentences as the referenced simulator (concatenated, each with CRLF).
    inline dp::Res<dp::String> pod_to_nmea_all(nonsens::pod::Gnss const &in) {
        auto gga = pod_to_nmea_gga(in);
        if (!gga.is_ok())
            return dp::Res<dp::String>::err(gga.error());
        auto rmc = pod_to_nmea_rmc(in);
        if (!rmc.is_ok())
            return dp::Res<dp::String>::err(rmc.error());
        auto gns = pod_to_nmea_gns(in);
        if (!gns.is_ok())
            return dp::Res<dp::String>::err(gns.error());
        auto gst = pod_to_nmea_gst(in);
        if (!gst.is_ok())
            return dp::Res<dp::String>::err(gst.error());
        auto gsv = pod_to_nmea_gsv(in);
        if (!gsv.is_ok())
            return dp::Res<dp::String>::err(gsv.error());
        auto phtg = pod_to_nmea_phtg(in);
        if (!phtg.is_ok())
            return dp::Res<dp::String>::err(phtg.error());

        std::string out;
        out.reserve(gga.value().size() + rmc.value().size() + gns.value().size() + gst.value().size() +
                    gsv.value().size() + phtg.value().size());
        out.append(gga.value().c_str(), gga.value().size());
        out.append(rmc.value().c_str(), rmc.value().size());
        out.append(gns.value().c_str(), gns.value().size());
        out.append(gst.value().c_str(), gst.value().size());
        out.append(gsv.value().c_str(), gsv.value().size());
        out.append(phtg.value().c_str(), phtg.value().size());
        return dp::Res<dp::String>::ok(dp::String(out.c_str()));
    }

} // namespace nonsens::codec::gnss

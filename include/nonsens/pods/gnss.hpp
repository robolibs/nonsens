#pragma once

#include <tuple>

#include <datapod/datapod.hpp>

namespace nonsens::pod {

    // =============================================================================================
    // GNSS (ROS-like)
    // =============================================================================================

    /// Equivalent to sensor_msgs/NavSatFix (without std_msgs/Header).
    struct Gnss {
        /// Equivalent to sensor_msgs/NavSatStatus.
        struct NavSatStatus {
            static constexpr dp::i8 STATUS_NO_FIX = -1;
            static constexpr dp::i8 STATUS_FIX = 0;
            static constexpr dp::i8 STATUS_SBAS_FIX = 1;
            static constexpr dp::i8 STATUS_GBAS_FIX = 2;

            static constexpr dp::u16 SERVICE_GPS = 1;
            static constexpr dp::u16 SERVICE_GLONASS = 2;
            static constexpr dp::u16 SERVICE_COMPASS = 4;
            static constexpr dp::u16 SERVICE_GALILEO = 8;

            dp::i8 status{STATUS_NO_FIX};
            dp::u16 service{0};

            auto members() noexcept { return std::tie(status, service); }
            auto members() const noexcept { return std::tie(status, service); }
        };

        static constexpr dp::u8 COVARIANCE_TYPE_UNKNOWN = 0;
        static constexpr dp::u8 COVARIANCE_TYPE_APPROXIMATED = 1;
        static constexpr dp::u8 COVARIANCE_TYPE_DIAGONAL_KNOWN = 2;
        static constexpr dp::u8 COVARIANCE_TYPE_KNOWN = 3;

        NavSatStatus status{};
        dp::f64 latitude{0.0};
        dp::f64 longitude{0.0};
        dp::f64 altitude{0.0};

        dp::Array<dp::f64, 9> position_covariance{};
        dp::u8 position_covariance_type{COVARIANCE_TYPE_UNKNOWN};

        // -------------------------------------------------------------------------------------
        // Extensions (to match common simulator/receiver outputs)
        // -------------------------------------------------------------------------------------

        /// RTK status as used by many GNSS receivers/simulators.
        enum class RtkStatus : dp::u8 {
            NO_FIX = 0,
            SINGLE = 1,
            DGPS = 2,
            RTK_FLOAT = 3,
            RTK_FIXED = 4,
        };

        /// Velocity in local North-East-Up frame [m/s] -> {vn, ve, vu}.
        dp::mat::Vector<dp::f64, 3> velocity_neu{};

        /// Track/course over ground [deg] (0 = North, 90 = East).
        dp::f64 track_deg{0.0};

        /// Speed over ground [m/s].
        dp::f64 speed_mps{0.0};

        /// Estimated horizontal accuracy [m].
        dp::f64 horizontal_accuracy{0.0};

        /// Estimated vertical accuracy [m].
        dp::f64 vertical_accuracy{0.0};

        /// NMEA-style HDOP (dimensionless).
        dp::f64 hdop{0.0};

        /// Number of satellites used in solution.
        dp::u8 num_satellites{0};

        /// RTK state.
        RtkStatus rtk_status{RtkStatus::NO_FIX};

        /// Galileo HAS / vendor-specific assist status (from PHTG-like sentences).
        dp::boolean phtg{false};

        auto members() noexcept {
            return std::tie(status, latitude, longitude, altitude, position_covariance, position_covariance_type,
                            velocity_neu, track_deg, speed_mps, horizontal_accuracy, vertical_accuracy, hdop,
                            num_satellites, rtk_status, phtg);
        }
        auto members() const noexcept {
            return std::tie(status, latitude, longitude, altitude, position_covariance, position_covariance_type,
                            velocity_neu, track_deg, speed_mps, horizontal_accuracy, vertical_accuracy, hdop,
                            num_satellites, rtk_status, phtg);
        }
    };

} // namespace nonsens::pod

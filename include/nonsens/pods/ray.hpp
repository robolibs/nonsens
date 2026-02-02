#pragma once

#include <tuple>

#include <datapod/datapod.hpp>

namespace nonsens::pod {

    // =============================================================================================
    // LiDAR / RAY (ROS-like)
    // =============================================================================================

    /// Equivalent to sensor_msgs/LaserScan (without std_msgs/Header).
    struct Ray {
        dp::f32 angle_min{0.0f};
        dp::f32 angle_max{0.0f};
        dp::f32 angle_increment{0.0f};
        dp::f32 time_increment{0.0f};
        dp::f32 scan_time{0.0f};
        dp::f32 range_min{0.0f};
        dp::f32 range_max{0.0f};

        dp::Vector<dp::f32> ranges{};
        dp::Vector<dp::f32> intensities{};

        auto members() noexcept {
            return std::tie(angle_min, angle_max, angle_increment, time_increment, scan_time, range_min, range_max,
                            ranges, intensities);
        }
        auto members() const noexcept {
            return std::tie(angle_min, angle_max, angle_increment, time_increment, scan_time, range_min, range_max,
                            ranges, intensities);
        }
    };

} // namespace nonsens::pod

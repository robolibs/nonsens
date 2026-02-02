#pragma once

#include <tuple>

#include <datapod/datapod.hpp>

namespace nonsens::pod {

    // =============================================================================================
    // Camera (ROS-like)
    // =============================================================================================

    /// Equivalent to sensor_msgs/Image (without std_msgs/Header).
    struct Camera {
        dp::u32 height{0};
        dp::u32 width{0};
        dp::String encoding{};
        dp::u8 is_bigendian{0};
        dp::u32 step{0};
        dp::Vector<dp::u8> data{};

        auto members() noexcept { return std::tie(height, width, encoding, is_bigendian, step, data); }
        auto members() const noexcept { return std::tie(height, width, encoding, is_bigendian, step, data); }
    };

} // namespace nonsens::pod

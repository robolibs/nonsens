#pragma once

#include <tuple>

#include <datapod/datapod.hpp>

namespace nonsens::pod {

    // =============================================================================================
    // IMU (ROS-like)
    // =============================================================================================

    /// Equivalent to sensor_msgs/Imu (without std_msgs/Header).
    struct Imu {
        dp::mat::Quaternion<dp::f64> orientation{dp::f64{1.0}, dp::f64{0.0}, dp::f64{0.0}, dp::f64{0.0}};
        dp::Array<dp::f64, 9> orientation_covariance{};

        dp::mat::Vector<dp::f64, 3> angular_velocity{};
        dp::Array<dp::f64, 9> angular_velocity_covariance{};

        dp::mat::Vector<dp::f64, 3> linear_acceleration{};
        dp::Array<dp::f64, 9> linear_acceleration_covariance{};

        auto members() noexcept {
            return std::tie(orientation, orientation_covariance, angular_velocity, angular_velocity_covariance,
                            linear_acceleration, linear_acceleration_covariance);
        }
        auto members() const noexcept {
            return std::tie(orientation, orientation_covariance, angular_velocity, angular_velocity_covariance,
                            linear_acceleration, linear_acceleration_covariance);
        }
    };

} // namespace nonsens::pod

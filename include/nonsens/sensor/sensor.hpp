#pragma once

#include <type_traits>

#include <datapod/datapod.hpp>

#include <wirebit/wirebit.hpp>

#include <nonsens/pods/camera.hpp>
#include <nonsens/pods/gnss.hpp>
#include <nonsens/pods/imu.hpp>
#include <nonsens/pods/ray.hpp>

#include <nonsens/sensor/can_protocol.hpp>
#include <nonsens/sensor/gnss.hpp>

namespace nonsens::sensor {

    enum class SensorType : dp::u8 {
        GNSS = 0,
        IMU = 1,
        RAY = 2,
        CAMERA = 3,
    };

    using Endpoint = dp::Variant<wirebit::SerialEndpoint *, wirebit::CanEndpoint *, wirebit::EthEndpoint *>;

    using PodPtr = dp::Variant<nonsens::pod::Gnss *, nonsens::pod::Imu *, nonsens::pod::Ray *, nonsens::pod::Camera *>;

    using PodConstPtr = dp::Variant<nonsens::pod::Gnss const *, nonsens::pod::Imu const *, nonsens::pod::Ray const *,
                                    nonsens::pod::Camera const *>;

    /// Unified runtime sensor wrapper.
    ///
    /// This is the single type you can create/configure from external code.
    /// Internally it wraps concrete typed sensors (currently GNSS only).
    class Sensor {
      public:
        static inline dp::Res<Sensor> create(SensorType type) {
            Sensor s;
            s.type_ = type;

            switch (type) {
            case SensorType::GNSS:
                s.impl_ = nonsens::sensor::Gnss{};
                return dp::Res<Sensor>::ok(std::move(s));
            case SensorType::IMU:
            case SensorType::RAY:
            case SensorType::CAMERA:
            default:
                return dp::Res<Sensor>::err(dp::Error::invalid_argument("sensor type not implemented"));
            }
        }

        SensorType type() const noexcept { return type_; }

        dp::String name() const {
            return impl_.apply([](auto const &s) { return dp::String(s.name()); });
        }

        dp::VoidRes add_input(Endpoint ep) {
            return impl_.apply([&](auto &s) { return ep.apply([&](auto *p) { return add_input_impl(s, p); }); });
        }

        dp::VoidRes add_output(Endpoint ep) {
            return impl_.apply([&](auto &s) { return ep.apply([&](auto *p) { return add_output_impl(s, p); }); });
        }

        dp::VoidRes step() {
            return impl_.apply([](auto &s) { return s.step(); });
        }
        dp::VoidRes push() {
            return impl_.apply([](auto &s) { return s.push(); });
        }

        dp::VoidRes set_can_input_protocol(CanProtocol proto) {
            return impl_.apply([&](auto &s) { return set_can_input_protocol_impl(s, proto); });
        }

        dp::VoidRes set_can_output_protocol(CanProtocol proto) {
            return impl_.apply([&](auto &s) { return set_can_output_protocol_impl(s, proto); });
        }

        PodPtr pod() {
            return impl_.apply([](auto &s) -> PodPtr {
                using S = std::decay_t<decltype(s)>;
                if constexpr (std::is_same_v<S, nonsens::sensor::Gnss>) {
                    return PodPtr{&s.pod()};
                } else {
                    return PodPtr{};
                }
            });
        }

        PodConstPtr pod() const {
            return impl_.apply([](auto const &s) -> PodConstPtr {
                using S = std::decay_t<decltype(s)>;
                if constexpr (std::is_same_v<S, nonsens::sensor::Gnss>) {
                    return PodConstPtr{&s.pod()};
                } else {
                    return PodConstPtr{};
                }
            });
        }

      private:
        template <typename S> static dp::VoidRes set_can_input_protocol_impl(S &s, CanProtocol proto) {
            if constexpr (requires { s.set_can_input_protocol(proto); }) {
                return s.set_can_input_protocol(proto);
            } else {
                return dp::VoidRes::err(dp::Error::invalid_argument("can input protocol not supported by this sensor"));
            }
        }

        template <typename S> static dp::VoidRes set_can_output_protocol_impl(S &s, CanProtocol proto) {
            if constexpr (requires { s.set_can_output_protocol(proto); }) {
                return s.set_can_output_protocol(proto);
            } else {
                return dp::VoidRes::err(
                    dp::Error::invalid_argument("can output protocol not supported by this sensor"));
            }
        }

        template <typename S> static dp::VoidRes add_input_impl(S &s, wirebit::SerialEndpoint *p) {
            if constexpr (requires { s.add_input(*p); }) {
                return s.add_input(*p);
            } else {
                return dp::VoidRes::err(dp::Error::invalid_argument("serial input not supported by this sensor"));
            }
        }

        template <typename S> static dp::VoidRes add_input_impl(S &s, wirebit::CanEndpoint *p) {
            if constexpr (requires { s.add_input(*p); }) {
                return s.add_input(*p);
            } else {
                return dp::VoidRes::err(dp::Error::invalid_argument("can input not supported by this sensor"));
            }
        }

        template <typename S> static dp::VoidRes add_input_impl(S &s, wirebit::EthEndpoint *p) {
            if constexpr (requires { s.add_input(*p); }) {
                return s.add_input(*p);
            } else {
                return dp::VoidRes::err(dp::Error::invalid_argument("eth input not supported by this sensor"));
            }
        }

        template <typename S> static dp::VoidRes add_output_impl(S &s, wirebit::SerialEndpoint *p) {
            if constexpr (requires { s.add_output(*p); }) {
                return s.add_output(*p);
            } else {
                return dp::VoidRes::err(dp::Error::invalid_argument("serial output not supported by this sensor"));
            }
        }

        template <typename S> static dp::VoidRes add_output_impl(S &s, wirebit::CanEndpoint *p) {
            if constexpr (requires { s.add_output(*p); }) {
                return s.add_output(*p);
            } else {
                return dp::VoidRes::err(dp::Error::invalid_argument("can output not supported by this sensor"));
            }
        }

        template <typename S> static dp::VoidRes add_output_impl(S &s, wirebit::EthEndpoint *p) {
            if constexpr (requires { s.add_output(*p); }) {
                return s.add_output(*p);
            } else {
                return dp::VoidRes::err(dp::Error::invalid_argument("eth output not supported by this sensor"));
            }
        }

        SensorType type_{SensorType::GNSS};
        dp::Variant<nonsens::sensor::Gnss> impl_{};
    };

} // namespace nonsens::sensor

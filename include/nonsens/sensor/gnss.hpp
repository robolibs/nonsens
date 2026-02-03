#pragma once

#include <string>

#include <datapod/datapod.hpp>
#include <wirebit/wirebit.hpp>

#include <nonsens/codec/gnss/j1939_to_pod.hpp>
#include <nonsens/codec/gnss/nmea_to_pod.hpp>
#include <nonsens/codec/gnss/pod_to_j1939.hpp>
#include <nonsens/codec/gnss/pod_to_nmea.hpp>

#include <nonsens/pods/gnss.hpp>
#include <nonsens/sensor/sensor_base.hpp>

namespace nonsens::sensor {

    /// GNSS sensor translator (pod in the middle).
    ///
    /// - SerialEndpoint: NMEA0183 (implemented)
    /// - CanEndpoint: J1939/ISOBUS (stub)
    /// - EthEndpoint: reserved (stub)
    class Gnss final : public SensorBase<nonsens::pod::Gnss> {
      public:
        char const *name() const noexcept override { return "gnss"; }

        dp::VoidRes add_input(wirebit::SerialEndpoint &ep) {
            if (has_input_) {
                return dp::VoidRes::err(dp::Error::already_exists("gnss input already set"));
            }
            in_kind_ = InKind::NMEA_SERIAL;
            serial_in_ = &ep;
            has_input_ = true;
            return dp::VoidRes::ok();
        }

        dp::VoidRes add_input(wirebit::CanEndpoint &ep) {
            if (has_input_) {
                return dp::VoidRes::err(dp::Error::already_exists("gnss input already set"));
            }
            in_kind_ = InKind::J1939_CAN;
            can_in_ = &ep;
            has_input_ = true;
            return dp::VoidRes::ok();
        }

        dp::VoidRes add_input(wirebit::EthEndpoint &ep) {
            if (has_input_) {
                return dp::VoidRes::err(dp::Error::already_exists("gnss input already set"));
            }
            in_kind_ = InKind::ETH;
            eth_in_ = &ep;
            has_input_ = true;
            return dp::VoidRes::ok();
        }

        dp::VoidRes add_output(wirebit::SerialEndpoint &ep) {
            if (has_output_) {
                return dp::VoidRes::err(dp::Error::already_exists("gnss output already set"));
            }
            out_kind_ = OutKind::NMEA_SERIAL;
            serial_out_ = &ep;
            has_output_ = true;
            return dp::VoidRes::ok();
        }

        dp::VoidRes add_output(wirebit::CanEndpoint &ep) {
            if (has_output_) {
                return dp::VoidRes::err(dp::Error::already_exists("gnss output already set"));
            }
            out_kind_ = OutKind::J1939_CAN;
            can_out_ = &ep;
            has_output_ = true;
            return dp::VoidRes::ok();
        }

        dp::VoidRes add_output(wirebit::EthEndpoint &ep) {
            if (has_output_) {
                return dp::VoidRes::err(dp::Error::already_exists("gnss output already set"));
            }
            out_kind_ = OutKind::ETH;
            eth_out_ = &ep;
            has_output_ = true;
            return dp::VoidRes::ok();
        }

        dp::Error const &last_error() const noexcept { return last_error_; }

      private:
        enum class InKind { NONE, NMEA_SERIAL, J1939_CAN, ETH };
        enum class OutKind { NONE, NMEA_SERIAL, J1939_CAN, ETH };

        dp::VoidRes do_step() override {
            if (!has_input_) {
                return dp::VoidRes::ok();
            }

            if (in_kind_ == InKind::NMEA_SERIAL) {
                auto r = serial_in_->recv();
                if (!r.is_ok()) {
                    if (r.error().code == dp::Error::TIMEOUT) {
                        return dp::VoidRes::ok();
                    }
                    last_error_ = r.error();
                    return dp::VoidRes::err(r.error());
                }
                append_bytes_to_line_buffer(r.value());
                return consume_lines();
            }

            if (in_kind_ == InKind::J1939_CAN) {
                wirebit::can_frame frame{};
                auto r = can_in_->recv_can(frame);
                if (!r.is_ok()) {
                    if (r.error().code == dp::Error::TIMEOUT) {
                        return dp::VoidRes::ok();
                    }
                    last_error_ = r.error();
                    return dp::VoidRes::err(r.error());
                }
                auto res = nonsens::codec::gnss::j1939_to_pod(frame, pod_);
                if (!res.is_ok())
                    last_error_ = res.error();
                return res;
            }

            if (in_kind_ == InKind::ETH) {
                auto r = eth_in_->recv_eth();
                if (!r.is_ok()) {
                    if (r.error().code == dp::Error::TIMEOUT) {
                        return dp::VoidRes::ok();
                    }
                    last_error_ = r.error();
                    return dp::VoidRes::err(r.error());
                }
                (void)r;
                last_error_ = dp::Error::invalid_argument("gnss eth decode not implemented");
                return dp::VoidRes::err(last_error_);
            }

            return dp::VoidRes::ok();
        }

        dp::VoidRes do_push() override {
            if (!has_output_) {
                return dp::VoidRes::ok();
            }

            if (out_kind_ == OutKind::NMEA_SERIAL) {
                auto all = nonsens::codec::gnss::pod_to_nmea_all(pod_);
                if (!all.is_ok()) {
                    last_error_ = all.error();
                    return dp::VoidRes::err(all.error());
                }

                wirebit::Bytes bytes;
                bytes.reserve(all.value().size());
                for (char c : std::string_view(all.value().c_str(), all.value().size())) {
                    bytes.push_back(static_cast<wirebit::Byte>(c));
                }

                auto s = serial_out_->send(bytes);
                if (!s.is_ok()) {
                    last_error_ = s.error();
                    return dp::VoidRes::err(s.error());
                }
                return dp::VoidRes::ok();
            }

            if (out_kind_ == OutKind::J1939_CAN) {
                auto frames = nonsens::codec::gnss::pod_to_j1939(pod_);
                if (!frames.is_ok()) {
                    last_error_ = frames.error();
                    return dp::VoidRes::err(frames.error());
                }
                for (auto const &f : frames.value()) {
                    auto s = can_out_->send_can(f);
                    if (!s.is_ok()) {
                        last_error_ = s.error();
                        return dp::VoidRes::err(s.error());
                    }
                }
                return dp::VoidRes::ok();
            }

            if (out_kind_ == OutKind::ETH) {
                last_error_ = dp::Error::invalid_argument("gnss eth encode not implemented");
                return dp::VoidRes::err(last_error_);
            }

            return dp::VoidRes::ok();
        }

        void append_bytes_to_line_buffer(wirebit::Bytes const &bytes) {
            for (auto b : bytes) {
                char c = static_cast<char>(b);
                if (c == '\r' || c == '\n' || (c >= 0x20 && c <= 0x7E)) {
                    line_buf_.push_back(c);
                }
                if (line_buf_.size() > 8192) {
                    line_buf_.erase(0, line_buf_.size() - 4096);
                }
            }
        }

        dp::VoidRes consume_lines() {
            while (true) {
                auto pos = line_buf_.find('\n');
                if (pos == std::string::npos) {
                    break;
                }
                std::string line = line_buf_.substr(0, pos + 1);
                line_buf_.erase(0, pos + 1);

                auto start = line.find('$');
                if (start == std::string::npos) {
                    continue;
                }
                std::string_view sv(line.data() + start, line.size() - start);
                auto r = nonsens::codec::gnss::nmea_to_pod(sv, pod_);
                if (!r.is_ok()) {
                    last_error_ = r.error();
                }
            }
            return dp::VoidRes::ok();
        }

        bool has_input_{false};
        bool has_output_{false};
        InKind in_kind_{InKind::NONE};
        OutKind out_kind_{OutKind::NONE};

        wirebit::SerialEndpoint *serial_in_{nullptr};
        wirebit::CanEndpoint *can_in_{nullptr};
        wirebit::EthEndpoint *eth_in_{nullptr};

        wirebit::SerialEndpoint *serial_out_{nullptr};
        wirebit::CanEndpoint *can_out_{nullptr};
        wirebit::EthEndpoint *eth_out_{nullptr};

        std::string line_buf_{};
        dp::Error last_error_{};
    };

} // namespace nonsens::sensor

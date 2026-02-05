#pragma once

#include <chrono>
#include <string>

#include <datapod/datapod.hpp>
#include <wirebit/wirebit.hpp>

#include <nonsens/codec/gnss/j1939_to_pod.hpp>
#include <nonsens/codec/gnss/nmea2000_to_pod.hpp>
#include <nonsens/codec/gnss/nmea_to_pod.hpp>
#include <nonsens/codec/gnss/pod_to_j1939.hpp>
#include <nonsens/codec/gnss/pod_to_nmea.hpp>
#include <nonsens/codec/gnss/pod_to_nmea2000.hpp>

#include <nonsens/pods/gnss.hpp>
#include <nonsens/sensor/can_protocol.hpp>
#include <nonsens/sensor/sensor_base.hpp>

namespace nonsens::sensor {

    /// GNSS sensor translator (pod in the middle).
    ///
    /// - SerialEndpoint: NMEA0183
    /// - CanEndpoint: J1939 (PGN 0xFEF3, 0xFEE8)
    /// - EthEndpoint: reserved (stub)
    class Gnss final : public SensorBase<nonsens::pod::Gnss> {
      public:
        char const *name() const noexcept override { return "gnss"; }

        dp::VoidRes set_can_input_protocol(nonsens::sensor::CanProtocol proto) {
            if (in_kind_ != InKind::J1939_CAN && in_kind_ != InKind::NMEA2000_CAN) {
                return dp::VoidRes::err(
                    dp::Error::invalid_argument("gnss can input protocol: no can input configured"));
            }
            in_kind_ = (proto == nonsens::sensor::CanProtocol::NMEA2000) ? InKind::NMEA2000_CAN : InKind::J1939_CAN;
            return dp::VoidRes::ok();
        }

        dp::VoidRes set_can_output_protocol(nonsens::sensor::CanProtocol proto) {
            if (out_kind_ != OutKind::J1939_CAN && out_kind_ != OutKind::NMEA2000_CAN) {
                return dp::VoidRes::err(
                    dp::Error::invalid_argument("gnss can output protocol: no can output configured"));
            }
            out_kind_ = (proto == nonsens::sensor::CanProtocol::NMEA2000) ? OutKind::NMEA2000_CAN : OutKind::J1939_CAN;
            return dp::VoidRes::ok();
        }

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

        dp::VoidRes add_input_nmea2000(wirebit::CanEndpoint &ep) {
            if (has_input_) {
                return dp::VoidRes::err(dp::Error::already_exists("gnss input already set"));
            }
            in_kind_ = InKind::NMEA2000_CAN;
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

        dp::VoidRes add_output_nmea2000(wirebit::CanEndpoint &ep) {
            if (has_output_) {
                return dp::VoidRes::err(dp::Error::already_exists("gnss output already set"));
            }
            out_kind_ = OutKind::NMEA2000_CAN;
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
        enum class InKind { NONE, NMEA_SERIAL, J1939_CAN, NMEA2000_CAN, ETH };
        enum class OutKind { NONE, NMEA_SERIAL, J1939_CAN, NMEA2000_CAN, ETH };

        dp::VoidRes do_step() override {
            if (!has_input_) {
                return dp::VoidRes::ok();
            }

            if (in_kind_ == InKind::NMEA_SERIAL) {
                // Drain the serial RX queue so we decode the most recent fix.
                // Without this, a fast NMEA source + slow loop rate can build up backlog,
                // causing CAN outputs to lag behind real-time.
                constexpr size_t MAX_BYTES_PER_STEP = 256 * 1024;
                constexpr int MAX_READS_PER_STEP = 1024;

                size_t total_bytes = 0;
                bool got_any = false;

                for (int i = 0; i < MAX_READS_PER_STEP && total_bytes < MAX_BYTES_PER_STEP; ++i) {
                    auto r = serial_in_->recv();
                    if (!r.is_ok()) {
                        if (r.error().code == dp::Error::TIMEOUT) {
                            break;
                        }
                        last_error_ = r.error();
                        return dp::VoidRes::err(r.error());
                    }

                    got_any = true;
                    total_bytes += r.value().size();
                    append_bytes_to_line_buffer(r.value());
                }

                if (!got_any) {
                    return dp::VoidRes::ok();
                }
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

            if (in_kind_ == InKind::NMEA2000_CAN) {
                wirebit::can_frame frame{};
                auto r = can_in_->recv_can(frame);
                if (!r.is_ok()) {
                    if (r.error().code == dp::Error::TIMEOUT) {
                        return dp::VoidRes::ok();
                    }
                    last_error_ = r.error();
                    return dp::VoidRes::err(r.error());
                }
                auto res = nonsens::codec::gnss::nmea2000_to_pod(frame, pod_);
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

            if (out_kind_ == OutKind::NMEA2000_CAN) {
                auto now = std::chrono::steady_clock::now();
                bool send_gnss_position = false;
                if (!nmea2000_gnss_sent_) {
                    send_gnss_position = true;
                } else if (now - nmea2000_last_gnss_tp_ >= std::chrono::seconds(1)) {
                    send_gnss_position = true;
                }

                auto frames =
                    nonsens::codec::gnss::pod_to_nmea2000(pod_, nmea2000_sid_, nmea2000_fast_seq_, send_gnss_position);
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
                if (send_gnss_position) {
                    nmea2000_last_gnss_tp_ = now;
                    nmea2000_gnss_sent_ = true;
                    nmea2000_sid_ = static_cast<uint8_t>((nmea2000_sid_ + 1) % 253);
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

        // NMEA2000 sequencing (Fast Packet + related messages)
        uint8_t nmea2000_sid_{0};
        uint8_t nmea2000_fast_seq_{0};
        bool nmea2000_gnss_sent_{false};
        std::chrono::steady_clock::time_point nmea2000_last_gnss_tp_{};
    };

} // namespace nonsens::sensor

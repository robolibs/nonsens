#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
#include <thread>

#include <wirebit/can/socketcan_link.hpp>
#include <wirebit/serial/tty_link.hpp>
#include <wirebit/wirebit.hpp>

#include <nonsens/nonsens.hpp>

int main(int argc, char **argv) {
    const char *tty_path = (argc >= 2) ? argv[1] : "/dev/ttyUSB0";
    uint32_t baud = (argc >= 3) ? static_cast<uint32_t>(std::strtoul(argv[2], nullptr, 10)) : 115200;
    const char *can_if = (argc >= 4) ? argv[3] : "can0";

    auto tty = wirebit::TtyLink::create({.device = wirebit::String(tty_path), .baud = baud}).value();
    auto tty_link = std::make_shared<wirebit::TtyLink>(std::move(tty));

    wirebit::SerialConfig serial_cfg{};
    serial_cfg.baud = baud;
    wirebit::SerialEndpoint serial(tty_link, serial_cfg, 1);

    auto can_link_res = wirebit::SocketCanLink::create({
        .interface_name = wirebit::String(can_if),
        .create_if_missing = false,
        .destroy_on_close = false,
    });
    if (!can_link_res.is_ok()) {
        std::cerr << "failed to open SocketCAN interface '" << can_if
                  << "' (create it first or use vcan0): " << can_link_res.error().message.c_str() << "\n";
        return 1;
    }
    auto can_link = std::make_shared<wirebit::SocketCanLink>(std::move(can_link_res.value()));

    wirebit::CanConfig can_cfg{};
    can_cfg.bitrate = 250000;
    wirebit::CanEndpoint can(can_link, can_cfg, 2);

    std::cout << "GNSS TTY:   " << tty_path << " @ " << baud << "\n";
    std::cout << "CAN iface:  " << can_if << " (NMEA2000 out)\n";
    std::cout << "Bridge:     NMEA0183 -> NMEA2000 (PGN 129025, 129026, 129029 fast packet)\n";

    auto sres = nonsens::sensor::Sensor::create(nonsens::sensor::SensorType::GNSS);
    if (!sres.is_ok()) {
        std::cerr << "failed to create sensor: " << sres.error().message.c_str() << "\n";
        return 1;
    }
    auto sensor = std::move(sres.value());

    auto podv = sensor.pod();
    auto *gnss = dp::get<nonsens::pod::Gnss *>(podv);
    if (gnss == nullptr) {
        std::cerr << "failed to access gnss pod\n";
        return 1;
    }

    auto in = sensor.add_input(nonsens::sensor::Endpoint{&serial});
    if (!in.is_ok()) {
        std::cerr << "failed to set input: " << in.error().message.c_str() << "\n";
        return 1;
    }
    auto out = sensor.add_output(nonsens::sensor::Endpoint{&can});
    if (!out.is_ok()) {
        std::cerr << "failed to set output: " << out.error().message.c_str() << "\n";
        return 1;
    }

    auto proto = sensor.set_can_output_protocol(nonsens::sensor::CanProtocol::NMEA2000);
    if (!proto.is_ok()) {
        std::cerr << "failed to set CAN protocol: " << proto.error().message.c_str() << "\n";
        return 1;
    }

    auto last_print = std::chrono::steady_clock::now();
    while (true) {
        auto st = sensor.step();
        if (!st.is_ok()) {
            std::cerr << "step error: " << st.error().message.c_str() << "\n";
        }

        auto now = std::chrono::steady_clock::now();
        if (now - last_print > std::chrono::milliseconds(500)) {
            last_print = now;
            double t = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
            std::cout << std::fixed << std::setprecision(6) << "t=" << t << " " << std::setprecision(8)
                      << "pod lat=" << gnss->latitude << " lon=" << gnss->longitude << " track_deg=" << gnss->track_deg
                      << " speed_mps=" << gnss->speed_mps << "\n";
        }

        auto pu = sensor.push();
        if (!pu.is_ok()) {
            std::cerr << "push error: " << pu.error().message.c_str() << "\n";
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

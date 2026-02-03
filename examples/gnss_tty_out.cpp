#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <thread>

#include <wirebit/serial/tty_link.hpp>
#include <wirebit/wirebit.hpp>

#include <nonsens/nonsens.hpp>

int main(int argc, char **argv) {
    const char *dev = (argc >= 2) ? argv[1] : "/dev/ttyUSB0";
    uint32_t baud = (argc >= 3) ? static_cast<uint32_t>(std::strtoul(argv[2], nullptr, 10)) : 115200;

    auto tty = wirebit::TtyLink::create({.device = wirebit::String(dev), .baud = baud}).value();
    auto link = std::make_shared<wirebit::TtyLink>(std::move(tty));

    wirebit::SerialConfig serial_cfg{};
    serial_cfg.baud = baud;
    wirebit::SerialEndpoint serial(link, serial_cfg, 1);

    std::cout << "GNSS TTY output: " << dev << " (" << baud << " baud)\n";
    std::cout << "This example only outputs (manual pod updates).\n";

    nonsens::sensor::Gnss gnss;
    gnss.add_output(serial);

    // Start state.
    gnss.pod().status.status = nonsens::pod::Gnss::NavSatStatus::STATUS_FIX;
    gnss.pod().status.service = nonsens::pod::Gnss::NavSatStatus::SERVICE_GPS;
    gnss.pod().rtk_status = nonsens::pod::Gnss::RtkStatus::RTK_FIXED;
    gnss.pod().num_satellites = 10;
    gnss.pod().hdop = 0.8;
    gnss.pod().horizontal_accuracy = 0.02;
    gnss.pod().vertical_accuracy = 0.03;
    gnss.pod().phtg = true;

    double lat0 = 45.0;
    double lon0 = -93.0;
    double t = 0.0;

    while (true) {
        // Manual change: small circular motion.
        t += 0.2;
        double dlat = 1e-5 * std::sin(t);
        double dlon = 1e-5 * std::cos(t);

        gnss.pod().latitude = lat0 + dlat;
        gnss.pod().longitude = lon0 + dlon;
        gnss.pod().altitude = 300.0 + 0.5 * std::sin(t * 0.5);

        // Fake velocity in NEU (m/s)
        gnss.pod().velocity_neu[0] = 0.5 * std::cos(t); // north
        gnss.pod().velocity_neu[1] = 0.5 * std::sin(t); // east
        gnss.pod().velocity_neu[2] = 0.0;
        gnss.pod().speed_mps = std::sqrt(gnss.pod().velocity_neu[0] * gnss.pod().velocity_neu[0] +
                                         gnss.pod().velocity_neu[1] * gnss.pod().velocity_neu[1]);
        gnss.pod().track_deg = std::atan2(gnss.pod().velocity_neu[1], gnss.pod().velocity_neu[0]) * (180.0 / M_PI);
        if (gnss.pod().track_deg < 0.0)
            gnss.pod().track_deg += 360.0;

        auto r = gnss.push();
        if (!r.is_ok()) {
            std::cerr << "push error: " << gnss.last_error().message.c_str() << "\n";
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}

#include <chrono>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <memory>
#include <thread>

#include <wirebit/wirebit.hpp>

#include <nonsens/nonsens.hpp>

int main(int argc, char **argv) {
    // Convenience path so users can point other tools at a stable filename.
    // This is a symlink to the real /dev/pts/N slave created by PtyLink.
    const char *link_path = (argc >= 2) ? argv[1] : "/tmp/pty07";

    auto pty = wirebit::PtyLink::create({.raw_bytes = true}).value();
    auto slave_path = std::string(pty.slave_path().c_str());
    auto link = std::make_shared<wirebit::PtyLink>(std::move(pty));

    std::error_code ec;
    std::filesystem::remove(link_path, ec);
    std::filesystem::create_symlink(slave_path, link_path, ec);

    if (ec) {
        std::cerr << "Warning: failed to create symlink " << link_path << " -> " << slave_path << ": " << ec.message()
                  << "\n";
    }

    wirebit::SerialConfig serial_cfg{};
    wirebit::SerialEndpoint serial(link, serial_cfg, 1);

    std::cout << "GNSS PTY slave: " << slave_path << "\n";
    std::cout << "Symlink:       " << link_path << "\n";
    std::cout << "This example only outputs (manual pod updates).\n";

    nonsens::Nonsens ns;

    auto res = ns.add("gnss", nonsens::sensor::SensorType::GNSS);
    if (!res.is_ok()) {
        std::cerr << "failed to create sensor: " << res.error().message.c_str() << "\n";
        return 1;
    }

    auto *sensor = ns.get("gnss");

    auto w = sensor->add_output(nonsens::sensor::Endpoint{&serial});
    if (!w.is_ok()) {
        std::cerr << "failed to set output: " << w.error().message.c_str() << "\n";
        return 1;
    }

    auto podv = sensor->pod();
    auto *gnss = dp::get<nonsens::pod::Gnss *>(podv);

    gnss->status.status = nonsens::pod::Gnss::NavSatStatus::STATUS_FIX;
    gnss->status.service = nonsens::pod::Gnss::NavSatStatus::SERVICE_GPS;
    gnss->rtk_status = nonsens::pod::Gnss::RtkStatus::RTK_FIXED;
    gnss->num_satellites = 10;
    gnss->hdop = 0.8;
    gnss->horizontal_accuracy = 0.02;
    gnss->vertical_accuracy = 0.03;
    gnss->phtg = true;

    double lat0 = 45.0;
    double lon0 = -93.0;
    double t = 0.0;

    while (true) {
        t += 0.2;
        double dlat = 1e-5 * std::sin(t);
        double dlon = 1e-5 * std::cos(t);

        gnss->latitude = lat0 + dlat;
        gnss->longitude = lon0 + dlon;
        gnss->altitude = 300.0 + 0.5 * std::sin(t * 0.5);

        gnss->velocity_neu[0] = 0.5 * std::cos(t);
        gnss->velocity_neu[1] = 0.5 * std::sin(t);
        gnss->velocity_neu[2] = 0.0;
        gnss->speed_mps =
            std::sqrt(gnss->velocity_neu[0] * gnss->velocity_neu[0] + gnss->velocity_neu[1] * gnss->velocity_neu[1]);
        gnss->track_deg = std::atan2(gnss->velocity_neu[1], gnss->velocity_neu[0]) * (180.0 / M_PI);
        if (gnss->track_deg < 0.0) {
            gnss->track_deg += 360.0;
        }

        auto r = ns.push_all();
        if (!r.is_ok()) {
            std::cerr << "push error: " << r.error().message.c_str() << "\n";
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}

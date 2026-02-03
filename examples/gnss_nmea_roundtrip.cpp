#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

#include <wirebit/wirebit.hpp>

#include <nonsens/nonsens.hpp>

int main() {
    auto pty = wirebit::PtyLink::create().value();
    auto link = std::make_shared<wirebit::PtyLink>(std::move(pty));

    wirebit::SerialConfig serial_cfg{};
    wirebit::SerialEndpoint serial(link, serial_cfg, 1);

    std::cout << "GNSS PTY slave: " << link->slave_path().c_str() << "\n";
    std::cout << "Write NMEA to it; program will echo flatsim-style burst back.\n";

    auto sres = nonsens::sensor::Sensor::create(nonsens::sensor::SensorType::GNSS);
    if (!sres.is_ok()) {
        std::cerr << "failed to create sensor: " << sres.error().message.c_str() << "\n";
        return 1;
    }
    auto sensor = std::move(sres.value());

    sensor.add_input(nonsens::sensor::Endpoint{&serial});
    sensor.add_output(nonsens::sensor::Endpoint{&serial});

    while (true) {
        sensor.step();
        sensor.push();

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

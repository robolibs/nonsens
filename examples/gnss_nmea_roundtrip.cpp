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

    nonsens::sensor::Gnss gnss;
    gnss.add_input(serial);
    gnss.add_output(serial);

    while (true) {
        gnss.step();

        // Push periodically. This keeps the example simple.
        gnss.push();

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

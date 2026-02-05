#include <iomanip>
#include <iostream>
#include <string>

#include <nonsens/nonsens.hpp>

int main() {
    nonsens::pod::Gnss pod;

    std::string line;
    while (std::getline(std::cin, line)) {
        if (line.empty() || line[0] != '$') {
            continue;
        }

        // Preserve original line including possible CR.
        line.push_back('\n');
        auto res = nonsens::codec::gnss::nmea_to_pod(std::string_view(line.data(), line.size()), pod);
        if (!res.is_ok()) {
            continue;
        }

        std::cout << std::fixed << std::setprecision(8) << "lat=" << pod.latitude << " lon=" << pod.longitude
                  << " track_deg=" << pod.track_deg << " speed_mps=" << pod.speed_mps << "\n";
    }

    return 0;
}

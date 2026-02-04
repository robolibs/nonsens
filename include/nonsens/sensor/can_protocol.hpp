#pragma once

#include <datapod/datapod.hpp>

namespace nonsens::sensor {

    /// CAN navigation protocol selection for sensors that support CAN.
    enum class CanProtocol : dp::u8 {
        J1939 = 0,
        NMEA2000 = 1,
    };

} // namespace nonsens::sensor

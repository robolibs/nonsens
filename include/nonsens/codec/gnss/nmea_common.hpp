#pragma once

#include <cstdint>
#include <string>
#include <string_view>

#include <datapod/datapod.hpp>

namespace nonsens::codec::gnss::detail {

    inline uint8_t nmea_xor_checksum(std::string_view payload) {
        uint8_t csum = 0;
        for (char ch : payload) {
            csum ^= static_cast<uint8_t>(ch);
        }
        return csum;
    }

    inline char nybble_hex(uint8_t v) {
        v &= 0xF;
        return v < 10 ? static_cast<char>('0' + v) : static_cast<char>('A' + (v - 10));
    }

    inline dp::String make_sentence(std::string_view payload) {
        uint8_t c = nmea_xor_checksum(payload);
        char tail[5];
        tail[0] = '*';
        tail[1] = nybble_hex(static_cast<uint8_t>(c >> 4));
        tail[2] = nybble_hex(static_cast<uint8_t>(c));
        tail[3] = '\r';
        tail[4] = '\n';

        std::string out;
        out.reserve(1 + payload.size() + sizeof(tail));
        out.push_back('$');
        out.append(payload);
        out.append(tail, sizeof(tail));
        return dp::String(out.c_str());
    }

} // namespace nonsens::codec::gnss::detail

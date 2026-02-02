#pragma once

#include <datapod/datapod.hpp>
#include <wirebit/wirebit.hpp>

#include <nonsens/pods/gnss.hpp>

namespace nonsens::codec::gnss {

    inline dp::VoidRes j1939_to_pod(wirebit::can_frame const & /*frame*/, nonsens::pod::Gnss & /*out*/) {
        return dp::VoidRes::err(dp::Error::invalid_argument("j1939 gnss decode not implemented"));
    }

} // namespace nonsens::codec::gnss

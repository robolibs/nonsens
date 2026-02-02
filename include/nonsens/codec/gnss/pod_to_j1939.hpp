#pragma once

#include <datapod/datapod.hpp>
#include <wirebit/wirebit.hpp>

#include <nonsens/pods/gnss.hpp>

namespace nonsens::codec::gnss {

    inline dp::Res<dp::Vector<wirebit::can_frame>> pod_to_j1939(nonsens::pod::Gnss const & /*in*/) {
        return dp::Res<dp::Vector<wirebit::can_frame>>::err(
            dp::Error::invalid_argument("j1939 gnss encode not implemented"));
    }

} // namespace nonsens::codec::gnss

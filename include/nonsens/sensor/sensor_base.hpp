#pragma once

#include <datapod/datapod.hpp>

namespace nonsens::sensor {

    /// Base for a concrete sensor that translates between wire formats and a datapod pod.
    ///
    /// The key design goal is a unified control surface (`step()`, `push()`, `pod()`), while
    /// keeping wiring manual and sensor-specific (only the concrete sensor exposes valid
    /// `add_input/add_output` overloads).
    template <typename PodT> class SensorBase {
      public:
        using Pod = PodT;

        virtual ~SensorBase() = default;

        virtual char const *name() const noexcept = 0;

        Pod const &pod() const noexcept { return pod_; }
        Pod &pod() noexcept { return pod_; }
        void set_pod(Pod const &p) noexcept { pod_ = p; }

        dp::VoidRes step() { return do_step(); }
        dp::VoidRes push() { return do_push(); }

      protected:
        virtual dp::VoidRes do_step() = 0;
        virtual dp::VoidRes do_push() = 0;

        Pod pod_{};
    };

} // namespace nonsens::sensor

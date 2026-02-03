#pragma once

#include <datapod/datapod.hpp>

#include <nonsens/sensor/sensor.hpp>

namespace nonsens {

    /// Main interface for managing multiple named sensors.
    ///
    /// Provides a simple interface to create, store, and operate on
    /// multiple sensors by name.
    class Nonsens {
      public:
        Nonsens() = default;

        /// Create and add a sensor with the given name and type.
        /// Returns error if sensor with that name already exists.
        dp::VoidRes add(dp::String name, sensor::SensorType type) {
            if (sensors_.contains(name)) {
                return dp::VoidRes::err(dp::Error::invalid_argument("sensor with this name already exists"));
            }

            auto res = sensor::Sensor::create(type);
            if (!res.is_ok()) {
                return dp::VoidRes::err(res.error());
            }

            sensors_.insert(dp::Pair<dp::String, sensor::Sensor>{std::move(name), std::move(res.value())});
            return dp::VoidRes::ok();
        }

        /// Add an already-created sensor with the given name.
        /// Returns error if sensor with that name already exists.
        dp::VoidRes add(dp::String name, sensor::Sensor sensor) {
            if (sensors_.contains(name)) {
                return dp::VoidRes::err(dp::Error::invalid_argument("sensor with this name already exists"));
            }

            sensors_.insert(dp::Pair<dp::String, sensor::Sensor>{std::move(name), std::move(sensor)});
            return dp::VoidRes::ok();
        }

        /// Get a sensor by name. Returns nullptr if not found.
        sensor::Sensor *get(dp::String const &name) {
            auto it = sensors_.find(name);
            if (it == sensors_.end()) {
                return nullptr;
            }
            return &it->second;
        }

        /// Get a sensor by name (const). Returns nullptr if not found.
        sensor::Sensor const *get(dp::String const &name) const {
            auto it = sensors_.find(name);
            if (it == sensors_.end()) {
                return nullptr;
            }
            return &it->second;
        }

        /// Check if a sensor with the given name exists.
        bool contains(dp::String const &name) const { return sensors_.contains(name); }

        /// Remove a sensor by name. Returns true if removed, false if not found.
        bool remove(dp::String const &name) { return sensors_.erase(name) > 0; }

        /// Get the number of sensors.
        dp::usize size() const noexcept { return sensors_.size(); }

        /// Check if empty.
        bool empty() const noexcept { return sensors_.empty(); }

        /// Clear all sensors.
        void clear() { sensors_.clear(); }

        /// Call step() on all sensors. Returns first error encountered, or ok.
        dp::VoidRes step_all() {
            for (auto &[name, sensor] : sensors_) {
                auto res = sensor.step();
                if (!res.is_ok()) {
                    return res;
                }
            }
            return dp::VoidRes::ok();
        }

        /// Call push() on all sensors. Returns first error encountered, or ok.
        dp::VoidRes push_all() {
            for (auto &[name, sensor] : sensors_) {
                auto res = sensor.push();
                if (!res.is_ok()) {
                    return res;
                }
            }
            return dp::VoidRes::ok();
        }

        /// Call step() then push() on all sensors.
        dp::VoidRes update_all() {
            auto res = step_all();
            if (!res.is_ok()) {
                return res;
            }
            return push_all();
        }

        /// Direct access to the underlying map.
        dp::Map<dp::String, sensor::Sensor> &sensors() noexcept { return sensors_; }
        dp::Map<dp::String, sensor::Sensor> const &sensors() const noexcept { return sensors_; }

        /// Iterator support for range-based for loops.
        auto begin() noexcept { return sensors_.begin(); }
        auto end() noexcept { return sensors_.end(); }
        auto begin() const noexcept { return sensors_.begin(); }
        auto end() const noexcept { return sensors_.end(); }

      private:
        dp::Map<dp::String, sensor::Sensor> sensors_{};
    };

} // namespace nonsens

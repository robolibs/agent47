#pragma once

namespace agent47::model {

    /// Base interface for all agent47 entities.
    ///
    /// This exists so the runtime can be generalized beyond robots later
    /// (e.g., implements, stationary beacons, maps, etc.).
    struct Entity {
        virtual ~Entity() = default;
    };

} // namespace agent47::model

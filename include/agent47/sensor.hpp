#pragma once

#include <type_traits>

#include <datapod/datapod.hpp>

namespace agent47 {

    /// Typed sensor interface.
    ///
    /// Each concrete sensor chooses its own `Output` type and implements `read(Output&)`.
    /// This avoids forcing unrelated sensors into a single unified feedback struct.
    template <typename Output> class Sensor {
      public:
        static_assert(!std::is_reference_v<Output>, "Sensor<Output>: Output must not be a reference");
        virtual ~Sensor() = default;

        virtual bool connect(const dp::String &endpoint) = 0;
        virtual void disconnect() = 0;
        virtual bool is_connected() const = 0;

        /// Read latest sample into `out`.
        /// Returns false if no sample is available.
        virtual bool read(Output &out) = 0;
    };

} // namespace agent47

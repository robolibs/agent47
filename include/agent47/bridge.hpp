#pragma once

#include <string>

#include <datapod/pods/temporal/stamp.hpp>

#include "agent47/types.hpp"

namespace agent47 {

    /// Abstract bridge interface for bidirectional communication with a backend
    /// (simulator, real robot, ROS2 node, etc.).
    class Bridge {
      public:
        virtual ~Bridge() = default;

        virtual bool connect(const std::string &endpoint) = 0;
        virtual void disconnect() = 0;
        virtual bool is_connected() const = 0;

        virtual bool send(const dp::Stamp<types::Command> &cmd) = 0;
        virtual bool recv(dp::Stamp<types::Feedback> &fb, dp::i32 timeout_ms = 100) = 0;
    };

} // namespace agent47

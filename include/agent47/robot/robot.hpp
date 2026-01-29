#pragma once

#include <datapod/pods/adapters/result.hpp>

#include "model.hpp"
#include "runtime.hpp"

namespace agent47 {
    namespace robot {

        struct Robot {
            Model model;
            Runtime runtime;

            static dp::Result<Robot> from_urdf_string(const dp::String &xml) {
                auto m = Model::from_urdf_string(xml);
                if (m.is_err()) {
                    return dp::Result<Robot>::err(m.error());
                }
                Robot r;
                r.model = m.value();
                r.runtime.resize(r.model.joints.size(), r.model.sensors.size());
                return dp::Result<Robot>::ok(r);
            }
        };

    } // namespace robot
} // namespace agent47

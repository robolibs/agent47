#pragma once

#include <echo/echo.hpp>
#include <memory>

#include <drivekit/tracker.hpp>
#include <echo/echo.hpp>
#include <farmtrax/farmtrax.hpp>
#include <filesystem>
#include <iostream>
#include <netpipe/netpipe.hpp>

#include <datapod/pods/adapters/error.hpp>
#include <datapod/pods/adapters/optional.hpp>
#include <datapod/pods/adapters/result.hpp>

#include <datapod/pods/temporal/stamp.hpp>

#include "agent47/bridge.hpp"
#include "agent47/model/urdf.hpp"
#include "agent47/types.hpp"

namespace agent47 {
    class Agent {
      public:
        dp::u8 speed = 1;
        dp::robot::Robot model_;
        std::shared_ptr<Bridge> bridge_;
        dp::Geo datum;
        dp::Odom odom;
        dp::Optional<dp::Twist> cmd;
        dp::Optional<dp::Path> path;

      private:
        dp::Optional<farmtrax::Farmtrax> farmtrax_;
        dp::Optional<drivekit::Tracker> tracker_;

      public:
        explicit Agent(dp::robot::Robot model, Bridge *bridge) : model_(std::move(model)) {
            if (bridge) {
                bridge_ = std::shared_ptr<Bridge>(bridge, [](Bridge *) {});
            }
        }

        explicit Agent(dp::String model_urdf, dp::robot::Identity RoboId, Bridge *bridge) {
            if (bridge) {
                bridge_ = std::shared_ptr<Bridge>(bridge, [](Bridge *) {});
            }
            std::filesystem::path urdf_path = std::filesystem::path(model_urdf.c_str());
            if (!std::filesystem::exists(urdf_path)) {
                echo::critical("Model file does not exist: ", urdf_path);
            }
            auto model_result = agent47::from_urdf_string(model_urdf);
            if (model_result.is_err()) {
                echo::critical("Failed to load model: ", model_result.error());
            }
            model_.model = model_result.value();
            model_.id = RoboId;
        }

        inline void set_velocity(const datapod::Twist &t) { cmd = t; }
        inline void brake() { cmd = datapod::Twist{}; }

        inline void update(const types::Feedback &fb) {
            odom.pose = fb.pose;
            odom.twist = fb.twist;
        }

        inline dp::Result<types::Command> tick(dp::f64 dt_s) {
            dp::Stamp<types::Feedback> fb;
            bool have_fb = false;
            if (bridge_) {
                have_fb = bridge_->recv(fb);
            }
            if (!have_fb) {
                fb.timestamp = dp::Stamp<types::Feedback>::now();
                fb.value.pose = odom.pose;
                fb.value.twist = odom.twist;
            }
            auto result = tick(fb, dt_s);

            echo::trace("Pose: ", fb.value.pose);
            echo::trace("Twist: ", fb.value.twist);

            if (bridge_ && result.is_ok()) {
                bridge_->send(dp::Stamp<types::Command>{fb.timestamp, result.value()});
            }
            return result;
        }

        inline dp::Result<types::Command> tick(const dp::Stamp<types::Feedback> &fb, dp::f64 dt_s) {
            update(fb.value);
            types::Command out;
            if (cmd.has_value()) {
                const auto s = speed;
                out.valid = true;
                out.twist.linear = odom.twist.linear * s;
                out.twist.angular = odom.twist.angular * s;
                return dp::Result<types::Command>::ok(out);
            }
            if (!tracker_) {
                return dp::Result<types::Command>::err(dp::Error::invalid_argument("drivekit not attached"));
            }
            drivekit::RobotState st;
            st.pose = fb.value.pose;
            st.timestamp = static_cast<dp::f64>(fb.timestamp) * 1e-9;
            st.velocity.linear = fb.value.twist.linear.vx;
            st.velocity.angular = fb.value.twist.angular.vz;
            auto cmd = tracker_->tick(st, static_cast<dp::f32>(dt_s), nullptr);
            out.valid = cmd.valid;
            if (cmd.valid) {
                const auto s = speed;
                out.twist.linear.vx = cmd.linear_velocity * s;
                out.twist.angular.vz = cmd.angular_velocity * s;
            }
            return dp::Result<types::Command>::ok(out);
        }

        inline drivekit::Tracker *drivekit_tracker() { return tracker_ ? &(*tracker_) : nullptr; }
        inline const drivekit::Tracker *drivekit_tracker() const { return tracker_ ? &(*tracker_) : nullptr; }
    };

} // namespace agent47

#pragma once

#include <memory>

#include <drivekit/tracker.hpp>
#include <echo/echo.hpp>
#include <farmtrax/farmtrax.hpp>
#include <netpipe/netpipe.hpp>

#include <datapod/pods/adapters/error.hpp>
#include <datapod/pods/adapters/optional.hpp>
#include <datapod/pods/adapters/result.hpp>

#include <datapod/pods/temporal/stamp.hpp>

#include "agent47/bridge.hpp"
#include "agent47/model/robot.hpp"
#include "agent47/types.hpp"

namespace agent47 {
    class Agent {
      public:
        model::Robot model_;
        std::shared_ptr<Bridge> bridge_;
        dp::Geo datum;
        dp::Pose pose;
        dp::Optional<dp::Twist> twist;
        dp::Optional<dp::Path> path;

      private:
        dp::Optional<farmtrax::Farmtrax> farmtrax_;
        dp::Optional<drivekit::Tracker> tracker_;

      public:
        /// Construct an agent around a robot model and an optional backend bridge.
        ///
        /// The bridge is non-owning and must outlive the Agent.
        explicit Agent(model::Robot model, Bridge *bridge) : model_(std::move(model)) {
            if (bridge) {
                bridge_ = std::shared_ptr<Bridge>(bridge, [](Bridge *) {
                    // Non-owning: do not delete.
                });
            }
        }

        inline void set_velocity(const datapod::Twist &t) { twist = t; }
        inline void clear_velocity() { twist.reset(); }
        inline void brake() { twist = datapod::Twist{}; }

        inline void update(const types::Feedback &fb) {
            model_.runtime.pose = fb.pose;
            model_.runtime.twist = fb.twist;

            // Keep the public convenience state in sync with the model runtime.
            // This allows users to read `agent.pose` directly after tick().
            pose = fb.pose;
            twist = fb.twist;
        }

        inline dp::Result<types::Command> tick(dp::f64 dt_s) {
            dp::Stamp<types::Feedback> fb;
            bool have_fb = false;
            if (bridge_) {
                have_fb = bridge_->recv(fb);
            }
            if (!have_fb) {
                fb.timestamp = dp::Stamp<types::Feedback>::now();
                fb.value.pose = model_.runtime.pose;
                fb.value.twist = model_.runtime.twist;
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
            out.timestamp_ns = fb.timestamp;
            if (twist.has_value()) {
                auto s = static_cast<dp::f64>(model_.runtime.speed_scale);
                out.valid = true;
                out.twist.linear = twist->linear * s;
                out.twist.angular = twist->angular * s;
                return dp::Result<types::Command>::ok(out);
            }
            if (!tracker_) {
                return dp::Result<types::Command>::err(dp::Error::invalid_argument("drivekit not attached"));
            }
            drivekit::RobotState st;
            st.pose = fb.value.pose;
            st.allow_move = model_.runtime.allow_move;
            st.allow_reverse = model_.runtime.allow_reverse;
            st.timestamp = static_cast<dp::f64>(fb.timestamp) * 1e-9;
            st.velocity.linear = fb.value.twist.linear.vx;
            st.velocity.angular = fb.value.twist.angular.vz;
            auto cmd = tracker_->tick(st, static_cast<dp::f32>(dt_s), nullptr);
            out.valid = cmd.valid;
            if (cmd.valid) {
                auto s = static_cast<dp::f64>(model_.runtime.speed_scale);
                out.twist.linear.vx = cmd.linear_velocity * s;
                out.twist.angular.vz = cmd.angular_velocity * s;
            }
            return dp::Result<types::Command>::ok(out);
        }

        inline drivekit::Tracker *drivekit_tracker() { return tracker_ ? &(*tracker_) : nullptr; }
        inline const drivekit::Tracker *drivekit_tracker() const { return tracker_ ? &(*tracker_) : nullptr; }
    };

} // namespace agent47

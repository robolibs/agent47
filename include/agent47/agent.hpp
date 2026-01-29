#pragma once

#include <memory>

#include <drivekit/tracker.hpp>
#include <farmtrax/farmtrax.hpp>
#include <netpipe/netpipe.hpp>

#include <datapod/pods/adapters/error.hpp>
#include <datapod/pods/adapters/optional.hpp>
#include <datapod/pods/adapters/result.hpp>

#include "agent47/bridge.hpp"
#include "agent47/model/robot.hpp"
#include "agent47/types.hpp"

namespace agent47 {
    class Agent {
      private:
        model::Robot model_;
        dp::Optional<farmtrax::Farmtrax> farmtrax_;
        dp::Optional<drivekit::Tracker> tracker_;
        std::shared_ptr<Bridge> bridge_;

        dp::Geo datum;
        dp::Pose pose;
        dp::Optional<dp::Twist> twist;
        dp::Optional<dp::Path> path;

      public:
        Agent() = default;

        explicit Agent(model::Robot model) : model_(std::move(model)) {}

        inline model::Robot &model() { return model_; }
        inline const model::Robot &model() const { return model_; }

        // -- Bridge --

        inline void set_bridge(std::shared_ptr<Bridge> b) { bridge_ = std::move(b); }
        inline Bridge *bridge() const { return bridge_.get(); }

        // -- Manual velocity control --

        inline void set_manual_velocity(const datapod::Twist &t) { twist = t; }

        inline void set_manual_velocity(dp::f64 linear, dp::f64 angular) {
            twist = datapod::Twist{{linear, 0.0, 0.0}, {0.0, 0.0, angular}};
        }

        inline void clear_manual_velocity() { twist.reset(); }

        inline void brake() { twist = datapod::Twist{}; }

        inline void set_speed_scale(dp::f32 s) { model_.runtime.speed_scale = s; }

        // -- Modules --

        inline dp::Result<bool> attach_farmtrax() {
            farmtrax_.emplace();
            return dp::Result<bool>::ok(true);
        }

        inline farmtrax::Farmtrax *farmtrax() { return farmtrax_ ? &(*farmtrax_) : nullptr; }
        inline const farmtrax::Farmtrax *farmtrax() const { return farmtrax_ ? &(*farmtrax_) : nullptr; }

        inline bool has_farmtrax() const { return farmtrax_.has_value(); }

        /// Attach DriveKit using constraints stored in the model.
        /// Returns false if constraints are missing.
        inline dp::Result<bool> attach_drivekit() {
            if (!model_.body.constraints.has_value()) {
                return dp::Result<bool>::err(dp::Error::invalid_argument("missing robot constraints"));
            }
            tracker_.emplace();
            tracker_->init(*model_.body.constraints, nullptr, "agent47");
            return dp::Result<bool>::ok(true);
        }

        /// Attach DriveKit and store constraints into the model.
        inline dp::Result<bool> attach_drivekit(const drivekit::RobotConstraints &constraints) {
            model_.body.constraints = constraints;
            return attach_drivekit();
        }

        // -- Update / Tick --

        /// Update model runtime from a Feedback message.
        inline void update(const types::Feedback &fb) {
            model_.runtime.pose = fb.pose;
            model_.runtime.twist = fb.twist;
        }

        /// Tick using the model's current runtime state (no bridge).
        inline dp::Result<types::Command> tick(dp::f64 dt_s) {
            types::Feedback fb;
            bool have_fb = false;

            if (bridge_) {
                have_fb = bridge_->recv(fb);
            }

            if (!have_fb) {
                // Build feedback from model state.
                fb.pose = model_.runtime.pose;
                fb.twist = model_.runtime.twist;
            }

            auto result = tick(fb, dt_s);

            if (bridge_ && result.is_ok()) {
                bridge_->send(result.value());
            }

            return result;
        }

        /// Tick using an explicit feedback (standalone / test mode).
        inline dp::Result<types::Command> tick(const types::Feedback &fb, dp::f64 dt_s) {
            update(fb);

            types::Command out;
            out.stamp_s = fb.stamp_s;

            // Manual override takes precedence.
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
            st.pose = fb.pose;
            st.allow_move = model_.runtime.allow_move;
            st.allow_reverse = model_.runtime.allow_reverse;
            st.timestamp = fb.stamp_s;
            st.velocity.linear = fb.twist.linear.vx;
            st.velocity.angular = fb.twist.angular.vz;

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

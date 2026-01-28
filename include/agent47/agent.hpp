#pragma once

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
        dp::Optional<netpipe::Pipe> pipe_;

        Bridge *bridge_ = nullptr;

        // Manual velocity override.
        dp::Optional<double> manual_linear_;
        dp::Optional<double> manual_angular_;

      public:
        Agent() = default;

        explicit Agent(model::Robot model) : model_(std::move(model)) {}

        inline model::Robot &model() { return model_; }
        inline const model::Robot &model() const { return model_; }

        // -- Bridge --

        inline void set_bridge(Bridge *b) { bridge_ = b; }
        inline Bridge *bridge() const { return bridge_; }

        // -- Manual velocity control --

        inline void set_manual_velocity(double linear, double angular) {
            manual_linear_ = linear;
            manual_angular_ = angular;
        }

        inline void clear_manual_velocity() {
            manual_linear_.reset();
            manual_angular_.reset();
        }

        inline void brake() { set_manual_velocity(0.0, 0.0); }

        inline void set_speed_scale(float s) { model_.runtime.speed_scale = s; }

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
            model_.runtime.tick_seq = fb.tick_seq;
            model_.runtime.stamp_s = fb.stamp_s;
            model_.runtime.pose = fb.pose;
            model_.runtime.linear_mps = fb.linear_mps;
            model_.runtime.angular_rps = fb.angular_rps;

            model_.sensors.data.tick_seq = fb.tick_seq;
            model_.sensors.data.stamp_s = fb.stamp_s;
            model_.sensors.data.has_lidar = fb.has_lidar;
            model_.sensors.data.has_gps = fb.has_gps;
            model_.sensors.data.has_imu = fb.has_imu;
            if (fb.has_lidar) {
                model_.sensors.data.lidar = fb.lidar;
            }
            if (fb.has_gps) {
                model_.sensors.data.gps = fb.gps;
            }
            if (fb.has_imu) {
                model_.sensors.data.imu = fb.imu;
            }
        }

        /// Tick using the model's current runtime state (no bridge).
        inline dp::Result<types::Command> tick(double dt_s) {
            types::Feedback fb;
            bool have_fb = false;

            if (bridge_) {
                have_fb = bridge_->recv(fb);
            }

            if (!have_fb) {
                // Build feedback from model state.
                fb.tick_seq = model_.runtime.tick_seq;
                fb.stamp_s = model_.runtime.stamp_s;
                fb.pose = model_.runtime.pose;
                if (model_.runtime.linear_mps.has_value()) {
                    fb.linear_mps = *model_.runtime.linear_mps;
                }
                if (model_.runtime.angular_rps.has_value()) {
                    fb.angular_rps = *model_.runtime.angular_rps;
                }
            }

            auto result = tick(fb, dt_s);

            if (bridge_ && result.is_ok()) {
                bridge_->send(result.value());
            }

            return result;
        }

        /// Tick using an explicit feedback (standalone / test mode).
        inline dp::Result<types::Command> tick(const types::Feedback &fb, double dt_s) {
            update(fb);

            types::Command out;
            out.stamp_s = fb.stamp_s;

            // Manual override takes precedence.
            if (manual_linear_.has_value() || manual_angular_.has_value()) {
                out.valid = true;
                out.linear_mps = manual_linear_.has_value() ? *manual_linear_ : 0.0;
                out.angular_rps = manual_angular_.has_value() ? *manual_angular_ : 0.0;
                out.linear_mps *= model_.runtime.speed_scale;
                out.angular_rps *= model_.runtime.speed_scale;
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
            st.velocity.linear = fb.linear_mps;
            st.velocity.angular = fb.angular_rps;

            auto cmd = tracker_->tick(st, static_cast<float>(dt_s), nullptr);
            out.valid = cmd.valid;
            if (cmd.valid) {
                out.linear_mps = cmd.linear_velocity * model_.runtime.speed_scale;
                out.angular_rps = cmd.angular_velocity * model_.runtime.speed_scale;
            }
            return dp::Result<types::Command>::ok(out);
        }

        inline drivekit::Tracker *drivekit_tracker() { return tracker_ ? &(*tracker_) : nullptr; }
        inline const drivekit::Tracker *drivekit_tracker() const { return tracker_ ? &(*tracker_) : nullptr; }
    };

} // namespace agent47

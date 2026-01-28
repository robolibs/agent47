#pragma once

#pragma once

#include <drivekit/tracker.hpp>
#include <farmtrax/farmtrax.hpp>
#include <netpipe/netpipe.hpp>

#include <datapod/pods/adapters/error.hpp>
#include <datapod/pods/adapters/optional.hpp>
#include <datapod/pods/adapters/result.hpp>

#include "agent47/model/robot.hpp"
#include "agent47/types.hpp"

namespace agent47 {
    class Agent {
      private:
        model::Robot model_;
        dp::Optional<farmtrax::Farmtrax> farmtrax_;
        dp::Optional<drivekit::Tracker> tracker_;
        dp::Optional<netpipe::Pipe> pipe_;

      public:
        Agent() = default;

        explicit Agent(model::Robot model) : model_(std::move(model)) {}

        inline model::Robot &model() { return model_; }
        inline const model::Robot &model() const { return model_; }

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

        /// Convenience: update model runtime from an Observation.
        inline void update(const types::Observation &obs) {
            model_.runtime.tick_seq = obs.tick_seq;
            model_.runtime.stamp_s = obs.stamp_s;
            model_.runtime.pose = obs.pose;
            model_.runtime.linear_mps = obs.linear_mps;
            model_.runtime.angular_rps = obs.angular_rps;
            model_.runtime.allow_move = obs.allow_move;
            model_.runtime.allow_reverse = obs.allow_reverse;
            if (!obs.robot_id.empty()) {
                model_.identity.uuid = obs.robot_id;
            }
        }

        /// Tick using the model's current runtime state.
        inline dp::Result<types::Command> tick(double dt_s) {
            types::Observation obs;
            obs.robot_id = model_.identity.uuid;
            obs.tick_seq = model_.runtime.tick_seq;
            obs.stamp_s = model_.runtime.stamp_s;
            obs.pose = model_.runtime.pose;
            obs.linear_mps = model_.runtime.linear_mps;
            obs.angular_rps = model_.runtime.angular_rps;
            obs.allow_move = model_.runtime.allow_move;
            obs.allow_reverse = model_.runtime.allow_reverse;
            return tick(obs, dt_s);
        }

        /// Tick using an explicit observation (also updates the model).
        inline dp::Result<types::Command> tick(const types::Observation &obs, double dt_s) {
            update(obs);

            types::Command out;
            out.stamp_s = obs.stamp_s;

            if (!tracker_) {
                return dp::Result<types::Command>::err(dp::Error::invalid_argument("drivekit not attached"));
            }

            drivekit::RobotState st;
            st.pose = obs.pose;
            st.allow_move = obs.allow_move;
            st.allow_reverse = obs.allow_reverse;
            st.timestamp = obs.stamp_s;
            if (obs.linear_mps.has_value()) {
                st.velocity.linear = *obs.linear_mps;
            }
            if (obs.angular_rps.has_value()) {
                st.velocity.angular = *obs.angular_rps;
            }

            auto cmd = tracker_->tick(st, static_cast<float>(dt_s), nullptr);
            out.valid = cmd.valid;
            if (cmd.valid) {
                out.linear_mps = cmd.linear_velocity;
                out.angular_rps = cmd.angular_velocity;
                out.status = cmd.status_message;
            } else {
                out.status = "no active goal/path";
            }
            return dp::Result<types::Command>::ok(out);
        }

        inline drivekit::Tracker *drivekit_tracker() { return tracker_ ? &(*tracker_) : nullptr; }
        inline const drivekit::Tracker *drivekit_tracker() const { return tracker_ ? &(*tracker_) : nullptr; }
    };

} // namespace agent47

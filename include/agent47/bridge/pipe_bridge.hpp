#pragma once

#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>

#include <netpipe/netpipe.hpp>

#include "agent47/bridge.hpp"

namespace agent47 {

    // Protocol method IDs shared between agent47 and any netpipe backend.
    static constexpr uint32_t AGENT47_METHOD_COMMAND = 1;  // agent47 -> backend
    static constexpr uint32_t AGENT47_METHOD_FEEDBACK = 2; // backend -> agent47

    /// Bridge implementation using netpipe::Pipe + netpipe::Remote<Bidirect>.
    ///
    /// Endpoint strings:
    ///   tcp://host:port
    ///   ipc:///path
    ///   shm://name:size
    class PipeBridge : public Bridge {
      public:
        PipeBridge() = default;
        ~PipeBridge() override { disconnect(); }

        PipeBridge(const PipeBridge &) = delete;
        PipeBridge &operator=(const PipeBridge &) = delete;

        bool connect(const std::string &endpoint) override {
            auto ep = parse_endpoint(endpoint);
            if (!ep.has_value()) {
                return false;
            }

            auto res = netpipe::Pipe::connect(*ep);
            if (res.is_err()) {
                return false;
            }

            pipe_.emplace(std::move(res.value()));

            rpc_ = std::make_unique<netpipe::Remote<netpipe::Bidirect>>(
                *pipe_->stream().get(),
                /*max_concurrent=*/100,
                /*enable_metrics=*/false,
                /*recv_timeout_ms=*/100,
                /*handler_threads=*/2,
                /*max_handler_queue=*/100,
                /*handler_timeout_ms=*/0,
                /*max_incoming=*/100);

            // Backend calls us on AGENT47_METHOD_FEEDBACK with serialised Feedback.
            rpc_->register_method(
                AGENT47_METHOD_FEEDBACK, [this](const netpipe::Message &msg) -> dp::Res<netpipe::Message> {
                    types::Feedback fb;
                    if (deserialize_feedback(msg, fb)) {
                        std::lock_guard<std::mutex> lock(fb_mutex_);
                        last_fb_ = std::move(fb);
                        fb_ready_ = true;
                        fb_cv_.notify_one();
                    }
                    return dp::result::ok(netpipe::Message{});
                });

            return true;
        }

        void disconnect() override {
            rpc_.reset();
            if (pipe_.has_value()) {
                pipe_->close();
                pipe_.reset();
            }
        }

        bool is_connected() const override { return pipe_.has_value() && pipe_->is_connected(); }

        bool send(const types::Command &cmd) override {
            if (!rpc_ || !is_connected()) {
                return false;
            }
            auto msg = serialize_command(cmd);
            auto res = rpc_->call(AGENT47_METHOD_COMMAND, msg, 1000);
            return res.is_ok();
        }

        bool recv(types::Feedback &fb, int timeout_ms = 100) override {
            std::unique_lock<std::mutex> lock(fb_mutex_);
            if (!fb_ready_) {
                fb_cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms), [this] { return fb_ready_; });
            }
            if (fb_ready_) {
                fb = last_fb_;
                fb_ready_ = false;
                return true;
            }
            return false;
        }

      private:
        dp::Optional<netpipe::Pipe> pipe_;
        std::unique_ptr<netpipe::Remote<netpipe::Bidirect>> rpc_;

        mutable std::mutex fb_mutex_;
        std::condition_variable fb_cv_;
        types::Feedback last_fb_;
        bool fb_ready_ = false;

        // ---------------------------------------------------------------------------
        // Endpoint parsing
        // ---------------------------------------------------------------------------

        static dp::Optional<netpipe::AnyEndpoint> parse_endpoint(const std::string &s) {
            if (s.rfind("tcp://", 0) == 0) {
                auto body = s.substr(6);
                auto colon = body.rfind(':');
                if (colon == std::string::npos) {
                    return {};
                }
                auto host = body.substr(0, colon);
                auto port = static_cast<uint16_t>(std::stoi(body.substr(colon + 1)));
                return netpipe::AnyEndpoint::tcp_endpoint(dp::String(host.c_str()), port);
            }
            if (s.rfind("ipc://", 0) == 0) {
                auto path = s.substr(6);
                return netpipe::AnyEndpoint::ipc_endpoint(dp::String(path.c_str()));
            }
            if (s.rfind("shm://", 0) == 0) {
                auto body = s.substr(6);
                auto colon = body.rfind(':');
                if (colon == std::string::npos) {
                    return {};
                }
                auto name = body.substr(0, colon);
                auto size = static_cast<size_t>(std::stoull(body.substr(colon + 1)));
                return netpipe::AnyEndpoint::shm_endpoint(dp::String(name.c_str()), size);
            }
            return {};
        }

        // ---------------------------------------------------------------------------
        // Wire helpers -- pack/unpack POD values into a netpipe::Message
        // ---------------------------------------------------------------------------

        template <typename T> static void write_val(netpipe::Message &buf, const T &val) {
            const uint8_t *p = reinterpret_cast<const uint8_t *>(&val);
            for (size_t i = 0; i < sizeof(T); ++i) {
                buf.push_back(p[i]);
            }
        }

        template <typename T> static T read_val(const uint8_t *&ptr) {
            T val;
            std::memcpy(&val, ptr, sizeof(T));
            ptr += sizeof(T);
            return val;
        }

        // ---------------------------------------------------------------------------
        // Command serialisation (agent47 -> backend)
        //
        // Wire: [stamp_s:8][linear_mps:8][angular_rps:8][valid:1]
        // ---------------------------------------------------------------------------

        static netpipe::Message serialize_command(const types::Command &cmd) {
            netpipe::Message msg;
            write_val(msg, cmd.stamp_s);
            write_val(msg, cmd.linear_mps);
            write_val(msg, cmd.angular_rps);
            write_val(msg, static_cast<uint8_t>(cmd.valid ? 1 : 0));
            return msg;
        }

        // ---------------------------------------------------------------------------
        // Feedback deserialisation (backend -> agent47)
        //
        // Wire layout:
        //   [tick_seq:8][stamp_s:8]
        //   [pose.point x/y/z:24][pose.rotation w/x/y/z:32]
        //   [linear_mps:8][angular_rps:8]
        //   [num_wheels:4] { [angle_rad:8][speed_rps:8] } * N
        //   [flags:1]  (bit0=lidar, bit1=gps, bit2=imu)
        //   if lidar:
        //     [n_ranges:4][ranges_m: n*4][n_angles:4][angles_rad: n*4]
        //     [n_valid:4][valid: n*1][min_range:4][max_range:4]
        //   if gps:
        //     [lat:8][lon:8][alt:8][heading:4][speed:4]
        //   if imu:
        //     [ax:4][ay:4][az:4][gz:4][yaw:4]
        // ---------------------------------------------------------------------------

        static constexpr size_t FEEDBACK_FIXED_SIZE = 8 + 8 + 24 + 32 + 8 + 8 + 4 + 1; // 93

        static bool deserialize_feedback(const netpipe::Message &msg, types::Feedback &fb) {
            if (msg.size() < FEEDBACK_FIXED_SIZE) {
                return false;
            }

            const uint8_t *ptr = msg.data();

            fb.tick_seq = read_val<uint64_t>(ptr);
            fb.stamp_s = read_val<double>(ptr);

            fb.pose.point.x = read_val<double>(ptr);
            fb.pose.point.y = read_val<double>(ptr);
            fb.pose.point.z = read_val<double>(ptr);
            fb.pose.rotation.w = read_val<double>(ptr);
            fb.pose.rotation.x = read_val<double>(ptr);
            fb.pose.rotation.y = read_val<double>(ptr);
            fb.pose.rotation.z = read_val<double>(ptr);

            fb.linear_mps = read_val<double>(ptr);
            fb.angular_rps = read_val<double>(ptr);

            uint32_t num_wheels = read_val<uint32_t>(ptr);
            fb.wheels.resize(num_wheels);
            for (uint32_t i = 0; i < num_wheels; ++i) {
                fb.wheels[i].angle_rad = read_val<double>(ptr);
                fb.wheels[i].speed_rps = read_val<double>(ptr);
            }

            uint8_t flags = read_val<uint8_t>(ptr);
            fb.has_lidar = (flags & 0x01) != 0;
            fb.has_gps = (flags & 0x02) != 0;
            fb.has_imu = (flags & 0x04) != 0;

            if (fb.has_lidar) {
                uint32_t n_ranges = read_val<uint32_t>(ptr);
                fb.lidar.ranges_m.resize(n_ranges);
                for (uint32_t i = 0; i < n_ranges; ++i) {
                    fb.lidar.ranges_m[i] = read_val<float>(ptr);
                }
                uint32_t n_angles = read_val<uint32_t>(ptr);
                fb.lidar.angles_rad.resize(n_angles);
                for (uint32_t i = 0; i < n_angles; ++i) {
                    fb.lidar.angles_rad[i] = read_val<float>(ptr);
                }
                uint32_t n_valid = read_val<uint32_t>(ptr);
                fb.lidar.valid.resize(n_valid);
                for (uint32_t i = 0; i < n_valid; ++i) {
                    fb.lidar.valid[i] = read_val<uint8_t>(ptr);
                }
                fb.lidar.min_range_m = read_val<float>(ptr);
                fb.lidar.max_range_m = read_val<float>(ptr);
            }

            if (fb.has_gps) {
                fb.gps.latitude = read_val<double>(ptr);
                fb.gps.longitude = read_val<double>(ptr);
                fb.gps.altitude = read_val<double>(ptr);
                fb.gps.heading_rad = read_val<float>(ptr);
                fb.gps.speed_mps = read_val<float>(ptr);
            }

            if (fb.has_imu) {
                fb.imu.accel_x = read_val<float>(ptr);
                fb.imu.accel_y = read_val<float>(ptr);
                fb.imu.accel_z = read_val<float>(ptr);
                fb.imu.gyro_z = read_val<float>(ptr);
                fb.imu.yaw_rad = read_val<float>(ptr);
            }

            return true;
        }
    };

} // namespace agent47

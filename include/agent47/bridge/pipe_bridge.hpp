#pragma once

#include <condition_variable>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>

#include <netpipe/netpipe.hpp>

#include "agent47/bridge.hpp"

namespace agent47 {

    // Protocol method IDs shared between agent47 and any netpipe backend.
    static constexpr dp::u32 AGENT47_METHOD_COMMAND = 1;  // agent47 -> backend
    static constexpr dp::u32 AGENT47_METHOD_FEEDBACK = 2; // backend -> agent47

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

            rpc_ = std::make_unique<netpipe::Remote<netpipe::Bidirect>>(*pipe_->stream().get(),
                                                                        /*max_concurrent=*/100,
                                                                        /*enable_metrics=*/false,
                                                                        /*recv_timeout_ms=*/100,
                                                                        /*handler_threads=*/2,
                                                                        /*max_handler_queue=*/100,
                                                                        /*handler_timeout_ms=*/0,
                                                                        /*max_incoming=*/100);

            // Backend calls us on AGENT47_METHOD_FEEDBACK with serialised Feedback.
            rpc_->register_method(AGENT47_METHOD_FEEDBACK,
                                  [this](const netpipe::Message &msg) -> dp::Res<netpipe::Message> {
                                      dp::Stamp<types::Feedback> fb;
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

        bool send(const dp::Stamp<types::Command> &cmd) override {
            if (!rpc_ || !is_connected()) {
                return false;
            }
            auto msg = serialize_command(cmd);
            auto res = rpc_->call(AGENT47_METHOD_COMMAND, msg, 1000);
            return res.is_ok();
        }

        bool recv(dp::Stamp<types::Feedback> &fb, dp::i32 timeout_ms = 100) override {
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
        dp::Stamp<types::Feedback> last_fb_;
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
                auto port = static_cast<dp::u16>(std::stoi(body.substr(colon + 1)));
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
                auto size = static_cast<dp::usize>(std::stoull(body.substr(colon + 1)));
                return netpipe::AnyEndpoint::shm_endpoint(dp::String(name.c_str()), size);
            }
            return {};
        }

        // ---------------------------------------------------------------------------
        // Wire helpers -- pack/unpack POD values into a netpipe::Message
        // ---------------------------------------------------------------------------

        template <typename T> static void write_val(netpipe::Message &buf, const T &val) {
            const dp::u8 *p = reinterpret_cast<const dp::u8 *>(&val);
            for (dp::usize i = 0; i < sizeof(T); ++i) {
                buf.push_back(p[i]);
            }
        }

        template <typename T> static T read_val(const dp::u8 *&ptr) {
            T val;
            std::memcpy(&val, ptr, sizeof(T));
            ptr += sizeof(T);
            return val;
        }

        // ---------------------------------------------------------------------------
        // Command serialisation (agent47 -> backend)
        //
        // Wire: [timestamp_ns:8]
        //       [twist.linear  vx/vy/vz:24]
        //       [twist.angular vx/vy/vz:24]
        //       [valid:1]
        // ---------------------------------------------------------------------------

        static netpipe::Message serialize_command(const dp::Stamp<types::Command> &cmd) {
            netpipe::Message msg;
            write_val(msg, static_cast<dp::i64>(cmd.timestamp));
            write_val(msg, cmd.value.twist.linear.vx);
            write_val(msg, cmd.value.twist.linear.vy);
            write_val(msg, cmd.value.twist.linear.vz);
            write_val(msg, cmd.value.twist.angular.vx);
            write_val(msg, cmd.value.twist.angular.vy);
            write_val(msg, cmd.value.twist.angular.vz);
            write_val(msg, static_cast<dp::u8>(cmd.value.valid ? 1 : 0));
            return msg;
        }

        // ---------------------------------------------------------------------------
        // Feedback deserialisation (backend -> agent47)
        //
        // Wire layout:
        //   [timestamp_ns:8]
        //   [pose.point x/y/z:24][pose.rotation w/x/y/z:32]
        //   [twist.linear vx/vy/vz:24][twist.angular vx/vy/vz:24]
        //   [num_wheels:4] { [angle_rad:8][speed_rps:8] } * N
        //   [flags:1]  (bit0=lidar, bit1=gnss, bit2=imu)
        //   if lidar:
        //     [n_ranges:4][ranges_m: n*4][n_angles:4][angles_rad: n*4]
        //     [n_valid:4][valid: n*1][min_range:4][max_range:4]
        //   if gnss:
        //     [lat:8][lon:8][alt:8][heading:4][speed:4]
        //   if imu:
        //     [ax:4][ay:4][az:4][gz:4][yaw:4]
        // ---------------------------------------------------------------------------

        static constexpr dp::usize FEEDBACK_FIXED_SIZE = 8 + 24 + 32 + 24 + 24 + 4 + 1; // 117

        static bool deserialize_feedback(const netpipe::Message &msg, dp::Stamp<types::Feedback> &fb) {
            if (msg.size() < FEEDBACK_FIXED_SIZE) {
                return false;
            }

            const dp::u8 *ptr = msg.data();

            fb.timestamp = read_val<dp::i64>(ptr);

            fb.value.pose.point.x = read_val<dp::f64>(ptr);
            fb.value.pose.point.y = read_val<dp::f64>(ptr);
            fb.value.pose.point.z = read_val<dp::f64>(ptr);
            fb.value.pose.rotation.w = read_val<dp::f64>(ptr);
            fb.value.pose.rotation.x = read_val<dp::f64>(ptr);
            fb.value.pose.rotation.y = read_val<dp::f64>(ptr);
            fb.value.pose.rotation.z = read_val<dp::f64>(ptr);

            fb.value.twist.linear.vx = read_val<dp::f64>(ptr);
            fb.value.twist.linear.vy = read_val<dp::f64>(ptr);
            fb.value.twist.linear.vz = read_val<dp::f64>(ptr);
            fb.value.twist.angular.vx = read_val<dp::f64>(ptr);
            fb.value.twist.angular.vy = read_val<dp::f64>(ptr);
            fb.value.twist.angular.vz = read_val<dp::f64>(ptr);

            dp::u32 num_wheels = read_val<dp::u32>(ptr);
            fb.value.wheels.resize(num_wheels);
            for (dp::u32 i = 0; i < num_wheels; ++i) {
                fb.value.wheels[i].angle_rad = read_val<dp::f64>(ptr);
                fb.value.wheels[i].speed_rps = read_val<dp::f64>(ptr);
            }
            dp::u8 flags = read_val<dp::u8>(ptr);
            return true;
        }
    };

} // namespace agent47

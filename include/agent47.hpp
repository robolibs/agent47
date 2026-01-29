#pragma once

// Single-include convenience header for agent47.

#include "agent47/agent.hpp"
#include "agent47/bridge.hpp"
#include "agent47/bridge/pipe_bridge.hpp"
#include "agent47/model/entity.hpp"
#include "agent47/model/robot.hpp"
#include "agent47/sensor.hpp"
#include "agent47/sensors/gnss.hpp"
#include "agent47/sensors/imu.hpp"
#include "agent47/sensors/lidar.hpp"
#include "agent47/types.hpp"

#ifdef AGENT47_HAS_ROS2
#include "agent47/bridge/ros2_bridge.hpp"
#endif

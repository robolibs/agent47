# Raw ROS2 linking helpers (no ament).
#
# This file is intentionally minimal and grows over time.
# It does not call find_package(rclcpp) because that pulls in ament and python.

function(agent47_enable_ros2_raw target)
    if(NOT TARGET ${target})
        message(FATAL_ERROR "agent47_enable_ros2_raw: target '${target}' does not exist")
    endif()

    set(ROS2_PREFIX "/opt/ros/jazzy" CACHE PATH "ROS2 install prefix")
    target_compile_definitions(${target} PUBLIC AGENT47_HAS_ROS2)
    target_include_directories(${target} SYSTEM PUBLIC
        "${ROS2_PREFIX}/include"
    )

    # Some ROS2 installs (notably some Jazzy layouts) place headers under
    # ${prefix}/include/<pkg>/<pkg>/... while including as <pkg/...>.
    # Add all one-level include subdirectories to avoid per-package whack-a-mole.
    file(GLOB _ros2_include_roots LIST_DIRECTORIES true "${ROS2_PREFIX}/include/*")
    foreach(_inc IN LISTS _ros2_include_roots)
        if(IS_DIRECTORY "${_inc}")
            target_include_directories(${target} SYSTEM PUBLIC "${_inc}")
        endif()
    endforeach()
    target_link_directories(${target} PUBLIC
        "${ROS2_PREFIX}/lib"
    )

    # Keep runtime lookup simple for non-standard prefixes.
    set_property(TARGET ${target} APPEND PROPERTY BUILD_RPATH "${ROS2_PREFIX}/lib")

    # Minimal set for rclcpp + message types used by ros2_bridge.hpp.
    # We'll extend this list as needed.
    target_link_libraries(${target}
        PUBLIC
        rclcpp
        libstatistics_collector
        rcl
        rcutils
        rmw
        rmw_implementation
        rcl_logging_interface
        rcl_yaml_param_parser
        tracetools

        # rosidl runtime / typesupport
        rosidl_runtime_c
        rosidl_typesupport_c
        rosidl_typesupport_cpp
        rosidl_typesupport_introspection_c
        rosidl_typesupport_introspection_cpp
        rosidl_typesupport_fastrtps_c
        rosidl_typesupport_fastrtps_cpp
        rosidl_dynamic_typesupport

        # message packages (C/C++ typesupport)
        std_msgs__rosidl_typesupport_cpp
        geometry_msgs__rosidl_typesupport_cpp
        nav_msgs__rosidl_typesupport_cpp
        sensor_msgs__rosidl_typesupport_cpp
        statistics_msgs__rosidl_typesupport_cpp
        turtlesim__rosidl_typesupport_cpp

        std_msgs__rosidl_typesupport_c
        geometry_msgs__rosidl_typesupport_c
        nav_msgs__rosidl_typesupport_c
        sensor_msgs__rosidl_typesupport_c
        statistics_msgs__rosidl_typesupport_c
        turtlesim__rosidl_typesupport_c
    )

    # Common POSIX libs used by ROS2.
    target_link_libraries(${target}
        PUBLIC
            dl
            pthread
    )
endfunction()

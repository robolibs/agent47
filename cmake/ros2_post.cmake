# Apply raw ROS2 linking to all targets when enabled.

if(NOT DEFINED PROJECT_NAME_UPPER)
    string(TOUPPER ${PROJECT_NAME} PROJECT_NAME_UPPER)
endif()

if(NOT ${PROJECT_NAME_UPPER}_HAS_ROS2)
    return()
endif()

if(NOT COMMAND agent47_enable_ros2_raw)
    include(ros2_raw)
endif()

if(NOT COMMAND agent47_enable_ros2_raw)
    message(FATAL_ERROR "ROS2 raw enabled but cmake/ros2_raw.cmake not available")
endif()

get_property(_all_targets DIRECTORY PROPERTY BUILDSYSTEM_TARGETS)
foreach(_t IN LISTS _all_targets)
    get_target_property(_type ${_t} TYPE)
    if(NOT _type)
        continue()
    endif()
    if(_type STREQUAL "INTERFACE_LIBRARY")
        continue()
    endif()

    agent47_enable_ros2_raw(${_t})
endforeach()

message(STATUS "ROS2 raw enabled")

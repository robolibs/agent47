# ROS2 toggle + raw linking hook (no ament)

if(NOT DEFINED PROJECT_NAME_UPPER)
    string(TOUPPER ${PROJECT_NAME} PROJECT_NAME_UPPER)
endif()

option(${PROJECT_NAME_UPPER}_HAS_ROS2 "Enable ROS2 bridge adapter" OFF)

function(agent47_ros2_raw_link target)
    if(NOT ${PROJECT_NAME_UPPER}_HAS_ROS2)
        return()
    endif()
    if(NOT COMMAND agent47_enable_ros2_raw)
        include(ros2_raw)
    endif()
    agent47_enable_ros2_raw(${target})
endfunction()

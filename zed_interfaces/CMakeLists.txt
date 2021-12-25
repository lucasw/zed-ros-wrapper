cmake_minimum_required(VERSION 3.5)

project(zed_interfaces)

#Fix for QtCretor
#list(APPEND CMAKE_PREFIX_PATH "/opt/ros/$ENV{ROS_DISTRO}/")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  sensor_msgs
  std_msgs
)

add_message_files(
    DIRECTORY msg
    FILES
    BoundingBox2Df.msg
    BoundingBox2Di.msg
    BoundingBox3D.msg
    Keypoint2Df.msg
    Keypoint2Di.msg
    Keypoint3D.msg
    Object.msg
    ObjectsStamped.msg
    RGBDSensors.msg
    Skeleton2D.msg
    Skeleton3D.msg
  )

add_service_files(
    DIRECTORY srv
    FILES
    reset_odometry.srv
    reset_tracking.srv
    set_led_status.srv
    set_pose.srv
    start_3d_mapping.srv
    start_object_detection.srv
    start_remote_stream.srv
    start_svo_recording.srv
    stop_3d_mapping.srv
    stop_object_detection.srv
    stop_remote_stream.srv
    stop_svo_recording.srv
    toggle_led.srv
  )

generate_messages(
    DEPENDENCIES
    geometry_msgs
    sensor_msgs
    std_msgs
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
    geometry_msgs
    message_generation
    message_runtime
    sensor_msgs
    std_msgs
)

###############################################################################

#Add all files in subdirectories of the project in
# a dummy_target so qtcreator have access to all files
file(GLOB_RECURSE all_files ${CMAKE_SOURCE_DIR}/*)
add_custom_target(all_files_${PROJECT_NAME} SOURCES ${all_files})



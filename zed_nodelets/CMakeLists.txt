cmake_minimum_required(VERSION 3.5)

project(zed_nodelets)

# if CMAKE_BUILD_TYPE is not specified, take 'Release' as default
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

function(check_package package customMessage)
    set(varName "${package}_FOUND")
    if (NOT "${${varName}}")
        string(REPLACE "_" "-" aptPackage ${package})
        if("${customMessage}" STREQUAL "")
            message(FATAL_ERROR "\n\n ${package} is missing, please try to install it with:\n sudo apt-get install ros-$(rosversion -d)-${aptPackage}\n\n")
        else()
            message(FATAL_ERROR "\n\n ${customMessage} \n\n")
        endif()
    endif()
endfunction()

find_package(ZED 3)
check_package("ZED" "ZED SDK v3.x not found, install it from:\n https://www.stereolabs.com/developers/")

exec_program(uname ARGS -p OUTPUT_VARIABLE CMAKE_SYSTEM_NAME2)
if ( CMAKE_SYSTEM_NAME2 MATCHES "aarch64" ) # Jetson TX
    set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
endif()

find_package(CUDA)
check_package("CUDA" "CUDA not found, install it from:\n https://developer.nvidia.com/cuda-downloads")

find_package(OpenMP)
check_package("OpenMP" "OpenMP not found, please install it to improve performances: 'sudo apt install libomp-dev'")
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

find_package(catkin REQUIRED COMPONENTS
    diagnostic_updater
    dynamic_reconfigure
    image_transport
    message_filters
    message_generation
    nav_msgs
    nodelet
    rosconsole
    roscpp
    rosgraph_msgs
    sensor_msgs
    std_msgs
    stereo_msgs
    tf2_geometry_msgs
    tf2_ros
    zed_interfaces
)

generate_dynamic_reconfigure_options(
  cfg/Zed.cfg
)

catkin_package(
  CATKIN_DEPENDS
    dynamic_reconfigure
    image_transport
    message_filters
    message_runtime
    nav_msgs
    nodelet
    rosconsole
    roscpp
    rosgraph_msgs
    sensor_msgs
    stereo_msgs
    tf2_geometry_msgs
    tf2_ros
    zed_interfaces
)

add_definitions(-Wno-deprecated-declarations)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${ZED_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/rgbd_sensor_demux.cpp
  src/rgbd_sensor_sync.cpp
  src/sl_tools.cpp
  src/zed_wrapper_nodelet.cpp
)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

# link_directories(${ZED_LIBRARY_DIR})
# TODO(lucasw) did the zed library libsl_nodelet.so used to get the full
# path baked in here?  It isn't getting found later during install,
# but putting the zed sdk lib dir on LD_LIBRARY_PATH makes it found
# where it is failing to be found)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${ZED_LIBRARIES}
  ${CUDA_LIBRARIES}
)

# TODO(lucasw) not sure if this is needed
# if(catkin_EXPORTED_LIBRARIES)
# message(STATUS "${catkin_EXPORTED_TARGETS}")
# add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
# endif()

###############################################################################
# INSTALL

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  nodelet_${PROJECT_NAME}.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

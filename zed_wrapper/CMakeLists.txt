cmake_minimum_required(VERSION 3.5)

project(zed_wrapper)

# if CMAKE_BUILD_TYPE is not specified, take 'Release' as default
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
  nodelet
  rosconsole
  roscpp
  xacro
  zed_nodelets
)

catkin_package(
  CATKIN_DEPENDS
    nodelet
    rosconsole
    roscpp
    zed_nodelets
)

###############################################################################
# SOURCES

set(NODE_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}_node.cpp)
###############################################################################

###############################################################################
# INCLUDES

# Specify locations of header files.
include_directories(
        ${catkin_INCLUDE_DIRS}
        ${CUDA_INCLUDE_DIRS}
        ${ZED_INCLUDE_DIRS}
)

###############################################################################

###############################################################################
# EXECUTABLE

set(LINK_LIBRARIES
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node ${NODE_SRC})
target_link_libraries(${PROJECT_NAME}_node ${LINK_LIBRARIES})
# target_link_directories(${PROJECT_NAME}_node ${ZED_LIBRARY_DIR})
# target_link_directories(${PROJECT_NAME}_node ${CUDA_LIBRARY_DIRS})
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})


###############################################################################
#Add all files in subdirectories of the project in

# a dummy_target so qtcreator have access to all files
file(GLOB_RECURSE all_files ${CMAKE_SOURCE_DIR}/*)
add_custom_target(all_files_${PROJECT_NAME} SOURCES ${all_files})

###############################################################################
# INSTALL

install(TARGETS
  ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY
  launch
  params
  urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

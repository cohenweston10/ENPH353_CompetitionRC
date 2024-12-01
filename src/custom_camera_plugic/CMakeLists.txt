cmake_minimum_required(VERSION 3.0.2)
project(custom_camera_plugin)

# Find dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  gazebo_plugins
)

find_package(gazebo REQUIRED)

# Include directories
include_directories(${GAZEBO_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

# Declare the plugin library
add_library(custom_camera_plugin SHARED src/custom_camera_plugin.cpp)
target_link_libraries(custom_camera_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

# Install plugin library
install(TARGETS custom_camera_plugin
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

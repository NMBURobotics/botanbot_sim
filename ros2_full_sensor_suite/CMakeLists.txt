################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(ros2_full_sensor_suite)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

find_package(ignition-common3 QUIET COMPONENTS profiler)
if(ignition-common3_FOUND)
  add_definitions("-DIGN_PROFILER_ENABLE=1" "-DIGN_PROFILER_REMOTERY=1")
  message(STATUS "Profiler is active")
else()
  message(WARNING "Can't find Ignition common3. Profiler will not be actived")
  add_definitions("-DIGN_PROFILER_ENABLE=0" "-DIGN_PROFILER_REMOTERY=0")
endif()
 
################################################################################
# Build
################################################################################
include_directories(
  include 
  ${GAZEBO_INCLUDE_DIRS}
 )

set(dependencies
    camera_info_manager  
    gazebo
    gazebo_dev
    gazebo_msgs
    gazebo_ros
    gazebo_ros_pkgs
    geometry_msgs
    image_transport
    cv_bridge
    nav_msgs
    rclcpp
    sensor_msgs
    std_msgs
    std_srvs
 )

#  REALSENSE CAMERA GAZEBO PLUGIN SHARED LIBRARY
add_library(gazebo_ros_realsense_camera_plugin SHARED  src/gazebo_ros_realsense_camera.cpp
                                            src/realsense_ros_plugin.cpp)
ament_target_dependencies(gazebo_ros_realsense_camera_plugin ${dependencies})
target_link_libraries(gazebo_ros_realsense_camera_plugin ${GAZEBO_LIBRARIES})

#  VELODYNE LIDAR GAZEBO PLUGIN SHARED LIBRARY
add_library(gazebo_ros_velodyne_laser SHARED src/gazebo_ros_velodyne_laser.cpp)
ament_target_dependencies(gazebo_ros_velodyne_laser ${dependencies})

#  HECTOR GPS SENSOR GAZEBO PLUGIN SHARED LIBRARY
add_library(gazebo_ros_gps SHARED src/gazebo_ros_gps.cpp)
ament_target_dependencies(gazebo_ros_gps ${dependencies})

#  HECTOR imu SENSOR GAZEBO PLUGIN SHARED LIBRARY
add_library(gazebo_ros_imu SHARED src/gazebo_ros_imu.cpp)
ament_target_dependencies(gazebo_ros_imu ${dependencies})

# this world plugin is used to attach sensor suite model to given robot body name
add_library(sensor_base_attacher_plugin SHARED src/sensor_base_attacher_plugin.cpp)
ament_target_dependencies(sensor_base_attacher_plugin ${dependencies})

ament_export_libraries(gazebo_ros_realsense_camera_plugin 
                       gazebo_ros_velodyne_laser 
                       gazebo_ros_gps
                       gazebo_ros_imu
                       sensor_base_attacher_plugin)
################################################################################
# Install
################################################################################
install(TARGETS 
        gazebo_ros_realsense_camera_plugin
        gazebo_ros_velodyne_laser
        gazebo_ros_gps
        gazebo_ros_imu
        sensor_base_attacher_plugin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(DIRECTORY launch models worlds urdf
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY include/
  DESTINATION include/
)

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()

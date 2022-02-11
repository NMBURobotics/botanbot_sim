################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(botanbot_gazebo)

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
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(vox_nav_msgs REQUIRED)
find_package(OCTOMAP REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED)

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
  ${OCTOMAP_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS})

set(dependencies
    camera_info_manager  
    gazebo
    gazebo_dev
    gazebo_msgs
    gazebo_ros
    geometry_msgs
    image_transport
    cv_bridge
    nav_msgs
    rclcpp
    sensor_msgs
    std_msgs
    std_srvs
    tf2
    tf2_geometry_msgs
    tf2_ros
    trajectory_msgs
    vox_nav_msgs
    OCTOMAP
    octomap_msgs
    Eigen3
    pcl_ros)

# botanbot ACKERMANN DRIVE GAZEBO PLUGIN SHARED LIBRARY
add_library(botanbot_ackermann_drive SHARED  src/botanbot_ackermann_drive.cpp)
target_include_directories(botanbot_ackermann_drive PUBLIC include)
ament_target_dependencies(botanbot_ackermann_drive ${dependencies})
target_include_directories(botanbot_ackermann_drive PUBLIC ${ignition-common3_INCLUDE_DIRS})
target_link_libraries(botanbot_ackermann_drive ${ignition-common3_LIBRARIES})

# botanbot GAZEBO WORLD TO OCTOMAP PLUGIN SHARED LIBRARY
add_library(gazebo_world_to_octomap SHARED src/gazebo_to_octomap.cpp)
ament_target_dependencies(gazebo_world_to_octomap ${dependencies})

# botanbot GAZEBO WORLD TO POINTCLOUD PLUGIN SHARED LIBRARY
add_library(gazebo_world_to_pointcloud SHARED src/gazebo_to_pointcloud.cpp)
ament_target_dependencies(gazebo_world_to_pointcloud ${dependencies})

ament_export_libraries(botanbot_ackermann_drive 
                       gazebo_world_to_octomap
                       gazebo_world_to_pointcloud)
################################################################################
# Install
################################################################################
install(TARGETS 
        botanbot_ackermann_drive
        gazebo_world_to_octomap
        gazebo_world_to_pointcloud
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(DIRECTORY launch models worlds
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

# Copyright (c) 2021 Fetullah Atas, Norwegian University of Life Sciences
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.substitutions import Command
import os

def generate_launch_description():

    # Get hare directories of thorvald packages
    ros2_full_sensor_suite_share_dir = get_package_share_directory(
        'ros2_full_sensor_suite')

    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_config = LaunchConfiguration("rviz_config")

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='whether to use or not sim time.')

    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(
            ros2_full_sensor_suite_share_dir, 'rviz', 'rviz.rviz'),
        description='...')

    # DECLARE THE ROBOT STATE PUBLISHER NODE
    xacro_full_dir = os.path.join(
        ros2_full_sensor_suite_share_dir, 'urdf', 'test.xacro')
    declare_robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': Command(['xacro ', xacro_full_dir])}],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
    )

    # START GAZEBO ONLY IF use_simulator IS SET TO TRUE
    gazebo_world = os.path.join(
        ros2_full_sensor_suite_share_dir, 'worlds', 'test.world')
    declare_start_gazebo_cmd = ExecuteProcess(
        cmd=['gazebo', '--verbose', gazebo_world,
             '-s', 'libgazebo_ros_factory.so'],
        output='screen')

    # SPAWN THE ROBOT TO GAZEBO IF use_simulator, FROM THE TOPIC "robot_description"
    declare_spawn_entity_to_gazebo_node = Node(package='gazebo_ros',
                                               executable='spawn_entity.py',
                                               arguments=[
                                                   '-entity', '',
                                                   '-topic', '/robot_description'],
                                               output='screen')

    return LaunchDescription([
        declare_use_sim_time,
        declare_rviz_config,
        declare_robot_state_publisher_node,
        declare_spawn_entity_to_gazebo_node,
        declare_start_gazebo_cmd,
    ])

# Copyright (c) 2018 Intel Corporation
# Copyright (c) 2020 Fetullah Atas, Norwegian University of Life Sciences
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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

GAZEBO_WORLD = os.environ['GAZEBO_WORLD']


def generate_launch_description():
    # Get the launch directory
    botanbot_bringup_dir = get_package_share_directory('botanbot_bringup')
    vox_nav_bringup_dir = get_package_share_directory('vox_nav_bringup')

    params = LaunchConfiguration('params')
    localization_params = LaunchConfiguration('localization_params')
    rviz_config = LaunchConfiguration('rviz_config')
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Launch configuration variables specific to simulation
    use_simulator = LaunchConfiguration('use_simulator')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')
    joy_config_filepath = LaunchConfiguration('config_filepath')

    urdf = os.path.join(get_package_share_directory('botanbot_description'),
                        'urdf/botanbot.urdf')

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    decleare_params = DeclareLaunchArgument(
        'params',
        default_value=os.path.join(
            botanbot_bringup_dir, 'params', 'params.yaml'),
        description='Path to the vox_nav parameters file.')

    decleare_localization_params = DeclareLaunchArgument(
        'localization_params',
        default_value=os.path.join(
            botanbot_bringup_dir, 'params', 'localization_params.yaml'),
        description='Path to the localization parameters file.')

    decleare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(
            botanbot_bringup_dir, 'rviz', 'vox_nav_default_view.rviz'),
        description='Path to the rviz config file.')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the vox_nav')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator',
        default_value='True',
        description='Whether to start the simulator')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz', default_value='True', description='Whether to start RVIZ')

    declare_simulator_cmd = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Whether to execute gzclient)')
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(
            get_package_share_directory('botanbot_gazebo'), 'worlds/',
            GAZEBO_WORLD, GAZEBO_WORLD + '.world'),
        description='Full path to world model file to load')

    declare_joy_config_filepath = DeclareLaunchArgument(
        'config_filepath',
        default_value=os.path.join(
            get_package_share_directory('botanbot_bringup'), 'params', 'xbox.yaml'),
        description='path to locks params.')

    # Specify the actions
    start_gazebo_server_cmd = ExecuteProcess(
        condition=IfCondition(use_simulator),
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so', world],
        cwd=[os.path.join(botanbot_bringup_dir, 'launch')],
        output='screen')

    start_gazebo_client_cmd = ExecuteProcess(condition=IfCondition(
        PythonExpression([use_simulator, ' and not ', headless])),
        cmd=['gzclient'],
        cwd=[os.path.join(botanbot_bringup_dir, 'launch')],
        output='screen')

    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        remappings=remappings,
        arguments=[urdf])

    bringup_cmd = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(vox_nav_bringup_dir, 'launch', 'bringup_vox_nav.launch.py')),
        launch_arguments={
        'params': params,
        'namespace': namespace,
        'use_namespace': use_namespace,
        'use_sim_time': use_sim_time,
    }.items())

    joy_config_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('teleop_twist_joy'), 'launch', 'teleop-launch.py')]),
        launch_arguments={
            'config_filepath': joy_config_filepath}.items()
    )

    twist_mux_cmd = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('twist_mux'), 'launch', 'twist_mux_launch.py')),
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(decleare_params)
    ld.add_action(decleare_localization_params)
    ld.add_action(decleare_rviz_config)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_joy_config_filepath)

    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)

    # Add the actions to launch all of the vox_nav nodes
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(bringup_cmd)
    ld.add_action(twist_mux_cmd)
    ld.add_action(joy_config_cmd)

    return ld

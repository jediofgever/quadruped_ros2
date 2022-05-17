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
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch.substitutions import PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

import os

# This will only take in effect if you are running THorvald in Simulation
os.environ['GAZEBO_MODEL_PATH'] = os.path.join(get_package_share_directory('champ_gazebo'),
                                               'models')


def generate_launch_description():

    # Get hare directories of thorvald packages
    champ_bringup_share_dir = get_package_share_directory(
        'champ_bringup')
    champ_description_share_dir = get_package_share_directory(
        'champ_description')

    use_simulator = LaunchConfiguration('use_simulator')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration("use_rviz")
    tf_prefix = LaunchConfiguration('tf_prefix')
    rviz_config = LaunchConfiguration("rviz_config")
    joy_config_filepath = LaunchConfiguration('config_filepath')
    link_params = LaunchConfiguration('link_params')

    declare_use_simulator = DeclareLaunchArgument(
        'use_simulator',
        default_value='True',
        description='whether to use Gazebo Simulation.')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='whether to use or not sim time.')
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='...')
    declare_tf_prefix = DeclareLaunchArgument(
        'tf_prefix',
        default_value='',
        description='...')
    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(
            champ_bringup_share_dir, 'rviz', 'thorvald_default_view.rviz'),
        description='...')
    declare_joy_config_filepath = DeclareLaunchArgument(
        'config_filepath',
        default_value=os.path.join(
            champ_bringup_share_dir, 'config', 'joystick_xbox.yaml'),
        description='path to locks params.')

    # DECLARE THE ROBOT STATE PUBLISHER NODE
    xacro_file_name = 'champ.urdf.xacro'
    xacro_full_dir = os.path.join(
        champ_description_share_dir, 'urdf', xacro_file_name)
    declare_robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{"use_sim_time": use_sim_time},
                    {'robot_description': Command(['xacro ', xacro_full_dir])}],
        remappings=[('/tf', 'tf'),
                    ('/tf_static', 'tf_static')])

    # SPAWN THE ROBOT TO GAZEBO IF use_simulator, FROM THE TOPIC "robot_description"
    declare_spawn_entity_to_gazebo_node = Node(package='gazebo_ros',
                                               condition=IfCondition(
                                                   use_simulator),
                                               executable='spawn_entity.py',
                                               parameters=[
                                                   {"use_sim_time": use_sim_time}],
                                               arguments=[
                                                   '-entity', '',
                                                   '-topic', '/robot_description'],
                                               output='screen')

    # START GAZEBO ONLY IF use_simulator IS SET TO TRUE
    gazebo_world = os.path.join(
        get_package_share_directory('champ_gazebo'), 'worlds/', 'void.world'),
    declare_start_gazebo_cmd = ExecuteProcess(
        cmd=[
            'gazebo', '--debug', '--verbose', gazebo_world,
            '-s', 'libgazebo_ros_factory.so', '-s', 'libgazebo_ros_init.so'
        ],
        condition=IfCondition(PythonExpression(
            [use_simulator])),
        output='screen')

    #  INCLUDE RVIZ LAUNCH FILE IF use_rviz IS SET TO TRUE
    declare_rviz_launch_include = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(champ_bringup_share_dir,
                     'launch',
                     'rviz.launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={
        'rviz_config': rviz_config
    }.items())

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [champ_description_share_dir,
                 "urdf", xacro_file_name]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("champ_bringup"),
            "config",
            "ros2_control.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"use_sim_time": use_sim_time},
                    robot_description,
                    robot_controllers],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_state_broadcaster'],
        output='screen'
    )

    joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_trajectory_controller'],
        output='screen'
    )

    return LaunchDescription([
        declare_use_simulator,
        declare_use_sim_time,
        declare_use_rviz,
        declare_tf_prefix,
        declare_rviz_config,
        declare_robot_state_publisher_node,
        declare_rviz_launch_include,
        declare_spawn_entity_to_gazebo_node,
        declare_start_gazebo_cmd,
        declare_joy_config_filepath,
        joint_state_broadcaster,
        joint_trajectory_controller
    ])

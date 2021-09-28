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
from launch.substitutions import Command
from launch.substitutions import PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    robot_name = LaunchConfiguration('robot_name')
    tf_prefix = LaunchConfiguration('tf_prefix')
    robot_model_params = LaunchConfiguration('robot_model_params')
    model_extras = LaunchConfiguration('model_extras')
    twist_mux_config = LaunchConfiguration('twist_mux_config')
    teleop_config = LaunchConfiguration('teleop_config')
    joy_config = LaunchConfiguration('joy_config')
    rviz_config = LaunchConfiguration("rviz_config")

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
    declare_robot_name = DeclareLaunchArgument(
        'robot_name',
        default_value=' ',
        description='...')
    declare_tf_prefix = DeclareLaunchArgument(
        'tf_prefix',
        default_value='',
        description='...')
    declare_robot_model_params = DeclareLaunchArgument(
        'robot_model_params',
        default_value='',
        description='...')
    declare_model_extras = DeclareLaunchArgument(
        'model_extras',
        default_value='',
        description='...')
    decleare_twist_mux_config = DeclareLaunchArgument(
        'twist_mux_config',
        default_value=os.path.join(
            champ_bringup_share_dir, 'config', 'twist_mux_config.yaml'),
        description='path to locks params.')
    declare_joy_config = DeclareLaunchArgument(
        'joy_config',
        default_value=os.path.join(
            champ_bringup_share_dir, 'config', 'joy_config.yaml'),
        description='...')
    declare_teleop_config = DeclareLaunchArgument(
        'teleop_config',
        default_value=os.path.join(
            champ_bringup_share_dir, 'config', 'teleop', 'teleop_xbox.yaml'),
        description='...')
    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(
            champ_bringup_share_dir, 'rviz', 'thorvald_default_view.rviz'),
        description='...')

    # DECLARE THE BASE DRIVER ROS2 NODE
    #declare_base_driver_node = Node(
    #    package='champ_base',
    #    executable='base_driver',
    #    name='base_driver',
    #    output='screen',
    #    namespace='',
    #    parameters=[robot_model_params],
    #    #prefix=['xterm -e gdb -ex run --args'],
    #    # uncomment this to run node in GDB debugger, note that you need xterm terminal installed with apt-get
    #    remappings=[('cmd_vel', 'twist_mux/cmd_vel')])

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

    # DECLARE TWIST MUX NODE
    declare_twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        namespace='',
        output='screen',
        parameters=[twist_mux_config],
        remappings=[('cmd_vel_out', 'twist_mux/cmd_vel')])

    # DECLARE JOY NODE
    declare_joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[joy_config])

    # DECLARE TELEOP NODE
    #declare_teleop_node = Node(
    #    package='thorvald_teleop',
    #    executable='teleop_node',
    #    name='teleop_node',
    #    output='screen',
    #    parameters=[teleop_config],
    #    remappings=[('cmd_vel', "teleop_joy/cmd_vel"),
    #                ('joy_priority', 'teleop_joy/joy_priority'),
    #                ('home_steering', 'base_driver/home_steering')])

    # SPAWN THE ROBOT TO GAZEBO IF use_simulator, FROM THE TOPIC "robot_description"
    declare_spawn_entity_to_gazebo_node = Node(package='gazebo_ros',
                                               condition=IfCondition(
                                                   use_simulator),
                                               executable='spawn_entity.py',
                                               arguments=[
                                                   '-entity', '',
                                                   '-topic', '/robot_description'],
                                               output='screen')

    # START GAZEBO ONLY IF use_simulator IS SET TO TRUE
    gazebo_world = os.path.join(
        get_package_share_directory('champ_gazebo'), 'worlds/', 'void.world'),
    declare_start_gazebo_cmd = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose', gazebo_world,
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

    return LaunchDescription([
        declare_use_simulator,
        declare_use_sim_time,
        declare_use_rviz,
        declare_robot_name,
        declare_tf_prefix,
        declare_robot_model_params,
        declare_model_extras,
        decleare_twist_mux_config,
        declare_joy_config,
        declare_teleop_config,
        declare_rviz_config,
        #declare_base_driver_node,
        declare_robot_state_publisher_node,
        declare_twist_mux_node,
        declare_joy_node,
        #declare_teleop_node,
        declare_rviz_launch_include,
        declare_spawn_entity_to_gazebo_node,
        declare_start_gazebo_cmd,
    ])

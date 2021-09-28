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
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    bringup_dir = get_package_share_directory('thorvald_bringup')
    rviz_config_file = LaunchConfiguration('rviz_config')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(bringup_dir, 'rviz',
                                   'thorvald_default_view.rviz'),
        description='Full path to the RVIZ config file to use')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output={'both': 'log'}, #change it to screen if you wanna see RVIZ output in terminal
        arguments=['-d', rviz_config_file,'--ros-args', '--log-level', 'ERROR']
        )

    return LaunchDescription([
        declare_rviz_config_file_cmd,
        rviz_node
    ])

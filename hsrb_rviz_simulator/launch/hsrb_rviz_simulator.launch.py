#!/usr/bin/env python3
'''
Copyright (c) 2024 TOYOTA MOTOR CORPORATION
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the disclaimer
below) provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of the copyright holder nor the names of its contributors may be used
  to endorse or promote products derived from this software without specific
  prior written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
'''
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from tmc_launch_ros_utils.tmc_launch_ros_utils import load_robot_description


def create_spanwer_node(controller_name, manager_name='/controller_manager'):
    return Node(package='controller_manager',
                executable='spawner',
                arguments=[controller_name, '--controller-manager', manager_name])


def declare_arguments():
    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument('description_package', default_value='hsrb_description',
                                                    description='Description package with robot URDF/xacro files.'))
    declared_arguments.append(DeclareLaunchArgument('description_file', default_value='hsrb4s.urdf.xacro',
                                                    description='URDF/XACRO description file with the robot.'))
    declared_arguments.append(DeclareLaunchArgument('runtime_config_package', default_value='hsrb_rviz_simulator',
                                                    description='Package with the controller\'s configuration.'))
    declared_arguments.append(DeclareLaunchArgument('controllers_file', default_value='controllers.yaml',
                                                    description='YAML file with the controllers configuration.'))

    declared_arguments.append(DeclareLaunchArgument('joint_limit_config_file', default_value='hsrb_joint_limits.yaml',
                                                    description='YAML file with the joint limits configuration.'))
    declared_arguments.append(DeclareLaunchArgument('timeopt_filter_launch',
                                                    default_value='hsrb_timeopt_filter.launch.py',
                                                    description='Launch file for timeopt filter'))
    declared_arguments.append(DeclareLaunchArgument('rrt_planner_launch', default_value='hsrb_planner.launch.py',
                                                    description='Launch file for rrt planner'))
    return declared_arguments


def generate_launch_description():
    robot_description = load_robot_description(xacro_arg='rviz_sim:=True')

    runtime_config_package = LaunchConfiguration('runtime_config_package')
    controllers_file = LaunchConfiguration('controllers_file')
    robot_controllers = PathJoinSubstitution([FindPackageShare(runtime_config_package), 'config', controllers_file])

    control_node = Node(package='controller_manager',
                        executable='ros2_control_node',
                        parameters=[robot_description, robot_controllers],
                        remappings=[('odom', '~/wheel_odom')])

    joint_state_publisher = Node(package='joint_state_publisher',
                                 executable='joint_state_publisher',
                                 parameters=[{'source_list': ['/joint_states']}],
                                 namespace='whole_body',
                                 remappings=[('robot_description', '/robot_description')])
    robot_state_pub_node = Node(package='robot_state_publisher',
                                executable='robot_state_publisher',
                                parameters=[robot_description],
                                namespace='whole_body',
                                output={'both': 'log'},
                                remappings=[('robot_description', '/robot_description')])
    wheel_odom_connector_tf = Node(package='tf2_ros',
                                   executable='static_transform_publisher',
                                   name='static_transform_publisher',
                                   output='log',
                                   arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0',
                                              'base_footprint_wheel', 'base_footprint'])

    rviz_config = os.path.join(get_package_share_directory('hsrb_rviz_simulator'), 'config/display_config.rviz')
    rviz_node = Node(package='rviz2',
                     executable='rviz2',
                     name='rviz2',
                     output='log',
                     arguments=['-d', rviz_config],
                     parameters=[robot_description])

    pseudo_endeffector_controller_node = Node(package='hsrb_pseudo_endeffector_controller',
                                              executable='hsrb_pseudo_endeffector_controller',
                                              name='pseudo_endeffector_controller_node',
                                              parameters=[robot_description],
                                              remappings=[('odom', 'omni_base_controller/wheel_odom')])

    hsrb_manipulation_launch_dir = get_package_share_directory('hsrb_manipulation_launch')
    safe_pose_changer_launch = os.path.join(hsrb_manipulation_launch_dir, 'launch/safe_pose_changer.launch.py')
    safe_pose_changer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(safe_pose_changer_launch),
        launch_arguments={'runtime_config_package': 'hsrb_manipulation_launch',
                          'configuration_file': LaunchConfiguration('joint_limit_config_file')}.items())

    timeopt_ros_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution(
        [hsrb_manipulation_launch_dir, 'launch', LaunchConfiguration('timeopt_filter_launch')])))
    planner_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution(
        [hsrb_manipulation_launch_dir, 'launch', LaunchConfiguration('rrt_planner_launch')])))

    nodes = [control_node,
             joint_state_publisher,
             robot_state_pub_node,
             wheel_odom_connector_tf,
             create_spanwer_node('joint_state_broadcaster'),
             create_spanwer_node('head_trajectory_controller'),
             create_spanwer_node('arm_trajectory_controller'),
             create_spanwer_node('omni_base_controller'),
             rviz_node,
             pseudo_endeffector_controller_node,
             safe_pose_changer,
             timeopt_ros_launch,
             planner_launch]

    return LaunchDescription(declare_arguments() + nodes)

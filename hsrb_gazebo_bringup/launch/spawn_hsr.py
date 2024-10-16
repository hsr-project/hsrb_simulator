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
# -*- coding: utf-8 -*-
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction
)
from launch.conditions import (
    IfCondition,
    UnlessCondition,
)
from launch.substitutions import (
    LaunchConfiguration,
)
from launch_ros.actions import Node


# (H-yaguchi) Humble launches a controller in this way
def create_spawner_node(controller_name, manager_name='/controller_manager'):
    return Node(package='controller_manager',
                executable='spawner',
                arguments=[controller_name, '--controller-manager', manager_name])


def declare_arguments(context, robot_name):
    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument('gazebo_visualization', default_value="false",
                                                    description='gazebo visualization status'))
    declared_arguments.append(DeclareLaunchArgument('use_odom_ground_truth', default_value='true',
                                                    description='Use ground truth odometry at omni base controller.'))
    declared_arguments.append(DeclareLaunchArgument('runtime_config_package', default_value='hsrb_gazebo_bringup',
                                                    description='Package with the controller\'s configuration.'))
    declared_arguments.append(DeclareLaunchArgument('controllers_file', default_value='gazebo_ros2_control.yaml',
                                                    description='YAML file with the controllers configuration.'))
    declared_arguments.append(DeclareLaunchArgument('robot_pos', default_value='0.0,0.0,0.0,0.0',
                                                    description='spawn_entity.py robot position.'))
    declared_arguments.append(DeclareLaunchArgument('ground_truth_xyz', default_value='0.0\\ 0.0\\ 0.0',
                                                    description='gazebo ground truth xyz position.'))
    declared_arguments.append(DeclareLaunchArgument('ground_truth_rpy', default_value='0.0\\ 0.0\\ 0.0',
                                                    description='gazebo ground truth rpy position.'))
    return declared_arguments


def launch_setup(context,
                 robot_name,
                 robot_pos,
                 ground_truth_xyz,
                 ground_truth_rpy,
                 gazebo_visualization,
                 robot_description):
    robot_pos_list = context.perform_substitution(robot_pos).split(',')
    robot_name_value = context.perform_substitution(robot_name)
    robot_description_value = context.perform_substitution(robot_description)

    spawn_entity = Node(
        package='gazebo_ros', executable='spawn_entity.py', output='screen',
        arguments=[
            '-topic',
            'robot_description',
            '-entity',
            robot_name_value,
            '-x',
            robot_pos_list[0],
            '-y',
            robot_pos_list[1],
            '-z',
            robot_pos_list[2],
            '-Y',
            robot_pos_list[3]])

    # The problem of here is the same as HSRB_BRINGUP for actual machines
    joint_state_publisher = Node(package='joint_state_publisher',
                                 executable='joint_state_publisher',
                                 parameters=[{'source_list': ['/joint_states']}, {'use_sim_time': True}],
                                 namespace='whole_body',
                                 remappings=[('robot_description', '/robot_description')])
    robot_state_publisher = Node(package='robot_state_publisher',
                                 executable='robot_state_publisher',
                                 parameters=[{'robot_description': robot_description_value}, {'use_sim_time': True}],
                                 namespace='whole_body',
                                 output={'both': 'log'},
                                 remappings=[('robot_description', '/robot_description')])
    wheel_odom_connector_tf = Node(package='tf2_ros',
                                   executable='static_transform_publisher',
                                   name='static_transform_publisher',
                                   output='log',
                                   arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0',
                                              'base_footprint_wheel', 'base_footprint'])

    # REMAP is difficult with Gazebo_ros2_control, so Relay
    # Topic_tools itself ROS2 compatible is not over
    odom_relay_node_gt = Node(package='hsrb_gazebo_bringup',
                              executable='odom_relay',
                              name='odom_relay',
                              remappings=[('~/input_odom', 'odom_ground_truth'),
                                          ('~/output_odom', 'odom')],
                              condition=IfCondition(LaunchConfiguration('use_odom_ground_truth')))
    odom_to_tf_converter = Node(package='hsrb_gazebo_bringup',
                                executable='odom_to_tf_converter',
                                name='odom_to_tf_converter',
                                parameters=[{'frame_id': 'odom'}],
                                remappings=[('~/input_odom', 'odom_ground_truth')],
                                condition=IfCondition(LaunchConfiguration('use_odom_ground_truth')))
    odom_relay_node_wheel = Node(package='hsrb_gazebo_bringup',
                                 executable='odom_relay',
                                 name='odom_relay',
                                 remappings=[('~/input_odom', 'omni_base_controller/wheel_odom'),
                                             ('~/output_odom', 'odom')],
                                 condition=UnlessCondition(LaunchConfiguration('use_odom_ground_truth')))

    # The problem of here is the same as HSRB_BRINGUP for actual machines
    nodes = [joint_state_publisher,
             robot_state_publisher,
             odom_relay_node_gt,
             odom_to_tf_converter,
             odom_relay_node_wheel,
             wheel_odom_connector_tf,
             create_spawner_node('joint_state_broadcaster'),
             create_spawner_node('head_trajectory_controller'),
             create_spawner_node('arm_trajectory_controller'),
             create_spawner_node('omni_base_controller'),
             spawn_entity]

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [OpaqueFunction(function=declare_arguments,
                        args=[LaunchConfiguration('robot_name')]),
         OpaqueFunction(function=launch_setup,
                        args=[LaunchConfiguration('robot_name'),
                              LaunchConfiguration('robot_pos'),
                              LaunchConfiguration('ground_truth_xyz'),
                              LaunchConfiguration('ground_truth_rpy'),
                              LaunchConfiguration('gazebo_visualization'),
                              LaunchConfiguration('robot_description')])])

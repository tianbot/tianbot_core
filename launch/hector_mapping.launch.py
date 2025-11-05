#!/usr/bin/env python3
"""ROS2 launch file for hector_mapping converted from ROS1 XML.

Converted params are passed as a single parameter dictionary. To run:

    ros2 launch tianbot_core hector_mapping.launch.py

Assumptions:
- The ROS2 package/executable is named 'hector_mapping' and exposes the
  'hector_mapping' executable.
- Parameter names are unchanged in the ROS2 port of hector_mapping.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = {
        "pub_map_odom_transform": True,
        "map_frame": "map",
        "base_frame": "base_link",
        "odom_frame": "odom",

        "use_tf_scan_transformation": True,
        "use_tf_pose_start_estimate": False,

        "map_resolution": 0.05,
        "map_size": 3096,
        "map_start_x": 0.5,
        "map_start_y": 0.5,
        "laser_z_min_value": -1.0,
        "laser_z_max_value": 1.0,
        "map_multi_res_levels": 2,
        "map_pub_period": 2,
        "laser_min_dist": 0.4,
        "laser_max_dist": 5.5,
        "output_timing": False,
        "pub_map_scanmatch_transform": True,

        "update_factor_free": 0.4,
        "update_factor_occupied": 0.7,
        "map_update_distance_thresh": 0.2,
        "map_update_angle_thresh": 0.06,

        "advertise_map_service": True,
        "scan_subscriber_queue_size": 5,
        "scan_topic": "scan",
    }

    hector_node = Node(
        package="hector_mapping",
        executable="hector_mapping",
        name="hector_mapping",
        output="screen",
        parameters=[params],
    )

    return LaunchDescription([hector_node])

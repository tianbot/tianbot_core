from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    base_footprint_to_base_link = Node(package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       name="base_footprint_to_base_link",
                       arguments = ["0", "0", "0", "0", "0", "0", "base_footprint", "base_link"])
    ld.add_action(base_footprint_to_base_link)

    base_link_to_imu = Node(package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       name="base_link_to_imu",
                       arguments = ["0.22", "0", "0.1", "0", "0", "0", "base_link", "imu_link"])
    ld.add_action(base_link_to_imu)

    base_link_to_laser = Node(package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       name="base_link_to_laser",
                       arguments = ["0.22", "0", "0.1", "0", "0", "0", "base_link", "laser"])
    ld.add_action(base_link_to_laser)

    base_link_to_camera_link = Node(package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       name="base_link_to_camera_link",
                       arguments = ["0.3", "0", "0.035", "0", "0", "0", "base_link", "camera_link"])

    ld.add_action(base_link_to_camera_link)

                       
    return ld
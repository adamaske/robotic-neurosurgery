from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([ Node(packge='demo_nodes_cpp', executable='talker')])
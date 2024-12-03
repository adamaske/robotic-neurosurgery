

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution


def generate_robot_description():
    safety_limits = "true" #values from the default configuration
    safety_pos_margin = "0.15"
    safety_k_position = "20"

    ur_description = "ur_description"
    ur_type = "ur5e"

    # Load all the information from the ur_desceription
    joint_limit_params = os.path.join(
        get_package_share_directory(ur_description), "config", ur_type, "joint_limits.yaml"
    )
    kinematic_params = os.path.join(
        get_package_share_directory(ur_description),"config", ur_type, "default_kinematics.yaml"
    )
    physical_params = os.path.join(
        get_package_share_directory(ur_description), "config", ur_type, "physical_parameters.yaml"
    )
    visual_params = os.path.join(
        get_package_share_directory(ur_description), "config", ur_type, "visual_parameters.yaml"
    )
    description_filepath = os.path.join( #use the ur_description urdf files
        get_package_share_directory("surgical_simulation"), "urdf", "ur5.urdf.xacro"
    )
    robot_description_content = Command( #define the xacro file
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            description_filepath,
            " ",
            "joint_limit_params:=", joint_limit_params,
            " ",
            "kinematic_params:=", kinematic_params,
            " ",
            "physical_params:=", physical_params,
            " ",
            "visual_params:=", visual_params,
            " ",
            "safety_limits:=", safety_limits,
            " ",
            "safety_pos_margin:=", safety_pos_margin,
            " ",
            "safety_k_position:=", safety_k_position,
            " ",
            "name:=", "ur",
            " ",
            "ur_type:=", ur_type,
            " ",
            "tf_prefix:=",
            '""',
        ]
    )
    
    return robot_description_content

def generate_launch_description():

    #find gazebo world
    surgery_room_world = os.path.join(
        get_package_share_directory('surgical_simulation'),
        'worlds',
        'surgery_room.world')
    
    # gazebo launch desc
    gazebo_node = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
                launch_arguments={'world': surgery_room_world}.items(),
             )
    
    
    robot_description = {"robot_description": generate_robot_description()}

    #publish robot info
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description], 
    )
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
    )

    rviz_config_file = os.path.join(
        get_package_share_directory("ur_description"), "rviz", "view_robot.rviz"
    )
    rviz_node = Node(package="rviz2", 
                     executable="rviz2", 
                     name="rviz2", 
                     output="log",
                     arguments=["-d", rviz_config_file]
    )

    #add robot to gazeborobot_state_publisher_node
    spawn_robot = Node(package='gazebo_ros', 
                       executable='spawn_entity.py',
                       arguments=['-topic', 'robot_description', '-entity', 'robot'],
                       output="screen"
                       )
    
    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
        gazebo_node,
        spawn_robot
    ])
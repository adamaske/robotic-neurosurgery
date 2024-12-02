
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
import yaml

def generate_launch_description():

    #find gazebo world
    
    surgery_room_world = os.path.join(
        get_package_share_directory('surgical_simulation'),
        'worlds',
        'surgery_room.world')
    
    #launch gazebo
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
                launch_arguments={'world': ur5_ros2_gazebo}.items(),
             )

    #find urdf (xacro) file
    robot_description_path = os.path.join(
        get_package_share_directory('surgical_simulation'))
    xacro_file = os.path.join(robot_description_path,
                              'urdf',
                              'ur5.urdf.xacro')
    #create robot description
    doc = xacro.parse(open(xacro_file))
    doc.process_doc(doc, mapping=)
    robot_description_config = doc.toxml()
    robot_description = {'robot_description': robot_description_config}
    
    #publish robot info
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    #add robot to gazebo
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'ur5'],
                        output='screen')
    
    return LaunchDescription([
        gazebo, 
        node_robot_state_publisher,
        spawn_entity
    ])

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration,  PathJoinSubstitution, FindExecutable
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro


def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('6_AXIS_BOT'))
    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ros2_control_demo_description"), "r6bot/rviz", "view_robot.rviz"]
    )

    # This causes the robot to appear
    publisher_ = Node(package='joint_state_publisher', executable='joint_state_publisher')
    # This causes the robot to appear
    gui = Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui')
    # This causes the robot to appear
    rviz = Node(package='rviz2', executable='rviz2', arguments=["-d", rviz_config_file])

    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),

        node_robot_state_publisher,
        publisher_,
        gui,
        rviz,
    ])

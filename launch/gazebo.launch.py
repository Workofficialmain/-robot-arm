import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

URDF = "/home/pavan/robot_arm_ws/src/robot_arm_description/urdf/robot_arm_gz.urdf"

def generate_launch_description():
    robot_description = open(URDF).read()
    gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")))
    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher", output="screen",
               parameters=[{"robot_description": robot_description, "use_sim_time": True}])
    spawn = Node(package="gazebo_ros", executable="spawn_entity.py", output="screen",
                 arguments=["-topic", "robot_description", "-entity", "robot_arm"])
    jsb = Node(package="controller_manager", executable="spawner", output="screen",
               arguments=["joint_state_broadcaster"])
    arm = Node(package="controller_manager", executable="spawner", output="screen",
               arguments=["arm_controller"])
    return LaunchDescription([
        gazebo, rsp, spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[arm])),
    ])

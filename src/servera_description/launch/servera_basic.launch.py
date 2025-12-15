from launch import LaunchDescription

from launch.substitutions import FindExecutable, PathJoinSubstitution, LaunchConfiguration , Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = FindPackageShare("servera_description")
    urdf_path = PathJoinSubstitution([pkg_share, 'urdf', 'servera.urdf.xacro'])
    rviz_config_path = PathJoinSubstitution([pkg_share, 'config', 'rviz_basic_conf.rviz'])


    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', urdf_path]),
        value_type=str
    )
    
    robot_state_publisher = Node(
        package= "robot_state_publisher",
        executable="robot_state_publisher",
        name="RSPub",
        parameters=[{"robot_description": robot_description}],
        output="screen",
        # remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    )
    
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="JSPGui",
        output="screen"
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        arguments=["-d", rviz_config_path]
    )
    ld = LaunchDescription()
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher_gui)
    ld.add_action(rviz)
    # ld.add_action()

    return ld
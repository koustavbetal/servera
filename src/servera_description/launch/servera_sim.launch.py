from launch import LaunchDescription

from launch.substitutions import FindExecutable, PathJoinSubstitution, LaunchConfiguration , Command
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = FindPackageShare("servera_description")

    urdf_path = PathJoinSubstitution([pkg_share, 'urdf', 'servera.urdf.xacro'])
    rviz_config_path = PathJoinSubstitution([pkg_share, 'config', 'rviz_sim_conf.rviz'])
    bridge_path = PathJoinSubstitution([pkg_share, 'config', 'gz_bridge.yaml'])

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

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        arguments=["-d", rviz_config_path]
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'), "launch", "gz_sim.launch.py"])]),
                launch_arguments={
                    "gz_args": ['-r -v4 ', "shapes.sdf"],
                    "on_exit_shutdown": "true"
                }.items()
    )

    parameter_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='parameter_bridge',
        parameters=[{"config_file": bridge_path}],
        output='screen'
    )

    spawn_model = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'), "launch", "gz_spawn_model.launch.py"])]),
                launch_arguments={
                    "topic": "robot_description",
                    "entity_name": "Servera",
                    "x": "-3.0",
                    "z": "0.02",
                }.items(),
    )

    ld = LaunchDescription()
    ld.add_action(robot_state_publisher)
    ld.add_action(parameter_bridge)
    ld.add_action(gz_sim)
    ld.add_action(spawn_model)
    ld.add_action(rviz)


    return ld
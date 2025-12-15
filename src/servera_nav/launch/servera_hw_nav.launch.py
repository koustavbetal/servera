from launch import LaunchDescription

from launch.substitutions import FindExecutable, PathJoinSubstitution, LaunchConfiguration , Command  # noqa: F401
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription # noqa: F401

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    servera_control_share = FindPackageShare("servera_control")
    servera_description_share = FindPackageShare("servera_description")
    servera_nav_share = FindPackageShare("servera_nav")


    urdf_path = PathJoinSubstitution([servera_description_share, 'hw_urdf', 'servera_hw.urdf.xacro'])
    # rviz_config_path = PathJoinSubstitution([servera_nav_share, 'config', 'nav2_default_view.rviz'])
    rviz_config_path = PathJoinSubstitution([servera_nav_share, 'config', 'rviz_slam.rviz'])    
    ros2_control_config_path = PathJoinSubstitution([servera_control_share, 'config', 'ros2_controller.yaml'])
    nav2_config_path = PathJoinSubstitution([servera_nav_share, 'config', 'nav2_params.yaml'])

    
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

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'lidar_link', 'Servera/base_footprint/lidar_link']
    )

    lidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[{'channel_type': 'serial',
                        'serial_port': '/dev/ttyUSB0',
                        'serial_baudrate': 1000000,
                        'frame_id': 'lidar_link',
                        'inverted': False,
                        'angle_compensate': True,
                        'scan_mode': 'DenseBoost'
                        }],
        output='screen')

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description}, ros2_control_config_path],
        output="screen"
    )

    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    load_diff_drive = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
    )

    laser_odom = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        parameters=[{
            'laser_scan_topic' : '/scan',
            'odom_topic' : '/odom',
            'base_frame_id' : 'base_footprint',
            'odom_frame_id' : 'odom',
            'publish_tf' : True,
            'init_pose_from_topic' : '',
            'freq' : 10.0}],
        output='screen'
    )
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([servera_nav_share, "launch", "bringup_launch.py"])]),
                launch_arguments={
                    'use_sim_time': 'true',
                    'params_file': nav2_config_path,
                }.items()
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([servera_nav_share, "launch", "slam_launch.py"])]),
                launch_arguments={
                    'use_sim_time': 'false',
                    'params_file': nav2_config_path,
                }.items()
    )


    ld = LaunchDescription()
    ld.add_action(robot_state_publisher)
    # ld.add_action(static_tf)
    ld.add_action(controller_manager)
    ld.add_action(lidar_node)
    ld.add_action(load_diff_drive)
    ld.add_action(load_joint_state_broadcaster)
    ld.add_action(rviz)
    ld.add_action(laser_odom)
    # ld.add_action(nav2_bringup) #bringup and slam launch should not be run together, bringup is for navigation with map, slam is for creating the map. This yet has not been tested properly.
    ld.add_action(slam_launch)

    return ld
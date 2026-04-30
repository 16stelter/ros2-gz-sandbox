import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction, ExecuteProcess
from launch.launch_context import LaunchContext
from launch.launch_description import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
import xacro


def spawn_robot(context: LaunchContext, namespace: LaunchConfiguration, x, y, z):
    robot_ns = context.perform_substitution(namespace)

    descr_pkg_share = get_package_share_directory("go2_description")

    joints_config = os.path.join(descr_pkg_share, "config/champ/joints.yaml")
    gait_config = os.path.join(descr_pkg_share, "config/champ/gait.yaml")
    links_config = os.path.join(descr_pkg_share, "config/champ/links.yaml")

    urdf_path = os.path.join(descr_pkg_share, "urdf", "unitree_go2_robot.xacro")

    robot_desc = xacro.process(
        urdf_path,
        mappings={"robot_ns": robot_ns},
    )

    if robot_ns == "":
        robot_gazebo_name = "go2"
        node_name_prefix = ""
    else:
        robot_gazebo_name = "go2_" + robot_ns
        node_name_prefix = robot_ns

    # Launch robot state publisher node
    robot_state_publisher = Node(
        namespace=robot_ns,
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            {"use_sim_time": True},
            {"robot_description": robot_desc},
        ],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    # Spawn a robot inside a simulation
    go2 = Node(
        namespace=robot_ns,
        package="ros_gz_sim",
        executable="create",
        name="ros_gz_sim_create",
        output="both",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            robot_gazebo_name,
            "-x",
            context.perform_substitution(x),
            "-y",
            context.perform_substitution(y),
            "-z",
            context.perform_substitution(z),
        ],
    )

    topic_bridge = Node(
        namespace="",
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name=node_name_prefix + "_parameter_bridge",
        output='screen',
        arguments=[
            # Gazebo to ROS
            '/imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
            '/velodyne_points/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            '/unitree_lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            # '/velodyne_points@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/rgb_image@sensor_msgs/msg/Image@gz.msgs.Image',
            # D455 RGBD camera bridges
            '/d455/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/d455/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/d455/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/d455/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',

            # ROS to Gazebo
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/joint_group_position_controller/joint_trajectory@trajectory_msgs/msg/JointTrajectory]gz.msgs.JointTrajectory',
        ],
        parameters=[
            {
                "qos_overrides./tf_static.publisher.durability": "transient_local",
                "use_sim_time": True,
            }
        ],
    )

    quadruped_controller_node = Node(
        namespace=robot_ns,
        package="champ_base",
        executable="quadruped_controller_node",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"gazebo": True},
            {"publish_joint_states": True},
            {"publish_joint_control": True},
            {"publish_foot_contacts": False},
            {"joint_controller_topic": "joint_group_position_controller/joint_trajectory"},
            {"robot_desc": robot_desc},
            joints_config,
            links_config,
            gait_config,
            {"hardware_connected": False},
            {"publish_foot_contacts": False},
            {"close_loop_odom": True},
        ],
        remappings=[("/cmd_vel/smooth", "/cmd_vel")],
    )

    state_estimator_node = Node(
        namespace=robot_ns,
        package="champ_base",
        executable="state_estimation_node",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"orientation_from_imu": True},
            {"urdf": robot_desc},
            joints_config,
            links_config,
            gait_config,
        ],
    )

    controller_spawner_js = TimerAction(
        period=20.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                output="screen",
                arguments=[
                    "--controller-manager-timeout", "120",
                    "joint_states_controller",  # No --inactive flag to ensure full activation
                ],
                parameters=[{"use_sim_time": True}],
            )
        ]
    )

    controller_spawner_position = TimerAction(
        period=30.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                output="screen",
                arguments=[
                    "--controller-manager-timeout", "120",
                    "joint_group_position_controller",  # No --inactive flag to ensure full activation
                ],
                parameters=[{"use_sim_time": True}],
            )
        ]
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(descr_pkg_share, "rviz/rviz.rviz")],
    )

    return [
        robot_state_publisher,
        go2,
        topic_bridge,
        quadruped_controller_node,
        state_estimator_node,
        controller_spawner_js,
        controller_spawner_position,
    ]


def generate_launch_description():
    config_pkg_share = get_package_share_directory("go2_description")

    ros_control_config = os.path.join(
        config_pkg_share, "config/ros_control/ros_control.yaml"  
    )

    name_argument = DeclareLaunchArgument(
        "robot_ns",
        default_value="",
        description="Robot namespace",
    )

    declare_ros_control_file = DeclareLaunchArgument(
        "ros_control_file",
        default_value=ros_control_config,
        description="Ros control config path",
    )

    namespace = LaunchConfiguration("robot_ns")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")

    return LaunchDescription([
        name_argument, 
        declare_ros_control_file,
        OpaqueFunction(function=spawn_robot, args=[namespace, x, y, z])
    ])

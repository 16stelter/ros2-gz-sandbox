import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.launch_context import LaunchContext
from launch.launch_description import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def spawn_robot(context: LaunchContext, namespace: LaunchConfiguration, x, y, z):
    robot_ns = context.perform_substitution(namespace)

    config_pkg_share = get_package_share_directory("go2_config")
    descr_pkg_share = get_package_share_directory("go2_description")

    joints_config = os.path.join(config_pkg_share, "config/joints/joints.yaml")
    gait_config = os.path.join(config_pkg_share, "config/gait/gait.yaml")
    links_config = os.path.join(config_pkg_share, "config/links/links.yaml")

    robot_desc = xacro.process(
        os.path.join(
            descr_pkg_share,
            "xacro",
            "robot_VLP.xacro",
        ),
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

    joint_state_publisher = Node(
        namespace=robot_ns,
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
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

    # Bridge ROS topics and Gazebo messages for establishing communication
    topic_bridge = Node(
        namespace="",
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name=node_name_prefix + "_parameter_bridge",
        arguments=[
            robot_ns + "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            #robot_ns + "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            robot_ns + "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            robot_ns + "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
        ],
        parameters=[
            {
                "qos_overrides./tf_static.publisher.durability": "transient_local",
            }
        ],
        output="screen",
    )

    return [
        robot_state_publisher,
        joint_state_publisher,
        go2,
        topic_bridge
    ]


def generate_launch_description():
    config_pkg_share = get_package_share_directory("go2_config")

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

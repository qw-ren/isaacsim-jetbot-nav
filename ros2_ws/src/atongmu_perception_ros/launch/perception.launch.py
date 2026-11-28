"""启动感知节点。

    ros2 launch atongmu_perception_ros perception.launch.py

按需覆盖参数，例如摄像头话题名不同：
    ros2 launch atongmu_perception_ros perception.launch.py \
        image_topic:=/rgb camera_info_topic:=/rgb/camera_info use_depth:=false \
        config_file:=/home/user/isaacsim-jetbot-nav/perception/configs/perception.yaml
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    default_config = PathJoinSubstitution([FindPackageShare("atongmu_perception_ros"), "config", "perception_ros.yaml"])

    args = [
        DeclareLaunchArgument("config_file", default_value=default_config,
                              description="感知 YAML 配置（结构与 perception/configs/perception.yaml 相同）"),
        DeclareLaunchArgument("image_topic", default_value="/camera/color/image_raw"),
        DeclareLaunchArgument("depth_topic", default_value="/camera/depth/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/camera/color/camera_info"),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument("use_depth", default_value="true"),
        DeclareLaunchArgument("use_scan", default_value="true"),
        DeclareLaunchArgument("use_tf_extrinsics", default_value="true"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("map_frame", default_value="map"),
        DeclareLaunchArgument("camera_frame", default_value="camera_color_optical_frame"),
        DeclareLaunchArgument("laser_frame", default_value="laser_frame"),
        DeclareLaunchArgument("publish_markers", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("require_map", default_value="false"),
        DeclareLaunchArgument("publish_legacy", default_value="true"),
        DeclareLaunchArgument("scan_slop", default_value="0.06"),
    ]

    node = Node(
        package="atongmu_perception_ros",
        executable="perception_node",
        name="atongmu_perception",
        output="screen",
        parameters=[
            {
                "config_file": LaunchConfiguration("config_file"),
                "image_topic": LaunchConfiguration("image_topic"),
                "depth_topic": LaunchConfiguration("depth_topic"),
                "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                "scan_topic": LaunchConfiguration("scan_topic"),
                "use_depth": LaunchConfiguration("use_depth"),
                "use_scan": LaunchConfiguration("use_scan"),
                "use_tf_extrinsics": LaunchConfiguration("use_tf_extrinsics"),
                "base_frame": LaunchConfiguration("base_frame"),
                "map_frame": LaunchConfiguration("map_frame"),
                "camera_frame": LaunchConfiguration("camera_frame"),
                "laser_frame": LaunchConfiguration("laser_frame"),
                "publish_markers": LaunchConfiguration("publish_markers"),
                "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
                "require_map": ParameterValue(LaunchConfiguration("require_map"), value_type=bool),
                "publish_legacy": ParameterValue(LaunchConfiguration("publish_legacy"), value_type=bool),
                "scan_slop": ParameterValue(LaunchConfiguration("scan_slop"), value_type=float),
            }
        ],
    )

    return LaunchDescription(args + [node])

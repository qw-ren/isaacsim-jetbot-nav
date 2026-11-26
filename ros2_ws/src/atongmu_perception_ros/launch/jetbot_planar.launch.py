"""PR #6 JetBot v2 planar perception profile; requires matching TF and RGB-D."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    share = FindPackageShare("atongmu_perception_ros")
    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=PathJoinSubstitution(
            [share, "config", "perception_ros.yaml"])),
        DeclareLaunchArgument("use_depth", default_value="true"),
        DeclareLaunchArgument("use_scan", default_value="true"),
        DeclareLaunchArgument("require_map", default_value="true"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([share, "launch", "perception.launch.py"])),
            launch_arguments={
                "config_file": LaunchConfiguration("config_file"),
                "use_depth": LaunchConfiguration("use_depth"),
                "use_scan": LaunchConfiguration("use_scan"),
                "require_map": LaunchConfiguration("require_map"),
                "image_topic": "/jetbot/image_raw",
                "depth_topic": "/jetbot/depth/image_rect_raw",
                "camera_info_topic": "/jetbot/camera_info",
                "scan_topic": "/jetbot/scan",
                "base_frame": "chassis",
                "map_frame": "map",
                "camera_frame": "jetbot/camera_optical",
                # /jetbot/scan is already expressed in chassis, not jetbot/lidar.
                "laser_frame": "chassis",
                "use_tf_extrinsics": "true",
                "use_sim_time": "true",
                "publish_markers": "false",
                "publish_legacy": "false",
            }.items()),
        Node(package="atongmu_perception_ros", executable="planar_map_node",
             name="atongmu_planar_map", output="screen", parameters=[{"use_sim_time": True}]),
    ])

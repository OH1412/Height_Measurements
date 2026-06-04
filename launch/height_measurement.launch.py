from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = [
        DeclareLaunchArgument("heightmap_path", default_value=""),
        DeclareLaunchArgument("metadata_path", default_value=""),
        DeclareLaunchArgument("odom_topic", default_value="/aft_mapped_in_map"),
        DeclareLaunchArgument("map_frame", default_value="map"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("publish_topic", default_value="/heightmap_points"),
        DeclareLaunchArgument("publish_rate", default_value="50.0"),
        DeclareLaunchArgument("height_formula", default_value="legged_gym"),
        DeclareLaunchArgument("measured_height_offset", default_value="0.3"),
        DeclareLaunchArgument("base_to_odom_x", default_value="0.16266"),
        DeclareLaunchArgument("base_to_odom_y", default_value="0.0"),
        DeclareLaunchArgument("base_to_odom_z", default_value="0.11703"),
        DeclareLaunchArgument("clip_min", default_value="-1.0"),
        DeclareLaunchArgument("clip_max", default_value="1.0"),
        DeclareLaunchArgument("use_bilinear", default_value="true"),
        DeclareLaunchArgument("default_height", default_value="0.0"),
    ]

    node = Node(
        package="height_measurements",
        executable="height_measurement_node",
        name="height_measurement_node",
        output="screen",
        parameters=[
            {
                "heightmap_path": LaunchConfiguration("heightmap_path"),
                "metadata_path": LaunchConfiguration("metadata_path"),
                "odom_topic": LaunchConfiguration("odom_topic"),
                "map_frame": LaunchConfiguration("map_frame"),
                "base_frame": LaunchConfiguration("base_frame"),
                "publish_topic": LaunchConfiguration("publish_topic"),
                "publish_rate": ParameterValue(LaunchConfiguration("publish_rate"), value_type=float),
                "height_formula": LaunchConfiguration("height_formula"),
                "measured_height_offset": ParameterValue(
                    LaunchConfiguration("measured_height_offset"), value_type=float
                ),
                "base_to_odom_x": ParameterValue(
                    LaunchConfiguration("base_to_odom_x"), value_type=float
                ),
                "base_to_odom_y": ParameterValue(
                    LaunchConfiguration("base_to_odom_y"), value_type=float
                ),
                "base_to_odom_z": ParameterValue(
                    LaunchConfiguration("base_to_odom_z"), value_type=float
                ),
                "clip_min": ParameterValue(LaunchConfiguration("clip_min"), value_type=float),
                "clip_max": ParameterValue(LaunchConfiguration("clip_max"), value_type=float),
                "use_bilinear": ParameterValue(LaunchConfiguration("use_bilinear"), value_type=bool),
                "default_height": ParameterValue(
                    LaunchConfiguration("default_height"), value_type=float
                ),
            }
        ],
    )

    return LaunchDescription(args + [node])

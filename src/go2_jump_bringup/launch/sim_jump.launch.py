from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_path = PathJoinSubstitution(
        [FindPackageShare("go2_jump_bringup"), "config", "jump_params.yaml"]
    )

    target_distance = LaunchConfiguration("target_distance_m")
    takeoff_angle = LaunchConfiguration("takeoff_angle_deg")
    takeoff_speed_scale = LaunchConfiguration("takeoff_speed_scale")
    use_takeoff_speed_scale_curve = LaunchConfiguration(
        "use_takeoff_speed_scale_curve"
    )
    auto_start = LaunchConfiguration("auto_start")
    push_front_tau_scale = LaunchConfiguration("push_front_tau_scale")
    push_rear_tau_scale = LaunchConfiguration("push_rear_tau_scale")
    landing_front_tau_scale = LaunchConfiguration("landing_front_tau_scale")
    landing_rear_tau_scale = LaunchConfiguration("landing_rear_tau_scale")
    push_pitch_target_deg = LaunchConfiguration("push_pitch_target_deg")
    push_pitch_compactness_gain = LaunchConfiguration(
        "push_pitch_compactness_gain"
    )
    flight_pitch_target_deg = LaunchConfiguration("flight_pitch_target_deg")
    flight_pitch_compactness_gain = LaunchConfiguration(
        "flight_pitch_compactness_gain"
    )
    landing_pitch_target_deg = LaunchConfiguration("landing_pitch_target_deg")
    landing_pitch_compactness_gain = LaunchConfiguration(
        "landing_pitch_compactness_gain"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("target_distance_m", default_value="0.25"),
            DeclareLaunchArgument("takeoff_angle_deg", default_value="45.0"),
            DeclareLaunchArgument("takeoff_speed_scale", default_value="1.06"),
            DeclareLaunchArgument(
                "use_takeoff_speed_scale_curve", default_value="true"
            ),
            DeclareLaunchArgument("auto_start", default_value="true"),
            DeclareLaunchArgument("push_front_tau_scale", default_value="0.96"),
            DeclareLaunchArgument("push_rear_tau_scale", default_value="1.12"),
            DeclareLaunchArgument("landing_front_tau_scale", default_value="0.90"),
            DeclareLaunchArgument("landing_rear_tau_scale", default_value="1.10"),
            DeclareLaunchArgument("push_pitch_target_deg", default_value="-5.0"),
            DeclareLaunchArgument(
                "push_pitch_compactness_gain", default_value="0.35"
            ),
            DeclareLaunchArgument("flight_pitch_target_deg", default_value="-2.0"),
            DeclareLaunchArgument(
                "flight_pitch_compactness_gain", default_value="0.55"
            ),
            DeclareLaunchArgument("landing_pitch_target_deg", default_value="-8.0"),
            DeclareLaunchArgument(
                "landing_pitch_compactness_gain", default_value="0.40"
            ),
            Node(
                package="go2_jump_planner",
                executable="jump_target_node",
                name="go2_jump_planner",
                output="screen",
                parameters=[
                    config_path,
                    {
                        "target_distance_m": target_distance,
                        "takeoff_angle_deg": takeoff_angle,
                        "takeoff_speed_scale": takeoff_speed_scale,
                        "use_takeoff_speed_scale_curve": use_takeoff_speed_scale_curve,
                    }
                ],
            ),
            Node(
                package="go2_jump_controller",
                executable="jump_controller_node",
                name="go2_jump_controller",
                output="screen",
                parameters=[
                    config_path,
                    {
                        "auto_start": auto_start,
                        "target_distance_m": target_distance,
                        "takeoff_angle_deg": takeoff_angle,
                        "takeoff_speed_scale": takeoff_speed_scale,
                        "use_takeoff_speed_scale_curve": use_takeoff_speed_scale_curve,
                        "push_front_tau_scale": push_front_tau_scale,
                        "push_rear_tau_scale": push_rear_tau_scale,
                        "landing_front_tau_scale": landing_front_tau_scale,
                        "landing_rear_tau_scale": landing_rear_tau_scale,
                        "push_pitch_target_deg": push_pitch_target_deg,
                        "push_pitch_compactness_gain": push_pitch_compactness_gain,
                        "flight_pitch_target_deg": flight_pitch_target_deg,
                        "flight_pitch_compactness_gain": flight_pitch_compactness_gain,
                        "landing_pitch_target_deg": landing_pitch_target_deg,
                        "landing_pitch_compactness_gain": landing_pitch_compactness_gain,
                    },
                ],
            ),
        ]
    )

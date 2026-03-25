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
    crouch_forward_bias_rad = LaunchConfiguration("crouch_forward_bias_rad")
    push_forward_bias_rad = LaunchConfiguration("push_forward_bias_rad")
    landing_absorption_blend = LaunchConfiguration("landing_absorption_blend")
    landing_kd = LaunchConfiguration("landing_kd")
    support_kp = LaunchConfiguration("support_kp")
    support_kd = LaunchConfiguration("support_kd")
    recovery_kd = LaunchConfiguration("recovery_kd")
    recovery_release_forward_speed_mps = LaunchConfiguration(
        "recovery_release_forward_speed_mps"
    )
    recovery_release_pitch_deg = LaunchConfiguration("recovery_release_pitch_deg")
    recovery_release_pitch_rate_degps = LaunchConfiguration(
        "recovery_release_pitch_rate_degps"
    )
    recovery_max_hold_s = LaunchConfiguration("recovery_max_hold_s")
    recovery_upright_blend = LaunchConfiguration("recovery_upright_blend")
    landing_support_blend = LaunchConfiguration("landing_support_blend")
    landing_touchdown_reference_blend = LaunchConfiguration(
        "landing_touchdown_reference_blend"
    )
    landing_hold_use_touchdown_pose = LaunchConfiguration(
        "landing_hold_use_touchdown_pose"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("target_distance_m", default_value="0.25"),
            DeclareLaunchArgument("takeoff_angle_deg", default_value="35.0"),
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
            DeclareLaunchArgument("crouch_forward_bias_rad", default_value="0.08"),
            DeclareLaunchArgument("push_forward_bias_rad", default_value="0.05"),
            DeclareLaunchArgument("landing_absorption_blend", default_value="0.45"),
            DeclareLaunchArgument("landing_kd", default_value="5.5"),
            DeclareLaunchArgument("support_kp", default_value="58.0"),
            DeclareLaunchArgument("support_kd", default_value="6.5"),
            DeclareLaunchArgument("recovery_kd", default_value="5.0"),
            DeclareLaunchArgument(
                "recovery_release_forward_speed_mps", default_value="0.08"
            ),
            DeclareLaunchArgument("recovery_release_pitch_deg", default_value="10.0"),
            DeclareLaunchArgument(
                "recovery_release_pitch_rate_degps", default_value="35.0"
            ),
            DeclareLaunchArgument("recovery_max_hold_s", default_value="0.55"),
            DeclareLaunchArgument("recovery_upright_blend", default_value="0.35"),
            DeclareLaunchArgument("landing_support_blend", default_value="0.40"),
            DeclareLaunchArgument(
                "landing_touchdown_reference_blend", default_value="0.80"
            ),
            DeclareLaunchArgument(
                "landing_hold_use_touchdown_pose", default_value="false"
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
                        "crouch_forward_bias_rad": crouch_forward_bias_rad,
                        "push_forward_bias_rad": push_forward_bias_rad,
                        "landing_absorption_blend": landing_absorption_blend,
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
                        "crouch_forward_bias_rad": crouch_forward_bias_rad,
                        "push_forward_bias_rad": push_forward_bias_rad,
                        "landing_absorption_blend": landing_absorption_blend,
                        "landing_kd": landing_kd,
                        "support_kp": support_kp,
                        "support_kd": support_kd,
                        "recovery_kd": recovery_kd,
                        "recovery_release_forward_speed_mps": recovery_release_forward_speed_mps,
                        "recovery_release_pitch_deg": recovery_release_pitch_deg,
                        "recovery_release_pitch_rate_degps": recovery_release_pitch_rate_degps,
                        "recovery_max_hold_s": recovery_max_hold_s,
                        "recovery_upright_blend": recovery_upright_blend,
                        "landing_support_blend": landing_support_blend,
                        "landing_touchdown_reference_blend": landing_touchdown_reference_blend,
                        "landing_hold_use_touchdown_pose": landing_hold_use_touchdown_pose,
                    },
                ],
            ),
        ]
    )

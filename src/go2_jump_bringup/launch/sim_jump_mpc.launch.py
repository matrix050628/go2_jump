from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_path = PathJoinSubstitution(
        [FindPackageShare("go2_jump_bringup"), "config", "mpc_params.yaml"]
    )

    target_distance = LaunchConfiguration("target_distance_m")
    takeoff_angle = LaunchConfiguration("takeoff_angle_deg")
    takeoff_speed_scale = LaunchConfiguration("takeoff_speed_scale")
    planner_backend = LaunchConfiguration("planner_backend")
    enable_intent_planner = LaunchConfiguration("enable_intent_planner")
    solver_backend = LaunchConfiguration("solver_backend")
    enable_lowcmd_output = LaunchConfiguration("enable_lowcmd_output")
    auto_start = LaunchConfiguration("auto_start")

    return LaunchDescription(
        [
            DeclareLaunchArgument("target_distance_m", default_value="0.25"),
            DeclareLaunchArgument("takeoff_angle_deg", default_value="34.0"),
            DeclareLaunchArgument("takeoff_speed_scale", default_value="1.03"),
            DeclareLaunchArgument(
                "planner_backend", default_value="heuristic_explicit"
            ),
            DeclareLaunchArgument(
                "enable_intent_planner", default_value="true"
            ),
            DeclareLaunchArgument(
                "solver_backend", default_value="reference_preview"
            ),
            DeclareLaunchArgument("enable_lowcmd_output", default_value="false"),
            DeclareLaunchArgument("auto_start", default_value="true"),
            Node(
                package="go2_jump_core",
                executable="jump_task_node",
                name="go2_jump_task_node",
                output="screen",
                parameters=[
                    config_path,
                    {
                        "target_distance_m": target_distance,
                        "takeoff_angle_deg": takeoff_angle,
                        "takeoff_speed_scale": takeoff_speed_scale,
                    },
                ],
            ),
            Node(
                package="go2_jump_planner",
                executable="jump_intent_node",
                name="go2_jump_intent_node",
                output="screen",
                condition=IfCondition(enable_intent_planner),
                parameters=[
                    config_path,
                    {
                        "planner_backend": planner_backend,
                    },
                ],
            ),
            Node(
                package="go2_jump_mpc",
                executable="whole_body_mpc_node",
                name="go2_jump_mpc_node",
                output="screen",
                parameters=[
                    config_path,
                    {
                        "solver_backend": solver_backend,
                        "enable_lowcmd_output": enable_lowcmd_output,
                        "auto_start": auto_start,
                    },
                ],
            ),
        ]
    )

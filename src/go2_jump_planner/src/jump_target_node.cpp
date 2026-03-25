#include "go2_jump_planner/jump_plan.hpp"

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class JumpTargetNode : public rclcpp::Node {
 public:
  JumpTargetNode() : Node("go2_jump_planner") {
    config_.target_distance_m = this->declare_parameter("target_distance_m", 0.25);
    config_.takeoff_angle_deg = this->declare_parameter("takeoff_angle_deg", 45.0);
    config_.takeoff_speed_scale =
        this->declare_parameter("takeoff_speed_scale", 1.06);
    config_.use_takeoff_speed_scale_curve =
        this->declare_parameter("use_takeoff_speed_scale_curve", false);
    config_.takeoff_speed_scale_distance_points_m = this->declare_parameter(
        "takeoff_speed_scale_distance_points_m", std::vector<double>{});
    config_.takeoff_speed_scale_values = this->declare_parameter(
        "takeoff_speed_scale_values", std::vector<double>{});
    config_.crouch_forward_bias_rad =
        this->declare_parameter("crouch_forward_bias_rad",
                                config_.crouch_forward_bias_rad);
    config_.push_forward_bias_rad =
        this->declare_parameter("push_forward_bias_rad",
                                config_.push_forward_bias_rad);
    config_.landing_absorption_blend =
        this->declare_parameter("landing_absorption_blend",
                                config_.landing_absorption_blend);
    config_.leg_link_length_m =
        this->declare_parameter("leg_link_length_m", config_.leg_link_length_m);
    config_.landing_capture_time_constant_s = this->declare_parameter(
        "landing_capture_time_constant_s",
        config_.landing_capture_time_constant_s);
    config_.landing_capture_rear_ratio = this->declare_parameter(
        "landing_capture_rear_ratio", config_.landing_capture_rear_ratio);
    config_.landing_capture_limit_m = this->declare_parameter(
        "landing_capture_limit_m", config_.landing_capture_limit_m);
    config_.landing_extension_m =
        this->declare_parameter("landing_extension_m",
                                config_.landing_extension_m);
    config_.support_capture_ratio =
        this->declare_parameter("support_capture_ratio",
                                config_.support_capture_ratio);
    config_.support_hip_rad =
        this->declare_parameter("support_hip_rad", config_.support_hip_rad);
    config_.support_thigh_rad =
        this->declare_parameter("support_thigh_rad", config_.support_thigh_rad);
    config_.support_calf_rad =
        this->declare_parameter("support_calf_rad", config_.support_calf_rad);
    config_.support_front_compact_delta_rad = this->declare_parameter(
        "support_front_compact_delta_rad",
        config_.support_front_compact_delta_rad);
    config_.support_rear_compact_delta_rad = this->declare_parameter(
        "support_rear_compact_delta_rad",
        config_.support_rear_compact_delta_rad);

    publisher_ =
        this->create_publisher<std_msgs::msg::Float64>("/jump_target_distance", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500), [this]() { PublishTargetDistance(); });

    const auto plan = go2_jump_planner::MakeJumpPlan(config_);
    RCLCPP_INFO(this->get_logger(),
                "Planner target=%.3f m, scale_mode=%s, ballistic_takeoff=%.3f m/s, speed_scale=%.3f, takeoff_speed=%.3f m/s, takeoff_vx=%.3f m/s, takeoff_vz=%.3f m/s, apex=%.3f m, capture_offset=%.3f m, flight=%.3f s",
                plan.target_distance_m,
                plan.using_takeoff_speed_scale_curve ? "curve" : "manual",
                plan.ballistic_takeoff_speed_mps,
                plan.takeoff_speed_scale, plan.takeoff_speed_mps,
                plan.takeoff_velocity_x_mps, plan.takeoff_velocity_z_mps,
                plan.apex_height_above_takeoff_m, plan.landing_capture_offset_m,
                plan.estimated_flight_time_s);
  }

 private:
  void PublishTargetDistance() {
    std_msgs::msg::Float64 msg;
    msg.data = config_.target_distance_m;
    publisher_->publish(msg);
  }

  go2_jump_planner::JumpPlannerConfig config_{};
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JumpTargetNode>());
  rclcpp::shutdown();
  return 0;
}

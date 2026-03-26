#include "go2_jump_core/jump_task.hpp"

#include <chrono>
#include <memory>

#include "go2_jump_msgs/msg/jump_task.hpp"
#include "rclcpp/rclcpp.hpp"

namespace {

go2_jump_msgs::msg::JumpTask ToRosMessage(
    const go2_jump_core::JumpTaskSpec& task, const rclcpp::Time& stamp) {
  go2_jump_msgs::msg::JumpTask msg;
  msg.header.stamp = stamp;
  msg.task_id = task.task_id;
  msg.target_distance_m = task.objective.target_distance_m;
  msg.takeoff_angle_deg = task.objective.takeoff_angle_deg;
  msg.target_takeoff_speed_mps = task.target_takeoff_speed_mps;
  msg.target_takeoff_velocity_x_mps = task.target_takeoff_velocity_x_mps;
  msg.target_takeoff_velocity_z_mps = task.target_takeoff_velocity_z_mps;
  msg.target_takeoff_pitch_deg = task.objective.target_takeoff_pitch_deg;
  msg.target_landing_pitch_deg = task.objective.target_landing_pitch_deg;
  msg.crouch_duration_s = task.crouch_duration_s;
  msg.push_duration_s = task.push_duration_s;
  msg.estimated_flight_time_s = task.estimated_flight_time_s;
  msg.landing_duration_s = task.landing_duration_s;
  msg.settle_duration_s = task.settle_duration_s;
  msg.horizon_duration_s = task.horizon_duration_s;
  msg.max_motion_duration_s = task.objective.max_motion_duration_s;
  return msg;
}

}  // namespace

class JumpTaskNode : public rclcpp::Node {
 public:
  JumpTaskNode() : Node("go2_jump_task_node") {
    objective_.target_distance_m =
        declare_parameter("target_distance_m", objective_.target_distance_m);
    objective_.takeoff_angle_deg =
        declare_parameter("takeoff_angle_deg", objective_.takeoff_angle_deg);
    objective_.takeoff_speed_scale =
        declare_parameter("takeoff_speed_scale", objective_.takeoff_speed_scale);
    objective_.target_takeoff_pitch_deg = declare_parameter(
        "target_takeoff_pitch_deg", objective_.target_takeoff_pitch_deg);
    objective_.target_landing_pitch_deg = declare_parameter(
        "target_landing_pitch_deg", objective_.target_landing_pitch_deg);
    objective_.max_motion_duration_s = declare_parameter(
        "max_motion_duration_s", objective_.max_motion_duration_s);

    config_.gravity_mps2 = declare_parameter("gravity_mps2", config_.gravity_mps2);
    config_.crouch_duration_base_s =
        declare_parameter("crouch_duration_base_s", config_.crouch_duration_base_s);
    config_.crouch_duration_gain_s_per_m = declare_parameter(
        "crouch_duration_gain_s_per_m", config_.crouch_duration_gain_s_per_m);
    config_.push_duration_base_s =
        declare_parameter("push_duration_base_s", config_.push_duration_base_s);
    config_.push_duration_gain_s_per_m = declare_parameter(
        "push_duration_gain_s_per_m", config_.push_duration_gain_s_per_m);
    config_.landing_duration_base_s = declare_parameter(
        "landing_duration_base_s", config_.landing_duration_base_s);
    config_.landing_duration_gain_s_per_m = declare_parameter(
        "landing_duration_gain_s_per_m", config_.landing_duration_gain_s_per_m);
    config_.settle_duration_base_s =
        declare_parameter("settle_duration_base_s", config_.settle_duration_base_s);
    config_.settle_duration_gain_s_per_m = declare_parameter(
        "settle_duration_gain_s_per_m", config_.settle_duration_gain_s_per_m);
    config_.horizon_duration_base_s = declare_parameter(
        "horizon_duration_base_s", config_.horizon_duration_base_s);
    config_.horizon_duration_gain_s_per_m = declare_parameter(
        "horizon_duration_gain_s_per_m", config_.horizon_duration_gain_s_per_m);
    config_.flight_margin_s =
        declare_parameter("flight_margin_s", config_.flight_margin_s);

    publisher_ =
        create_publisher<go2_jump_msgs::msg::JumpTask>("/go2_jump/task", 10);

    task_ = go2_jump_core::BuildJumpTaskSpec(objective_, config_);
    RCLCPP_INFO(
        get_logger(),
        "JumpTask task_id=%s distance=%.3f takeoff_speed=%.3f vx=%.3f vz=%.3f flight=%.3f horizon=%.3f",
        task_.task_id.c_str(), task_.objective.target_distance_m,
        task_.target_takeoff_speed_mps, task_.target_takeoff_velocity_x_mps,
        task_.target_takeoff_velocity_z_mps, task_.estimated_flight_time_s,
        task_.horizon_duration_s);

    timer_ = create_wall_timer(std::chrono::milliseconds(200), [this]() {
      publisher_->publish(ToRosMessage(task_, now()));
    });
  }

 private:
  go2_jump_core::JumpObjective objective_{};
  go2_jump_core::JumpTaskConfig config_{};
  go2_jump_core::JumpTaskSpec task_{};
  rclcpp::Publisher<go2_jump_msgs::msg::JumpTask>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JumpTaskNode>());
  rclcpp::shutdown();
  return 0;
}

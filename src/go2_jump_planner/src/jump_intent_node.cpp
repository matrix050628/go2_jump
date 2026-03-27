#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "go2_jump_core/jump_task.hpp"
#include "go2_jump_msgs/msg/jump_intent.hpp"
#include "go2_jump_msgs/msg/jump_task.hpp"
#include "rclcpp/rclcpp.hpp"

namespace {

bool NearlyEqual(double a, double b, double eps = 1e-6) {
  return std::abs(a - b) <= eps;
}

go2_jump_core::JumpTaskSpec TaskFromMsg(const go2_jump_msgs::msg::JumpTask& msg,
                                        double gravity_mps2) {
  go2_jump_core::JumpTaskSpec task{};
  task.task_id = msg.task_id;
  task.objective.target_distance_m = msg.target_distance_m;
  task.objective.takeoff_angle_deg = msg.takeoff_angle_deg;
  task.objective.takeoff_speed_scale = 1.0;
  task.objective.target_takeoff_pitch_deg = msg.target_takeoff_pitch_deg;
  task.objective.target_landing_pitch_deg = msg.target_landing_pitch_deg;
  task.objective.max_motion_duration_s = msg.max_motion_duration_s;
  task.target_takeoff_speed_mps = msg.target_takeoff_speed_mps;
  task.target_takeoff_velocity_x_mps = msg.target_takeoff_velocity_x_mps;
  task.target_takeoff_velocity_z_mps = msg.target_takeoff_velocity_z_mps;
  task.estimated_flight_time_s = msg.estimated_flight_time_s;
  task.crouch_duration_s = msg.crouch_duration_s;
  task.push_duration_s = msg.push_duration_s;
  task.landing_duration_s = msg.landing_duration_s;
  task.settle_duration_s = msg.settle_duration_s;
  task.horizon_duration_s = msg.horizon_duration_s;
  task.total_motion_duration_s = msg.crouch_duration_s + msg.push_duration_s +
                                 msg.estimated_flight_time_s + msg.landing_duration_s +
                                 msg.settle_duration_s;
  return go2_jump_core::NormalizeJumpTaskSpec(std::move(task), gravity_mps2);
}

bool SameTaskSpec(const go2_jump_core::JumpTaskSpec& a,
                  const go2_jump_core::JumpTaskSpec& b) {
  return a.task_id == b.task_id &&
         NearlyEqual(a.objective.target_distance_m, b.objective.target_distance_m) &&
         NearlyEqual(a.objective.takeoff_angle_deg, b.objective.takeoff_angle_deg) &&
         NearlyEqual(a.objective.target_takeoff_pitch_deg,
                     b.objective.target_takeoff_pitch_deg) &&
         NearlyEqual(a.objective.target_landing_pitch_deg,
                     b.objective.target_landing_pitch_deg) &&
         NearlyEqual(a.target_takeoff_speed_mps, b.target_takeoff_speed_mps) &&
         NearlyEqual(a.target_takeoff_velocity_x_mps,
                     b.target_takeoff_velocity_x_mps) &&
         NearlyEqual(a.target_takeoff_velocity_z_mps,
                     b.target_takeoff_velocity_z_mps) &&
         NearlyEqual(a.estimated_flight_time_s, b.estimated_flight_time_s) &&
         NearlyEqual(a.crouch_duration_s, b.crouch_duration_s) &&
         NearlyEqual(a.push_duration_s, b.push_duration_s) &&
         NearlyEqual(a.landing_duration_s, b.landing_duration_s) &&
         NearlyEqual(a.settle_duration_s, b.settle_duration_s) &&
         NearlyEqual(a.horizon_duration_s, b.horizon_duration_s);
}

go2_jump_msgs::msg::JumpIntent ToIntentMsg(
    const go2_jump_core::JumpKinodynamicIntent& intent, const rclcpp::Time& stamp) {
  go2_jump_msgs::msg::JumpIntent msg;
  msg.header.stamp = stamp;
  msg.valid = intent.valid;
  msg.task_id = intent.task_id;
  msg.planner_backend = intent.planner_backend;
  msg.target_distance_m = intent.target_distance_m;
  msg.target_takeoff_velocity_x_mps = intent.target_takeoff_velocity_x_mps;
  msg.target_takeoff_velocity_z_mps = intent.target_takeoff_velocity_z_mps;
  msg.target_takeoff_pitch_deg = intent.target_takeoff_pitch_deg;
  msg.target_landing_pitch_deg = intent.target_landing_pitch_deg;
  msg.crouch_duration_s = intent.crouch_duration_s;
  msg.push_duration_s = intent.push_duration_s;
  msg.estimated_flight_time_s = intent.estimated_flight_time_s;
  msg.landing_duration_s = intent.landing_duration_s;
  msg.settle_duration_s = intent.settle_duration_s;
  msg.horizon_duration_s = intent.horizon_duration_s;
  msg.crouch_height_offset_m = intent.crouch_height_offset_m;
  msg.push_height_offset_m = intent.push_height_offset_m;
  msg.flight_height_offset_m = intent.flight_height_offset_m;
  msg.landing_height_offset_m = intent.landing_height_offset_m;
  msg.leg_retraction_scale = intent.leg_retraction_scale;
  msg.landing_brace_scale = intent.landing_brace_scale;
  msg.front_push_foot_x_bias_m = intent.front_push_foot_x_bias_m;
  msg.rear_push_foot_x_bias_m = intent.rear_push_foot_x_bias_m;
  msg.front_landing_foot_x_bias_m = intent.front_landing_foot_x_bias_m;
  msg.rear_landing_foot_x_bias_m = intent.rear_landing_foot_x_bias_m;
  msg.swing_foot_height_m = intent.swing_foot_height_m;
  msg.planned_apex_height_m = intent.planned_apex_height_m;
  return msg;
}

}  // namespace

class JumpIntentNode : public rclcpp::Node {
 public:
  JumpIntentNode() : Node("go2_jump_intent_node") {
    planner_backend_ = declare_parameter("planner_backend", planner_backend_);
    publish_period_ms_ = declare_parameter("publish_period_ms", publish_period_ms_);
    velocity_x_scale_ = declare_parameter("velocity_x_scale", velocity_x_scale_);
    velocity_z_scale_ = declare_parameter("velocity_z_scale", velocity_z_scale_);
    push_duration_scale_ =
        declare_parameter("push_duration_scale", push_duration_scale_);
    flight_duration_scale_ =
        declare_parameter("flight_duration_scale", flight_duration_scale_);

    config_.gravity_mps2 = declare_parameter("gravity_mps2", config_.gravity_mps2);
    config_.crouch_height_offset_m = declare_parameter(
        "crouch_height_offset_m", config_.crouch_height_offset_m);
    config_.push_height_offset_m = declare_parameter(
        "push_height_offset_m", config_.push_height_offset_m);
    config_.flight_height_offset_m = declare_parameter(
        "flight_height_offset_m", config_.flight_height_offset_m);
    config_.landing_height_offset_m = declare_parameter(
        "landing_height_offset_m", config_.landing_height_offset_m);

    task_sub_ = create_subscription<go2_jump_msgs::msg::JumpTask>(
        "/go2_jump/task", 10,
        [this](const go2_jump_msgs::msg::JumpTask::SharedPtr msg) { OnTask(*msg); });
    intent_pub_ =
        create_publisher<go2_jump_msgs::msg::JumpIntent>("/go2_jump/intent", 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(publish_period_ms_),
                               [this]() { PublishIntent(); });
  }

 private:
  void OnTask(const go2_jump_msgs::msg::JumpTask& msg) {
    const auto task = TaskFromMsg(msg, config_.gravity_mps2);
    if (have_task_ && SameTaskSpec(task, active_task_)) {
      return;
    }

    active_task_ = task;
    have_task_ = true;
    active_intent_ =
        go2_jump_core::BuildHeuristicJumpKinodynamicIntent(active_task_, config_);
    active_intent_.planner_backend = planner_backend_;
    active_intent_.target_takeoff_velocity_x_mps *= velocity_x_scale_;
    active_intent_.target_takeoff_velocity_z_mps *= velocity_z_scale_;
    active_intent_.push_duration_s *= push_duration_scale_;
    active_intent_.estimated_flight_time_s *= flight_duration_scale_;

    RCLCPP_INFO(
        get_logger(),
        "Planned JumpIntent task_id=%s backend=%s vx=%.3f vz=%.3f push=%.3f flight=%.3f",
        active_intent_.task_id.c_str(), active_intent_.planner_backend.c_str(),
        active_intent_.target_takeoff_velocity_x_mps,
        active_intent_.target_takeoff_velocity_z_mps, active_intent_.push_duration_s,
        active_intent_.estimated_flight_time_s);
    PublishIntent();
  }

  void PublishIntent() {
    if (!have_task_ || !active_intent_.valid) {
      return;
    }
    intent_pub_->publish(ToIntentMsg(active_intent_, now()));
  }

  std::string planner_backend_{"heuristic_explicit"};
  int publish_period_ms_{200};
  double velocity_x_scale_{1.0};
  double velocity_z_scale_{1.0};
  double push_duration_scale_{1.0};
  double flight_duration_scale_{1.0};
  go2_jump_core::JumpTaskConfig config_{};
  go2_jump_core::JumpTaskSpec active_task_{};
  go2_jump_core::JumpKinodynamicIntent active_intent_{};
  bool have_task_{false};

  rclcpp::Subscription<go2_jump_msgs::msg::JumpTask>::SharedPtr task_sub_;
  rclcpp::Publisher<go2_jump_msgs::msg::JumpIntent>::SharedPtr intent_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JumpIntentNode>());
  rclcpp::shutdown();
  return 0;
}

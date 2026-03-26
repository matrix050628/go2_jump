#pragma once

#include <array>
#include <vector>

namespace go2_jump_planner {

struct JumpPlannerConfig {
  double target_distance_m{0.25};
  double takeoff_angle_deg{45.0};
  double takeoff_speed_scale{1.0};
  bool use_takeoff_speed_scale_curve{false};
  std::vector<double> takeoff_speed_scale_distance_points_m{};
  std::vector<double> takeoff_speed_scale_values{};
  double gravity_mps2{9.81};
  double leg_link_length_m{0.213};
  double landing_capture_time_constant_s{0.06};
  double landing_capture_rear_ratio{0.6};
  double landing_capture_limit_m{0.08};
  double landing_extension_m{0.015};
  double support_capture_ratio{0.6};

  double stand_hip_rad{0.00571868};
  double stand_thigh_rad{0.608813};
  double stand_calf_rad{-1.21763};
  double stand_front_compact_delta_rad{0.06};
  double stand_rear_compact_delta_rad{-0.18};

  double crouch_hip_rad{0.0473455};
  double crouch_thigh_rad{1.22187};
  double crouch_calf_rad{-2.44375};
  double crouch_front_compact_delta_rad{-0.06};
  double crouch_rear_compact_delta_rad{0.14};
  double crouch_forward_bias_rad{0.06};

  double push_hip_rad{0.0};
  double push_thigh_rad{0.82};
  double push_calf_rad{-1.08};
  double push_front_compact_delta_rad{0.04};
  double push_rear_compact_delta_rad{-0.18};
  double push_forward_bias_rad{0.04};

  double flight_hip_rad{0.0};
  double flight_thigh_rad{0.92};
  double flight_calf_rad{-1.55};
  double flight_front_compact_delta_rad{0.12};
  double flight_rear_compact_delta_rad{-0.04};

  double landing_hip_rad{0.025};
  double landing_thigh_rad{1.05};
  double landing_calf_rad{-2.05};
  double landing_front_compact_delta_rad{0.08};
  double landing_rear_compact_delta_rad{0.02};
  double landing_absorption_blend{0.45};

  double support_hip_rad{0.036};
  double support_thigh_rad{1.126};
  double support_calf_rad{-2.215};
  double support_front_compact_delta_rad{0.164};
  double support_rear_compact_delta_rad{-0.164};

  double crouch_duration_s{0.32};
  double push_duration_s{0.16};
  double landing_duration_s{0.20};
  double recovery_duration_s{0.32};

  double push_thigh_tau_ff{2.0};
  double push_calf_tau_ff{3.5};
  double landing_thigh_tau_ff{1.2};
  double landing_calf_tau_ff{2.2};
};

struct JumpPlan {
  double target_distance_m{0.25};
  double takeoff_angle_deg{45.0};
  double ballistic_takeoff_speed_mps{0.0};
  double takeoff_speed_scale{1.0};
  bool using_takeoff_speed_scale_curve{false};
  double takeoff_speed_mps{0.0};
  double takeoff_velocity_x_mps{0.0};
  double takeoff_velocity_z_mps{0.0};
  double touchdown_velocity_x_mps{0.0};
  double touchdown_velocity_z_mps{0.0};
  double apex_height_above_takeoff_m{0.0};
  double landing_capture_offset_m{0.0};
  double effective_support_capture_ratio{0.0};
  double effective_support_capture_offset_m{0.0};
  double effective_support_capture_offset_limit_m{0.0};
  double estimated_flight_time_s{0.0};

  double crouch_duration_s{0.0};
  double push_duration_s{0.0};
  double flight_duration_s{0.0};
  double landing_duration_s{0.0};
  double recovery_duration_s{0.0};

  double push_thigh_tau_ff{0.0};
  double push_calf_tau_ff{0.0};
  double landing_thigh_tau_ff{0.0};
  double landing_calf_tau_ff{0.0};

  std::array<double, 12> stand_pose{};
  std::array<double, 12> crouch_pose{};
  std::array<double, 12> push_pose{};
  std::array<double, 12> flight_pose{};
  std::array<double, 12> landing_pose{};
  std::array<double, 12> support_pose{};
};

JumpPlan MakeJumpPlan(const JumpPlannerConfig& config);

std::array<double, 12> InterpolatePose(const std::array<double, 12>& start,
                                       const std::array<double, 12>& end,
                                       double alpha);

}  // namespace go2_jump_planner

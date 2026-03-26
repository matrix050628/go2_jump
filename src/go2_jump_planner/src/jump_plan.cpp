#include "go2_jump_planner/jump_plan.hpp"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace go2_jump_planner {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kCompactnessToCalfScale = 1.6;

struct SagittalFootPoint {
  double x_m{0.0};
  double z_m{0.0};
};

struct SagittalLegAngles {
  double thigh_rad{0.0};
  double calf_rad{0.0};
};

double Clamp(double value, double low, double high) {
  return std::max(low, std::min(high, value));
}

double SmoothClamp01(double value) {
  const double x = Clamp(value, 0.0, 1.0);
  return x * x * (3.0 - 2.0 * x);
}

double ResolveTakeoffSpeedScale(const JumpPlannerConfig& config,
                                double target_distance_m,
                                bool& using_curve) {
  using_curve = false;
  const double manual_scale = Clamp(config.takeoff_speed_scale, 0.85, 1.30);
  if (!config.use_takeoff_speed_scale_curve) {
    return manual_scale;
  }

  const auto& distance_points = config.takeoff_speed_scale_distance_points_m;
  const auto& scale_values = config.takeoff_speed_scale_values;
  if (distance_points.empty() || distance_points.size() != scale_values.size()) {
    return manual_scale;
  }

  std::vector<std::pair<double, double>> calibration_points;
  calibration_points.reserve(distance_points.size());
  for (std::size_t i = 0; i < distance_points.size(); ++i) {
    calibration_points.emplace_back(
        Clamp(distance_points[i], 0.05, 0.60),
        Clamp(scale_values[i], 0.85, 1.30));
  }

  std::sort(calibration_points.begin(), calibration_points.end(),
            [](const auto& lhs, const auto& rhs) { return lhs.first < rhs.first; });
  if (calibration_points.empty()) {
    return manual_scale;
  }

  using_curve = true;
  if (calibration_points.size() == 1 ||
      target_distance_m <= calibration_points.front().first) {
    return calibration_points.front().second;
  }
  if (target_distance_m >= calibration_points.back().first) {
    return calibration_points.back().second;
  }

  for (std::size_t i = 1; i < calibration_points.size(); ++i) {
    const auto& low = calibration_points[i - 1];
    const auto& high = calibration_points[i];
    if (target_distance_m > high.first) {
      continue;
    }

    const double segment_width = std::max(high.first - low.first, 1e-6);
    const double alpha = Clamp((target_distance_m - low.first) / segment_width,
                               0.0, 1.0);
    return low.second + (high.second - low.second) * alpha;
  }

  return calibration_points.back().second;
}

std::array<double, 12> MakeFrontRearPose(double front_hip,
                                         double front_thigh,
                                         double front_calf,
                                         double rear_hip,
                                         double rear_thigh,
                                         double rear_calf) {
  return {
      -front_hip, front_thigh, front_calf,
      front_hip,  front_thigh, front_calf,
      -rear_hip,  rear_thigh,  rear_calf,
      rear_hip,   rear_thigh,  rear_calf,
  };
}

std::array<double, 12> MakeCompactnessBiasedPose(double hip,
                                                 double thigh,
                                                 double calf,
                                                 double front_compactness,
                                                 double rear_compactness) {
  return MakeFrontRearPose(
      hip, thigh + front_compactness,
      calf - kCompactnessToCalfScale * front_compactness,
      hip, thigh + rear_compactness,
      calf - kCompactnessToCalfScale * rear_compactness);
}

SagittalFootPoint ForwardKinematics(double thigh_rad,
                                    double calf_rad,
                                    double leg_link_length_m) {
  const double leg_angle = thigh_rad + calf_rad;
  return {
      leg_link_length_m * std::sin(thigh_rad) +
          leg_link_length_m * std::sin(leg_angle),
      -(leg_link_length_m * std::cos(thigh_rad) +
        leg_link_length_m * std::cos(leg_angle)),
  };
}

SagittalLegAngles SolveSagittalLegIk(double x_m,
                                     double z_m,
                                     double leg_link_length_m) {
  const double l1 = leg_link_length_m;
  const double l2 = leg_link_length_m;
  const double min_radius_m = 1e-5;
  const double max_radius_m = std::max(min_radius_m, l1 + l2 - 1e-5);
  const double radius_m = Clamp(std::sqrt(x_m * x_m + z_m * z_m), min_radius_m,
                                max_radius_m);
  const double cos_calf = Clamp((radius_m * radius_m - l1 * l1 - l2 * l2) /
                                    (2.0 * l1 * l2),
                                -1.0, 1.0);
  const double calf_rad = -std::acos(cos_calf);
  const double k1 = l1 + l2 * std::cos(calf_rad);
  const double k2 = l2 * std::sin(calf_rad);
  const double thigh_rad =
      std::atan2(x_m, -z_m) - std::atan2(k2, k1);
  return {thigh_rad, calf_rad};
}

std::array<double, 12> MakeSagittalPlacementPose(
    double hip_rad,
    const SagittalFootPoint& front_foot,
    const SagittalFootPoint& rear_foot,
    double leg_link_length_m) {
  const auto front_leg =
      SolveSagittalLegIk(front_foot.x_m, front_foot.z_m, leg_link_length_m);
  const auto rear_leg =
      SolveSagittalLegIk(rear_foot.x_m, rear_foot.z_m, leg_link_length_m);
  return MakeFrontRearPose(hip_rad, front_leg.thigh_rad, front_leg.calf_rad,
                           hip_rad, rear_leg.thigh_rad, rear_leg.calf_rad);
}

}  // namespace

JumpPlan MakeJumpPlan(const JumpPlannerConfig& config) {
  JumpPlan plan{};
  plan.target_distance_m = Clamp(config.target_distance_m, 0.05, 0.60);
  plan.takeoff_angle_deg = Clamp(config.takeoff_angle_deg, 25.0, 65.0);
  plan.takeoff_speed_scale = ResolveTakeoffSpeedScale(
      config, plan.target_distance_m, plan.using_takeoff_speed_scale_curve);

  const double takeoff_angle_rad = plan.takeoff_angle_deg * kPi / 180.0;
  const double ballistic_denominator =
      std::max(std::sin(2.0 * takeoff_angle_rad), 0.2);
  plan.ballistic_takeoff_speed_mps =
      std::sqrt(plan.target_distance_m * config.gravity_mps2 / ballistic_denominator);
  // Expose a small empirical calibration hook so we can keep the target
  // distance honest while compensating for low-level execution losses.
  plan.takeoff_speed_mps =
      plan.ballistic_takeoff_speed_mps * plan.takeoff_speed_scale;
  plan.takeoff_velocity_x_mps = plan.takeoff_speed_mps * std::cos(takeoff_angle_rad);
  plan.takeoff_velocity_z_mps = plan.takeoff_speed_mps * std::sin(takeoff_angle_rad);
  plan.touchdown_velocity_x_mps = plan.takeoff_velocity_x_mps;
  plan.touchdown_velocity_z_mps = -plan.takeoff_velocity_z_mps;
  plan.apex_height_above_takeoff_m =
      (plan.takeoff_velocity_z_mps * plan.takeoff_velocity_z_mps) /
      (2.0 * std::max(config.gravity_mps2, 1e-6));
  plan.landing_capture_offset_m = Clamp(
      plan.touchdown_velocity_x_mps * config.landing_capture_time_constant_s,
      0.0, config.landing_capture_limit_m);
  const double support_distance_alpha =
      SmoothClamp01((plan.target_distance_m - 0.20) / 0.10);
  plan.effective_support_capture_ratio = Clamp(
      config.support_capture_ratio * (0.80 + 0.30 * support_distance_alpha),
      0.0, 1.20);
  const double raw_support_capture_offset_m =
      plan.effective_support_capture_ratio * plan.landing_capture_offset_m;
  plan.effective_support_capture_offset_limit_m = Clamp(
      plan.target_distance_m * (0.22 + 0.06 * support_distance_alpha), 0.025,
      config.landing_capture_limit_m);
  plan.effective_support_capture_offset_m =
      std::min(raw_support_capture_offset_m,
               plan.effective_support_capture_offset_limit_m);
  plan.estimated_flight_time_s = Clamp(
      2.0 * plan.takeoff_velocity_z_mps / config.gravity_mps2,
      0.16, 0.45);

  const double speed_scale = SmoothClamp01((plan.takeoff_speed_mps - 0.9) / 1.2);
  const double crouch_forward_bias_rad =
      Clamp(config.crouch_forward_bias_rad * (0.75 + 0.35 * speed_scale), -0.20,
            0.20);
  const double push_forward_bias_rad =
      Clamp(config.push_forward_bias_rad * (0.80 + 0.40 * speed_scale), -0.20,
            0.20);
  const double landing_absorption_blend =
      Clamp(config.landing_absorption_blend * (0.85 + 0.20 * speed_scale), 0.0,
            1.0);

  plan.crouch_duration_s = config.crouch_duration_s + 0.04 * speed_scale;
  plan.push_duration_s = config.push_duration_s + 0.03 * speed_scale;
  plan.flight_duration_s = plan.estimated_flight_time_s;
  plan.landing_duration_s = config.landing_duration_s + 0.02 * speed_scale;
  plan.recovery_duration_s = config.recovery_duration_s;

  plan.push_thigh_tau_ff = config.push_thigh_tau_ff * (0.8 + 0.4 * speed_scale);
  plan.push_calf_tau_ff = config.push_calf_tau_ff * (0.8 + 0.4 * speed_scale);
  plan.landing_thigh_tau_ff =
      config.landing_thigh_tau_ff * (0.8 + 0.4 * speed_scale);
  plan.landing_calf_tau_ff =
      config.landing_calf_tau_ff * (0.8 + 0.4 * speed_scale);

  plan.stand_pose = MakeCompactnessBiasedPose(
      config.stand_hip_rad, config.stand_thigh_rad, config.stand_calf_rad,
      config.stand_front_compact_delta_rad,
      config.stand_rear_compact_delta_rad);

  plan.crouch_pose = MakeCompactnessBiasedPose(
      config.crouch_hip_rad, config.crouch_thigh_rad + 0.12 * speed_scale,
      config.crouch_calf_rad - 0.18 * speed_scale,
      config.crouch_front_compact_delta_rad + crouch_forward_bias_rad,
      config.crouch_rear_compact_delta_rad - crouch_forward_bias_rad);

  plan.push_pose = MakeCompactnessBiasedPose(
      config.push_hip_rad, config.push_thigh_rad - 0.05 * speed_scale,
      config.push_calf_rad + 0.10 * speed_scale,
      config.push_front_compact_delta_rad + push_forward_bias_rad,
      config.push_rear_compact_delta_rad - push_forward_bias_rad);

  plan.flight_pose = MakeCompactnessBiasedPose(
      config.flight_hip_rad, config.flight_thigh_rad + 0.08 * speed_scale,
      config.flight_calf_rad - 0.08 * speed_scale,
      config.flight_front_compact_delta_rad,
      config.flight_rear_compact_delta_rad);

  const double landing_front_thigh =
      config.landing_thigh_rad + 0.10 * speed_scale +
      config.landing_front_compact_delta_rad;
  const double landing_front_calf =
      config.landing_calf_rad - 0.12 * speed_scale -
      kCompactnessToCalfScale * config.landing_front_compact_delta_rad;
  const double landing_rear_thigh =
      config.landing_thigh_rad + 0.10 * speed_scale +
      config.landing_rear_compact_delta_rad;
  const double landing_rear_calf =
      config.landing_calf_rad - 0.12 * speed_scale -
      kCompactnessToCalfScale * config.landing_rear_compact_delta_rad;

  auto landing_front_foot = ForwardKinematics(
      landing_front_thigh, landing_front_calf, config.leg_link_length_m);
  auto landing_rear_foot = ForwardKinematics(
      landing_rear_thigh, landing_rear_calf, config.leg_link_length_m);
  landing_front_foot.x_m += plan.landing_capture_offset_m;
  landing_rear_foot.x_m -=
      config.landing_capture_rear_ratio * plan.landing_capture_offset_m;
  landing_front_foot.z_m -= config.landing_extension_m;
  landing_rear_foot.z_m -= 0.5 * config.landing_extension_m;

  const auto nominal_landing_pose = MakeSagittalPlacementPose(
      config.landing_hip_rad, landing_front_foot, landing_rear_foot,
      config.leg_link_length_m);
  plan.landing_pose = InterpolatePose(
      nominal_landing_pose, plan.crouch_pose, 0.5 * landing_absorption_blend);

  const double support_front_thigh =
      config.support_thigh_rad + 0.06 * speed_scale +
      config.support_front_compact_delta_rad;
  const double support_front_calf =
      config.support_calf_rad - 0.08 * speed_scale -
      kCompactnessToCalfScale * config.support_front_compact_delta_rad;
  const double support_rear_thigh =
      config.support_thigh_rad + 0.06 * speed_scale +
      config.support_rear_compact_delta_rad;
  const double support_rear_calf =
      config.support_calf_rad - 0.08 * speed_scale -
      kCompactnessToCalfScale * config.support_rear_compact_delta_rad;
  auto support_front_foot = ForwardKinematics(
      support_front_thigh, support_front_calf, config.leg_link_length_m);
  auto support_rear_foot = ForwardKinematics(
      support_rear_thigh, support_rear_calf, config.leg_link_length_m);
  support_front_foot.x_m += plan.effective_support_capture_offset_m;
  support_rear_foot.x_m -=
      config.landing_capture_rear_ratio *
      plan.effective_support_capture_offset_m;
  plan.support_pose = MakeSagittalPlacementPose(
      config.support_hip_rad, support_front_foot, support_rear_foot,
      config.leg_link_length_m);

  return plan;
}

std::array<double, 12> InterpolatePose(const std::array<double, 12>& start,
                                       const std::array<double, 12>& end,
                                       double alpha) {
  const double clamped_alpha = Clamp(alpha, 0.0, 1.0);
  std::array<double, 12> output{};
  for (std::size_t i = 0; i < output.size(); ++i) {
    output[i] =
        start[i] + (end[i] - start[i]) * SmoothClamp01(clamped_alpha);
  }
  return output;
}

}  // namespace go2_jump_planner

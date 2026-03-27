#include "go2_jump_mpc/whole_body_mpc.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <mujoco/mujoco.h>

namespace go2_jump_mpc {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr double kDegToRad = kPi / 180.0;
constexpr std::size_t kFootCount = 4;

using JointArray = std::array<double, kControlledJointCount>;

constexpr std::array<const char*, kControlledJointCount> kJointNames{{
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
}};

constexpr std::array<const char*, kControlledJointCount> kActuatorNames{{
    "FR_hip", "FR_thigh", "FR_calf",
    "FL_hip", "FL_thigh", "FL_calf",
    "RR_hip", "RR_thigh", "RR_calf",
    "RL_hip", "RL_thigh", "RL_calf",
}};

constexpr std::array<const char*, kFootCount> kFootGeomNames{{
    "FR", "FL", "RR", "RL",
}};

constexpr std::array<const char*, kFootCount> kFootBodyNames{{
    "FR_foot", "FL_foot", "RR_foot", "RL_foot",
}};

constexpr std::array<std::array<double, 3>, kFootCount> kHipOffsets{{
    {{0.1934, -0.0465, 0.0}},
    {{0.1934, 0.0465, 0.0}},
    {{-0.1934, -0.0465, 0.0}},
    {{-0.1934, 0.0465, 0.0}},
}};

constexpr std::array<double, kFootCount> kHipLinkOffsetsM{{
    -0.0955, 0.0955, -0.0955, 0.0955,
}};

constexpr double kUpperLegLengthM = 0.213;
constexpr double kLowerLegLengthM = 0.213;

constexpr JointArray kJointLowerBounds{{
    -1.0472, -1.5708, -2.7227,
    -1.0472, -1.5708, -2.7227,
    -1.0472, -0.5236, -2.7227,
    -1.0472, -0.5236, -2.7227,
}};

constexpr JointArray kJointUpperBounds{{
    1.0472, 3.4907, -0.83776,
    1.0472, 3.4907, -0.83776,
    1.0472, 4.5379, -0.83776,
    1.0472, 4.5379, -0.83776,
}};

struct CandidateAction {
  double push_extension_scale{0.0};
  double pitch_adjust_scale{0.0};
  double flight_tuck_scale{0.0};
  double landing_brace_scale{0.0};
  double forward_drive_scale{0.0};
  double lift_drive_scale{0.0};
  double rear_drive_bias_scale{0.0};
};

struct RolloutResult {
  double score{std::numeric_limits<double>::infinity()};
  CandidateAction candidate{};
  go2_jump_core::JumpReferenceSample first_sample{};
  JointArray first_q_ref{};
  JointArray first_tau_ff{};
  double first_kp{0.0};
  double first_kd{0.0};
};

struct FrontRearForcePlan {
  bool valid{false};
  double front_fx_n{0.0};
  double front_fz_n{0.0};
  double rear_fx_n{0.0};
  double rear_fz_n{0.0};
};

double Clamp(double value, double low, double high) {
  return std::max(low, std::min(high, value));
}

double Square(double value) { return value * value; }

double Lerp(double a, double b, double alpha) {
  return a + (b - a) * Clamp(alpha, 0.0, 1.0);
}

double Smooth01(double value) {
  const double x = Clamp(value, 0.0, 1.0);
  return x * x * (3.0 - 2.0 * x);
}

FrontRearForcePlan SolveFrontRearForcePlan(double total_fx_n, double total_fz_n,
                                           double pitch_moment_nm,
                                           bool front_active,
                                           double front_x_m, double front_z_m,
                                           double front_support_n,
                                           bool rear_active, double rear_x_m,
                                           double rear_z_m, double rear_support_n,
                                           double front_fx_pref_n,
                                           double front_fz_pref_n,
                                           double friction_coeff,
                                           double max_group_delta_force_n) {
  FrontRearForcePlan plan{};
  if (!front_active && !rear_active) {
    return plan;
  }

  if (!rear_active) {
    plan.valid = true;
    plan.front_fx_n = total_fx_n;
    plan.front_fz_n = total_fz_n;
    return plan;
  }
  if (!front_active) {
    plan.valid = true;
    plan.rear_fx_n = total_fx_n;
    plan.rear_fz_n = total_fz_n;
    return plan;
  }

  const double a = front_z_m - rear_z_m;
  const double b = rear_x_m - front_x_m;
  const double c =
      pitch_moment_nm - (rear_z_m * total_fx_n - rear_x_m * total_fz_n);
  const double fx_weight = 1.0;
  const double fz_weight = 0.7;
  const double denom = a * a / fx_weight + b * b / fz_weight;

  double front_fx_n = front_fx_pref_n;
  double front_fz_n = front_fz_pref_n;
  if (denom > 1e-6) {
    const double residual = a * front_fx_pref_n + b * front_fz_pref_n - c;
    front_fx_n -= (a / fx_weight) * residual / denom;
    front_fz_n -= (b / fz_weight) * residual / denom;
  }

  const double front_min_fz_n =
      std::max(-0.85 * front_support_n, -max_group_delta_force_n);
  const double rear_min_fz_n =
      std::max(-0.85 * rear_support_n, -max_group_delta_force_n);
  front_fz_n = Clamp(front_fz_n, front_min_fz_n, max_group_delta_force_n);
  double rear_fz_n = total_fz_n - front_fz_n;
  rear_fz_n = Clamp(rear_fz_n, rear_min_fz_n, max_group_delta_force_n);
  front_fz_n = total_fz_n - rear_fz_n;

  const double front_normal_n = std::max(10.0, front_support_n + front_fz_n);
  const double rear_normal_n = std::max(10.0, rear_support_n + rear_fz_n);
  const double front_fx_limit_n = friction_coeff * front_normal_n;
  const double rear_fx_limit_n = friction_coeff * rear_normal_n;
  front_fx_n = Clamp(front_fx_n, -front_fx_limit_n, front_fx_limit_n);
  double rear_fx_n = total_fx_n - front_fx_n;
  rear_fx_n = Clamp(rear_fx_n, -rear_fx_limit_n, rear_fx_limit_n);
  front_fx_n = Clamp(total_fx_n - rear_fx_n, -front_fx_limit_n, front_fx_limit_n);

  plan.valid = true;
  plan.front_fx_n = front_fx_n;
  plan.front_fz_n = front_fz_n;
  plan.rear_fx_n = rear_fx_n;
  plan.rear_fz_n = rear_fz_n;
  return plan;
}

JointArray BlendPose(const JointArray& a, const JointArray& b, double alpha) {
  JointArray out{};
  for (std::size_t i = 0; i < out.size(); ++i) {
    out[i] = Lerp(a[i], b[i], alpha);
  }
  return out;
}

int CountContacts(const std::array<bool, kFootCount>& foot_contact) {
  return static_cast<int>(std::count(foot_contact.begin(), foot_contact.end(), true));
}

std::array<double, 4> QuaternionFromRpy(const std::array<double, 3>& rpy) {
  const double half_roll = 0.5 * rpy[0];
  const double half_pitch = 0.5 * rpy[1];
  const double half_yaw = 0.5 * rpy[2];
  const double cr = std::cos(half_roll);
  const double sr = std::sin(half_roll);
  const double cp = std::cos(half_pitch);
  const double sp = std::sin(half_pitch);
  const double cy = std::cos(half_yaw);
  const double sy = std::sin(half_yaw);
  return {
      cr * cp * cy + sr * sp * sy,
      sr * cp * cy - cr * sp * sy,
      cr * sp * cy + sr * cp * sy,
      cr * cp * sy - sr * sp * cy,
  };
}

std::array<double, 3> RpyFromQuaternion(const mjtNum* quat_wxyz) {
  const double w = quat_wxyz[0];
  const double x = quat_wxyz[1];
  const double y = quat_wxyz[2];
  const double z = quat_wxyz[3];

  std::array<double, 3> rpy{};
  rpy[0] = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
  rpy[1] = std::asin(Clamp(2.0 * (w * y - z * x), -1.0, 1.0));
  rpy[2] = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
  return rpy;
}

std::filesystem::path ResolveMujocoModelPath(const std::string& configured_path) {
  std::vector<std::filesystem::path> candidates;
  if (!configured_path.empty()) {
    candidates.emplace_back(configured_path);
  }
  candidates.emplace_back("/workspace/src/unitree_mujoco/unitree_robots/go2/scene.xml");
  candidates.emplace_back("/home/hayan/go2_jump_ws/src/unitree_mujoco/unitree_robots/go2/scene.xml");
  candidates.emplace_back(std::filesystem::current_path() /
                          "src/unitree_mujoco/unitree_robots/go2/scene.xml");

  for (const auto& candidate : candidates) {
    if (!candidate.empty() && std::filesystem::exists(candidate)) {
      return candidate;
    }
  }
  return {};
}

bool IsMujocoNativeBackend(const std::string& backend_name) {
  return backend_name == "mujoco_sampling" ||
         backend_name == "mujoco_native_mpc";
}

double JumpDistanceAlpha(const go2_jump_core::JumpTaskSpec& task) {
  return Clamp((task.objective.target_distance_m - 0.18) / 0.18, 0.0, 1.2);
}

bool UseTaskSpaceReferenceMode(const WholeBodyMpcConfig& config) {
  return config.reference_builder_mode == "taskspace_ik";
}

std::array<std::array<double, 3>, kFootCount> FootPositionsFromPose(
    const JointArray& pose) {
  std::array<std::array<double, 3>, kFootCount> foot_positions{};
  for (std::size_t leg_index = 0; leg_index < kFootCount; ++leg_index) {
    const double q_ab = pose[3 * leg_index + 0];
    const double q_hip = pose[3 * leg_index + 1];
    const double q_knee = pose[3 * leg_index + 2];
    const double leg_distance = std::sqrt(
        std::max(1e-7, kUpperLegLengthM * kUpperLegLengthM +
                           kLowerLegLengthM * kLowerLegLengthM +
                           2.0 * kUpperLegLengthM * kLowerLegLengthM *
                               std::cos(q_knee)));
    const double effective_swing = q_hip + 0.5 * q_knee;
    const double off_x_hip = -leg_distance * std::sin(effective_swing);
    const double off_z_hip = -leg_distance * std::cos(effective_swing);
    const double off_y_hip = kHipLinkOffsetsM[leg_index];

    foot_positions[leg_index][0] = kHipOffsets[leg_index][0] + off_x_hip;
    foot_positions[leg_index][1] =
        kHipOffsets[leg_index][1] +
        std::cos(q_ab) * off_y_hip - std::sin(q_ab) * off_z_hip;
    foot_positions[leg_index][2] =
        kHipOffsets[leg_index][2] +
        std::sin(q_ab) * off_y_hip + std::cos(q_ab) * off_z_hip;
  }
  return foot_positions;
}

JointArray PoseFromFootTargets(
    const std::array<std::array<double, 3>, kFootCount>& foot_targets,
    const JointArray& fallback_pose) {
  JointArray pose = fallback_pose;
  for (std::size_t leg_index = 0; leg_index < kFootCount; ++leg_index) {
    const double x = foot_targets[leg_index][0] - kHipOffsets[leg_index][0];
    const double y = foot_targets[leg_index][1] - kHipOffsets[leg_index][1];
    const double z = foot_targets[leg_index][2] - kHipOffsets[leg_index][2];
    const double l_hip = kHipLinkOffsetsM[leg_index];

    const double knee_cos = Clamp(
        (Square(x) + Square(y) + Square(z) - Square(l_hip) -
         Square(kLowerLegLengthM) - Square(kUpperLegLengthM)) /
            (2.0 * kLowerLegLengthM * kUpperLegLengthM),
        -1.0, 1.0);
    const double q_knee = -std::acos(knee_cos);
    const double leg_distance = std::sqrt(
        std::max(1e-7, Square(kUpperLegLengthM) + Square(kLowerLegLengthM) +
                           2.0 * kUpperLegLengthM * kLowerLegLengthM *
                               std::cos(q_knee)));
    const double q_hip =
        std::asin(Clamp(-x / leg_distance, -1.0, 1.0)) - 0.5 * q_knee;
    const double cos_eff = std::cos(q_hip + 0.5 * q_knee);
    const double c1 = l_hip * y - leg_distance * cos_eff * z;
    const double s1 = leg_distance * cos_eff * y + l_hip * z;
    const double q_ab = std::atan2(s1, c1);

    const std::array<double, 3> solved_leg{
        Clamp(q_ab, kJointLowerBounds[3 * leg_index + 0],
              kJointUpperBounds[3 * leg_index + 0]),
        Clamp(q_hip, kJointLowerBounds[3 * leg_index + 1],
              kJointUpperBounds[3 * leg_index + 1]),
        Clamp(q_knee, kJointLowerBounds[3 * leg_index + 2],
              kJointUpperBounds[3 * leg_index + 2]),
    };
    if (!std::isfinite(solved_leg[0]) || !std::isfinite(solved_leg[1]) ||
        !std::isfinite(solved_leg[2])) {
      continue;
    }

    pose[3 * leg_index + 0] = solved_leg[0];
    pose[3 * leg_index + 1] = solved_leg[1];
    pose[3 * leg_index + 2] = solved_leg[2];
  }
  return pose;
}

std::vector<CandidateAction> CandidateActionsForPhase(
    go2_jump_core::JumpPhase phase) {
  std::vector<CandidateAction> candidates{
      {},
      {1.0, 0.0, 0.0, 0.0},
      {1.6, 0.0, 0.0, 0.0},
      {-1.0, 0.0, 0.0, 0.0},
      {0.0, 1.0, 0.0, 0.0},
      {0.0, -1.0, 0.0, 0.0},
      {1.0, 1.0, 0.0, 0.0},
      {1.5, 0.8, 0.0, 0.0},
      {0.0, 0.0, 1.0, 0.0},
      {0.0, 0.0, -1.0, 0.0},
      {0.0, 0.0, 0.0, 1.0},
      {0.0, 0.0, 0.0, 1.4},
      {0.8, 1.6, 0.0, 0.0},
      {0.8, -1.6, 0.0, 0.0},
      {1.6, 1.4, 0.0, 0.0},
      {1.6, -1.4, 0.0, 0.0},
  };
  if (phase == go2_jump_core::JumpPhase::kFlight) {
    candidates.push_back({0.0, 0.5, 1.2, 0.0});
    candidates.push_back({0.0, 0.9, 1.6, 0.0});
    candidates.push_back({0.0, -0.5, -1.0, 0.5});
  } else if (phase == go2_jump_core::JumpPhase::kLanding ||
             phase == go2_jump_core::JumpPhase::kSettle) {
    candidates.push_back({0.0, 0.3, 0.3, 1.4});
    candidates.push_back({0.0, 0.6, 0.0, 1.8});
    candidates.push_back({0.0, -0.3, -0.2, 0.8});
  } else {
    candidates.push_back({1.2, 0.6, 0.0, 0.0});
    candidates.push_back({2.0, 0.9, 0.0, 0.0});
    candidates.push_back({-0.8, -0.6, 0.0, 0.0});
    candidates.push_back({1.0, 0.2, 0.0, 0.0, 0.8, 0.3, 0.8});
    candidates.push_back({1.2, 0.4, 0.0, 0.0, 1.0, 0.4, 1.0});
    candidates.push_back({1.4, 0.3, 0.0, 0.0, 1.3, 0.4, 1.3});
    candidates.push_back({1.6, 0.1, 0.0, 0.0, 1.5, 0.2, 1.4});
    candidates.push_back({0.8, 0.0, 0.0, 0.0, 0.4, 1.0, 0.6});
    candidates.push_back({1.0, -0.2, 0.0, 0.0, 0.9, 0.0, 1.2});
    candidates.push_back({1.1, 0.2, 0.0, 0.0, 1.0, -0.3, 1.0});
    candidates.push_back({1.3, 0.1, 0.0, 0.0, 1.2, -0.5, 1.3});
  }

  return candidates;
}

go2_jump_core::JumpReferenceSample ApplyCandidateToSample(
    go2_jump_core::JumpReferenceSample sample, const CandidateAction& action) {
  if (sample.phase == go2_jump_core::JumpPhase::kCrouch) {
    sample.desired_body_pitch_deg +=
        2.0 * action.pitch_adjust_scale - 1.0 * action.rear_drive_bias_scale;
  } else if (sample.phase == go2_jump_core::JumpPhase::kPush) {
    sample.desired_body_pitch_deg += 5.5 * action.pitch_adjust_scale -
                                     0.8 * action.forward_drive_scale -
                                     0.2 * action.rear_drive_bias_scale;
    sample.desired_forward_velocity_mps *=
        1.0 + 0.10 * action.push_extension_scale +
        0.08 * action.pitch_adjust_scale +
        0.20 * action.forward_drive_scale +
        0.08 * action.rear_drive_bias_scale;
    sample.desired_vertical_velocity_mps *=
        1.0 + 0.11 * action.push_extension_scale -
        0.03 * std::abs(action.pitch_adjust_scale) +
        0.12 * action.lift_drive_scale +
        0.01 * action.rear_drive_bias_scale -
        0.08 * action.forward_drive_scale;
  } else if (sample.phase == go2_jump_core::JumpPhase::kFlight) {
    sample.desired_body_pitch_deg += 2.2 * action.pitch_adjust_scale;
    sample.leg_retraction_ratio = Clamp(
        sample.leg_retraction_ratio + 0.22 * action.flight_tuck_scale, 0.0, 1.0);
    sample.landing_brace_factor = Clamp(
        sample.landing_brace_factor + 0.08 * action.landing_brace_scale, 0.0, 1.0);
  } else if (sample.phase == go2_jump_core::JumpPhase::kLanding ||
             sample.phase == go2_jump_core::JumpPhase::kSettle) {
    sample.desired_body_pitch_deg += 2.0 * action.pitch_adjust_scale;
    sample.landing_brace_factor = Clamp(
        sample.landing_brace_factor + 0.22 * action.landing_brace_scale, 0.0, 1.5);
  }
  return sample;
}

JointArray ApplyCandidateToPose(const JointArray& nominal_pose,
                                const go2_jump_core::JumpReferenceSample& sample,
                                const CandidateAction& action) {
  JointArray pose = nominal_pose;
  const double forward_scale = Clamp(
      std::abs(sample.desired_forward_velocity_mps) / 1.2, 0.5, 1.8);

  auto apply_leg_delta = [&pose](std::size_t thigh_idx, std::size_t calf_idx,
                                 double thigh_delta, double calf_delta) {
    pose[thigh_idx] += thigh_delta;
    pose[calf_idx] += calf_delta;
  };

  if (sample.phase == go2_jump_core::JumpPhase::kCrouch) {
    const double front_thigh_delta =
        (-0.04 * action.pitch_adjust_scale +
         0.03 * action.rear_drive_bias_scale) * forward_scale;
    const double front_calf_delta =
        (0.08 * action.pitch_adjust_scale -
         0.05 * action.rear_drive_bias_scale) * forward_scale;
    const double rear_thigh_delta =
        (0.07 * action.pitch_adjust_scale -
         0.08 * action.rear_drive_bias_scale) * forward_scale;
    const double rear_calf_delta =
        (-0.12 * action.pitch_adjust_scale +
         0.14 * action.rear_drive_bias_scale) * forward_scale;
    for (std::size_t thigh_idx : {1u, 4u}) {
      apply_leg_delta(thigh_idx, thigh_idx + 1, front_thigh_delta, front_calf_delta);
    }
    for (std::size_t thigh_idx : {7u, 10u}) {
      apply_leg_delta(thigh_idx, thigh_idx + 1, rear_thigh_delta, rear_calf_delta);
    }
  } else if (sample.phase == go2_jump_core::JumpPhase::kPush) {
    const double thigh_delta =
        -0.16 * action.push_extension_scale - 0.12 * action.lift_drive_scale;
    const double calf_delta =
        0.18 * action.push_extension_scale + 0.12 * action.lift_drive_scale;
    for (std::size_t thigh_idx : {1u, 4u, 7u, 10u}) {
      const std::size_t calf_idx = thigh_idx + 1;
      apply_leg_delta(thigh_idx, calf_idx, thigh_delta, calf_delta);
    }
    const double front_thigh_bias =
        (0.10 * action.pitch_adjust_scale +
         0.06 * action.forward_drive_scale) * forward_scale;
    const double front_calf_bias =
        (-0.16 * action.pitch_adjust_scale -
         0.09 * action.forward_drive_scale) * forward_scale;
    const double rear_thigh_bias =
        (-0.18 * action.pitch_adjust_scale -
         0.14 * (action.forward_drive_scale + action.rear_drive_bias_scale)) *
        forward_scale;
    const double rear_calf_bias =
        (0.12 * action.pitch_adjust_scale +
         0.08 * (action.forward_drive_scale + action.rear_drive_bias_scale)) *
        forward_scale;
    for (std::size_t thigh_idx : {1u, 4u}) {
      apply_leg_delta(thigh_idx, thigh_idx + 1, front_thigh_bias, front_calf_bias);
    }
    for (std::size_t thigh_idx : {7u, 10u}) {
      apply_leg_delta(thigh_idx, thigh_idx + 1, rear_thigh_bias, rear_calf_bias);
    }
  } else if (sample.phase == go2_jump_core::JumpPhase::kFlight) {
    const double thigh_delta = 0.14 * action.flight_tuck_scale;
    const double calf_delta = -0.22 * action.flight_tuck_scale;
    for (std::size_t thigh_idx : {1u, 4u, 7u, 10u}) {
      const std::size_t calf_idx = thigh_idx + 1;
      apply_leg_delta(thigh_idx, calf_idx, thigh_delta, calf_delta);
    }
    const double front_thigh_bias =
        -0.04 * action.pitch_adjust_scale * forward_scale;
    const double front_calf_bias =
        0.06 * action.pitch_adjust_scale * forward_scale;
    const double rear_thigh_bias =
        0.04 * action.pitch_adjust_scale * forward_scale;
    const double rear_calf_bias =
        -0.06 * action.pitch_adjust_scale * forward_scale;
    for (std::size_t thigh_idx : {1u, 4u}) {
      apply_leg_delta(thigh_idx, thigh_idx + 1, front_thigh_bias, front_calf_bias);
    }
    for (std::size_t thigh_idx : {7u, 10u}) {
      apply_leg_delta(thigh_idx, thigh_idx + 1, rear_thigh_bias, rear_calf_bias);
    }
  } else if (sample.phase == go2_jump_core::JumpPhase::kLanding ||
             sample.phase == go2_jump_core::JumpPhase::kSettle) {
    const double thigh_delta = 0.10 * action.landing_brace_scale;
    const double calf_delta = -0.18 * action.landing_brace_scale;
    for (std::size_t thigh_idx : {1u, 4u, 7u, 10u}) {
      const std::size_t calf_idx = thigh_idx + 1;
      apply_leg_delta(thigh_idx, calf_idx, thigh_delta, calf_delta);
    }
  }

  return pose;
}

JointArray ApplyCandidateToFeedforward(const JointArray& nominal_tau,
                                       const go2_jump_core::JumpReferenceSample& sample,
                                       const CandidateAction& action,
                                       double max_feedforward_torque_nm) {
  JointArray tau = nominal_tau;
  const double forward_scale = Clamp(
      std::abs(sample.desired_forward_velocity_mps) / 1.2, 0.5, 1.8);

  if (sample.phase == go2_jump_core::JumpPhase::kPush) {
    const double thigh_delta =
        Clamp(3.2 * action.push_extension_scale + 2.2 * action.lift_drive_scale,
              -max_feedforward_torque_nm,
              max_feedforward_torque_nm);
    const double calf_delta =
        Clamp(2.4 * action.push_extension_scale + 1.8 * action.lift_drive_scale,
              -max_feedforward_torque_nm,
              max_feedforward_torque_nm);
    for (std::size_t idx : {1u, 4u, 7u, 10u}) {
      tau[idx] = Clamp(tau[idx] + thigh_delta, -max_feedforward_torque_nm,
                       max_feedforward_torque_nm);
    }
    for (std::size_t idx : {2u, 5u, 8u, 11u}) {
      tau[idx] = Clamp(tau[idx] + calf_delta, -max_feedforward_torque_nm,
                       max_feedforward_torque_nm);
    }
    const double front_thigh_bias =
        Clamp((-1.6 * action.pitch_adjust_scale -
               1.2 * action.forward_drive_scale) * forward_scale,
              -max_feedforward_torque_nm, max_feedforward_torque_nm);
    const double front_calf_bias =
        Clamp((-1.6 * action.pitch_adjust_scale -
               1.0 * action.forward_drive_scale) * forward_scale,
              -max_feedforward_torque_nm, max_feedforward_torque_nm);
    const double rear_thigh_bias =
        Clamp((3.0 * action.pitch_adjust_scale +
               2.6 * (action.forward_drive_scale + action.rear_drive_bias_scale)) *
                  forward_scale,
              -max_feedforward_torque_nm, max_feedforward_torque_nm);
    const double rear_calf_bias =
        Clamp((2.2 * action.pitch_adjust_scale +
               1.6 * (action.forward_drive_scale + action.rear_drive_bias_scale)) *
                  forward_scale,
              -max_feedforward_torque_nm, max_feedforward_torque_nm);
    for (std::size_t idx : {1u, 4u}) {
      tau[idx] = Clamp(tau[idx] + front_thigh_bias, -max_feedforward_torque_nm,
                       max_feedforward_torque_nm);
    }
    for (std::size_t idx : {2u, 5u}) {
      tau[idx] = Clamp(tau[idx] + front_calf_bias, -max_feedforward_torque_nm,
                       max_feedforward_torque_nm);
    }
    for (std::size_t idx : {7u, 10u}) {
      tau[idx] = Clamp(tau[idx] + rear_thigh_bias, -max_feedforward_torque_nm,
                       max_feedforward_torque_nm);
    }
    for (std::size_t idx : {8u, 11u}) {
      tau[idx] = Clamp(tau[idx] + rear_calf_bias, -max_feedforward_torque_nm,
                       max_feedforward_torque_nm);
    }
  } else if (sample.phase == go2_jump_core::JumpPhase::kLanding) {
    const double brace_delta =
        Clamp(-2.0 * action.landing_brace_scale, -max_feedforward_torque_nm,
              max_feedforward_torque_nm);
    for (std::size_t idx : {1u, 4u, 7u, 10u, 2u, 5u, 8u, 11u}) {
      tau[idx] = Clamp(tau[idx] + brace_delta, -max_feedforward_torque_nm,
                       max_feedforward_torque_nm);
    }
  }

  return tau;
}

}  // namespace

struct WholeBodyMpc::MujocoBackend {
  explicit MujocoBackend(const WholeBodyMpcConfig& config) {
    const auto model_path_candidate = ResolveMujocoModelPath(config.mujoco_model_path);
    if (model_path_candidate.empty()) {
      error_message =
          "Unable to locate Go2 MuJoCo scene.xml for the MuJoCo-native backend.";
      return;
    }

    model_path = model_path_candidate.string();
    char error_buffer[1024] = {};
    model = mj_loadXML(model_path.c_str(), nullptr, error_buffer,
                       sizeof(error_buffer));
    if (!model) {
      error_message = error_buffer;
      return;
    }

    data = mj_makeData(model);
    if (!data) {
      error_message = "mj_makeData failed for the MuJoCo-native backend.";
      return;
    }

    home_key_id = mj_name2id(model, mjOBJ_KEY, "home");
    base_body_id = mj_name2id(model, mjOBJ_BODY, "base_link");
    imu_site_id = mj_name2id(model, mjOBJ_SITE, "imu");

    for (std::size_t i = 0; i < kJointNames.size(); ++i) {
      const int joint_id = mj_name2id(model, mjOBJ_JOINT, kJointNames[i]);
      if (joint_id < 0) {
        error_message = std::string("Missing joint in MuJoCo model: ") + kJointNames[i];
        return;
      }
      joint_qposadr[i] = model->jnt_qposadr[joint_id];
      joint_dofadr[i] = model->jnt_dofadr[joint_id];

      const int actuator_id = mj_name2id(model, mjOBJ_ACTUATOR, kActuatorNames[i]);
      if (actuator_id < 0) {
        error_message =
            std::string("Missing actuator in MuJoCo model: ") + kActuatorNames[i];
        return;
      }
      actuator_ids[i] = actuator_id;
      actuator_limits[i] = model->actuator_ctrllimited[actuator_id]
                               ? model->actuator_ctrlrange[2 * actuator_id + 1]
                               : 45.0;
    }

    for (std::size_t i = 0; i < kFootGeomNames.size(); ++i) {
      foot_geom_ids[i] = mj_name2id(model, mjOBJ_GEOM, kFootGeomNames[i]);
      foot_body_ids[i] = mj_name2id(model, mjOBJ_BODY, kFootBodyNames[i]);
      if (foot_geom_ids[i] >= 0) {
        foot_geom_radius[i] = model->geom_size[3 * foot_geom_ids[i]];
      }
    }

    if (home_key_id >= 0) {
      mj_resetDataKeyframe(model, data, home_key_id);
    } else {
      mj_resetData(model, data);
    }
    mj_forward(model, data);

    for (int body_index = 0; body_index < model->nbody; ++body_index) {
      total_mass += model->body_mass[body_index];
    }

    ready = true;
  }

  ~MujocoBackend() {
    if (data) {
      mj_deleteData(data);
      data = nullptr;
    }
    if (model) {
      mj_deleteModel(model);
      model = nullptr;
    }
  }

  void SyncObservation(const RobotObservation& observation) {
    if (!ready || !model || !data) {
      return;
    }

    if (home_key_id >= 0) {
      mj_resetDataKeyframe(model, data, home_key_id);
    } else {
      mj_resetData(model, data);
    }

    std::fill(data->ctrl, data->ctrl + model->nu, 0.0);
    if (model->nq >= 7) {
      data->qpos[0] = 0.0;
      data->qpos[1] = 0.0;
      data->qpos[2] = 0.0;
      const auto quat = QuaternionFromRpy(observation.body_rpy);
      data->qpos[3] = quat[0];
      data->qpos[4] = quat[1];
      data->qpos[5] = quat[2];
      data->qpos[6] = quat[3];
    }

    for (std::size_t i = 0; i < joint_qposadr.size(); ++i) {
      data->qpos[joint_qposadr[i]] = observation.q[i];
    }

    mj_fwdPosition(model, data);

    if (observation.sportstate_received && imu_site_id >= 0) {
      data->qpos[0] = observation.position[0] - data->site_xpos[3 * imu_site_id + 0];
      data->qpos[1] = observation.position[1] - data->site_xpos[3 * imu_site_id + 1];
      data->qpos[2] = observation.position[2] - data->site_xpos[3 * imu_site_id + 2];
    } else {
      double required_z_shift = 0.0;
      for (std::size_t i = 0; i < foot_geom_ids.size(); ++i) {
        const int geom_id = foot_geom_ids[i];
        if (geom_id < 0) {
          continue;
        }
        const double geom_z = data->geom_xpos[3 * geom_id + 2];
        required_z_shift =
            std::max(required_z_shift, foot_geom_radius[i] - geom_z);
      }
      data->qpos[2] += required_z_shift;
    }

    std::fill(data->qvel, data->qvel + model->nv, 0.0);
    if (model->nv >= 6) {
      data->qvel[0] = observation.body_velocity[0];
      data->qvel[1] = observation.body_velocity[1];
      data->qvel[2] = observation.body_velocity[2];
      data->qvel[3] = observation.body_angular_velocity[0];
      data->qvel[4] = observation.body_angular_velocity[1];
      data->qvel[5] = observation.body_angular_velocity[2];
    }
    for (std::size_t i = 0; i < joint_dofadr.size(); ++i) {
      data->qvel[joint_dofadr[i]] = observation.dq[i];
    }

    mj_forward(model, data);
  }

  std::array<double, kFootCount> FootContactForces() const {
    std::array<double, kFootCount> forces{};
    if (!ready || !model || !data || data->ncon <= 0) {
      return forces;
    }

    std::array<double, 3> world_up{0.0, 0.0, 1.0};
    const double gravity_norm = std::sqrt(
        Square(model->opt.gravity[0]) + Square(model->opt.gravity[1]) +
        Square(model->opt.gravity[2]));
    if (gravity_norm > 1e-6) {
      world_up = {
          -model->opt.gravity[0] / gravity_norm,
          -model->opt.gravity[1] / gravity_norm,
          -model->opt.gravity[2] / gravity_norm,
      };
    }

    for (int contact_index = 0; contact_index < data->ncon; ++contact_index) {
      const auto& contact = data->contact[contact_index];
      double contact_wrench[6]{};
      mj_contactForce(model, data, contact_index, contact_wrench);
      const double contact_force_world[3]{
          contact.frame[0] * contact_wrench[0] +
              contact.frame[3] * contact_wrench[1] +
              contact.frame[6] * contact_wrench[2],
          contact.frame[1] * contact_wrench[0] +
              contact.frame[4] * contact_wrench[1] +
              contact.frame[7] * contact_wrench[2],
          contact.frame[2] * contact_wrench[0] +
              contact.frame[5] * contact_wrench[1] +
              contact.frame[8] * contact_wrench[2],
      };
      const double support_load_n = std::max(
          std::abs(contact_wrench[0]),
          std::abs(contact_force_world[0] * world_up[0] +
                   contact_force_world[1] * world_up[1] +
                   contact_force_world[2] * world_up[2]));
      if (support_load_n <= 0.0) {
        continue;
      }

      for (std::size_t foot_index = 0; foot_index < foot_geom_ids.size();
           ++foot_index) {
        const int foot_geom_id = foot_geom_ids[foot_index];
        if (foot_geom_id < 0) {
          continue;
        }
        if (contact.geom1 == foot_geom_id || contact.geom2 == foot_geom_id) {
          forces[foot_index] += support_load_n;
        }
      }
    }

    return forces;
  }

  std::array<bool, kFootCount> FootContacts(double threshold) const {
    std::array<bool, kFootCount> contacts{};
    const auto forces = FootContactForces();
    for (std::size_t i = 0; i < contacts.size(); ++i) {
      contacts[i] = forces[i] >= threshold;
    }
    return contacts;
  }

  std::array<double, 3> FootBodyPositionRelativeToBase(
      std::size_t foot_index) const {
    std::array<double, 3> position{};
    if (!ready || !model || !data || base_body_id < 0 ||
        foot_index >= foot_body_ids.size()) {
      return position;
    }
    const int foot_body_id = foot_body_ids[foot_index];
    if (foot_body_id < 0) {
      return position;
    }
    position[0] = data->xpos[3 * foot_body_id + 0] - data->xpos[3 * base_body_id + 0];
    position[1] = data->xpos[3 * foot_body_id + 1] - data->xpos[3 * base_body_id + 1];
    position[2] = data->xpos[3 * foot_body_id + 2] - data->xpos[3 * base_body_id + 2];
    return position;
  }

  JointArray FootForceToJointTorques(
      const std::array<std::array<double, 3>, kFootCount>& foot_forces_world) const {
    JointArray tau{};
    if (!ready || !model || !data) {
      return tau;
    }

    std::vector<mjtNum> jacp(3 * model->nv, 0.0);
    for (std::size_t foot_index = 0; foot_index < foot_body_ids.size(); ++foot_index) {
      const int foot_body_id = foot_body_ids[foot_index];
      if (foot_body_id < 0) {
        continue;
      }
      const auto& force = foot_forces_world[foot_index];
      if (std::abs(force[0]) < 1e-6 && std::abs(force[1]) < 1e-6 &&
          std::abs(force[2]) < 1e-6) {
        continue;
      }
      std::fill(jacp.begin(), jacp.end(), 0.0);
      mj_jacBodyCom(model, data, jacp.data(), nullptr, foot_body_id);
      for (std::size_t joint_index = 0; joint_index < kControlledJointCount;
           ++joint_index) {
        const int dof = joint_dofadr[joint_index];
        tau[joint_index] += jacp[dof] * force[0] +
                            jacp[model->nv + dof] * force[1] +
                            jacp[2 * model->nv + dof] * force[2];
      }
    }
    return tau;
  }

  std::array<double, 3> BaseRpy() const {
    if (!ready || !model || !data || base_body_id < 0) {
      return {};
    }
    return RpyFromQuaternion(&data->xquat[4 * base_body_id]);
  }

  mjModel* model{nullptr};
  mjData* data{nullptr};
  bool ready{false};
  std::string error_message;
  std::string model_path;

  int home_key_id{-1};
  int base_body_id{-1};
  int imu_site_id{-1};
  std::array<int, kControlledJointCount> joint_qposadr{};
  std::array<int, kControlledJointCount> joint_dofadr{};
  std::array<int, kControlledJointCount> actuator_ids{};
  std::array<double, kControlledJointCount> actuator_limits{};
  std::array<int, kFootCount> foot_geom_ids{};
  std::array<int, kFootCount> foot_body_ids{};
  std::array<double, kFootCount> foot_geom_radius{};
  double total_mass{0.0};
};

WholeBodyMpc::WholeBodyMpc(WholeBodyMpcConfig config)
    : config_(std::move(config)) {
  if (IsMujocoNativeBackend(config_.solver_backend)) {
    mujoco_backend_ = std::make_unique<MujocoBackend>(config_);
  }
}

WholeBodyMpc::~WholeBodyMpc() = default;

void WholeBodyMpc::SetTask(const go2_jump_core::JumpTaskSpec& task) {
  goal_task_ = task;
  task_ = task;
  have_task_ = true;
  have_intent_ = false;
  intent_ = {};
  execution_phase_state_ = {};
}

void WholeBodyMpc::SetIntent(const go2_jump_core::JumpKinodynamicIntent& intent) {
  intent_ = intent;
  have_intent_ = intent.valid;
  if (have_task_) {
    task_ = have_intent_ ? go2_jump_core::ApplyJumpKinodynamicIntent(goal_task_, intent_)
                         : goal_task_;
  }
  execution_phase_state_ = {};
}

bool WholeBodyMpc::HasTask() const { return have_task_; }

go2_jump_core::JumpReferenceProfile WholeBodyMpc::BuildActiveReferenceProfile() const {
  if (have_intent_) {
    return go2_jump_core::BuildJumpReferenceProfile(task_, config_.reference_config,
                                                    intent_);
  }
  return go2_jump_core::BuildJumpReferenceProfile(task_, config_.reference_config);
}

go2_jump_core::JumpReferenceSample WholeBodyMpc::SampleReference(
    double elapsed_s) const {
  if (have_intent_) {
    return go2_jump_core::SampleJumpReference(task_, config_.reference_config,
                                              elapsed_s, intent_);
  }
  return go2_jump_core::SampleJumpReference(task_, config_.reference_config,
                                            elapsed_s);
}

PhaseEventState WholeBodyMpc::BuildExecutionPhaseEventState(
    const RobotObservation& observation, double task_elapsed_s) {
  const int contact_count = CountContacts(observation.foot_contact);
  UpdatePhaseEventState(execution_phase_state_, contact_count,
                        observation.contact_signal_valid,
                        observation.body_velocity[2], task_elapsed_s);
  return execution_phase_state_;
}

void WholeBodyMpc::UpdatePhaseEventState(PhaseEventState& state, int contact_count,
                                         bool contact_signal_valid,
                                         double body_vertical_velocity_mps,
                                         double task_elapsed_s) const {
  state.contact_signal_valid = state.contact_signal_valid || contact_signal_valid;
  if (!state.contact_signal_valid) {
    return;
  }

  const double takeoff_gate_time_s =
      task_.crouch_duration_s + 0.30 * task_.push_duration_s;
  const double strong_takeoff_gate_time_s =
      task_.crouch_duration_s + 0.55 * task_.push_duration_s;
  const bool takeoff_candidate =
      task_elapsed_s >= takeoff_gate_time_s &&
      contact_count <= config_.flight_contact_count_max;
  const bool strong_takeoff_candidate =
      task_elapsed_s >= strong_takeoff_gate_time_s &&
      contact_count <= config_.strong_takeoff_contact_count_max &&
      body_vertical_velocity_mps >= config_.strong_takeoff_vertical_velocity_mps;
  if (!state.takeoff_latched) {
    if (takeoff_candidate || strong_takeoff_candidate) {
      if (state.takeoff_candidate_start_s < 0.0) {
        state.takeoff_candidate_start_s = task_elapsed_s;
      }
      if (task_elapsed_s - state.takeoff_candidate_start_s >=
              config_.takeoff_latch_dwell_s ||
          strong_takeoff_candidate) {
        state.takeoff_latched = true;
        state.takeoff_time_s =
            strong_takeoff_candidate ? task_elapsed_s
                                     : state.takeoff_candidate_start_s;
      }
    } else {
      state.takeoff_candidate_start_s = -1.0;
    }
  }
  if (state.takeoff_latched && state.takeoff_time_s >= 0.0) {
    state.takeoff_candidate_start_s = state.takeoff_time_s;
  }

  if (state.takeoff_latched && !state.touchdown_latched) {
    const double flight_elapsed_s = task_elapsed_s - state.takeoff_time_s;
    const bool touchdown_candidate =
        flight_elapsed_s >= config_.min_flight_time_before_touchdown_s &&
        contact_count >= config_.touchdown_contact_count_threshold &&
        (body_vertical_velocity_mps <= 0.35 || contact_count == 4);
    const bool strong_touchdown_candidate =
        flight_elapsed_s >= 0.5 * config_.min_flight_time_before_touchdown_s &&
        contact_count >= config_.touchdown_contact_count_threshold &&
        body_vertical_velocity_mps <=
            config_.strong_touchdown_vertical_velocity_mps;
    if (touchdown_candidate) {
      if (state.touchdown_candidate_start_s < 0.0) {
        state.touchdown_candidate_start_s = task_elapsed_s;
      }
      if (task_elapsed_s - state.touchdown_candidate_start_s >=
              config_.touchdown_latch_dwell_s ||
          strong_touchdown_candidate) {
        state.touchdown_latched = true;
        state.touchdown_time_s = state.touchdown_candidate_start_s;
      }
    } else {
      state.touchdown_candidate_start_s = -1.0;
    }
  }
  if (state.touchdown_latched && state.touchdown_time_s >= 0.0) {
    state.touchdown_candidate_start_s = state.touchdown_time_s;
  }

  if (state.touchdown_latched && !state.settle_latched) {
    const double touchdown_elapsed_s = task_elapsed_s - state.touchdown_time_s;
    const bool settle_candidate =
        contact_count == 4 &&
        body_vertical_velocity_mps <=
            std::min(0.08, config_.settle_vertical_velocity_threshold_mps) &&
        std::abs(body_vertical_velocity_mps) <=
            0.75 * config_.settle_vertical_velocity_threshold_mps &&
        touchdown_elapsed_s >= 0.85 * task_.landing_duration_s;
    if (settle_candidate) {
      if (state.settle_candidate_start_s < 0.0) {
        state.settle_candidate_start_s = task_elapsed_s;
      }
      if (task_elapsed_s - state.settle_candidate_start_s >=
          config_.settle_latch_dwell_s) {
        state.settle_latched = true;
        state.settle_time_s = state.settle_candidate_start_s;
      }
    } else {
      state.settle_candidate_start_s = -1.0;
    }
  }
  if (state.settle_latched && state.settle_time_s >= 0.0) {
    state.settle_candidate_start_s = state.settle_time_s;
  }
}

WholeBodyMpcCommand WholeBodyMpc::Solve(const RobotObservation& observation,
                                        double task_elapsed_s) {
  if (config_.solver_backend == "reference_preview") {
    return SolveReferencePreview(observation, task_elapsed_s);
  }
  if (IsMujocoNativeBackend(config_.solver_backend)) {
    return SolveMujocoSampling(observation, task_elapsed_s);
  }

  auto fallback = SolveReferencePreview(observation, task_elapsed_s);
  fallback.backend_name = config_.solver_backend;
  fallback.backend_ready = false;
  return fallback;
}

WholeBodyMpcCommand WholeBodyMpc::SolveReferencePreview(
    const RobotObservation& observation, double task_elapsed_s) {
  WholeBodyMpcCommand command{};
  if (!have_task_ || !observation.lowstate_received) {
    return command;
  }

  command.valid = true;
  command.backend_name = "reference_preview";
  command.backend_ready = true;
  command.lowcmd_enabled = config_.enable_lowcmd_output;
  command.contact_signal_valid = observation.contact_signal_valid;
  command.foot_contact = observation.foot_contact;
  command.contact_count = CountContacts(observation.foot_contact);

  const auto phase_state =
      BuildExecutionPhaseEventState(observation, task_elapsed_s);
  const auto planned_sample = SampleReference(task_elapsed_s);
  const auto current_sample =
      ApplyContactOverrides(planned_sample, phase_state, command.contact_count,
                            observation.body_velocity[2], task_elapsed_s);
  command.contact_override = (current_sample.phase != planned_sample.phase);
  command.phase = current_sample.phase;
  command.desired_forward_velocity_mps =
      current_sample.desired_forward_velocity_mps;
  command.desired_vertical_velocity_mps =
      current_sample.desired_vertical_velocity_mps;
  command.desired_body_pitch_deg = current_sample.desired_body_pitch_deg;
  command.desired_body_height_offset_m =
      current_sample.desired_body_height_offset_m;
  command.q_ref = BuildPoseForSample(current_sample);
  command.dq_ref.fill(0.0);
  command.tau_ff = BuildFeedforwardForSample(task_, current_sample);
  command.uniform_kp = GainsForPhase(current_sample.phase, false);
  command.uniform_kd = GainsForPhase(current_sample.phase, true);

  command.preview.reserve(std::max(config_.horizon_steps, 1));
  for (int step = 0; step < std::max(config_.horizon_steps, 1); ++step) {
    const double time_from_now = step * config_.control_dt_s;
    const auto sample = SampleReference(task_elapsed_s + time_from_now);
    command.preview.push_back(PreviewStep{
        time_from_now,
        sample.phase,
        sample.desired_forward_velocity_mps,
        sample.desired_vertical_velocity_mps,
        sample.desired_body_pitch_deg,
        sample.desired_body_height_offset_m,
        sample.landing_brace_factor,
    });
  }

  return command;
}

WholeBodyMpcCommand WholeBodyMpc::SolveMujocoSampling(
    const RobotObservation& observation, double task_elapsed_s) {
  auto command = SolveReferencePreview(observation, task_elapsed_s);
  if (!command.valid) {
    return command;
  }

  command.backend_name = "mujoco_native_mpc";
  command.backend_ready = mujoco_backend_ && mujoco_backend_->ready;
  if (!command.backend_ready) {
    return command;
  }

  auto& backend = *mujoco_backend_;
  backend.SyncObservation(observation);

  std::vector<mjtNum> qpos_start(backend.model->nq, 0.0);
  std::vector<mjtNum> qvel_start(backend.model->nv, 0.0);
  std::memcpy(qpos_start.data(), backend.data->qpos,
              sizeof(mjtNum) * backend.model->nq);
  std::memcpy(qvel_start.data(), backend.data->qvel,
              sizeof(mjtNum) * backend.model->nv);

  const int rollout_steps =
      std::max(1, std::min(config_.mujoco_rollout_steps, config_.horizon_steps));
  const int substeps = std::max(1, config_.mujoco_rollout_substeps);
  const double rollout_dt = substeps * backend.model->opt.timestep;
  const double gravity_mps2 = std::max(
      std::abs(backend.model->opt.gravity[2]), config_.reference_config.gravity_mps2);
  const auto reference_profile = BuildActiveReferenceProfile();
  const double distance_alpha = JumpDistanceAlpha(task_);
  const double horizontal_priority = 1.0 + 0.5 * distance_alpha;
  const double vertical_priority = 1.0 - 0.10 * distance_alpha;
  const double push_end = task_.crouch_duration_s + task_.push_duration_s;
  const double x_start = backend.data->qpos[0];
  const double gravity_magnitude_mps2 =
      std::max(std::abs(backend.model->opt.gravity[2]), 1e-6);
  const auto add_torque_arrays =
      [&](const JointArray& base, const JointArray& delta, double limit_nm) {
        JointArray out = base;
        for (std::size_t joint_index = 0; joint_index < out.size(); ++joint_index) {
          out[joint_index] = Clamp(out[joint_index] + delta[joint_index],
                                   -limit_nm, limit_nm);
        }
        return out;
      };

  const auto build_push_wrench_torques =
      [&](const go2_jump_core::JumpReferenceSample& sample,
          const std::array<bool, kFootCount>& foot_contact,
          const std::array<double, kFootCount>& support_loads) {
        JointArray tau{};
        if (!config_.enable_push_wrench_control ||
            sample.phase != go2_jump_core::JumpPhase::kPush ||
            backend.total_mass <= 1e-6) {
          return tau;
        }

        const int front_contact_count =
            static_cast<int>(foot_contact[0]) + static_cast<int>(foot_contact[1]);
        const int rear_contact_count =
            static_cast<int>(foot_contact[2]) + static_cast<int>(foot_contact[3]);
        const int contact_count = front_contact_count + rear_contact_count;
        if (contact_count <= 0) {
          return tau;
        }

        const double push_progress = Clamp(
            sample.time_in_phase_s / std::max(task_.push_duration_s, 1e-6), 0.0, 1.0);
        double assist_scale =
            config_.push_wrench_assist_gain *
            Lerp(0.35, 1.0, Smooth01(Clamp((push_progress - 0.08) / 0.55, 0.0, 1.0)));
        if (front_contact_count == 0 && rear_contact_count > 0 &&
            push_progress > 0.65) {
          assist_scale *=
              Lerp(1.0, 0.18, Smooth01(Clamp((push_progress - 0.65) / 0.20, 0.0, 1.0)));
        }

        const double remaining_time_s =
            std::max(0.03, task_.push_duration_s - sample.time_in_phase_s);
        const double desired_vx = std::max(
            sample.desired_forward_velocity_mps,
            Lerp(0.55, 0.95, push_progress) * task_.target_takeoff_velocity_x_mps);
        const double desired_vz = std::max(
            sample.desired_vertical_velocity_mps,
            Lerp(0.55, 0.95, push_progress) * task_.target_takeoff_velocity_z_mps);
        const double ax_cmd = Clamp(
            (desired_vx - backend.data->qvel[0]) / remaining_time_s, -1.5,
            4.0 + 1.0 * std::min(distance_alpha, 1.0));
        const double az_cmd =
            Clamp((desired_vz - backend.data->qvel[2]) / remaining_time_s, -2.0, 4.8);

        double total_fx_n = assist_scale * backend.total_mass * ax_cmd;
        double total_fz_n = assist_scale * backend.total_mass *
                            std::max(0.0, az_cmd) *
                            config_.push_wrench_vertical_gain;
        const double max_total_delta_force_n =
            std::max(config_.push_wrench_max_delta_force_n, 1.0) *
            (front_contact_count > 0 && rear_contact_count > 0 ? 2.0 : 1.0);
        total_fx_n = Clamp(total_fx_n, -max_total_delta_force_n, max_total_delta_force_n);
        total_fz_n = Clamp(total_fz_n, 0.0, max_total_delta_force_n);

        double front_x_m = 0.0;
        double front_z_m = 0.0;
        double rear_x_m = 0.0;
        double rear_z_m = 0.0;
        if (front_contact_count > 0) {
          for (std::size_t foot_index : {0u, 1u}) {
            if (!foot_contact[foot_index]) {
              continue;
            }
            const auto foot_position = backend.FootBodyPositionRelativeToBase(foot_index);
            front_x_m += foot_position[0];
            front_z_m += foot_position[2];
          }
          front_x_m /= static_cast<double>(front_contact_count);
          front_z_m /= static_cast<double>(front_contact_count);
        }
        if (rear_contact_count > 0) {
          for (std::size_t foot_index : {2u, 3u}) {
            if (!foot_contact[foot_index]) {
              continue;
            }
            const auto foot_position = backend.FootBodyPositionRelativeToBase(foot_index);
            rear_x_m += foot_position[0];
            rear_z_m += foot_position[2];
          }
          rear_x_m /= static_cast<double>(rear_contact_count);
          rear_z_m /= static_cast<double>(rear_contact_count);
        }

        const double front_support_n = support_loads[0] + support_loads[1];
        const double rear_support_n = support_loads[2] + support_loads[3];
        const auto base_rpy = backend.BaseRpy();
        const double desired_pitch_rad = sample.desired_body_pitch_deg * kDegToRad;
        const double pitch_error_rad = desired_pitch_rad - base_rpy[1];
        const double pitch_rate_radps =
            backend.model->nv > 4 ? backend.data->qvel[4] : 0.0;
        const double max_pitch_moment_nm =
            0.20 * backend.total_mass * gravity_magnitude_mps2;
        const double pitch_moment_nm = Clamp(
            config_.push_wrench_pitch_kp * pitch_error_rad -
                config_.push_wrench_pitch_kd * pitch_rate_radps,
            -max_pitch_moment_nm, max_pitch_moment_nm);

        double front_fx_pref_n = 0.0;
        double front_fz_pref_n = 0.0;
        if (front_contact_count > 0 && rear_contact_count > 0) {
          front_fx_pref_n = total_fx_n * Lerp(0.34, 0.18, push_progress);
          front_fz_pref_n = total_fz_n * Lerp(0.48, 0.28, push_progress);
        } else if (front_contact_count > 0) {
          front_fx_pref_n = total_fx_n;
          front_fz_pref_n = total_fz_n;
        }

        const auto force_plan = SolveFrontRearForcePlan(
            total_fx_n, total_fz_n, pitch_moment_nm, front_contact_count > 0,
            front_x_m, front_z_m, front_support_n, rear_contact_count > 0, rear_x_m,
            rear_z_m, rear_support_n, front_fx_pref_n, front_fz_pref_n,
            config_.push_wrench_friction_coeff,
            config_.push_wrench_max_delta_force_n);
        if (!force_plan.valid) {
          return tau;
        }

        std::array<std::array<double, 3>, kFootCount> foot_forces_world{};
        for (std::size_t foot_index : {0u, 1u}) {
          if (!foot_contact[foot_index] || front_contact_count <= 0) {
            continue;
          }
          foot_forces_world[foot_index][0] =
              force_plan.front_fx_n / static_cast<double>(front_contact_count);
          foot_forces_world[foot_index][2] =
              force_plan.front_fz_n / static_cast<double>(front_contact_count);
        }
        for (std::size_t foot_index : {2u, 3u}) {
          if (!foot_contact[foot_index] || rear_contact_count <= 0) {
            continue;
          }
          foot_forces_world[foot_index][0] =
              force_plan.rear_fx_n / static_cast<double>(rear_contact_count);
          foot_forces_world[foot_index][2] =
              force_plan.rear_fz_n / static_cast<double>(rear_contact_count);
        }

        tau = backend.FootForceToJointTorques(foot_forces_world);
        const double assist_limit_nm = 0.60 * config_.max_feedforward_torque_nm;
        for (std::size_t joint_index = 0; joint_index < tau.size(); ++joint_index) {
          if (joint_index == 0 || joint_index == 3 || joint_index == 6 ||
              joint_index == 9) {
            tau[joint_index] = 0.0;
          } else {
            tau[joint_index] = Clamp(tau[joint_index], -assist_limit_nm, assist_limit_nm);
          }
        }
        return tau;
      };

  auto rollout_phase_state = execution_phase_state_;
  const bool use_taskspace_reference = UseTaskSpaceReferenceMode(config_);
  const auto seed_contact = backend.FootContacts(config_.contact_release_threshold_n);
  UpdatePhaseEventState(rollout_phase_state, CountContacts(seed_contact),
                        rollout_phase_state.contact_signal_valid,
                        backend.data->qvel[2], task_elapsed_s);

  RolloutResult best_result;
  const auto seed_sample = ApplyContactOverrides(
      SampleReference(task_elapsed_s), rollout_phase_state,
      CountContacts(seed_contact), backend.data->qvel[2], task_elapsed_s);

  for (const auto& candidate : CandidateActionsForPhase(seed_sample.phase)) {
    std::memcpy(backend.data->qpos, qpos_start.data(),
                sizeof(mjtNum) * backend.model->nq);
    std::memcpy(backend.data->qvel, qvel_start.data(),
                sizeof(mjtNum) * backend.model->nv);
    std::fill(backend.data->ctrl, backend.data->ctrl + backend.model->nu, 0.0);
    mj_forward(backend.model, backend.data);

    RolloutResult rollout{};
    double score = 0.0;
    double target_dx = 0.0;
    double accumulated_control_energy = 0.0;
    auto candidate_phase_state = rollout_phase_state;
    bool takeoff_recorded = false;
    bool touchdown_recorded = false;
    double takeoff_x = 0.0;
    double takeoff_vx = 0.0;
    double takeoff_vz = 0.0;
    double takeoff_pitch_deg = 0.0;
    double predicted_ballistic_distance_m = 0.0;
    double touchdown_x = 0.0;

    for (int step = 0; step < rollout_steps; ++step) {
      const double current_time = task_elapsed_s + step * rollout_dt;
      const auto planned_sample = SampleReference(current_time);
      const auto predicted_contact =
          backend.FootContacts(config_.contact_release_threshold_n);
      const int predicted_contact_count = CountContacts(predicted_contact);
      const auto contact_adjusted_sample = ApplyContactOverrides(
          planned_sample, candidate_phase_state, predicted_contact_count,
          backend.data->qvel[2], current_time);
      const auto sampled_reference =
          ApplyCandidateToSample(contact_adjusted_sample, candidate);
      const auto nominal_q_ref = use_taskspace_reference
                                     ? BuildTaskSpacePoseForSample(sampled_reference)
                                     : BuildPoseForSample(sampled_reference);
      const auto q_ref = use_taskspace_reference
                             ? nominal_q_ref
                             : ApplyCandidateToPose(nominal_q_ref,
                                                    sampled_reference, candidate);
      const auto nominal_tau_ff =
          BuildFeedforwardForSample(task_, sampled_reference);
      auto tau_ff = ApplyCandidateToFeedforward(
          nominal_tau_ff, sampled_reference, candidate,
          config_.max_feedforward_torque_nm);
      const auto support_loads = backend.FootContactForces();
      const auto push_wrench_tau = build_push_wrench_torques(
          sampled_reference, predicted_contact, support_loads);
      tau_ff = add_torque_arrays(tau_ff, push_wrench_tau,
                                 config_.max_feedforward_torque_nm);
      const double kp = GainsForPhase(sampled_reference.phase, false);
      const double kd = GainsForPhase(sampled_reference.phase, true);

      if (step == 0) {
        rollout.first_sample = sampled_reference;
        rollout.first_q_ref = q_ref;
        rollout.first_tau_ff = tau_ff;
        rollout.first_kp = kp;
        rollout.first_kd = kd;
      }

      JointArray applied_torque{};
      for (std::size_t joint_index = 0; joint_index < kControlledJointCount;
           ++joint_index) {
        const double q = backend.data->qpos[backend.joint_qposadr[joint_index]];
        const double dq = backend.data->qvel[backend.joint_dofadr[joint_index]];
        double torque =
            tau_ff[joint_index] + kp * (q_ref[joint_index] - q) - kd * dq;
        torque = Clamp(torque, -backend.actuator_limits[joint_index],
                       backend.actuator_limits[joint_index]);
        applied_torque[joint_index] = torque;
        backend.data->ctrl[backend.actuator_ids[joint_index]] = torque;
        accumulated_control_energy += std::abs(torque);
      }

      for (int substep = 0; substep < substeps; ++substep) {
        mj_step(backend.model, backend.data);
      }

      const auto base_rpy = backend.BaseRpy();
      const auto next_contact =
          backend.FootContacts(config_.contact_release_threshold_n);
      const int next_contact_count = CountContacts(next_contact);
      const double vx = backend.data->qvel[0];
      const double vz = backend.data->qvel[2];
      const double pitch_deg = base_rpy[1] * kRadToDeg;
      const double roll_deg = base_rpy[0] * kRadToDeg;
      const double next_time = current_time + rollout_dt;
      const double ballistic_time_s = std::max(0.0, next_time - push_end);
      const double ballistic_vz =
          reference_profile.push_vertical_velocity_mps -
          gravity_mps2 * ballistic_time_s;

      const auto phase_state_before = candidate_phase_state;
      UpdatePhaseEventState(candidate_phase_state, next_contact_count,
                            candidate_phase_state.contact_signal_valid, vz,
                            next_time);
      if (!phase_state_before.takeoff_latched &&
          candidate_phase_state.takeoff_latched) {
        takeoff_recorded = true;
        takeoff_x = backend.data->qpos[0];
        takeoff_vx = vx;
        takeoff_vz = vz;
        takeoff_pitch_deg = pitch_deg;
        predicted_ballistic_distance_m =
            std::max(0.0, takeoff_vx) *
            std::max(0.0, 2.0 * takeoff_vz / gravity_mps2);
      }
      if (!phase_state_before.touchdown_latched &&
          candidate_phase_state.touchdown_latched) {
        touchdown_recorded = true;
        touchdown_x = backend.data->qpos[0];
      }

      target_dx += sampled_reference.desired_forward_velocity_mps * rollout_dt;

      switch (sampled_reference.phase) {
        case go2_jump_core::JumpPhase::kCrouch:
          score += 0.04 * Square(vx);
          score += 0.03 * Square(vz);
          score += 0.03 * Square(pitch_deg - sampled_reference.desired_body_pitch_deg);
          if (next_contact_count < 2) {
            score += 8.0;
          }
          break;
        case go2_jump_core::JumpPhase::kPush: {
          score += 0.28 * horizontal_priority *
                   Square(vx - sampled_reference.desired_forward_velocity_mps);
          score += 0.16 * vertical_priority *
                   Square(vz - sampled_reference.desired_vertical_velocity_mps);
          score += 0.05 * Square(pitch_deg - sampled_reference.desired_body_pitch_deg);
          const double push_progress = Clamp(
              sampled_reference.time_in_phase_s / std::max(task_.push_duration_s, 1e-6),
              0.0, 1.0);
          score += 0.22 * horizontal_priority * push_progress *
                   Square(vx - reference_profile.push_forward_velocity_mps);
          score += 0.10 * vertical_priority * push_progress *
                   Square(vz - reference_profile.push_vertical_velocity_mps);
          const double front_thigh_util =
              0.5 *
              (std::abs(applied_torque[1]) / backend.actuator_limits[1] +
               std::abs(applied_torque[4]) / backend.actuator_limits[4]);
          const double front_calf_util =
              0.5 *
              (std::abs(applied_torque[2]) / backend.actuator_limits[2] +
               std::abs(applied_torque[5]) / backend.actuator_limits[5]);
          const double rear_thigh_util =
              0.5 *
              (std::abs(applied_torque[7]) / backend.actuator_limits[7] +
               std::abs(applied_torque[10]) / backend.actuator_limits[10]);
          const double rear_calf_util =
              0.5 *
              (std::abs(applied_torque[8]) / backend.actuator_limits[8] +
               std::abs(applied_torque[11]) / backend.actuator_limits[11]);
          score += 0.12 * Square(std::max(0.0, rear_calf_util -
                                                   1.05 * rear_thigh_util));
          score += 0.06 * Square(std::max(0.0, front_calf_util -
                                                   1.15 * front_thigh_util));
          if (push_progress > 0.60 &&
              next_contact_count > config_.flight_contact_count_max) {
            score += 5.0;
          }
          if (push_progress < 0.25 &&
              next_contact_count <= config_.flight_contact_count_max) {
            score += 7.0;
          }
          const int next_front_contact_count =
              static_cast<int>(next_contact[0]) + static_cast<int>(next_contact[1]);
          const int next_rear_contact_count =
              static_cast<int>(next_contact[2]) + static_cast<int>(next_contact[3]);
          if (push_progress > 0.62 && next_front_contact_count == 0 &&
              next_rear_contact_count > 0 &&
              next_contact_count > config_.flight_contact_count_max) {
            const double rear_only_alpha = Smooth01(
                Clamp((push_progress - 0.62) / 0.22, 0.0, 1.0));
            score += 2.0 + 5.0 * rear_only_alpha;
          }
          break;
        }
        case go2_jump_core::JumpPhase::kFlight:
          score += 14.0 * next_contact_count;
          score += 0.24 * horizontal_priority *
                   Square(vx - reference_profile.flight_forward_velocity_mps);
          score += 0.12 * vertical_priority * Square(vz - ballistic_vz);
          score += 0.05 * Square(pitch_deg - sampled_reference.desired_body_pitch_deg);
          score += 0.02 * Square(roll_deg);
          break;
        case go2_jump_core::JumpPhase::kLanding:
          if (next_contact_count < config_.touchdown_contact_count_threshold) {
            score += 18.0;
          }
          score += 0.08 * Square(vx - sampled_reference.desired_forward_velocity_mps);
          score += 0.14 * Square(vz - sampled_reference.desired_vertical_velocity_mps);
          score += 0.08 * Square(pitch_deg - sampled_reference.desired_body_pitch_deg);
          score += 0.03 * Square(roll_deg);
          break;
        case go2_jump_core::JumpPhase::kSettle:
          if (next_contact_count < 3) {
            score += 10.0;
          }
          score += 0.06 * Square(vx);
          score += 0.10 * Square(vz);
          score += 0.06 * Square(pitch_deg);
          score += 0.03 * Square(roll_deg);
          break;
      }
    }

    const double end_dx = backend.data->qpos[0] - x_start;
    const double end_vx = backend.data->qvel[0];
    const double end_vz = backend.data->qvel[2];
    const auto end_contact =
        backend.FootContacts(config_.contact_release_threshold_n);
    const int end_contact_count = CountContacts(end_contact);
    const double horizon_end_time = task_elapsed_s + rollout_steps * rollout_dt;
    const auto end_reference = ApplyContactOverrides(
        SampleReference(horizon_end_time), candidate_phase_state,
        end_contact_count, end_vz, horizon_end_time);
    const double target_end_vx = (end_reference.phase == go2_jump_core::JumpPhase::kFlight)
                                     ? reference_profile.flight_forward_velocity_mps
                                     : end_reference.desired_forward_velocity_mps;
    const double target_end_vz = (end_reference.phase == go2_jump_core::JumpPhase::kFlight)
                                     ? (reference_profile.push_vertical_velocity_mps -
                                        gravity_mps2 *
                                            std::max(0.0, horizon_end_time - push_end))
                                     : end_reference.desired_vertical_velocity_mps;
    if (takeoff_recorded) {
      const double preferred_takeoff_pitch_deg =
          Lerp(-8.5, -10.5, std::min(distance_alpha, 1.0));
      const double min_takeoff_vx = std::max(
          0.48 + 0.14 * distance_alpha,
          0.34 * reference_profile.flight_forward_velocity_mps);
      const double min_takeoff_vz =
          0.78 + 0.10 * (1.0 - std::min(distance_alpha, 1.0));
      score += 1.55 * horizontal_priority *
               Square(takeoff_vx - reference_profile.flight_forward_velocity_mps);
      score += 0.75 * vertical_priority *
               Square(takeoff_vz - reference_profile.push_vertical_velocity_mps);
      score += 0.22 *
               Square(std::max(0.0, takeoff_pitch_deg - preferred_takeoff_pitch_deg));
      score += 2.60 * horizontal_priority *
               Square(std::max(0.0, min_takeoff_vx - takeoff_vx));
      score += 1.80 * vertical_priority *
               Square(std::max(0.0, min_takeoff_vz - takeoff_vz));
      score += 0.18 *
               Square(std::max(0.0, -(takeoff_pitch_deg + 12.0)));
      const double target_takeoff_ratio =
          reference_profile.push_vertical_velocity_mps /
          std::max(0.35, reference_profile.flight_forward_velocity_mps);
      score += 2.10 * horizontal_priority * Square(std::max(
          0.0, takeoff_vz - target_takeoff_ratio * std::max(0.0, takeoff_vx)));
      score += 2.80 *
               Square(predicted_ballistic_distance_m - task_.objective.target_distance_m);
      score += 2.00 * horizontal_priority * Square(std::max(
          0.0, task_.objective.target_distance_m - predicted_ballistic_distance_m));
      score += 0.12 * horizontal_priority * Square(std::max(
          0.0, takeoff_vz - 1.20 * reference_profile.push_vertical_velocity_mps));
    } else {
      score += 28.0;
    }
    if (takeoff_recorded && touchdown_recorded) {
      const double airborne_dx = touchdown_x - takeoff_x;
      score += 1.55 * horizontal_priority *
               Square(airborne_dx - task_.objective.target_distance_m);
    }
    score += 2.20 * Square(end_dx - target_dx);
    score += 0.65 * horizontal_priority * Square(end_vx - target_end_vx);
    score += 0.22 * vertical_priority * Square(end_vz - target_end_vz);
    score += 0.0008 * accumulated_control_energy;

    rollout.score = score;
    rollout.candidate = candidate;
    if (rollout.score < best_result.score) {
      best_result = rollout;
    }
  }

  if (!std::isfinite(best_result.score)) {
    command.backend_ready = false;
    return command;
  }

  const auto planned_now = SampleReference(task_elapsed_s);
  const auto actual_sample =
      ApplyContactOverrides(planned_now, execution_phase_state_,
                            command.contact_count, observation.body_velocity[2],
                            task_elapsed_s);
  const auto command_sample =
      ApplyCandidateToSample(actual_sample, best_result.candidate);
  const auto nominal_q_ref = use_taskspace_reference
                                 ? BuildTaskSpacePoseForSample(command_sample)
                                 : BuildPoseForSample(command_sample);
  const auto nominal_tau_ff =
      BuildFeedforwardForSample(task_, command_sample);

  command.phase = command_sample.phase;
  command.contact_override = (command_sample.phase != planned_now.phase);
  command.desired_forward_velocity_mps =
      command_sample.desired_forward_velocity_mps;
  command.desired_vertical_velocity_mps =
      command_sample.desired_vertical_velocity_mps;
  command.desired_body_pitch_deg = command_sample.desired_body_pitch_deg;
  command.desired_body_height_offset_m =
      command_sample.desired_body_height_offset_m;
  command.q_ref = use_taskspace_reference
                      ? nominal_q_ref
                      : ApplyCandidateToPose(nominal_q_ref, command_sample,
                                             best_result.candidate);
  command.dq_ref.fill(0.0);
  command.tau_ff = ApplyCandidateToFeedforward(
      nominal_tau_ff, command_sample, best_result.candidate,
      config_.max_feedforward_torque_nm);
  const auto command_support_loads = backend.FootContactForces();
  const auto command_push_wrench_tau = build_push_wrench_torques(
      command_sample, command.foot_contact, command_support_loads);
  command.tau_ff = add_torque_arrays(command.tau_ff, command_push_wrench_tau,
                                     config_.max_feedforward_torque_nm);
  command.uniform_kp = GainsForPhase(command_sample.phase, false);
  command.uniform_kd = GainsForPhase(command_sample.phase, true);
  return command;
}

std::array<double, kControlledJointCount> WholeBodyMpc::BuildPoseForSample(
    const go2_jump_core::JumpReferenceSample& sample) const {
  std::array<double, kControlledJointCount> pose = config_.stand_pose;
  const double distance_alpha = JumpDistanceAlpha(task_);
  switch (sample.phase) {
    case go2_jump_core::JumpPhase::kCrouch:
      pose = BlendPose(config_.stand_pose, config_.crouch_pose, 0.9);
      break;
    case go2_jump_core::JumpPhase::kPush:
      pose = config_.push_pose;
      break;
    case go2_jump_core::JumpPhase::kFlight:
      pose = BlendPose(config_.flight_pose, config_.landing_pose,
                       sample.landing_brace_factor);
      break;
    case go2_jump_core::JumpPhase::kLanding:
      pose = config_.landing_pose;
      break;
    case go2_jump_core::JumpPhase::kSettle:
      pose = BlendPose(config_.landing_pose, config_.settle_pose, 0.8);
      break;
  }

  const double pitch_rad = sample.desired_body_pitch_deg * kPi / 180.0;
  double phase_pitch_scale = 1.0;
  switch (sample.phase) {
    case go2_jump_core::JumpPhase::kCrouch:
      phase_pitch_scale = 1.2;
      break;
    case go2_jump_core::JumpPhase::kPush:
      phase_pitch_scale = 2.0;
      break;
    case go2_jump_core::JumpPhase::kFlight:
      phase_pitch_scale = 1.8;
      break;
    case go2_jump_core::JumpPhase::kLanding:
      phase_pitch_scale = 1.3;
      break;
    case go2_jump_core::JumpPhase::kSettle:
      phase_pitch_scale = 1.0;
      break;
  }
  const double hip_delta =
      Clamp(0.18 * phase_pitch_scale * pitch_rad, -0.14, 0.14);
  const double thigh_delta =
      Clamp(0.28 * phase_pitch_scale * pitch_rad, -0.22, 0.22);
  const double calf_delta =
      Clamp(-0.40 * phase_pitch_scale * pitch_rad, -0.28, 0.28);

  pose[0] -= hip_delta;
  pose[3] += hip_delta;
  pose[6] -= 0.5 * hip_delta;
  pose[9] += 0.5 * hip_delta;

  pose[1] += thigh_delta;
  pose[4] += thigh_delta;
  pose[7] -= thigh_delta;
  pose[10] -= thigh_delta;

  pose[2] += calf_delta;
  pose[5] += calf_delta;
  pose[8] -= 0.6 * calf_delta;
  pose[11] -= 0.6 * calf_delta;

  auto apply_leg_delta = [&pose](std::size_t thigh_idx, std::size_t calf_idx,
                                 double thigh_delta_value,
                                 double calf_delta_value) {
    pose[thigh_idx] += thigh_delta_value;
    pose[calf_idx] += calf_delta_value;
  };

  if (sample.phase == go2_jump_core::JumpPhase::kCrouch) {
    const double front_preload = 0.35 * distance_alpha;
    const double rear_preload = 0.45 * distance_alpha;
    for (std::size_t thigh_idx : {1u, 4u}) {
      apply_leg_delta(thigh_idx, thigh_idx + 1, -0.020 * front_preload,
                      0.030 * front_preload);
    }
    for (std::size_t thigh_idx : {7u, 10u}) {
      apply_leg_delta(thigh_idx, thigh_idx + 1, 0.035 * rear_preload,
                      -0.055 * rear_preload);
    }
  } else if (sample.phase == go2_jump_core::JumpPhase::kPush) {
    const double drive_alpha = 0.60 * distance_alpha;
    const double push_progress = Clamp(
        sample.time_in_phase_s / std::max(task_.push_duration_s, 1e-6), 0.0, 1.0);
    const double late_push_alpha =
        Smooth01(Clamp((push_progress - 0.45) / 0.32, 0.0, 1.0));
    const double front_release_alpha =
        Smooth01(Clamp((push_progress - 0.58) / 0.18, 0.0, 1.0));
    const double rear_release_alpha =
        Smooth01(Clamp((push_progress - 0.92) / 0.08, 0.0, 1.0));
    for (std::size_t thigh_idx : {1u, 4u}) {
      apply_leg_delta(thigh_idx, thigh_idx + 1, 0.026 * drive_alpha,
                      -0.038 * drive_alpha);
    }
    for (std::size_t thigh_idx : {7u, 10u}) {
      apply_leg_delta(thigh_idx, thigh_idx + 1, -0.072 * drive_alpha,
                      0.070 * drive_alpha);
    }
    for (std::size_t thigh_idx : {1u, 4u}) {
      apply_leg_delta(thigh_idx, thigh_idx + 1,
                      0.028 * late_push_alpha * distance_alpha,
                      -0.040 * late_push_alpha * distance_alpha);
    }
    for (std::size_t thigh_idx : {7u, 10u}) {
      apply_leg_delta(thigh_idx, thigh_idx + 1,
                      -0.056 * late_push_alpha * distance_alpha,
                      0.042 * late_push_alpha * distance_alpha);
    }
    const double front_release_scale =
        front_release_alpha * (0.70 + 0.45 * distance_alpha);
    const double rear_release_scale =
        rear_release_alpha * (0.25 + 0.20 * distance_alpha);
    for (std::size_t thigh_idx : {1u, 4u}) {
      apply_leg_delta(thigh_idx, thigh_idx + 1, 0.034 * front_release_scale,
                      -0.052 * front_release_scale);
    }
    for (std::size_t thigh_idx : {7u, 10u}) {
      apply_leg_delta(thigh_idx, thigh_idx + 1, 0.018 * rear_release_scale,
                      -0.028 * rear_release_scale);
    }
  } else if (sample.phase == go2_jump_core::JumpPhase::kFlight) {
    const double tuck = Clamp(sample.leg_retraction_ratio, 0.0, 1.0);
    for (std::size_t thigh_idx : {1u, 4u}) {
      apply_leg_delta(thigh_idx, thigh_idx + 1, 0.06 * tuck, -0.10 * tuck);
    }
    for (std::size_t thigh_idx : {7u, 10u}) {
      apply_leg_delta(thigh_idx, thigh_idx + 1, 0.05 * tuck, -0.08 * tuck);
    }
  }

  return pose;
}

std::array<double, kControlledJointCount> WholeBodyMpc::BuildTaskSpacePoseForSample(
    const go2_jump_core::JumpReferenceSample& sample) const {
  const JointArray seed_pose = BuildPoseForSample(sample);
  auto foot_targets = FootPositionsFromPose(seed_pose);
  const auto reference_profile = BuildActiveReferenceProfile();

  const double distance_alpha = Clamp(JumpDistanceAlpha(task_), 0.0, 1.0);
  const double forward_ratio = Clamp(
      sample.desired_forward_velocity_mps /
          std::max(reference_profile.push_forward_velocity_mps, 0.20),
      0.0, 1.35);
  const double vertical_ratio = Clamp(
      std::max(0.0, sample.desired_vertical_velocity_mps) /
          std::max(reference_profile.push_vertical_velocity_mps, 0.20),
      0.0, 1.35);
  const double crouch_progress =
      (sample.phase == go2_jump_core::JumpPhase::kCrouch)
          ? Smooth01(sample.time_in_phase_s /
                     std::max(task_.crouch_duration_s, 1e-6))
          : 0.0;
  const double push_progress =
      (sample.phase == go2_jump_core::JumpPhase::kPush)
          ? Clamp(sample.time_in_phase_s / std::max(task_.push_duration_s, 1e-6),
                  0.0, 1.0)
          : 0.0;
  const double push_alpha = Smooth01(push_progress);
  const double late_push_alpha =
      (sample.phase == go2_jump_core::JumpPhase::kPush)
          ? Smooth01(Clamp((push_progress - 0.52) / 0.24, 0.0, 1.0))
          : 0.0;
  const double flight_progress =
      (sample.phase == go2_jump_core::JumpPhase::kFlight)
          ? Clamp(sample.time_in_phase_s /
                      std::max(task_.estimated_flight_time_s, 1e-6),
                  0.0, 1.0)
          : 0.0;
  const double descent_alpha =
      (sample.phase == go2_jump_core::JumpPhase::kFlight)
          ? Smooth01(std::max(0.0, (flight_progress - 0.5) / 0.5))
          : 0.0;
  const double landing_alpha =
      (sample.phase == go2_jump_core::JumpPhase::kLanding)
          ? Smooth01(sample.time_in_phase_s /
                     std::max(task_.landing_duration_s, 1e-6))
          : 0.0;

  const double front_forward_shift_m =
      Lerp(0.006, 0.018, distance_alpha) * forward_ratio;
  const double rear_forward_shift_m =
      Lerp(0.012, 0.032, distance_alpha) * forward_ratio;
  const double front_release_lift_m =
      Lerp(0.004, 0.014, distance_alpha) * (0.8 + 0.4 * forward_ratio);
  const double rear_extension_m =
      Lerp(0.0, 0.010, distance_alpha) * std::max(0.0, vertical_ratio - 0.85);
  const double flight_tuck_lift_m = Lerp(0.008, 0.024, distance_alpha);
  const double landing_reach_m = Lerp(0.012, 0.032, distance_alpha);

  auto apply_front_delta = [&](double dx, double dz) {
    for (std::size_t leg_index : {0u, 1u}) {
      foot_targets[leg_index][0] += dx;
      foot_targets[leg_index][2] += dz;
    }
  };
  auto apply_rear_delta = [&](double dx, double dz) {
    for (std::size_t leg_index : {2u, 3u}) {
      foot_targets[leg_index][0] += dx;
      foot_targets[leg_index][2] += dz;
    }
  };
  auto apply_all_lift = [&](double dz) {
    for (auto& foot_target : foot_targets) {
      foot_target[2] += dz;
    }
  };

  switch (sample.phase) {
    case go2_jump_core::JumpPhase::kCrouch:
      apply_front_delta(0.0, 0.0);
      apply_rear_delta(0.004 * std::max(0.0, forward_ratio - 1.0) *
                           crouch_progress,
                       0.0);
      break;
    case go2_jump_core::JumpPhase::kPush:
      apply_front_delta(-front_forward_shift_m * push_alpha,
                        front_release_lift_m * late_push_alpha);
      apply_rear_delta(-rear_forward_shift_m * push_alpha,
                       -rear_extension_m * late_push_alpha);
      break;
    case go2_jump_core::JumpPhase::kFlight:
      apply_all_lift(flight_tuck_lift_m * sample.leg_retraction_ratio);
      apply_front_delta(landing_reach_m * descent_alpha, 0.0);
      apply_rear_delta(0.45 * landing_reach_m * descent_alpha, 0.0);
      break;
    case go2_jump_core::JumpPhase::kLanding:
      apply_front_delta(landing_reach_m * (1.0 - 0.35 * landing_alpha),
                        0.008 * sample.landing_brace_factor);
      apply_rear_delta(0.55 * landing_reach_m * (1.0 - 0.50 * landing_alpha),
                       0.004 * sample.landing_brace_factor);
      break;
    case go2_jump_core::JumpPhase::kSettle:
      break;
  }

  return PoseFromFootTargets(foot_targets, seed_pose);
}

std::array<double, kControlledJointCount> WholeBodyMpc::BuildFeedforwardForSample(
    const go2_jump_core::JumpTaskSpec& task,
    const go2_jump_core::JumpReferenceSample& sample) const {
  std::array<double, kControlledJointCount> tau{};
  const double distance_alpha = JumpDistanceAlpha(task);
  const double push_scale = Clamp(task.target_takeoff_speed_mps / 2.2, 0.0, 1.0);
  const double forward_bias_scale =
      Clamp(std::max(task.target_takeoff_velocity_x_mps,
                     sample.desired_forward_velocity_mps) /
                1.4,
            0.0, 1.6);
  const double landing_scale = Clamp(sample.landing_brace_factor, 0.0, 1.0);

  if (sample.phase == go2_jump_core::JumpPhase::kPush) {
    const double push_progress = Clamp(
        sample.time_in_phase_s / std::max(task.push_duration_s, 1e-6), 0.0, 1.0);
    const double late_push_alpha =
        Smooth01(Clamp((push_progress - 0.45) / 0.32, 0.0, 1.0));
    const double front_release_alpha =
        Smooth01(Clamp((push_progress - 0.58) / 0.18, 0.0, 1.0));
    const double rear_release_alpha =
        Smooth01(Clamp((push_progress - 0.92) / 0.08, 0.0, 1.0));
    const double thigh_tau =
        Clamp(6.2 + 6.2 * push_scale + 0.6 * distance_alpha, 0.0,
              config_.max_feedforward_torque_nm);
    const double calf_tau =
        Clamp(6.8 + 5.2 * push_scale + 0.4 * distance_alpha, 0.0,
              config_.max_feedforward_torque_nm);
    const double front_thigh_tau = Clamp(
        thigh_tau - (1.4 + 0.7 * distance_alpha) * forward_bias_scale, 0.0,
        config_.max_feedforward_torque_nm);
    const double front_calf_tau = Clamp(
        calf_tau - (1.2 + 0.5 * distance_alpha) * forward_bias_scale, 0.0,
        config_.max_feedforward_torque_nm);
    const double rear_thigh_tau = Clamp(
        thigh_tau + (3.0 + 1.2 * distance_alpha) * forward_bias_scale, 0.0,
        config_.max_feedforward_torque_nm);
    const double rear_calf_tau = Clamp(
        calf_tau + (1.8 + 0.7 * distance_alpha) * forward_bias_scale, 0.0,
        config_.max_feedforward_torque_nm);
    for (std::size_t idx : {1u, 4u}) {
      tau[idx] = front_thigh_tau;
    }
    for (std::size_t idx : {2u, 5u}) {
      tau[idx] = front_calf_tau;
    }
    for (std::size_t idx : {7u, 10u}) {
      tau[idx] = rear_thigh_tau;
    }
    for (std::size_t idx : {8u, 11u}) {
      tau[idx] = rear_calf_tau;
    }

    const double front_taper_thigh =
        Clamp(-1.2 * late_push_alpha * distance_alpha, -config_.max_feedforward_torque_nm,
              config_.max_feedforward_torque_nm);
    const double front_taper_calf =
        Clamp(-1.2 * late_push_alpha * distance_alpha, -config_.max_feedforward_torque_nm,
              config_.max_feedforward_torque_nm);
    const double rear_boost_thigh =
        Clamp(1.8 * late_push_alpha * distance_alpha, -config_.max_feedforward_torque_nm,
              config_.max_feedforward_torque_nm);
    const double rear_boost_calf =
        Clamp(0.8 * late_push_alpha * distance_alpha, -config_.max_feedforward_torque_nm,
              config_.max_feedforward_torque_nm);
    const double front_release_taper_thigh =
        Clamp(-0.9 * front_release_alpha * (0.5 + 0.5 * distance_alpha),
              -config_.max_feedforward_torque_nm, config_.max_feedforward_torque_nm);
    const double front_release_taper_calf =
        Clamp(-1.2 * front_release_alpha * (0.5 + 0.5 * distance_alpha),
              -config_.max_feedforward_torque_nm, config_.max_feedforward_torque_nm);
    const double rear_release_taper_thigh =
        Clamp(-0.4 * rear_release_alpha * (0.4 + 0.4 * distance_alpha),
              -config_.max_feedforward_torque_nm, config_.max_feedforward_torque_nm);
    const double rear_release_taper_calf =
        Clamp(-0.6 * rear_release_alpha * (0.4 + 0.4 * distance_alpha),
              -config_.max_feedforward_torque_nm, config_.max_feedforward_torque_nm);
    for (std::size_t idx : {1u, 4u}) {
      tau[idx] = Clamp(tau[idx] + front_taper_thigh + front_release_taper_thigh, 0.0,
                       config_.max_feedforward_torque_nm);
    }
    for (std::size_t idx : {2u, 5u}) {
      tau[idx] = Clamp(tau[idx] + front_taper_calf + front_release_taper_calf, 0.0,
                       config_.max_feedforward_torque_nm);
    }
    for (std::size_t idx : {7u, 10u}) {
      tau[idx] = Clamp(tau[idx] + rear_boost_thigh + rear_release_taper_thigh, 0.0,
                       config_.max_feedforward_torque_nm);
    }
    for (std::size_t idx : {8u, 11u}) {
      tau[idx] = Clamp(tau[idx] + rear_boost_calf + rear_release_taper_calf, 0.0,
                       config_.max_feedforward_torque_nm);
    }
  } else if (sample.phase == go2_jump_core::JumpPhase::kLanding) {
    const double thigh_tau = Clamp(-3.0 - 5.0 * landing_scale,
                                   -config_.max_feedforward_torque_nm, 0.0);
    const double calf_tau = Clamp(-4.5 - 7.5 * landing_scale,
                                  -config_.max_feedforward_torque_nm, 0.0);
    for (std::size_t idx : {1u, 4u, 7u, 10u}) {
      tau[idx] = thigh_tau;
    }
    for (std::size_t idx : {2u, 5u, 8u, 11u}) {
      tau[idx] = calf_tau;
    }
  }

  return tau;
}

double WholeBodyMpc::GainsForPhase(go2_jump_core::JumpPhase phase,
                                   bool derivative) const {
  switch (phase) {
    case go2_jump_core::JumpPhase::kCrouch:
      return derivative ? config_.default_kd : config_.default_kp;
    case go2_jump_core::JumpPhase::kPush:
      return derivative ? config_.push_kd : config_.push_kp;
    case go2_jump_core::JumpPhase::kFlight:
      return derivative ? config_.flight_kd : config_.flight_kp;
    case go2_jump_core::JumpPhase::kLanding:
      return derivative ? config_.landing_kd : config_.landing_kp;
    case go2_jump_core::JumpPhase::kSettle:
      return derivative ? config_.settle_kd : config_.settle_kp;
  }
  return derivative ? config_.default_kd : config_.default_kp;
}

go2_jump_core::JumpReferenceSample WholeBodyMpc::ApplyContactOverrides(
    const go2_jump_core::JumpReferenceSample& planned_sample,
    const RobotObservation& observation, double task_elapsed_s) {
  const auto phase_state =
      BuildExecutionPhaseEventState(observation, task_elapsed_s);
  return ApplyContactOverrides(planned_sample, phase_state,
                               CountContacts(observation.foot_contact),
                               observation.body_velocity[2], task_elapsed_s);
}

go2_jump_core::JumpReferenceSample WholeBodyMpc::ApplyContactOverrides(
    const go2_jump_core::JumpReferenceSample& planned_sample,
    const PhaseEventState& state, int contact_count,
    double body_vertical_velocity_mps, double task_elapsed_s) const {
  auto sample = planned_sample;
  if (!state.contact_signal_valid) {
    return sample;
  }

  const double push_end = task_.crouch_duration_s + task_.push_duration_s;
  const double max_push_extension_s = std::max(
      config_.late_takeoff_window_s,
      std::min(0.34, std::max(0.18, task_.estimated_flight_time_s + 0.10)));
  const double max_flight_hold_s = task_.estimated_flight_time_s + 0.12;
  const double flight_duration_s = std::max(task_.estimated_flight_time_s, 1e-6);
  const double landing_duration_s = std::max(task_.landing_duration_s, 1e-6);
  const double settle_duration_s = std::max(task_.settle_duration_s, 1e-6);
  const double takeoff_time_s =
      state.takeoff_latched ? state.takeoff_time_s : state.takeoff_candidate_start_s;
  const double touchdown_time_s = state.touchdown_latched
                                      ? state.touchdown_time_s
                                      : state.touchdown_candidate_start_s;
  const auto reference_profile = BuildActiveReferenceProfile();

  const auto build_push_hold_sample = [&]() {
    auto hold_sample = SampleReference(std::max(0.0, push_end - 1e-4));
    hold_sample.phase = go2_jump_core::JumpPhase::kPush;
    hold_sample.time_in_phase_s = task_.push_duration_s;
    hold_sample.desired_forward_velocity_mps =
        reference_profile.push_forward_velocity_mps;
    hold_sample.desired_vertical_velocity_mps =
        reference_profile.push_vertical_velocity_mps;
    hold_sample.desired_body_pitch_deg =
        reference_profile.effective_takeoff_pitch_deg;
    hold_sample.desired_body_height_offset_m =
        reference_profile.push_height_offset_m;
    hold_sample.leg_retraction_ratio = 0.0;
    hold_sample.landing_brace_factor = 0.0;
    return hold_sample;
  };

  const auto build_flight_sample = [&](double flight_elapsed_s) {
    go2_jump_core::JumpReferenceSample flight_sample{};
    const double alpha = Clamp(flight_elapsed_s / flight_duration_s, 0.0, 1.0);
    const double descent_alpha = Smooth01(std::max(0.0, (alpha - 0.5) / 0.5));
    flight_sample.phase = go2_jump_core::JumpPhase::kFlight;
    flight_sample.time_in_phase_s = flight_elapsed_s;
    flight_sample.desired_forward_velocity_mps =
        reference_profile.flight_forward_velocity_mps;
    flight_sample.desired_vertical_velocity_mps =
        reference_profile.push_vertical_velocity_mps -
        config_.reference_config.gravity_mps2 * flight_elapsed_s;
    flight_sample.desired_body_pitch_deg =
        Lerp(reference_profile.effective_takeoff_pitch_deg,
             task_.objective.target_landing_pitch_deg, descent_alpha);
    flight_sample.desired_body_height_offset_m = Lerp(
        reference_profile.flight_height_offset_m,
        reference_profile.landing_height_offset_m, descent_alpha);
    flight_sample.leg_retraction_ratio =
        Smooth01(alpha) * reference_profile.leg_retraction_scale;
    flight_sample.landing_brace_factor = Clamp(
        descent_alpha * reference_profile.landing_brace_scale, 0.0, 1.2);
    return flight_sample;
  };

  const auto build_landing_sample = [&](double landing_elapsed_s) {
    go2_jump_core::JumpReferenceSample landing_sample{};
    const double alpha = Smooth01(landing_elapsed_s / landing_duration_s);
    landing_sample.phase = go2_jump_core::JumpPhase::kLanding;
    landing_sample.time_in_phase_s = landing_elapsed_s;
    landing_sample.desired_forward_velocity_mps = 0.0;
    landing_sample.desired_vertical_velocity_mps = 0.0;
    landing_sample.desired_body_pitch_deg =
        Lerp(task_.objective.target_landing_pitch_deg, 0.0, alpha);
    landing_sample.desired_body_height_offset_m =
        Lerp(reference_profile.landing_height_offset_m, 0.0, alpha);
    landing_sample.leg_retraction_ratio = 0.0;
    landing_sample.landing_brace_factor = Lerp(
        Clamp(reference_profile.landing_brace_scale, 0.8, 1.2), 0.3, alpha);
    return landing_sample;
  };

  const auto build_settle_sample = [&](double settle_elapsed_s) {
    go2_jump_core::JumpReferenceSample settle_sample{};
    const double alpha = Smooth01(settle_elapsed_s / settle_duration_s);
    settle_sample.phase = go2_jump_core::JumpPhase::kSettle;
    settle_sample.time_in_phase_s = settle_elapsed_s;
    settle_sample.desired_forward_velocity_mps = 0.0;
    settle_sample.desired_vertical_velocity_mps = 0.0;
    settle_sample.desired_body_pitch_deg = 0.0;
    settle_sample.desired_body_height_offset_m = 0.0;
    settle_sample.leg_retraction_ratio = 0.0;
    settle_sample.landing_brace_factor = Lerp(0.3, 0.0, alpha);
    return settle_sample;
  };

  if (!state.takeoff_latched &&
      contact_count > config_.flight_contact_count_max &&
      task_elapsed_s <= push_end + max_push_extension_s &&
      (planned_sample.phase == go2_jump_core::JumpPhase::kFlight ||
       planned_sample.phase == go2_jump_core::JumpPhase::kLanding ||
       planned_sample.phase == go2_jump_core::JumpPhase::kSettle) &&
      body_vertical_velocity_mps >= -0.20) {
    return build_push_hold_sample();
  }

  if (takeoff_time_s >= 0.0 && !state.touchdown_latched) {
    const double flight_elapsed_s = std::max(0.0, task_elapsed_s - takeoff_time_s);
    if (planned_sample.phase == go2_jump_core::JumpPhase::kPush ||
        planned_sample.phase == go2_jump_core::JumpPhase::kFlight ||
        ((planned_sample.phase == go2_jump_core::JumpPhase::kLanding ||
          planned_sample.phase == go2_jump_core::JumpPhase::kSettle) &&
         flight_elapsed_s <= max_flight_hold_s)) {
      return build_flight_sample(flight_elapsed_s);
    }
  }

  if (touchdown_time_s >= 0.0 && !state.settle_latched) {
    const double landing_elapsed_s =
        std::max(0.0, task_elapsed_s - touchdown_time_s);
    return build_landing_sample(landing_elapsed_s);
  }

  if (state.settle_latched) {
    const double settle_start_time_s =
        state.settle_time_s >= 0.0 ? state.settle_time_s : task_elapsed_s;
    const double settle_elapsed_s =
        std::max(0.0, task_elapsed_s - settle_start_time_s);
    return build_settle_sample(settle_elapsed_s);
  }

  return sample;
}

}  // namespace go2_jump_mpc

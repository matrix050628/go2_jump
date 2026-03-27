#!/usr/bin/env python3
import argparse
import bisect
import json
import os
import time
from typing import Any, Dict, List, Optional

import rclpy
from rclpy.node import Node

from go2_jump_msgs.msg import JumpControllerState
from unitree_go.msg import SportModeState

try:
    from unitree_go.msg import LowCmd
except ImportError:  # pragma: no cover
    LowCmd = None

try:
    from unitree_go.msg import LowState
except ImportError:  # pragma: no cover
    LowState = None


GO2_CONTROLLED_JOINT_COUNT = 12
GO2_TORQUE_LIMITS_NM = [
    23.7,
    23.7,
    45.43,
    23.7,
    23.7,
    45.43,
    23.7,
    23.7,
    45.43,
    23.7,
    23.7,
    45.43,
]


def intent_signature(intent: Optional[Dict[str, Any]]) -> Optional[tuple]:
    if intent is None:
        return None
    return (
        intent.get("planner_backend"),
        intent.get("target_distance_m"),
        intent.get("target_takeoff_velocity_x_mps"),
        intent.get("target_takeoff_velocity_z_mps"),
        intent.get("target_takeoff_pitch_deg"),
        intent.get("target_landing_pitch_deg"),
        intent.get("crouch_duration_s"),
        intent.get("push_duration_s"),
        intent.get("estimated_flight_time_s"),
        intent.get("landing_duration_s"),
        intent.get("settle_duration_s"),
        intent.get("horizon_duration_s"),
        intent.get("crouch_height_offset_m"),
        intent.get("push_height_offset_m"),
        intent.get("flight_height_offset_m"),
        intent.get("landing_height_offset_m"),
        intent.get("leg_retraction_scale"),
        intent.get("landing_brace_scale"),
        intent.get("front_push_foot_x_bias_m"),
        intent.get("rear_push_foot_x_bias_m"),
        intent.get("front_landing_foot_x_bias_m"),
        intent.get("rear_landing_foot_x_bias_m"),
        intent.get("swing_foot_height_m"),
        intent.get("planned_apex_height_m"),
    )


def collapse_intent_events(samples: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    events: List[Dict[str, Any]] = []
    last_signature = object()
    for sample in samples:
        signature = intent_signature(sample.get("intent"))
        if signature == last_signature:
            continue
        last_signature = signature
        events.append(
            {
                "t": sample["t"],
                "intent_active": sample.get("intent_active", False),
                "planner_backend": (
                    sample["intent"]["planner_backend"]
                    if sample.get("intent") is not None
                    else "none"
                ),
                "intent": sample.get("intent"),
            }
        )
    return events


def collapse_phase_segments(samples: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    segments: List[Dict[str, Any]] = []
    for sample in samples:
        phase = sample["phase"]
        if not segments or segments[-1]["phase"] != phase:
            segments.append(
                {
                    "phase": phase,
                    "start_s": sample["t"],
                    "end_s": sample["t"],
                    "samples": 1,
                    "min_contact_count": sample["contact_count"],
                    "max_contact_count": sample["contact_count"],
                }
            )
            continue
        segment = segments[-1]
        segment["end_s"] = sample["t"]
        segment["samples"] += 1
        segment["min_contact_count"] = min(
            segment["min_contact_count"], sample["contact_count"]
        )
        segment["max_contact_count"] = max(
            segment["max_contact_count"], sample["contact_count"]
        )
    return segments


def first_index(
    samples: List[Dict[str, Any]], predicate, start_index: int = 0
) -> Optional[int]:
    for index in range(start_index, len(samples)):
        if predicate(samples[index]):
            return index
    return None


def nearest_sample(
    samples: List[Dict[str, Any]], target_time_s: Optional[float]
) -> Optional[Dict[str, Any]]:
    if not samples or target_time_s is None:
        return None
    sample_times = [sample["t"] for sample in samples]
    insert_index = bisect.bisect_left(sample_times, target_time_s)
    candidates = []
    if insert_index < len(samples):
        candidates.append(samples[insert_index])
    if insert_index > 0:
        candidates.append(samples[insert_index - 1])
    if not candidates:
        return None
    return min(candidates, key=lambda sample: abs(sample["t"] - target_time_s))


def compute_rate_hz(sample_times: List[float]) -> Optional[float]:
    if len(sample_times) < 2:
        return None
    elapsed = sample_times[-1] - sample_times[0]
    if elapsed <= 1e-6:
        return None
    return (len(sample_times) - 1) / elapsed


def compute_summary(
    controller_samples: List[Dict[str, Any]],
    sport_samples: List[Dict[str, Any]],
    lowcmd_times: List[float],
    lowcmd_samples: List[Dict[str, Any]],
    duration_s: float,
) -> Dict[str, Any]:
    segments = collapse_phase_segments(controller_samples)
    phase_sequence = [segment["phase"] for segment in segments]

    takeoff_index = first_index(
        controller_samples,
        lambda sample: sample["phase"] == "flight" and sample["contact_count"] <= 1,
    )
    if takeoff_index is None:
        takeoff_index = first_index(
            controller_samples, lambda sample: sample["phase"] == "flight"
        )

    touchdown_index = None
    if takeoff_index is not None:
        takeoff_time_s = controller_samples[takeoff_index]["t"]
        touchdown_index = first_index(
            controller_samples,
            lambda sample: sample["t"] >= takeoff_time_s + 0.02
            and (
                sample["phase"] in ("landing", "settle")
                or sample["contact_count"] >= 2
            ),
            takeoff_index + 1,
        )
    settle_index = first_index(
        controller_samples, lambda sample: sample["phase"] == "settle"
    )

    takeoff_time_s = (
        controller_samples[takeoff_index]["t"] if takeoff_index is not None else None
    )
    touchdown_time_s = (
        controller_samples[touchdown_index]["t"]
        if touchdown_index is not None
        else None
    )
    settle_time_s = (
        controller_samples[settle_index]["t"] if settle_index is not None else None
    )

    first_landing_segment_index = None
    for index, segment in enumerate(segments):
        if segment["phase"] in ("landing", "settle"):
            first_landing_segment_index = index
            break
    has_flight_relapse_after_landing = False
    if first_landing_segment_index is not None:
        for segment in segments[first_landing_segment_index + 1 :]:
            if segment["phase"] == "flight":
                has_flight_relapse_after_landing = True
                break

    flight_contact_counts = [
        sample["contact_count"]
        for sample in controller_samples
        if sample["phase"] == "flight"
    ]
    max_contact_count_in_flight = (
        max(flight_contact_counts) if flight_contact_counts else None
    )

    initial_sport = sport_samples[0] if sport_samples else None
    takeoff_sport = nearest_sample(sport_samples, takeoff_time_s)
    touchdown_sport = nearest_sample(sport_samples, touchdown_time_s)
    final_sport = (
        nearest_sample(sport_samples, settle_time_s)
        if settle_time_s is not None
        else (sport_samples[-1] if sport_samples else None)
    )

    initial_x = initial_sport["x"] if initial_sport else None
    takeoff_x = takeoff_sport["x"] if takeoff_sport else None
    touchdown_x = touchdown_sport["x"] if touchdown_sport else None
    final_x = final_sport["x"] if final_sport else None

    initial_z = initial_sport["z"] if initial_sport else None
    max_z = max((sample["z"] for sample in sport_samples), default=None)
    max_height_gain_m = (
        max_z - initial_z if max_z is not None and initial_z is not None else None
    )
    total_distance_m = (
        final_x - initial_x if final_x is not None and initial_x is not None else None
    )
    airborne_distance_m = (
        touchdown_x - takeoff_x
        if touchdown_x is not None and takeoff_x is not None
        else None
    )
    post_touchdown_distance_m = (
        final_x - touchdown_x
        if final_x is not None and touchdown_x is not None
        else None
    )
    airborne_share = None
    if (
        total_distance_m is not None
        and airborne_distance_m is not None
        and abs(total_distance_m) > 1e-6
    ):
        airborne_share = airborne_distance_m / total_distance_m

    controller_rate_hz = compute_rate_hz([sample["t"] for sample in controller_samples])
    lowcmd_rate_hz = compute_rate_hz(lowcmd_times)
    max_forward_velocity_mps = max(
        (sample["vx"] for sample in sport_samples), default=None
    )
    max_upward_velocity_mps = max(
        (sample["vz"] for sample in sport_samples), default=None
    )
    min_vertical_velocity_mps = min(
        (sample["vz"] for sample in sport_samples), default=None
    )

    target_distance_m = (
        controller_samples[0]["target_distance_m"] if controller_samples else None
    )
    solver_backend_name = (
        controller_samples[0]["backend_name"] if controller_samples else None
    )
    planner_events = collapse_intent_events(controller_samples)
    active_planner_events = [
        event for event in planner_events if event.get("intent_active")
    ]
    active_intent = (
        active_planner_events[0]["intent"] if active_planner_events else None
    )
    planner_backend_name = (
        active_intent["planner_backend"] if active_intent is not None else "none"
    )
    mainly_airborne_translation = (
        airborne_share is not None and airborne_share >= 0.6
    )

    push_effort_samples = [
        sample for sample in lowcmd_samples if sample.get("phase") == "push"
    ]
    push_peak_abs_tau_nm = None
    push_mean_abs_tau_nm = None
    push_mean_abs_tau_ff_nm = None
    push_feedforward_fraction = None
    push_joint_peak_abs_tau_nm = None
    push_joint_limit_utilization = None
    if push_effort_samples:
        push_joint_peak_abs_tau_nm = [
            max(abs(sample["command_torque_est_nm"][joint]) for sample in push_effort_samples)
            for joint in range(GO2_CONTROLLED_JOINT_COUNT)
        ]
        push_peak_abs_tau_nm = max(push_joint_peak_abs_tau_nm)
        push_mean_abs_tau_nm = sum(
            sum(abs(value) for value in sample["command_torque_est_nm"])
            / GO2_CONTROLLED_JOINT_COUNT
            for sample in push_effort_samples
        ) / len(push_effort_samples)
        push_mean_abs_tau_ff_nm = sum(
            sum(abs(value) for value in sample["tau_ff_nm"])
            / GO2_CONTROLLED_JOINT_COUNT
            for sample in push_effort_samples
        ) / len(push_effort_samples)
        total_abs_command_tau = sum(
            sum(abs(value) for value in sample["command_torque_est_nm"])
            for sample in push_effort_samples
        )
        total_abs_tau_ff = sum(
            sum(abs(value) for value in sample["tau_ff_nm"])
            for sample in push_effort_samples
        )
        if total_abs_command_tau > 1e-6:
            push_feedforward_fraction = total_abs_tau_ff / total_abs_command_tau
        push_joint_limit_utilization = [
            peak / max(limit, 1e-6)
            for peak, limit in zip(push_joint_peak_abs_tau_nm, GO2_TORQUE_LIMITS_NM)
        ]

    return {
        "duration_s": duration_s,
        "target_distance_m": target_distance_m,
        "backends": {
            "solver_backend": solver_backend_name,
            "planner_backend": planner_backend_name,
            "intent_active": active_intent is not None,
        },
        "planner": {
            "event_count": len(planner_events),
            "active_event_count": len(active_planner_events),
            "first_activation_time_s": (
                active_planner_events[0]["t"] if active_planner_events else None
            ),
            "target_distance_m": (
                active_intent["target_distance_m"]
                if active_intent is not None
                else None
            ),
            "target_takeoff_velocity_x_mps": (
                active_intent["target_takeoff_velocity_x_mps"]
                if active_intent is not None
                else None
            ),
            "target_takeoff_velocity_z_mps": (
                active_intent["target_takeoff_velocity_z_mps"]
                if active_intent is not None
                else None
            ),
            "target_takeoff_pitch_deg": (
                active_intent["target_takeoff_pitch_deg"]
                if active_intent is not None
                else None
            ),
            "target_landing_pitch_deg": (
                active_intent["target_landing_pitch_deg"]
                if active_intent is not None
                else None
            ),
            "crouch_duration_s": (
                active_intent["crouch_duration_s"]
                if active_intent is not None
                else None
            ),
            "push_duration_s": (
                active_intent["push_duration_s"]
                if active_intent is not None
                else None
            ),
            "estimated_flight_time_s": (
                active_intent["estimated_flight_time_s"]
                if active_intent is not None
                else None
            ),
            "landing_duration_s": (
                active_intent["landing_duration_s"]
                if active_intent is not None
                else None
            ),
            "settle_duration_s": (
                active_intent["settle_duration_s"]
                if active_intent is not None
                else None
            ),
            "planned_apex_height_m": (
                active_intent["planned_apex_height_m"]
                if active_intent is not None
                else None
            ),
        },
        "message_counts": {
            "controller_state": len(controller_samples),
            "sport_state": len(sport_samples),
            "lowcmd": len(lowcmd_times),
        },
        "rates_hz": {
            "controller_state": controller_rate_hz,
            "lowcmd": lowcmd_rate_hz,
        },
        "phase": {
            "segments": segments,
            "sequence": phase_sequence,
            "takeoff_time_s": takeoff_time_s,
            "touchdown_time_s": touchdown_time_s,
            "settle_time_s": settle_time_s,
            "has_flight_relapse_after_landing": has_flight_relapse_after_landing,
            "max_contact_count_in_flight": max_contact_count_in_flight,
        },
        "motion": {
            "initial_x_m": initial_x,
            "takeoff_x_m": takeoff_x,
            "touchdown_x_m": touchdown_x,
            "final_x_m": final_x,
            "total_distance_m": total_distance_m,
            "airborne_distance_m": airborne_distance_m,
            "post_touchdown_distance_m": post_touchdown_distance_m,
            "airborne_share": airborne_share,
            "mainly_airborne_translation": mainly_airborne_translation,
            "max_height_gain_m": max_height_gain_m,
            "max_forward_velocity_mps": max_forward_velocity_mps,
            "max_upward_velocity_mps": max_upward_velocity_mps,
            "min_vertical_velocity_mps": min_vertical_velocity_mps,
        },
        "command_effort": {
            "push_samples": len(push_effort_samples),
            "push_peak_abs_tau_nm": push_peak_abs_tau_nm,
            "push_mean_abs_tau_nm": push_mean_abs_tau_nm,
            "push_mean_abs_tau_ff_nm": push_mean_abs_tau_ff_nm,
            "push_feedforward_fraction": push_feedforward_fraction,
            "push_joint_peak_abs_tau_nm": push_joint_peak_abs_tau_nm,
            "push_joint_limit_utilization": push_joint_limit_utilization,
        },
    }


def print_summary(summary: Dict[str, Any]) -> None:
    backends = summary["backends"]
    planner = summary["planner"]
    phase = summary["phase"]
    motion = summary["motion"]
    rates = summary["rates_hz"]
    command_effort = summary["command_effort"]

    def fmt(value: Optional[float], digits: int = 3) -> str:
        if value is None:
            return "n/a"
        return f"{value:.{digits}f}"

    print("Jump trial summary")
    print(f"  target distance: {fmt(summary['target_distance_m'])} m")
    print(f"  duration: {fmt(summary['duration_s'])} s")
    print(
        "  backends: "
        f"solver={backends['solver_backend'] or 'n/a'} "
        f"planner={backends['planner_backend'] or 'n/a'} "
        f"intent_active={backends['intent_active']}"
    )
    if planner["target_takeoff_velocity_x_mps"] is not None:
        print(
            "  intent vx / vz / apex: "
            f"{fmt(planner['target_takeoff_velocity_x_mps'])} / "
            f"{fmt(planner['target_takeoff_velocity_z_mps'])} mps / "
            f"{fmt(planner['planned_apex_height_m'])} m"
        )
        print(
            "  intent crouch / push / flight: "
            f"{fmt(planner['crouch_duration_s'])} / "
            f"{fmt(planner['push_duration_s'])} / "
            f"{fmt(planner['estimated_flight_time_s'])} s"
        )
    print("  phase sequence: " + " -> ".join(phase["sequence"]))
    print(
        "  takeoff/touchdown/settle: "
        f"{fmt(phase['takeoff_time_s'])} / {fmt(phase['touchdown_time_s'])} / {fmt(phase['settle_time_s'])} s"
    )
    print(
        "  displacement total/airborne/post-touchdown: "
        f"{fmt(motion['total_distance_m'])} / {fmt(motion['airborne_distance_m'])} / "
        f"{fmt(motion['post_touchdown_distance_m'])} m"
    )
    print(
        "  airborne share: "
        f"{fmt(motion['airborne_share'] * 100.0 if motion['airborne_share'] is not None else None, 1)} %"
    )
    print(
        "  peak height gain / vx / vz+: "
        f"{fmt(motion['max_height_gain_m'])} m / {fmt(motion['max_forward_velocity_mps'])} mps / "
        f"{fmt(motion['max_upward_velocity_mps'])} mps"
    )
    print(
        "  controller rate / lowcmd rate: "
        f"{fmt(rates['controller_state'], 1)} / {fmt(rates['lowcmd'], 1)} Hz"
    )
    print(
        "  flight relapsed after landing: "
        f"{phase['has_flight_relapse_after_landing']}"
    )
    print(
        "  max contact count during flight: "
        f"{phase['max_contact_count_in_flight'] if phase['max_contact_count_in_flight'] is not None else 'n/a'}"
    )
    print(
        "  mainly airborne translation: "
        f"{motion['mainly_airborne_translation']}"
    )
    if command_effort["push_samples"] > 0:
        max_utilization = max(command_effort["push_joint_limit_utilization"])
        print(
            "  push torque peak / mean / max utilization: "
            f"{fmt(command_effort['push_peak_abs_tau_nm'])} Nm / "
            f"{fmt(command_effort['push_mean_abs_tau_nm'])} Nm / "
            f"{fmt(max_utilization * 100.0, 1)} %"
        )
        print(
            "  push tau_ff mean / feedforward share: "
            f"{fmt(command_effort['push_mean_abs_tau_ff_nm'])} Nm / "
            f"{fmt(command_effort['push_feedforward_fraction'] * 100.0 if command_effort['push_feedforward_fraction'] is not None else None, 1)} %"
        )


class TrialRecorder(Node):
    def __init__(self, start_time_s: float) -> None:
        super().__init__("go2_jump_trial_recorder")
        self.start_time_s = start_time_s
        self.controller_samples: List[Dict[str, Any]] = []
        self.sport_samples: List[Dict[str, Any]] = []
        self.lowcmd_times: List[float] = []
        self.lowcmd_samples: List[Dict[str, Any]] = []
        self.first_controller_time_s: Optional[float] = None
        self.latest_phase: Optional[str] = None
        self.latest_lowstate_q: Optional[List[float]] = None
        self.latest_lowstate_dq: Optional[List[float]] = None

        self.create_subscription(
            JumpControllerState,
            "/go2_jump/controller_state",
            self.on_controller_state,
            100,
        )
        self.create_subscription(
            SportModeState, "/sportmodestate", self.on_sport_state, 100
        )
        if LowCmd is not None:
            self.create_subscription(LowCmd, "/lowcmd", self.on_lowcmd, 200)
        if LowState is not None:
            self.create_subscription(LowState, "/lowstate", self.on_lowstate, 200)

    def now_s(self) -> float:
        return time.monotonic() - self.start_time_s

    def on_controller_state(self, msg: JumpControllerState) -> None:
        self.latest_phase = msg.phase
        if self.first_controller_time_s is None:
            self.first_controller_time_s = self.now_s()
        self.controller_samples.append(
            {
                "t": self.now_s(),
                "phase": msg.phase,
                "backend_name": msg.backend_name,
                "target_distance_m": msg.target_distance_m,
                "task_elapsed_s": msg.task_elapsed_s,
                "contact_count": int(msg.contact_count),
                "contact_override": bool(msg.contact_override),
                "desired_forward_velocity_mps": msg.desired_forward_velocity_mps,
                "desired_vertical_velocity_mps": msg.desired_vertical_velocity_mps,
                "desired_body_pitch_deg": msg.desired_body_pitch_deg,
                "body_pitch_deg": msg.body_pitch_deg,
                "forward_velocity_mps": msg.forward_velocity_mps,
                "vertical_velocity_mps": msg.vertical_velocity_mps,
                "foot_force_est": [float(value) for value in msg.foot_force_est],
                "foot_contact": [bool(value) for value in msg.foot_contact],
                "intent_active": bool(msg.intent_active),
                "intent": (
                    {
                        "task_id": msg.active_intent.task_id,
                        "planner_backend": msg.active_intent.planner_backend,
                        "target_distance_m": msg.active_intent.target_distance_m,
                        "target_takeoff_velocity_x_mps": msg.active_intent.target_takeoff_velocity_x_mps,
                        "target_takeoff_velocity_z_mps": msg.active_intent.target_takeoff_velocity_z_mps,
                        "target_takeoff_pitch_deg": msg.active_intent.target_takeoff_pitch_deg,
                        "target_landing_pitch_deg": msg.active_intent.target_landing_pitch_deg,
                        "crouch_duration_s": msg.active_intent.crouch_duration_s,
                        "push_duration_s": msg.active_intent.push_duration_s,
                        "estimated_flight_time_s": msg.active_intent.estimated_flight_time_s,
                        "landing_duration_s": msg.active_intent.landing_duration_s,
                        "settle_duration_s": msg.active_intent.settle_duration_s,
                        "horizon_duration_s": msg.active_intent.horizon_duration_s,
                        "crouch_height_offset_m": msg.active_intent.crouch_height_offset_m,
                        "push_height_offset_m": msg.active_intent.push_height_offset_m,
                        "flight_height_offset_m": msg.active_intent.flight_height_offset_m,
                        "landing_height_offset_m": msg.active_intent.landing_height_offset_m,
                        "leg_retraction_scale": msg.active_intent.leg_retraction_scale,
                        "landing_brace_scale": msg.active_intent.landing_brace_scale,
                        "front_push_foot_x_bias_m": msg.active_intent.front_push_foot_x_bias_m,
                        "rear_push_foot_x_bias_m": msg.active_intent.rear_push_foot_x_bias_m,
                        "front_landing_foot_x_bias_m": msg.active_intent.front_landing_foot_x_bias_m,
                        "rear_landing_foot_x_bias_m": msg.active_intent.rear_landing_foot_x_bias_m,
                        "swing_foot_height_m": msg.active_intent.swing_foot_height_m,
                        "planned_apex_height_m": msg.active_intent.planned_apex_height_m,
                    }
                    if msg.intent_active and msg.active_intent.valid
                    else None
                ),
            }
        )

    def on_sport_state(self, msg: SportModeState) -> None:
        self.sport_samples.append(
            {
                "t": self.now_s(),
                "x": float(msg.position[0]),
                "y": float(msg.position[1]),
                "z": float(msg.position[2]),
                "vx": float(msg.velocity[0]),
                "vy": float(msg.velocity[1]),
                "vz": float(msg.velocity[2]),
                "body_height": float(msg.body_height),
                "foot_force": [int(value) for value in msg.foot_force],
            }
        )

    def on_lowcmd(self, _msg: LowCmd) -> None:
        now_s = self.now_s()
        self.lowcmd_times.append(now_s)
        if self.latest_lowstate_q is None or self.latest_lowstate_dq is None:
            return

        command_torque_est_nm = []
        tau_ff_nm = []
        q_ref_rad = []
        dq_ref_radps = []
        kp = []
        kd = []
        for joint_index in range(GO2_CONTROLLED_JOINT_COUNT):
            motor_cmd = _msg.motor_cmd[joint_index]
            q_error = float(motor_cmd.q) - self.latest_lowstate_q[joint_index]
            dq_error = float(motor_cmd.dq) - self.latest_lowstate_dq[joint_index]
            tau_ff = float(motor_cmd.tau)
            q_ref_rad.append(float(motor_cmd.q))
            dq_ref_radps.append(float(motor_cmd.dq))
            kp.append(float(motor_cmd.kp))
            kd.append(float(motor_cmd.kd))
            command_torque_est_nm.append(
                float(motor_cmd.kp) * q_error + float(motor_cmd.kd) * dq_error + tau_ff
            )
            tau_ff_nm.append(tau_ff)

        self.lowcmd_samples.append(
            {
                "t": now_s,
                "phase": self.latest_phase,
                "command_torque_est_nm": command_torque_est_nm,
                "tau_ff_nm": tau_ff_nm,
                "q_ref_rad": q_ref_rad,
                "dq_ref_radps": dq_ref_radps,
                "kp": kp,
                "kd": kd,
            }
        )

    def on_lowstate(self, msg: LowState) -> None:
        self.latest_lowstate_q = [
            float(msg.motor_state[i].q) for i in range(GO2_CONTROLLED_JOINT_COUNT)
        ]
        self.latest_lowstate_dq = [
            float(msg.motor_state[i].dq) for i in range(GO2_CONTROLLED_JOINT_COUNT)
        ]


def main() -> int:
    parser = argparse.ArgumentParser(description="Record and summarize one jump trial.")
    parser.add_argument("--duration", type=float, default=6.0)
    parser.add_argument("--output-json", type=str, default="")
    parser.add_argument("--wait-for-controller-timeout", type=float, default=20.0)
    args = parser.parse_args()

    rclpy.init()
    start_time_s = time.monotonic()
    node = TrialRecorder(start_time_s)

    try:
        initial_deadline_s = start_time_s + args.wait_for_controller_timeout
        final_deadline_s: Optional[float] = None
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            now_s = time.monotonic()
            if node.first_controller_time_s is not None and final_deadline_s is None:
                final_deadline_s = now_s + args.duration
            if final_deadline_s is not None and now_s >= final_deadline_s:
                break
            if final_deadline_s is None and now_s >= initial_deadline_s:
                break
    finally:
        summary = compute_summary(
            node.controller_samples,
            node.sport_samples,
            node.lowcmd_times,
            node.lowcmd_samples,
            args.duration,
        )
        report = {
            "summary": summary,
            "controller_samples": node.controller_samples,
            "sport_samples": node.sport_samples,
            "lowcmd_times_s": node.lowcmd_times,
            "lowcmd_samples": node.lowcmd_samples,
        }
        if args.output_json:
            os.makedirs(os.path.dirname(args.output_json), exist_ok=True)
            with open(args.output_json, "w", encoding="utf-8") as handle:
                json.dump(report, handle, indent=2, ensure_ascii=False)
        print_summary(summary)
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

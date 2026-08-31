import json
import math
import threading
import time
from datetime import datetime
from pathlib import Path

import rclpy
from bw_interface.srv import (
    GetStatus,
    PauseRecord,
    PlaybackStart,
    PlaybackStop,
    ResumeRecord,
    StartRecord,
    StopRecord,
)
from control_msgs.action import FollowJointTrajectory, GripperCommand
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


LEFT = [f"A_left_Degree{i}_joint" for i in range(1, 8)]
RIGHT = [f"A_right_Degree{i}_joint" for i in range(1, 8)]
ARMS = LEFT + RIGHT
GRIPPER_JOINTS = ["A_left_Degree8_joint", "A_right_Degree8_joint"]
GRIPPER_TRAVEL = 0.04965


class MotionRecorderNode(Node):
    def __init__(self):
        super().__init__("motion_recorder")
        self.declare_parameter("feedback_topic", "joint_states")
        self.declare_parameter("sample_rate", 25.0)
        self.declare_parameter("output_directory", "motions")
        self.declare_parameter("initial_move_velocity", 0.15)
        self.declare_parameter("initial_move_min_duration", 3.0)
        self.declare_parameter("action_wait_timeout", 5.0)
        self.declare_parameter("trim_stationary_start", True)
        self.declare_parameter("stationary_arm_epsilon", 0.002)
        self.declare_parameter("stationary_gripper_epsilon", 0.02)

        self._lock = threading.RLock()
        self._state = "IDLE"
        self._samples = []
        self._record_started = 0.0
        self._paused_at = 0.0
        self._paused_duration = 0.0
        self._last_sample = 0.0
        self._latest = None
        self._stop_playback = threading.Event()
        self._goal_handles = []
        self._last_error = ""
        self._last_file = ""
        self._playback_progress = "idle"
        callbacks = ReentrantCallbackGroup()

        self.create_subscription(
            JointState, self.get_parameter("feedback_topic").value,
            self._on_joint_state, 20, callback_group=callbacks)
        self._arm_clients = {
            "left": ActionClient(
                self, FollowJointTrajectory,
                "arm_l_controller/follow_joint_trajectory", callback_group=callbacks),
            "right": ActionClient(
                self, FollowJointTrajectory,
                "arm_r_controller/follow_joint_trajectory", callback_group=callbacks),
        }
        self._gripper_clients = {
            "left": ActionClient(
                self, GripperCommand, "hand_l_controller/gripper_cmd",
                callback_group=callbacks),
            "right": ActionClient(
                self, GripperCommand, "hand_r_controller/gripper_cmd",
                callback_group=callbacks),
        }

        self.create_service(StartRecord, "motion/record/start", self._start_recording)
        self.create_service(PauseRecord, "motion/record/pause", self._pause_recording)
        self.create_service(ResumeRecord, "motion/record/resume", self._resume_recording)
        self.create_service(StopRecord, "motion/record/stop", self._stop_recording)
        self.create_service(GetStatus, "motion/status", self._get_status)
        self.create_service(PlaybackStart, "motion/playback/start", self._start_playback)
        self.create_service(PlaybackStop, "motion/playback/stop", self._stop_playback_service)
        self.get_logger().info("Motion recorder READY; source=/joint_states")

    def _on_joint_state(self, msg):
        values = dict(zip(msg.name, msg.position))
        required = ARMS + GRIPPER_JOINTS
        if not all(name in values and math.isfinite(values[name]) for name in required):
            return
        snapshot = {
            "arms": [float(values[name]) for name in ARMS],
            "grippers": [
                min(max(float(values[name]) / GRIPPER_TRAVEL, 0.0), 1.0)
                for name in GRIPPER_JOINTS
            ],
        }
        now = time.monotonic()
        with self._lock:
            self._latest = snapshot
            if self._state != "RECORDING":
                return
            period = 1.0 / max(float(self.get_parameter("sample_rate").value), 1.0)
            if now - self._last_sample < period:
                return
            self._last_sample = now
            self._samples.append({
                "time": now - self._record_started - self._paused_duration,
                **snapshot,
            })

    def _start_recording(self, _request, response):
        with self._lock:
            if self._state == "PLAYING":
                return self._response(response, False, "Cannot record during playback")
            if self._latest is None:
                return self._response(response, False, "No complete /joint_states feedback")
            self._samples = []
            self._record_started = time.monotonic()
            self._paused_duration = 0.0
            self._last_sample = 0.0
            self._state = "RECORDING"
            self._last_error = ""
        self.get_logger().info("Motion recording started")
        return self._response(response, True, "recording")

    def _pause_recording(self, _request, response):
        with self._lock:
            if self._state != "RECORDING":
                return self._response(response, False, "Recorder is not recording")
            self._paused_at = time.monotonic()
            self._state = "PAUSED"
        return self._response(response, True, "paused")

    def _resume_recording(self, _request, response):
        with self._lock:
            if self._state != "PAUSED":
                return self._response(response, False, "Recorder is not paused")
            self._paused_duration += time.monotonic() - self._paused_at
            self._last_sample = 0.0
            self._state = "RECORDING"
        return self._response(response, True, "recording")

    def _stop_recording(self, request, response):
        with self._lock:
            if self._state not in ("RECORDING", "PAUSED"):
                return self._response(response, False, "Recorder is not active")
            samples = list(self._samples)
            self._state = "IDLE"
        if len(samples) < 2:
            return self._response(response, False, "Need at least two recorded samples")
        try:
            path = self._save_recording(request.file_path, samples)
        except Exception as exc:
            return self._response(response, False, f"Save failed: {exc}")
        self.get_logger().info(f"Saved {len(samples)} samples to {path}")
        return self._response(response, True, path)

    def _save_recording(self, requested_path, samples):
        output_dir = Path(str(self.get_parameter("output_directory").value)).expanduser()
        if not output_dir.is_absolute():
            output_dir = Path.cwd() / output_dir
        output_dir.mkdir(parents=True, exist_ok=True)
        if requested_path:
            path = Path(requested_path).expanduser()
            if not path.is_absolute():
                path = output_dir / path
        else:
            path = output_dir / f"motion_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        if path.suffix.lower() != ".json":
            path = path.with_suffix(".json")
        path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "schema": "standard_motion_v1",
            "joint_names": ARMS,
            "gripper_names": ["left", "right"],
            "gripper_encoding": "urdf_position_normalized",
            "sample_rate": float(self.get_parameter("sample_rate").value),
            "samples": samples,
        }
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_text(json.dumps(payload, indent=2), encoding="utf-8")
        temporary.replace(path)
        return str(path.resolve())

    def _start_playback(self, request, response):
        with self._lock:
            if self._state != "IDLE":
                return self._response(response, False, f"Node is {self._state}")
        try:
            path = self._resolve_recording_path(request.file_path)
            data = self._load_recording(path)
        except Exception as exc:
            with self._lock:
                self._last_error = str(exc)
            return self._response(response, False, f"Load failed: {exc}")
        speed = float(request.speed) if request.speed > 0.0 else 1.0
        self._stop_playback.clear()
        with self._lock:
            self._state = "PLAYING"
            self._last_error = ""
            self._last_file = str(path.resolve())
            self._playback_progress = f"loaded {len(data)} samples"
        threading.Thread(
            target=self._play_worker,
            args=(data, speed, bool(request.loop)), daemon=True).start()
        return self._response(response, True, "playback started")

    def _resolve_recording_path(self, file_path):
        if not file_path:
            raise ValueError("file_path is empty")
        path = Path(file_path).expanduser()
        if not path.is_absolute():
            output_dir = Path(str(self.get_parameter("output_directory").value)).expanduser()
            if not output_dir.is_absolute():
                output_dir = Path.cwd() / output_dir
            path = output_dir / path
        if path.suffix.lower() != ".json":
            path = path.with_suffix(".json")
        return path

    def _load_recording(self, file_path):
        data = json.loads(Path(file_path).read_text(encoding="utf-8"))
        if data.get("schema") != "standard_motion_v1":
            raise ValueError("unsupported motion schema")
        if data.get("joint_names") != ARMS:
            raise ValueError("recorded arm joint order does not match this robot")
        samples = data.get("samples") or []
        if len(samples) < 2:
            raise ValueError("recording has fewer than two samples")
        previous = -1.0
        for sample in samples:
            stamp = float(sample["time"])
            arms = sample["arms"]
            grippers = sample["grippers"]
            if (stamp < previous or len(arms) != 14 or len(grippers) != 2 or
                    not all(math.isfinite(float(value)) for value in arms + grippers)):
                raise ValueError("recording contains an invalid sample")
            previous = stamp
        return self._trim_stationary_prefix(samples)

    def _trim_stationary_prefix(self, samples):
        if not bool(self.get_parameter("trim_stationary_start").value):
            return samples
        first = samples[0]
        arm_epsilon = float(self.get_parameter("stationary_arm_epsilon").value)
        gripper_epsilon = float(
            self.get_parameter("stationary_gripper_epsilon").value)
        moving_index = None
        for index, sample in enumerate(samples[1:], start=1):
            arm_delta = max(abs(float(a) - float(b))
                            for a, b in zip(sample["arms"], first["arms"]))
            gripper_delta = max(abs(float(a) - float(b))
                                for a, b in zip(sample["grippers"], first["grippers"]))
            if arm_delta > arm_epsilon or gripper_delta > gripper_epsilon:
                moving_index = index
                break
        if moving_index is None or moving_index <= 1:
            return samples
        period = 1.0 / max(float(self.get_parameter("sample_rate").value), 1.0)
        shift = float(samples[moving_index]["time"]) - period
        trimmed = [{**first, "time": 0.0}]
        trimmed.extend(
            {**sample, "time": max(period, float(sample["time"]) - shift)}
            for sample in samples[moving_index:])
        self.get_logger().info(
            f"Trimmed {float(samples[moving_index]['time']):.1f}s stationary "
            f"prefix ({moving_index - 1} samples)")
        return trimmed

    def _play_worker(self, samples, speed, loop):
        try:
            while rclpy.ok() and not self._stop_playback.is_set():
                with self._lock:
                    current = dict(self._latest) if self._latest else None
                if current is None:
                    raise RuntimeError("hardware feedback is unavailable")
                initial_duration = self._initial_duration(current["arms"], samples[0]["arms"])
                with self._lock:
                    self._playback_progress = (
                        f"moving to recorded start pose ({initial_duration:.1f}s)")
                goals = {
                    "left": self._arm_goal(LEFT, current["arms"][:7], samples, 0, initial_duration, speed),
                    "right": self._arm_goal(RIGHT, current["arms"][7:], samples, 7, initial_duration, speed),
                }
                handles = self._send_arm_goals(goals)
                self._goal_handles = list(handles.values())
                with self._lock:
                    self._playback_progress = "left/right arm goals accepted"
                gripper_thread = threading.Thread(
                    target=self._play_grippers,
                    args=(samples, initial_duration, speed), daemon=True)
                gripper_thread.start()
                results = {
                    side: self._wait_result(handle) for side, handle in handles.items()
                }
                success = all(status == 4 for status in results.values())
                with self._lock:
                    self._playback_progress = f"arm result status={results}"
                    if not success:
                        self._last_error = f"arm playback failed: status={results}"
                gripper_thread.join(timeout=1.0)
                self._goal_handles = []
                if not success or self._stop_playback.is_set() or not loop:
                    break
            self.get_logger().info("Motion playback finished")
        except Exception as exc:
            with self._lock:
                self._last_error = str(exc)
                self._playback_progress = "failed"
            self.get_logger().error(f"Motion playback failed: {exc}")
        finally:
            with self._lock:
                self._state = "IDLE"

    def _initial_duration(self, current, target):
        distance = max(abs(float(a) - float(b)) for a, b in zip(current, target))
        velocity = max(float(self.get_parameter("initial_move_velocity").value), 0.01)
        return max(
            float(self.get_parameter("initial_move_min_duration").value),
            1.5 * distance / velocity)

    @staticmethod
    def _arm_goal(names, current, samples, offset, initial_duration, speed):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(names)
        start = JointTrajectoryPoint()
        start.positions = [float(value) for value in current]
        goal.trajectory.points.append(start)
        first_stamp = float(samples[0]["time"])
        for sample in samples:
            point = JointTrajectoryPoint()
            point.positions = [float(value) for value in sample["arms"][offset:offset + 7]]
            stamp = initial_duration + (float(sample["time"]) - first_stamp) / speed
            point.time_from_start.sec = int(stamp)
            point.time_from_start.nanosec = int((stamp - int(stamp)) * 1e9)
            goal.trajectory.points.append(point)
        return goal

    def _send_arm_goals(self, goals):
        timeout = float(self.get_parameter("action_wait_timeout").value)
        for side, client in self._arm_clients.items():
            if not client.wait_for_server(timeout_sec=timeout):
                raise RuntimeError(f"{side} arm action server is unavailable")
        futures = {side: self._arm_clients[side].send_goal_async(goal) for side, goal in goals.items()}
        handles = {side: self._wait_future(future, timeout) for side, future in futures.items()}
        rejected = [side for side, handle in handles.items()
                    if not handle or not handle.accepted]
        if rejected:
            raise RuntimeError(
                f"arm playback goal rejected by bridge: {','.join(rejected)}; "
                "check that hardware bridge status is READY")
        return handles

    def _play_grippers(self, samples, initial_duration, speed):
        started = time.monotonic()
        first_stamp = float(samples[0]["time"])
        previous = [None, None]
        for sample in samples:
            due = initial_duration + (float(sample["time"]) - first_stamp) / speed
            while not self._stop_playback.is_set() and time.monotonic() - started < due:
                time.sleep(0.01)
            if self._stop_playback.is_set():
                return
            for index, side in enumerate(("left", "right")):
                target = min(max(float(sample["grippers"][index]), 0.0), 1.0)
                if previous[index] is None or abs(target - previous[index]) >= 0.02:
                    self._send_gripper(side, target)
                    previous[index] = target

    def _send_gripper(self, side, normalized):
        client = self._gripper_clients[side]
        if not client.wait_for_server(timeout_sec=1.0):
            raise RuntimeError(f"{side} gripper action server unavailable")
        goal = GripperCommand.Goal()
        goal.command.position = normalized * GRIPPER_TRAVEL
        future = client.send_goal_async(goal)
        handle = self._wait_future(future, 2.0)
        if not handle or not handle.accepted:
            raise RuntimeError(f"{side} gripper goal rejected")

    def _wait_result(self, handle):
        future = handle.get_result_async()
        result = self._wait_future(future, 3600.0)
        return int(result.status) if result is not None else -1

    @staticmethod
    def _wait_future(future, timeout):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        return future.result() if future.done() else None

    def _stop_playback_service(self, _request, response):
        self._stop_playback.set()
        for handle in list(self._goal_handles):
            handle.cancel_goal_async()
        return self._response(response, True, "playback stop requested")

    def _get_status(self, _request, response):
        with self._lock:
            response.status = (
                f"{self._state}; samples={len(self._samples)}; "
                f"file={self._last_file or '-'}; "
                f"progress={self._playback_progress}; "
                f"error={self._last_error or '-'}")
        return response

    @staticmethod
    def _response(response, success, message):
        response.success = bool(success)
        response.message = str(message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MotionRecorderNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

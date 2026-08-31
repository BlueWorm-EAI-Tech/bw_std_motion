import math
import threading
import time

import rclpy
from bw_serial.srv import SetCtrlSrc
from control_msgs.action import FollowJointTrajectory, GripperCommand
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState


LEFT = [f"A_left_Degree{i}_joint" for i in range(1, 8)]
RIGHT = [f"A_right_Degree{i}_joint" for i in range(1, 8)]
JOINTS = LEFT + RIGHT
FEEDBACK_NAMES = [
    "left_shoulder_pitch_joint", "left_shoulder_yaw_joint",
    "left_shoulder_roll_joint", "left_elbow_pitch_joint",
    "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_yaw_joint",
    "right_shoulder_roll_joint", "right_elbow_pitch_joint",
    "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
]
FULL_STATE_JOINTS = [
    "D_left_joint", "D_right_joint", "D_behind_joint", "C_joint",
    *LEFT, "A_left_Degree8_joint", "A_left_Degree9_joint",
    *RIGHT, "A_right_Degree8_joint", "A_right_Degree9_joint",
    "H_Degree1_joint", "H_Degree2_joint", "H_Degree3_joint",
]


class TrajectoryBridge(Node):
    def __init__(self):
        super().__init__("trajectory_bridge")
        for name, default in (
            ("publish_rate", 100.0), ("feedback_timeout", 0.5),
            ("goal_tolerance", 0.035), ("goal_time_tolerance", 2.0),
            ("max_joint_velocity", 0.35), ("auto_enable", True),
            ("control_flag", 0x19),
            ("home_on_start", True), ("home_grippers_open", True),
            ("home_velocity", 0.15),
            ("home_min_duration", 3.0), ("home_timeout", 8.0),
            ("home_tolerance", 0.035), ("tracking_error_fault", 0.6),
            ("tracking_error_fault_dwell", 0.3),
            ("gripper_travel", 0.04965), ("gripper_tolerance", 0.002),
            ("gripper_timeout", 3.0),
            ("home_positions", [0.0] * len(JOINTS)),
        ):
            self.declare_parameter(name, default)

        self._lock = threading.RLock()
        self._command = dict.fromkeys(JOINTS, 0.0)
        self._velocity = dict.fromkeys(JOINTS, 0.0)
        self._feedback = {}
        self._feedback_time = 0.0
        self._active = set()
        self._active_grippers = set()
        self._gripper_command = {"left": 0.0, "right": 0.0}
        self._gripper_feedback = {"left": 0.0, "right": 0.0}
        self._enable_pending = False
        self._enabled = False
        self._state = "WAIT_FEEDBACK"
        self._home_start = []
        self._home_target = []
        self._home_started = 0.0
        self._home_duration = 0.0
        self._home_deadline = 0.0
        self._tracking_fault_started = 0.0
        callbacks = ReentrantCallbackGroup()
        self._command_pub = self.create_publisher(
            JointState, "Teleop/joint_angle_solution/smooth", 10)
        self._state_pub = self.create_publisher(JointState, "joint_states", 20)
        self._gripper_pub = self.create_publisher(
            JointState, "Teleop/gripper_pos", 10)
        self.create_subscription(
            JointState, "joint_states_fdb_V3", self._on_feedback, 20,
            callback_group=callbacks)
        rate = float(self.get_parameter("publish_rate").value)
        self.create_timer(1.0 / rate, self._publish_command, callback_group=callbacks)
        self._enable_client = self.create_client(
            SetCtrlSrc, "set_ctrl_src", callback_group=callbacks)
        self.create_timer(0.5, self._try_enable, callback_group=callbacks)
        self._actions = [
            self._make_action("arm_l_controller", "left", LEFT, callbacks),
            self._make_action("arm_r_controller", "right", RIGHT, callbacks),
        ]
        self._gripper_actions = [
            self._make_gripper_action("hand_l_controller", "left", callbacks),
            self._make_gripper_action("hand_r_controller", "right", callbacks),
        ]
        self.get_logger().info("Waiting for V3 hardware feedback before accepting trajectories")

    def _make_action(self, controller, group, joints, callbacks):
        return ActionServer(
            self, FollowJointTrajectory,
            f"{controller}/follow_joint_trajectory",
            execute_callback=lambda handle: self._execute(handle, group, joints),
            goal_callback=lambda goal: self._validate_goal(goal, group, joints),
            cancel_callback=lambda _handle: CancelResponse.ACCEPT,
            callback_group=callbacks)

    def _make_gripper_action(self, controller, side, callbacks):
        return ActionServer(
            self, GripperCommand, f"{controller}/gripper_cmd",
            execute_callback=lambda handle: self._execute_gripper(handle, side),
            goal_callback=lambda goal: self._validate_gripper_goal(goal, side),
            cancel_callback=lambda _handle: CancelResponse.ACCEPT,
            callback_group=callbacks)

    def _on_feedback(self, msg):
        raw = dict(zip(msg.name, msg.position))
        if not all(name in raw and math.isfinite(raw[name]) for name in FEEDBACK_NAMES):
            self.get_logger().warning(
                "Incomplete V3 arm feedback ignored", throttle_duration_sec=2.0)
            return
        mapped = dict(zip(JOINTS, (raw[name] for name in FEEDBACK_NAMES)))
        with self._lock:
            self._feedback = mapped
            self._feedback_time = time.monotonic()
            self._gripper_feedback["left"] = min(
                max(raw.get("left_gripper_joint", 0.0), 0.0), 1.0)
            self._gripper_feedback["right"] = min(
                max(raw.get("right_gripper_joint", 0.0), 0.0), 1.0)
            # Before enabling, command exactly the measured pose so arm enable cannot jump.
            # Once homing starts, keep command targets latched; following noisy feedback here
            # creates a slow uncommanded drift while MoveIt is only planning.
            if self._state in ("WAIT_FEEDBACK", "ENABLING"):
                for name in JOINTS:
                    self._command[name] = mapped[name]
                self._gripper_command.update(self._gripper_feedback)
        state = JointState()
        state.header.stamp = self.get_clock().now().to_msg()
        gripper_scale = float(self.get_parameter("gripper_travel").value)
        left_gripper = min(max(raw.get("left_gripper_joint", 0.0), 0.0), 1.0)
        right_gripper = min(max(raw.get("right_gripper_joint", 0.0), 0.0), 1.0)
        full_state = {
            "D_left_joint": 0.0,
            "D_right_joint": 0.0,
            "D_behind_joint": 0.0,
            "C_joint": min(max(raw.get("pelvis_joint", 0.0) / 1000.0, -0.231), 0.231),
            **mapped,
            "A_left_Degree8_joint": left_gripper * gripper_scale,
            "A_left_Degree9_joint": left_gripper * gripper_scale,
            "A_right_Degree8_joint": right_gripper * gripper_scale,
            "A_right_Degree9_joint": right_gripper * gripper_scale,
            "H_Degree1_joint": raw.get("head_yaw_joint", 0.0),
            "H_Degree2_joint": 0.0,
            "H_Degree3_joint": raw.get("head_pitch_joint", 0.0),
        }
        state.name = FULL_STATE_JOINTS
        state.position = [full_state[name] for name in FULL_STATE_JOINTS]
        self._state_pub.publish(state)

    def _feedback_fresh(self):
        with self._lock:
            return bool(self._feedback) and time.monotonic() - self._feedback_time <= float(
                self.get_parameter("feedback_timeout").value)

    def _publish_command(self):
        if not self._feedback_fresh():
            return
        with self._lock:
            self._advance_homing(time.monotonic())
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = JOINTS
            msg.position = [self._command[name] for name in JOINTS]
            msg.velocity = [max(abs(self._velocity[name]), 0.01) for name in JOINTS]
        self._command_pub.publish(msg)
        gripper = JointState()
        gripper.header.stamp = msg.header.stamp
        gripper.name = ["left_gripper_joint", "right_gripper_joint"]
        with self._lock:
            gripper.position = [
                self._gripper_command["left"], self._gripper_command["right"]]
        self._gripper_pub.publish(gripper)

    def _try_enable(self):
        if (self._enabled or self._enable_pending or
                self._state == "FAULT" or
                not self.get_parameter("auto_enable").value or
                not self._feedback_fresh() or not self._enable_client.service_is_ready()):
            return
        request = SetCtrlSrc.Request()
        request.value = int(self.get_parameter("control_flag").value)
        self._enable_pending = True
        self._state = "ENABLING"
        self._enable_client.call_async(request).add_done_callback(self._enable_done)

    def _enable_done(self, future):
        self._enable_pending = False
        try:
            response = future.result()
            self._enabled = bool(response.success)
            log = self.get_logger().info if response.success else self.get_logger().error
            log(response.message)
            if response.success:
                self._start_homing()
            else:
                self._state = "FAULT"
        except Exception as exc:
            self._state = "FAULT"
            self.get_logger().error(f"set_ctrl_src failed: {exc}")

    def _start_homing(self):
        with self._lock:
            if not self.get_parameter("home_on_start").value:
                self._state = "READY"
                self.get_logger().info("V3 bridge READY (startup homing disabled)")
                return
            target = list(self.get_parameter("home_positions").value)
            if len(target) != len(JOINTS) or not all(math.isfinite(value) for value in target):
                self._state = "FAULT"
                self.get_logger().error("Invalid home_positions; expected 14 finite values")
                return
            self._home_start = [self._feedback[name] for name in JOINTS]
            self._home_target = target
            if self.get_parameter("home_grippers_open").value:
                # V3 normalized position 0.0 is fully open.
                self._gripper_command["left"] = 0.0
                self._gripper_command["right"] = 0.0
            max_distance = max(abs(b - a) for a, b in zip(self._home_start, target))
            velocity = max(0.01, float(self.get_parameter("home_velocity").value))
            self._home_duration = max(
                float(self.get_parameter("home_min_duration").value),
                1.5 * max_distance / velocity,
            )
            self._home_started = time.monotonic()
            self._home_deadline = (
                self._home_started + self._home_duration +
                float(self.get_parameter("home_timeout").value)
            )
            self._tracking_fault_started = 0.0
            self._active.update(("left", "right"))
            self._state = "HOMING"
            gripper_mode = (
                "open" if self.get_parameter("home_grippers_open").value
                else "hold-current")
            self.get_logger().warning(
                f"Startup homing started; duration={self._home_duration:.1f}s; "
                f"grippers={gripper_mode}")

    def _advance_homing(self, now):
        if self._state != "HOMING":
            return
        elapsed = now - self._home_started
        ratio = min(1.0, max(0.0, elapsed / self._home_duration))
        blend = ratio * ratio * (3.0 - 2.0 * ratio)
        blend_rate = 6.0 * ratio * (1.0 - ratio) / self._home_duration
        for name, start, target in zip(JOINTS, self._home_start, self._home_target):
            self._command[name] = start + (target - start) * blend
            self._velocity[name] = abs(target - start) * blend_rate

        max_error = max(
            abs(self._command[name] - self._feedback[name]) for name in JOINTS)
        fault_limit = float(self.get_parameter("tracking_error_fault").value)
        if max_error > fault_limit:
            if self._tracking_fault_started == 0.0:
                self._tracking_fault_started = now
            elif now - self._tracking_fault_started >= float(
                    self.get_parameter("tracking_error_fault_dwell").value):
                self._abort_homing(
                    f"tracking error {max_error:.3f} rad exceeded {fault_limit:.3f} rad")
                return
        else:
            self._tracking_fault_started = 0.0

        if ratio < 1.0:
            return
        tolerance = float(self.get_parameter("home_tolerance").value)
        target_error = max(
            abs(self._feedback[name] - target)
            for name, target in zip(JOINTS, self._home_target))
        if target_error <= tolerance:
            self._state = "READY"
            self._active.difference_update(("left", "right"))
            for name in JOINTS:
                self._velocity[name] = 0.0
            self.get_logger().info(
                f"Startup homing complete; V3 bridge READY (max error={target_error:.3f} rad)")
        elif now >= self._home_deadline:
            self._abort_homing(
                f"timeout waiting for home pose (max error={target_error:.3f} rad)")

    def _abort_homing(self, reason):
        self._state = "FAULT"
        self._active.difference_update(("left", "right"))
        for name in JOINTS:
            self._velocity[name] = 0.0
        self.get_logger().error("Startup homing aborted: " + reason)
        request = SetCtrlSrc.Request()
        request.value = 0x01
        if self._enable_client.service_is_ready():
            self._enable_client.call_async(request)
        self._enabled = False

    def _validate_goal(self, request, group, joints):
        trajectory = request.trajectory
        if (self._state != "READY" or not self._feedback_fresh() or not trajectory.points or
                set(trajectory.joint_names) != set(joints)):
            self.get_logger().warning(
                f"Rejecting {group} trajectory while bridge state is {self._state}",
                throttle_duration_sec=1.0)
            return GoalResponse.REJECT
        previous = -1.0
        for point in trajectory.points:
            current = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            if (current <= previous or len(point.positions) != len(joints) or
                    not all(math.isfinite(value) for value in point.positions)):
                return GoalResponse.REJECT
            previous = current
        with self._lock:
            if group in self._active:
                self.get_logger().warning(f"Rejecting {group} trajectory: controller is busy")
                return GoalResponse.REJECT
        self.get_logger().info(
            f"Accepted {group} Execute trajectory with {len(trajectory.points)} points")
        return GoalResponse.ACCEPT

    def _validate_gripper_goal(self, request, side):
        travel = float(self.get_parameter("gripper_travel").value)
        position = request.command.position
        if (self._state != "READY" or not self._feedback_fresh() or
                not math.isfinite(position) or position < 0.0 or position > travel + 1e-6):
            self.get_logger().warning(
                f"Rejecting {side} gripper goal position={position:.5f}m in state {self._state}")
            return GoalResponse.REJECT
        with self._lock:
            if side in self._active_grippers:
                return GoalResponse.REJECT
        self.get_logger().info(f"Accepted {side} gripper goal position={position:.5f}m")
        return GoalResponse.ACCEPT

    def _execute_gripper(self, handle, side):
        result = GripperCommand.Result()
        travel = float(self.get_parameter("gripper_travel").value)
        tolerance = float(self.get_parameter("gripper_tolerance").value)
        target_m = min(max(handle.request.command.position, 0.0), travel)
        target_normalized = target_m / travel
        with self._lock:
            self._active_grippers.add(side)
            self._gripper_command[side] = target_normalized
        self.get_logger().info(
            f"Executing {side} gripper on V3 hardware: {target_normalized:.3f}")
        deadline = time.monotonic() + float(self.get_parameter("gripper_timeout").value)
        try:
            while self._feedback_fresh() and time.monotonic() < deadline:
                with self._lock:
                    current_normalized = self._gripper_feedback[side]
                current_m = current_normalized * travel
                feedback = GripperCommand.Feedback()
                feedback.position = current_m
                feedback.effort = 0.0
                feedback.stalled = False
                feedback.reached_goal = abs(current_m - target_m) <= tolerance
                handle.publish_feedback(feedback)
                if handle.is_cancel_requested:
                    with self._lock:
                        self._gripper_command[side] = current_normalized
                    handle.canceled()
                    result.position = current_m
                    result.effort = 0.0
                    result.stalled = False
                    result.reached_goal = False
                    return result
                if feedback.reached_goal:
                    handle.succeed()
                    result.position = current_m
                    result.effort = 0.0
                    result.stalled = False
                    result.reached_goal = True
                    self.get_logger().info(f"{side} gripper goal reached")
                    return result
                time.sleep(0.02)
            with self._lock:
                current_m = self._gripper_feedback[side] * travel
            handle.abort()
            result.position = current_m
            result.effort = 0.0
            result.stalled = False
            result.reached_goal = False
            self.get_logger().error(f"{side} gripper goal timed out")
            return result
        finally:
            with self._lock:
                self._active_grippers.discard(side)

    def _execute(self, handle, group, joints):
        result = FollowJointTrajectory.Result()
        trajectory = handle.request.trajectory
        order = [trajectory.joint_names.index(name) for name in joints]
        with self._lock:
            self._active.add(group)
            start = [self._command[name] for name in joints]
        self.get_logger().info(f"Executing {group} trajectory on V3 hardware")
        started = time.monotonic()
        segment_start = 0.0
        try:
            for point in trajectory.points:
                segment_end = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
                target = [point.positions[index] for index in order]
                duration = segment_end - segment_start
                if duration <= 1e-9:
                    with self._lock:
                        for name, value in zip(joints, target):
                            self._command[name] = value
                            self._velocity[name] = 0.0
                    start, segment_start = target, segment_end
                    continue
                while True:
                    if handle.is_cancel_requested:
                        handle.canceled()
                        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                        return result
                    if not self._feedback_fresh():
                        handle.abort()
                        result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                        result.error_string = "V3 feedback timeout"
                        return result
                    elapsed = time.monotonic() - started
                    ratio = min(1.0, max(0.0, (elapsed - segment_start) / duration))
                    values = [a + (b - a) * ratio for a, b in zip(start, target)]
                    speeds = [(b - a) / duration for a, b in zip(start, target)]
                    limit = float(self.get_parameter("max_joint_velocity").value)
                    with self._lock:
                        for name, value, speed in zip(joints, values, speeds):
                            self._command[name] = value
                            self._velocity[name] = min(abs(speed), limit)
                    if ratio >= 1.0:
                        break
                    time.sleep(0.005)
                start, segment_start = target, segment_end

            deadline = time.monotonic() + float(
                self.get_parameter("goal_time_tolerance").value)
            tolerance = float(self.get_parameter("goal_tolerance").value)
            while self._feedback_fresh() and time.monotonic() < deadline:
                with self._lock:
                    error = max(abs(self._feedback[name] - value)
                                for name, value in zip(joints, start))
                if error <= tolerance:
                    handle.succeed()
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    self.get_logger().info(
                        f"{group} hardware trajectory complete (max error={error:.3f} rad)")
                    return result
                time.sleep(0.02)
            handle.abort()
            result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
            result.error_string = "Hardware failed to reach the planned target"
            self.get_logger().error(
                f"{group} hardware trajectory aborted: {result.error_string}")
            return result
        finally:
            with self._lock:
                self._active.discard(group)
                for name in joints:
                    self._velocity[name] = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryBridge()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

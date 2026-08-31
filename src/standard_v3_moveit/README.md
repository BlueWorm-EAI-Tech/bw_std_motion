# Standard V3 MoveIt 真机执行

该功能将 MoveIt 的 `FollowJointTrajectory` 轨迹转换为 V3 通信节点订阅的
`Teleop/joint_angle_solution/smooth`，并将真机反馈 `joint_states_fdb_V3` 映射到
MoveIt/RViz 使用的 `joint_states`。

## 构建

V3 串口驱动及其接口依赖已经包含在当前工作区，直接构建即可：

```zsh
source /opt/ros/humble/setup.zsh
cd /home/lanchong/standrad_movelt_motion
colcon build --symlink-install
source install/setup.zsh
```

如果系统还没有 MoveIt 2，需要先安装 `ros-humble-moveit`。

## 启动与操作

释放急停，清空机械臂工作范围，并确认串口设备名：

默认假硬件演示（可拖动交互球、规划和模拟执行）：

```zsh
ros2 launch robot_movelt_config demo.launch.py
```

使用 V3 真机作为执行后端：

```zsh
ros2 launch robot_movelt_config demo.launch.py \
  hardware:=v3 port_name:=/dev/ttyACM0
```

启动时节点先读取 V3 真机反馈并保持当前位置，再使能双臂，以 `0.15 rad/s` 的保守速度
平滑回到 MoveIt 的双臂零位。日志出现 `V3 bridge READY` 后，RViz 中才允许执行规划轨迹。
点击 **Plan** 只生成轨迹，点击 **Execute** 后真机执行，RViz 继续显示真机反馈。
执行时终端应依次出现 `Accepted ... Execute trajectory`、`Executing ... on V3 hardware`
和 `hardware trajectory complete`。只有点击 **Execute** 才会出现这些日志并改变保持目标。
左右夹爪分别通过 `hand_l_controller/gripper_cmd` 和
`hand_r_controller/gripper_cmd` 执行，MoveIt 的 `0~0.04965 m` 行程会转换为 V3 的
`0~1` 命令，并使用真机夹爪反馈确认到位。

如需只同步当前真机姿态、跳过自动回初始位：

```zsh
ros2 launch robot_movelt_config demo.launch.py \
  hardware:=v3 home_on_start:=false
```

在 RViz 的 MotionPlanning 面板选择 `arm_l` 或 `arm_r`，拖动末端交互球。先按
**Plan** 检查轨迹，再按 **Execute**；规划成功后真机会按轨迹运动，RViz 机器人状态由
真机反馈驱动。

首次联调或只检查反馈时可关闭自动使能：

```zsh
ros2 launch robot_movelt_config demo.launch.py \
  hardware:=v3 port_name:=/dev/ttyACM0 auto_enable:=false
ros2 service call /set_ctrl_src bw_serial/srv/SetCtrlSrc "{value: 25}"
```

启动后应满足：

```zsh
ros2 topic hz /joint_states_fdb_V3
ros2 topic hz /joint_states
ros2 action list | grep follow_joint_trajectory
```

桥接节点在 V3 反馈缺失或超过 0.5 秒未更新时拒绝/中止轨迹，不会在未知姿态下发送运动。

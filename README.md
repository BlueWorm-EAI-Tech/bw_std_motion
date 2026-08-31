# Standard MoveIt 运动控制工作空间

本工作空间用于 Standard 双臂机器人在 ROS 2 MoveIt 2 中进行模型显示、运动规划、
仿真执行和 V3 真机同步控制，并提供真机动作录制与回放功能。

主要入口是：

```zsh
ros2 launch robot_movelt_config demo.launch.py
```

不指定参数时使用模拟硬件；添加 `hardware:=v3` 后连接 V3 实际机械臂。

## 实现的功能

- 在 RViz 中加载 Standard v1 或 v2 机器人模型。
- 使用 RViz 交互球设置左右臂目标位姿。
- 使用 MoveIt 进行轨迹规划和碰撞检查。
- 在模拟硬件中执行规划轨迹。
- 通过 V3 串口协议读取实际机械臂反馈。
- 将 MoveIt 轨迹发送给实际机械臂，并在 RViz 中同步显示真机运动。
- 真机启动时自动回到初始位置。
- 录制双臂运动并保存为 JSON 文件。
- 将保存的动作按原始时序回放到实际机械臂。

## 代码框架

```text
standrad_movelt_motion/
├── README.md
├── motion.zsh                         # 录制和回放快捷入口
└── src/
    ├── standrad_description/          # 机器人 URDF、网格模型和显示配置
    ├── robot_movelt_config/           # MoveIt、RViz、控制器及主启动文件
    ├── standard_v3_moveit/            # MoveIt 与 V3 真机之间的轨迹桥接
    ├── standard_motion_recorder/      # 动作录制、保存和回放
    ├── bw_serial/                     # V3 串口通信和协议解析
    ├── bw_interface/                  # ROS 消息与服务接口定义
    └── bw_utils/                      # 通信节点使用的公共工具
```

### 关键文件

| 文件 | 作用 |
| --- | --- |
| `src/robot_movelt_config/launch/demo.launch.py` | 整个系统的主启动文件，根据参数选择模拟或 V3 真机 |
| `src/robot_movelt_config/config/moveit.rviz` | RViz 的 MoveIt 面板和显示配置 |
| `src/robot_movelt_config/config/standard.srdf.xacro` | MoveIt 规划组、末端和碰撞配置 |
| `src/robot_movelt_config/config/kinematics.yaml` | 运动学求解器配置 |
| `src/robot_movelt_config/config/joint_limits.yaml` | MoveIt 关节速度和位置限制 |
| `src/standrad_description/urdf/` | v1、v2 机器人 URDF 模型 |
| `src/standrad_description/meshes/` | RViz 使用的机器人 STL 网格 |
| `src/standard_v3_moveit/standard_v3_moveit/trajectory_bridge.py` | 接收 MoveIt action、下发 V3 命令、发布真机状态 |
| `src/standard_v3_moveit/config/bridge.yaml` | 真机反馈超时、归位速度、容差等参数 |
| `src/bw_serial/src/mantis_comm_node.cpp` | 串口收发和 V3 协议通信节点 |
| `src/standard_motion_recorder/standard_motion_recorder/motion_recorder_node.py` | 动作采样、文件保存和 action 回放 |
| `src/standard_motion_recorder/config/recorder.yaml` | 录制频率、存储目录及回放参数 |
| `src/standard_motion_recorder/motions/` | 所有录制动作文件的统一存放目录 |
| `motion.zsh` | 自动加载当前工作区后调用录制控制命令 |

## 环境要求

推荐环境：

- Ubuntu 22.04
- ROS 2 Humble
- MoveIt 2
- zsh
- Python 3
- colcon
- V3 真机模式需要可用的串口设备，例如 `/dev/ttyACM0`

安装 ROS 依赖前，确保系统已经安装 ROS 2 Humble。常用依赖可通过以下命令安装：

```zsh
sudo apt update
sudo apt install ros-humble-moveit ros-humble-rviz2 python3-colcon-common-extensions
```

也可以让 `rosdep` 根据各功能包的 `package.xml` 安装依赖：

```zsh
cd /home/lanchong/standrad_movelt_motion
source /opt/ros/humble/setup.zsh
rosdep install --from-paths src --ignore-src -r -y
```

## 构建工作空间

首次使用或修改代码后执行：

```zsh
cd /home/lanchong/standrad_movelt_motion
source /opt/ros/humble/setup.zsh
colcon build --symlink-install
source install/setup.zsh
```

`source` 只对当前终端有效。每次打开新的 zsh 终端，都需要加载 ROS 和当前工作空间：

```zsh
source /opt/ros/humble/setup.zsh
source /home/lanchong/standrad_movelt_motion/install/setup.zsh
```

`./motion.zsh` 已经自动完成上述加载，因此录制和回放命令不需要手动 `source`。

## 运行模拟规划

模拟模式不连接实际机械臂，适合检查模型、交互球和规划功能：

```zsh
cd /home/lanchong/standrad_movelt_motion
source /opt/ros/humble/setup.zsh
source install/setup.zsh
ros2 launch robot_movelt_config demo.launch.py
```

指定模型版本：

```zsh
ros2 launch robot_movelt_config demo.launch.py variant:=v1
ros2 launch robot_movelt_config demo.launch.py variant:=v2
```

RViz 打开后，在 MotionPlanning 面板中选择规划组 `arm_l` 或 `arm_r`，拖动末端
交互球设置目标。点击 **Plan** 生成轨迹，确认规划成功后点击 **Execute** 执行模拟运动。

## 运行 V3 实际机械臂

运行前确认急停状态、机械臂活动范围和串口设备名。默认串口为 `/dev/ttyACM0`：

```zsh
cd /home/lanchong/standrad_movelt_motion
source /opt/ros/humble/setup.zsh
source install/setup.zsh
ros2 launch robot_movelt_config demo.launch.py hardware:=v3
```

指定串口或模型版本：

```zsh
ros2 launch robot_movelt_config demo.launch.py \
  variant:=v2 hardware:=v3 port_name:=/dev/ttyACM0
```

启动后程序会先读取真机反馈，然后使机械臂回到初始位置。等待终端出现：

```text
V3 bridge READY
```

再在 RViz 中规划和执行。点击 **Plan** 只进行规划，不会驱动真机；点击
**Execute** 后，实际机械臂才会按照轨迹运动，RViz 同时显示真机反馈。

跳过启动归位、只同步当前真机姿态：

```zsh
ros2 launch robot_movelt_config demo.launch.py \
  hardware:=v3 home_on_start:=false
```

此选项只建议调试反馈时使用。正常运行应保留默认的 `home_on_start:=true`。

如果串口没有访问权限，可检查：

```zsh
ls -l /dev/ttyACM0
groups
```

通常需要将当前用户加入 `dialout` 组，重新登录后生效：

```zsh
sudo usermod -aG dialout $USER
```

## 动作录制与回放

录制节点在 `hardware:=v3` 时默认随主程序启动。另开一个 zsh 终端，在工作区中运行：

```zsh
cd /home/lanchong/standrad_movelt_motion
./motion.zsh start
```

暂停、继续并保存：

```zsh
./motion.zsh pause
./motion.zsh resume
./motion.zsh stop action_01.json
```

文件会保存在：

```text
src/standard_motion_recorder/motions/
```

可以使用不同名称保存多组动作。回放指定动作：

```zsh
./motion.zsh play action_01.json
```

以 `0.5` 倍速度回放或循环回放：

```zsh
./motion.zsh play action_01.json 0.5 false
./motion.zsh play action_01.json 1.0 true
```

查看状态或停止回放：

```zsh
./motion.zsh status
./motion.zsh stop-playback
```

回放前必须确认主程序仍在运行，并且终端已经显示 `V3 bridge READY`。回放过程中不要
同时在 RViz 点击 Execute。

## RViz 加载较慢

本项目启动时需要依次加载 URDF/STL 模型、MoveIt 规划场景、运动学插件和控制接口。
主启动文件还会等待 MoveGroup 完成初始化后再启动 RViz，避免 RViz 提前打开却找不到
规划服务。因此首次启动或电脑负载较高时，RViz 可能需要几十秒才出现，属于正常现象。

等待期间不要重复执行 launch。可以观察主终端：只要进程没有退出，并持续出现 MoveIt、
MoveGroup 或模型加载日志，就继续等待。RViz 出现后，机器人模型和交互球仍可能再延迟
几秒显示。

如果长时间没有打开，可在另一个已加载环境的 zsh 终端检查：

```zsh
ros2 node list
ros2 service list | rg move_group
ros2 topic echo /joint_states --once
```

真机模式还应检查反馈：

```zsh
ros2 topic hz /joint_states_fdb_V3
```

若没有真机反馈，检查串口名、串口权限和机械臂连接；若 MoveGroup 不存在，查看主启动
终端中的首个报错。不要通过连续启动多个 launch 来解决加载慢问题，否则会产生重复节点
和串口占用。

## 常用启动参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `variant` | `v1` | 机器人模型，可选 `v1`、`v2` |
| `hardware` | `mock` | 执行后端，可选 `mock`、`v3` |
| `port_name` | `/dev/ttyACM0` | V3 真机串口 |
| `baud_rate` | `2000000` | 串口波特率 |
| `auto_enable` | `true` | 收到真机反馈后自动获取控制权 |
| `home_on_start` | `true` | 真机启动后自动回初始位置 |
| `motion_recorder` | `true` | 真机模式下启动录制节点 |

## 安全注意事项

- 第一次连接真机时使用较大的安全空间，并随时准备急停。
- 必须等待 `V3 bridge READY` 后再执行 RViz 轨迹或动作回放。
- 点击 Plan 不会运动，点击 Execute 会驱动实际机械臂。
- 不要同时使用 RViz Execute 和动作回放控制机械臂。
- 真机反馈超时或桥接未就绪时，轨迹会被拒绝或中止。
- 关闭主程序后再拔插串口或修改通信设备。

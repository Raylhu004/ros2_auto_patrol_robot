# 基于 ROS 2 和 Navigation 2 的自动巡检机器人

## 1. 项目介绍

本项目在 Gazebo 仿真环境中实现了一个自动巡检机器人。机器人是一辆两轮差速小车 fishbot，装有激光雷达、IMU 和深度相机。

整个项目分三步完成：

1. **建图**：用键盘遥控小车在房间里走一圈，slam_toolbox 根据激光雷达数据画出房间地图。
2. **导航**：Navigation 2（简称 Nav2）在地图上确定小车的位置（定位），规划路径，并控制小车走到目标点。
3. **巡检**：巡检程序依次把目标点发给 Nav2。每到达一个点，就用语音播报并拍一张照片保存下来，然后循环巡逻。

通过这个项目，你可以把 ROS 2 的几种通信方式（话题、服务、动作、TF）和 Nav2 的使用串起来练一遍。

### 1.1 先了解几个概念

| 概念 | 一句话解释 | 在本项目中的例子 |
|---|---|---|
| 功能包（package） | ROS 2 组织代码的单位，一个包完成一类功能 | `fishbot_description` 放机器人模型 |
| 节点（node） | 一个运行中的 ROS 程序 | `patrol_node` 巡检节点、`speaker` 语音节点 |
| 话题（topic） | 一方持续发布、另一方订阅的数据流，单向 | 相机图像 `/camera_sensor/image_raw`、雷达 `/scan` |
| 服务（service） | 一问一答，调用方等对方处理完后拿到结果 | `patrol_node` 请求 `speaker` 朗读一句话 |
| 动作（action） | 耗时较长的任务，执行过程中会持续反馈进度，可以取消 | `patrol_node` 让 Nav2 导航到某个点，并收到“剩余距离” |
| TF | 各个坐标系之间的位置关系 | `map → odom → base_footprint` 表示小车在地图中的位置 |
| launch 文件 | 一条命令同时启动多个节点，并传入参数 | `auto_patrol.launch.py` 同时启动巡检和语音两个节点 |

### 1.2 功能包

| 功能包 | 编程语言 | 作用 |
|---|---|---|
| `fishbot_description` | （配置文件） | 机器人模型（URDF/xacro）、轮子控制器配置、Gazebo 仿真房间 |
| `fishbot_navigation` | （配置文件） | 建图参数、Nav2 参数、建好的地图 `room.pgm` 和 `room.yaml` |
| `fishbot_application` | Python | 4 个独立的小例子：设置初始位置、查询位置、单点导航、多点导航。建议先看这里，再看巡检程序 |
| `autopatrol_interfaces` | （接口定义） | 自定义服务接口 `SpeechText.srv`：请求是一段文字，回复是成功与否 |
| `autopatrol_robot` | Python | 巡检节点 `patrol_node` 和语音节点 `speaker` |

### 1.3 各部分如何连接

```
Gazebo 仿真（gazebo_sim.launch.py）
 ├─ 模拟传感器：雷达 /scan、相机 /camera_sensor/image_raw
 └─ 轮子控制器：接收 /cmd_vel 速度指令，根据轮子转动计算里程计（TF odom→base_footprint）

Nav2 导航（nav2.launch.py）
 ├─ map_server：加载地图 room.yaml
 ├─ AMCL：用雷达数据和地图做比对，确定小车在地图中的位置（TF map→odom）
 └─ 规划器 + 控制器：规划路径，并发出速度指令 /cmd_vel

自动巡检（auto_patrol.launch.py）
 ├─ patrol_node ──动作 NavigateToPose──► Nav2       “去这个点”
 │      ├─ 订阅话题 /camera_sensor/image_raw         到点后保存照片
 │      └─ 服务 speech_text ──► speaker             “读这句话”
 └─ speaker：收到文字后，用 espeak-ng 朗读中文
```

### 1.4 巡检程序 `patrol_node` 的流程

1. 播报“正在准备初始化位置”，把 `initial_point` 作为小车的初始位置告诉 AMCL，然后等待 Nav2 全部启动完成。
2. 依次前往 `target_points` 中的每个目标点：
   - 出发前，播报目标点坐标；
   - 导航成功后，播报“已经到达”并拍照。照片名为 `img_<x>_<y>.png`，x、y 是拍照时小车在地图中的位置；
   - 导航失败时，播报失败信息，跳过拍照，直接去下一个点。
3. 所有目标点走完后，从第一个点重新开始。

参数写在 `autopatrol_robot/config/patrol_config.yaml` 中：

| 参数 | 说明 |
|---|---|
| `initial_point` | 初始位置 `[x, y, yaw]`。地图原点就是小车在仿真里出生的位置，所以一般填 `[0.0, 0.0, 0.0]` |
| `target_points` | 目标点列表，每 3 个数为一组 `x, y, yaw`，单位是米和弧度。**要写成小数**，比如 `1.0`，不要写 `1` |
| `img_save_path` | 照片保存目录。不设置时，保存到执行 `ros2 launch` 命令时所在的目录 |
| `use_sim_time` | 是否使用仿真时间。在仿真中必须为 `true` |

> yaml 文件最顶层的名字 `/patrol_node` 必须和节点名一致，否则参数不会被读进去。

目标点可以这样找：启动导航后，在 RViz 顶部工具栏选择 **Publish Point** 工具，然后把鼠标移到地图上，窗口底部的状态栏会显示该点的坐标。注意不要选在墙边或很窄的角落里，小车可能进不去，或者进去之后出不来。

## 2. 使用方法

### 2.1 开发环境

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic 11
- Python 3.10

### 2.2 安装依赖

ROS 2 功能包：

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox \
    ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control ros-humble-ros2-controllers \
    ros-humble-xacro ros-humble-robot-state-publisher ros-humble-cv-bridge \
    ros-humble-tf-transformations ros-humble-teleop-twist-keyboard
```

语音合成与四元数转换：

```bash
sudo apt install espeak-ng python3-transforms3d
pip3 install espeakng
```

### 2.3 编译

```bash
cd chapter7_ws          # 工作空间目录，src/ 的上一级
colcon build
source install/setup.bash
```

- `colcon build` 会把所有功能包编译并安装到 `install/` 目录。
- `source install/setup.bash` 的作用是让当前终端能找到这些包。**每打开一个新终端都要执行一次**，否则会报 `Package 'xxx' not found`。
- 修改 Python 代码、launch、yaml、urdf 之后，都要**重新编译**才会生效。因为运行时用的是 `install/` 里的副本，而不是 `src/` 里的原文件。

### 2.4 运行自动巡检

打开三个终端，每个终端都先执行 `source install/setup.bash`，然后**按顺序**启动：

```bash
# 终端 1：仿真
ros2 launch fishbot_description gazebo_sim.launch.py
```
等待 Gazebo 窗口出现小车，并且终端里打印出控制器加载成功的信息。

```bash
# 终端 2：导航
ros2 launch fishbot_navigation nav2.launch.py
```
RViz 窗口会打开并显示地图。

```bash
# 终端 3：自动巡检
ros2 launch autopatrol_robot auto_patrol.launch.py
```
你会先听到“正在准备初始化位置”，然后在 RViz 中看到小车出现在地图上。之后小车开始依次前往各个目标点，终端里会不断打印“剩余距离”。

### 2.5 重新建图（可选）

仓库里已经有建好的地图。如果修改了仿真房间，需要重新建图：

```bash
# 终端 1：仿真
ros2 launch fishbot_description gazebo_sim.launch.py

# 终端 2：建图（在工作空间目录下执行，因为路径里用到了 $PWD）
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True \
    slam_params_file:=$PWD/src/fishbot_navigation/config/mapper_params_online_async.yaml

# 终端 3：用键盘控制小车。建议把转速降到 0.3~0.5 rad/s，转太快地图会变形
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 走完整个房间后，保存地图
ros2 run nav2_map_server map_saver_cli -f src/fishbot_navigation/maps/room
```

### 2.6 入门小例子

`fishbot_application` 里的 4 个程序各只做一件事，适合在读 `patrol_node` 之前先看。运行前要先启动仿真和导航（2.4 的终端 1、2）。

```bash
ros2 run fishbot_application init_robot_pose     # 设置初始位置
ros2 run fishbot_application get_robot_pose      # 每秒查询并打印小车在地图中的位置
ros2 run fishbot_application nav_to_pose         # 导航到一个点
ros2 run fishbot_application waypoint_follower   # 依次经过多个点
```

### 2.7 常见问题

| 现象 | 可能的原因和解决办法 |
|---|---|
| `Package 'xxx' not found` | 当前终端没有执行 `source install/setup.bash` |
| 改了代码或参数，但运行效果没变 | 没有重新执行 `colcon build` |
| 巡检程序启动后马上退出，日志显示 `process has died` | 往上翻终端，找 Python 的报错信息（Traceback）。程序出错时，详细报错只会显示在终端里 |
| 一直打印“语音服务未上线，等待中” | `speaker` 节点没有启动，或者启动时出错了。用 `ros2 service list` 检查是否有 `/speech_text` |
| 小车停在原地不动 | 检查 Nav2 是否已经启动；检查目标点是不是在墙里或障碍物上 |
| 小车在某个点附近来回转圈，最后导航失败 | 目标点离墙太近，或者在狭窄的角落里，换一个更空旷的位置 |
| 照片颜色发蓝 | 把图像转成 OpenCV 格式时没有指定 `'bgr8'`（本项目代码中已经处理） |

## 3. 作者

- Robotato

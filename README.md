# fastumi_replay

`fastumi_replay` 是一个 ROS1/catkin 包，用于回放 FastUMI 采集数据，并在 RViz 中可视化左右夹爪、桌面和夹爪末端轨迹。

## 功能

- 交互式选择 `~/fastumi/DATA` 下的采集日期和 session。
- 同步发布左右手 RGB 图像、夹爪末端位姿和夹爪开合距离。
- 使用 URDF/xacro 在 RViz 中显示左右夹爪简化模型。
- 以 `world` 为根坐标，显示左右夹爪基坐标、白色桌面和最近 2 秒末端红色轨迹。
- 所有常用参数集中在 `config/replay.yaml`，便于迁移和调参。

## 目录结构

```text
fastumi_replay/
├── config/replay.yaml                 # 数据路径、topic、坐标、桌面、轨迹等配置
├── launch/replay.launch               # 一键启动 RViz、RobotModel、visualizer 和回放终端
├── meshes/triangle_finger.stl         # 三角夹爪薄片 mesh
├── rviz/replay.rviz                   # RViz 配置
├── scripts/clamp_visualizer.py        # 夹爪 joint_states 与末端轨迹发布节点
├── scripts/replay_session.py          # 交互式数据回放脚本
└── urdf/fastumi_clamps.urdf.xacro     # 夹爪和桌面可视化模型
```

## 构建

在 catkin 工作空间根目录执行：

```bash
source /opt/ros/noetic/setup.bash
cd ~/fastumi
catkin_make
source ~/fastumi/devel/setup.bash
```

## 启动

```bash
source /opt/ros/noetic/setup.bash
source ~/fastumi/devel/setup.bash
roslaunch fastumi_replay replay.launch
```

`replay.launch` 会启动：

- `robot_state_publisher`
- `scripts/clamp_visualizer.py`
- RViz
- 一个新的 `gnome-terminal`，用于运行交互式 `scripts/replay_session.py`

如果只想检查某个 session 的数据是否能被解析：

```bash
source /opt/ros/noetic/setup.bash
source ~/fastumi/devel/setup.bash
rosrun fastumi_replay replay_session.py \
  --config ~/fastumi/src/fastumi_replay/config/replay.yaml \
  --check-session ~/fastumi/DATA/multi_session_20260429/session_165008
```

若当前系统没有 `rosrun`，可以直接执行脚本：

```bash
~/fastumi/src/fastumi_replay/scripts/replay_session.py \
  --config ~/fastumi/src/fastumi_replay/config/replay.yaml \
  --check-session ~/fastumi/DATA/multi_session_20260429/session_165008
```

## 配置

主要参数在 `config/replay.yaml`：

- `data_root`：采集数据根目录。
- `playback_rate`：回放倍率。
- `topics`：图像、位姿、夹爪距离、joint states 和轨迹 topic。
- `frames`：`world`、左右 `base` 坐标名及左右基座间距。
- `visualizer`：夹爪厚度、距离单位和发布频率。
- `table`：桌面显示开关、中心、尺寸和颜色。
- `trail`：末端轨迹显示开关、topic、保留时长、线宽和颜色。

## 坐标约定

- `world` 是可视化根坐标。
- `left_base` 和 `right_base` 是左右夹爪回放位姿的父坐标。
- `/left_clamp/pose` 和 `/right_clamp/pose` 表示夹爪尖端中心点的位置和朝向。
- `/left_clamp/distance` 和 `/right_clamp/distance` 表示两片夹爪之间的开合距离，默认单位为 `mm`。

`frames` 中的 `lr_dist`、`base_x`、`base_z` 会用于生成 `world -> left_base/right_base` 的固定变换。

## 常用验证

```bash
source /opt/ros/noetic/setup.bash
source ~/fastumi/devel/setup.bash
xacro ~/fastumi/src/fastumi_replay/urdf/fastumi_clamps.urdf.xacro \
  config_file:=~/fastumi/src/fastumi_replay/config/replay.yaml \
  > /tmp/fastumi_clamps.urdf
check_urdf /tmp/fastumi_clamps.urdf
```

```bash
rostopic echo /joint_states
rostopic echo /clamp_tip_trails
```

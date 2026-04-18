# Quadruped Sim

本仓库包含两个部分：

- `quadruped_sim/`：MuJoCo 四轮足仿真（使用 `pixi` 管理 Python 环境）
- `rh1/`：ROS 2 的 `rh1` 机器人描述包（URDF/launch/RViz/Gazebo）

## 1. 环境要求

建议系统：Linux (已在 wsl Unbuntu24.04 下编译测试通过)

### 必需软件

1. Git
2. Pixi（用于管理 `quadruped_sim` 目录下的 Python 依赖）
3. 图形环境（X11/Wayland）用于 MuJoCo viewer

### 可选软件（如果要用 ROS 2 部分,仅查看模型无仿真）

1. ROS 2（建议 Humble 或同版本生态）
2. `colcon`
3. `gazebo_ros`、`robot_state_publisher`、`joint_state_publisher_gui`、`rviz2`

## 2. 仓库结构与进入路径

仓库根目录下有两个主要子目录：

- `quadruped_sim/`：运行 MuJoCo 仿真时进入这里
- `rh1/`：gezebo模型文件

下面命令都从仓库根目录执行（即本 README 所在目录）。

## 3. MuJoCo 仿真：完整配置与运行

### 3.1 安装 Pixi

如果系统还没有 Pixi，先安装：

```bash
curl -fsSL https://pixi.sh/install.sh | bash
```

安装后重开终端，或执行：

```bash
source ~/.bashrc
# 或 source ~/.zshrc
```

验证：

```bash
pixi --version
```

### 3.2 创建/同步项目环境

进入仿真目录：

```bash
cd quadruped_sim
```

根据 `pixi.toml` + `pixi.lock` 同步环境：

```bash
pixi install
```

依赖包括：

- Python 3.13
- mujoco
- numpy

### 3.3 运行仿真

```bash
pixi run python simulate.py
```

运行后会自动：

1. 读取 `rh1.urdf`
2. 生成 `quadruped_generated.xml`
3. 打开 MuJoCo 可视化窗口

### 3.4 键盘控制

- `W / S`：前进 / 后退
- `A / D`：左转 / 右转
- `Q / E`：腿高上调 / 下调
- `Space`：停止（清零速度）

## 4. ROS 2 包（rh1）：构建与使用

如果你只关心 MuJoCo，这一节可以跳过。

### 4.1 安装 ROS 2 依赖（示例）

先确保已安装 ROS 2，并可执行：

```bash
source /opt/ros/humble/setup.bash
```

安装常用依赖（按需）：

```bash
sudo apt update
sudo apt install -y \
	ros-humble-joint-state-publisher-gui \
	ros-humble-robot-state-publisher \
	ros-humble-rviz2 \
	ros-humble-gazebo-ros-pkgs
```

### 4.2 构建 `rh1` 包

在仓库根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select rh1 --base-paths rh1
```

加载工作区环境：

```bash
source install/setup.bash
```

### 4.3 启动显示（URDF + RViz）

```bash
ros2 launch rh1 display_robot.launch.py
```

### 4.4 启动 Gazebo

```bash
ros2 launch rh1 gazebo.launch.py
```

## 5. 常见问题

### 5.1 为什么 `.pixi` 没有被 Git 跟踪？

仓库里的 `.gitignore` 默认忽略 `.pixi/*`，因为 `.pixi/envs` 是本地环境缓存，一般不提交。

同平台可复现相同依赖版本需要:
- `quadruped_sim/pixi.toml`
- `quadruped_sim/pixi.lock`


### 5.2 MuJoCo 窗口打不开或无图形

检查是否在有桌面图形会话的终端运行；远程环境需要正确转发显示（如 X11 forwarding/桌面会话）。

### 5.3 `pixi run` 很慢或首次下载时间长

首次会拉取依赖与构建环境，属于正常现象。后续复用环境会快很多。

## 6. 常用命令速查

```bash
# 运行 MuJoCo 仿真
cd quadruped_sim
pixi run python simulate.py

# 重新同步 pixi 环境
cd quadruped_sim
pixi install

```

## 7. 贡献建议

1. 提交依赖变化时，同时更新 `pixi.toml` 和 `pixi.lock`。

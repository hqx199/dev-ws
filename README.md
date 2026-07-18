# SMART1 — 智能车自动驾驶系统

**SMART1** 是面向 **全国大学生智能汽车竞赛** 的自主驾驶智能车项目。基于阿克曼底盘（STM32 下位机 + RDK X5 上位机），使用 ROS2 Humble 构建全栈自动驾驶系统，融合惯性导航、视觉感知与传感器融合技术。

---

## 项目概述

| 项目         | 内容                                                     |
| ------------ | -------------------------------------------------------- |
| **硬件平台** | RDK X5 + STM32 + 阿克曼转向底盘                          |
| **操作系统** | Ubuntu 22.04 (ROS2 Humble)                               |
| **开发语言** | C++ / Python                                             |
| **核心算法** | Pure Pursuit 路径跟踪、YOLOv5 障碍物检测、EKF 传感器融合 |
| **竞赛任务** | 自主导航、避障行驶、QR码扫描、视觉识别                   |

---

## 目录结构

```
SMART1/
├── start_origicar.sh                 # 底盘+IMU+摄像头 一键启动
├── start_yolo.sh                     # YOLO 障碍物检测启动
├── run_inertial_nav.sh               # 惯性导航控制器启动
├── stop_all.sh                       # 停止所有服务
├── start_aurora_qr_detection.sh      # QR码检测启动
├── stop_aurora_qr_detection.sh       # 停止QR码检测
├── setup_origicar_env.sh             # 工作环境配置
│
├── src/
│   ├── origincar/                    # 核心ROS2元包
│   │   ├── origincar_base/          # 底盘驱动节点 (STM32串口通信、IMU、里程计)
│   │   ├── origincar_bringup/       # 启动管理 (摄像头、WebSocket、可视化)
│   │   ├── origincar_description/   # URDF/XACRO 机器人模型
│   │   ├── origincar_msg/           # 自定义消息接口
│   │   ├── utils/                   # 图像传输工具
│   │   └── 3rdparty/                # 第三方依赖 (ackermann_msgs, serial_ros2)
│   │       └── aurora930/           # Aurora930 深度相机驱动
│   │
│   ├── racing/                       # 竞赛功能包
│   │   ├── racing_inertial_nav/     # 惯性导航控制器 (Pure Pursuit/航点跟踪/避障)
│   │   ├── racing_obstacle_detection_yolo/  # YOLOv5s 障碍物检测 (锥桶)
│   │   └── qr_mem/                  # QR码 BPU 推理检测
│   │
│   ├── deptrum-ros-driver/           # Aurora930 相机独立驱动
│   ├── imu_tools/                    # IMU 滤波工具 (Madgwick/Complementary)
│   └── qr_code_detection/           # QR码检测 Python 节点
│
├── docs/                             # 项目文档
└── README.md                         # 本文档
```

---

## 系统架构

### 整体架构

```
┌─────────────────────────────────────────────────────────┐
│                 inertial_nav_controller                  │
│              Pure Pursuit 路径跟踪 + YOLO避障             │
│             航点序列导航 · 动态预瞄 · 倒车控制              │
└──────────────┬──────────────────────────┬──────────────┘
               │ /cmd_vel                 │ YOLO检测输入
               ▼                          ▼
┌──────────────────────────┐  ┌────────────────────────────┐
│     Origicar Base        │  │  racing_obstacle_detection │
│   STM32 串口通信驱动      │  │     YOLOv5s BPU 推理       │
│   /odom · /imu/data_raw  │  │  锥桶检测 · 透视校正        │
│   EKF → /odom_combined   │  └───────────┬────────────────┘
└──────────┬───────────────┘              │
           │                              │
           │  ┌───────────────────────┐   │
           │  │    Aurora930 相机      │   │
           └──│   /image_raw → 视觉    │───┘
              │   QR码检测 · 图传       │
              └───────────────────────┘
```

### 传感器与数据流

```
┌──────────┐  /imu/data_raw  ┌──────────┐  /odom_combined
│ RDK IMU  │───────────────▶│   EKF    │──────────────▶ inertial_nav
│ BMI088   │                │   Fusion │      + /tf    controller
└──────────┘                └────┬─────┘
                                 │
┌──────────┐  /odom              │
│ STM32   │──────────────────────┘
│轮式里程计 │
└──────────┘

┌──────────┐  /image_raw  ┌────────────────┐  /qr_result
│Aurora930 │─────────────▶│ qr_mem/bpu     │────────────▶\
│ RGB相机  │              │ QR码BPU推理+ROI │              \
└──────────┘              └────────────────┘               \
                                                            \
┌──────────┐  /image_raw  ┌────────────────┐  /racing_      \  /cmd_vel
│Aurora930 │─────────────▶│ YOLOv5s BPU    │  obstacle_      ─▶ STM32
│ RGB相机  │              │ 锥桶检测        │  detection      /
└──────────┘              └────────────────┘               /
                                                           /
┌──────────┐             ┌────────────────┐               /
│ ROSBridge│◀────────────│ WebSocket      │──────────────/
│ 端口9090 │             │ 图像压缩中继    │
└──────────┘             └────────────────┘
```

---

## 核心功能包

### 1. racing_inertial_nav — 惯性导航控制器

| 项目     | 内容                                                                  |
| -------- | --------------------------------------------------------------------- |
| 语言     | C++                                                                   |
| 功能     | Pure Pursuit 路径跟踪、航点序列导航、动态预瞄、倒车控制、YOLO避障绕行 |
| 发布     | `/cmd_vel` (Twist)                                                    |
| 订阅     | `/odom_combined`, `/racing_obstacle_detection`                        |
| 关键文件 | `src/inertial_nav_controller.cpp`, `config/params.yaml`               |

包含完整的航点导航能力：
- **Pure Pursuit** 核心跟踪算法，支持曲率限速
- **动态预瞄**：根据到目标距离自动调整预瞄距离
- **渐进式姿态混合**：阿克曼底盘边行驶边调整朝向
- **倒车控制**：目标在后方时自动倒车（效率优先）
- **避障绕行**：基于 YOLO 检测结果自动插入绕行航点

### 2. racing_obstacle_detection_yolo — YOLO 障碍物检测

| 项目 | 内容                               |
| ---- | ---------------------------------- |
| 语言 | C++                                |
| 模型 | YOLOv5s (TensorRT .bin，BPU 推理)  |
| 功能 | 锥桶目标检测，含透视校正和距离估算 |
| 发布 | `/racing_obstacle_detection`       |
| 配置 | `config/yolov5sconfig.json`        |

### 3. origincar_base — 底盘驱动核心

| 项目 | 内容                                                            |
| ---- | --------------------------------------------------------------- |
| 语言 | C++                                                             |
| 功能 | STM32 串口通信、IMU 数据解析、轮式里程计计算、舵机/电机指令下发 |
| 发布 | `/odom`, `/imu/data_raw`, `/PowerVoltage`, `/robotpose`         |
| 订阅 | `/cmd_vel`                                                      |
| 启动 | `ros2 launch origincar_base origincar_bringup.launch.py`        |

### 4. origincar_bringup — 启动管理

| 项目 | 内容                                           |
| ---- | ---------------------------------------------- |
| 语言 | Python (launch)                                |
| 功能 | USB 摄像头、WebSocket 图传、ROSBridge 启动配置 |
| 模式 | 支持压缩模式、hobot 模式、USB only、无图传模式 |

### 5. qr_mem — QR 码 BPU 检测

| 项目 | 内容                         |
| ---- | ---------------------------- |
| 语言 | Python                       |
| 推理 | RDK X5 BPU 推理，零 CPU 占用 |
| 功能 | 图像中 QR 码检测 + ROI 解码  |
| 发布 | `/qr_result`                 |

### 6. 辅助包

| 包名                       | 语言   | 功能                   |
| -------------------------- | ------ | ---------------------- |
| `utils`                    | C++    | 图像传输协议转换       |
| `imu_filter_madgwick`      | C++    | Madgwick IMU 滤波      |
| `imu_complementary_filter` | C++    | 互补 IMU 滤波          |
| `qr_code_detection`        | Python | QR 码检测 Python 节点  |
| `deptrum-ros-driver`       | C++    | Aurora930 深度相机驱动 |

---

## 技术栈

| 层级       | 技术                           |
| ---------- | ------------------------------ |
| 操作系统   | Ubuntu 22.04 (RDK X5 arm64)    |
| 中间件     | ROS2 Humble (colcon build)     |
| AI 推理    | tros-humble BPU (YOLOv5s .bin) |
| 视觉       | Aurora930 RGB 相机             |
| 传感器融合 | robot_localization (EKF)       |
| 下位机     | STM32 (串口协议)               |
| 底盘       | 阿克曼转向 (Ackermann Drive)   |
| 远程调试   | ROSBridge WebSocket + Foxglove |

---

## 快速启动

### 1. 启动底盘与传感器

```bash
# 阿克曼模式完整启动（底盘+IMU+摄像头+ROSBridge）
bash start_origicar.sh --akmcar

# 如需禁用WebSocket图传（节省资源）
bash start_origicar.sh --akmcar --no-websocket
```

### 2. 启动 YOLO 障碍物检测

```bash
# 新开终端
bash start_yolo.sh
```

### 3. 启动惯性导航控制器

```bash
# 新开终端
bash run_inertial_nav.sh
```

### 4. 停止所有服务

```bash
bash stop_all.sh
```

---

## 关键参数调优

所有导航参数在 `src/racing/racing_inertial_nav/config/params.yaml` 中配置，修改后重启节点生效，无需重新编译。

| 参数                 | 默认值    | 说明                  |
| -------------------- | --------- | --------------------- |
| `max_linear_speed`   | 0.5 m/s   | 最大前进速度          |
| `lookahead_distance` | 0.5 m     | Pure Pursuit 预瞄距离 |
| `min_turning_radius` | 0.5 m     | 最小转弯半径          |
| `position_tolerance` | 0.05 m    | 位置到达容差          |
| `control_frequency`  | 50 Hz     | 控制循环频率          |
| `avoid_area`         | 18000 px² | 避障触发面积阈值      |

详细调参指南：`src/racing/racing_inertial_nav/README.md`

### 快速调参口诀

```
撞上了       → area↓  bottom↓  lateral↑  forward↑  speed↓
绕太早/太远  → area↑  bottom↑  lateral↓  forward↓
转弯擦边     → forward↑  speed↓
直道太慢     → max_linear_speed↑
转弯失控     → max_linear_speed↓  min_turning_radius↑
```

---

## 构建命令

```bash
# 全量编译
cd /home/sunrise/RacingDev/dev_ws
colcon build --symlink-install

# 仅编译特定包
colcon build --packages-select racing_inertial_nav
colcon build --packages-select racing_obstacle_detection_yolo
```

---

## 调试命令

```bash
# 查看融合里程计
ros2 topic echo /odom_combined | grep position

# 查看控制输出
ros2 topic echo /cmd_vel --once

# 查看 YOLO 检测
ros2 topic echo /racing_obstacle_detection --no-arr --once --field targets

# 运行时修改参数（无需重启）
ros2 param set /inertial_nav_controller max_linear_speed 0.6
ros2 param set /inertial_nav_controller dynamic_lookahead_max 0.9

# 查看所有参数
ros2 param dump /inertial_nav_controller
```

---

## 主要话题

| 话题                         | 类型            | 说明                           |
| ---------------------------- | --------------- | ------------------------------ |
| `/cmd_vel`                   | Twist           | 底盘速度控制指令               |
| `/odom_combined`             | Odometry        | EKF 融合里程计（导航主数据源） |
| `/odom`                      | Odometry        | 轮式里程计                     |
| `/imu/data_raw`              | Imu             | IMU 原始数据                   |
| `/racing_obstacle_detection` | 自定义          | YOLO 障碍物检测结果            |
| `/qr_result`                 | String          | QR 码检测/方向结果             |
| `/image`                     | CompressedImage | USB 摄像头画面                 |
| `/PowerVoltage`              | Float32         | 电池电压                       |

---

## 当前开发状态

| 模块                 | 状态     |
| -------------------- | -------- |
| 底盘驱动 (STM32通信) | ✅ 已完成 |
| EKF 传感器融合       | ✅ 已完成 |
| RDK IMU 驱动         | ✅ 已完成 |
| USB 摄像头 + 图传    | ✅ 已完成 |
| Pure Pursuit 导航    | ✅ 已完成 |
| 动态预瞄调节         | ✅ 已完成 |
| 倒车控制             | ✅ 已完成 |
| YOLO 障碍物检测      | ✅ 已完成 |
| 避障绕行             | ✅ 已完成 |
| QR 码检测            | ✅ 已完成 |
| ROSBridge 远程调试   | ✅ 已完成 |
| 实车赛道联调         | 🟡 进行中 |

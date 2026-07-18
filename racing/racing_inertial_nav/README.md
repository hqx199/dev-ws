# params.yaml 调参指南

> 本文档是 `racing_inertial_nav` 功能包的 **唯一调参参考**。  
> 所有参数均在 `config/params.yaml` 中配置，修改后重启节点即可生效，无需重新编译。

## 目录

- [参数总览](#参数总览)
- [基础运动参数](#基础运动参数)
- [导航容差](#导航容差)
- [Pure Pursuit 预瞄控制](#pure-pursuit-预瞄控制)
- [动态预瞄（距离自适应）](#动态预瞄距离自适应)
- [倒车控制](#倒车控制)
- [PID 角速度控制器](#pid-角速度控制器)
- [渐进式姿态混合](#渐进式姿态混合)
- [控制循环](#控制循环)
- [避障绕行](#避障绕行)
- [航点序列](#航点序列)
- [常见场景调参方案](#常见场景调参方案)
- [调试命令](#调试命令)

---

## 参数总览

| 参数名 | 默认值 | 单位 | 说明 |
|--------|--------|------|------|
| `max_linear_speed` | 0.5 | m/s | 最大前进速度 |
| `max_reverse_speed` | 0.3 | m/s | 最大倒车速度 |
| `max_angular_speed` | 3.0 | rad/s | 最大旋转角速度 |
| `max_acceleration` | 0.5 | m/s² | 加速度限制 |
| `position_tolerance` | 0.05 | m | 默认位置容差 |
| `angle_tolerance` | 0.1 | rad | 默认角度容差 |
| `waypoint_timeout` | 10.0 | s | 单航点超时 |
| `lookahead_distance` | 0.5 | m | 默认预瞄距离（绕行模式锁定值） |
| `min_lookahead_distance` | 0.1 | m | 最小预瞄距离 |
| `min_turning_radius` | 0.5 | m | 最小转弯半径 |
| `dynamic_lookahead_max` | 0.8 | m | 远距离最大预瞄 |
| `dynamic_lookahead_min` | 0.2 | m | 近距离最小预瞄 |
| `dist_far_threshold` | 1.5 | m | 远距离阈值 |
| `dist_near_threshold` | 0.3 | m | 近距离阈值 |
| `lookahead_lpf_alpha` | 0.15 | - | 预瞄低通滤波系数 |
| `enable_reverse` | true | - | 是否启用倒车 |
| `reverse_angle_threshold` | 0.8 | rad | 倒车横向偏角阈值 |
| `angular_kp` | 3.0 | - | 角度 PID 比例增益 |
| `angular_ki` | 0.0 | - | 角度 PID 积分增益 |
| `angular_kd` | 0.5 | - | 角度 PID 微分增益 |
| `yaw_blend_start_distance` | 1.0 | m | 姿态混合起始距离 |
| `control_frequency` | 50.0 | Hz | 控制循环频率 |
| `avoid_area` | 18000 | px² | 避障触发面积阈值 |
| `avoid_trigger_bottom` | 280 | px | 避障触发底部 y 坐标 |
| `avoid_detour_lateral_offset` | 0.3 | m | 绕行侧向偏移 |
| `avoid_detour_forward_step` | 0.5 | m | 绕行前进步长 |
| `avoid_detour_cooldown` | 3.0 | s | 绕行冷却时间 |
| `avoid_linear_speed` | 0.4 | m/s | 绕行速度 |

---

## 基础运动参数

```yaml
max_linear_speed: 0.5      # m/s 最大前进速度
max_reverse_speed: 0.3     # m/s 最大倒车速度
max_angular_speed: 3.0     # rad/s 最大旋转速度
max_acceleration: 0.5      # m/s² 最大加速度（平滑加减速）
```

### `max_linear_speed`

Pure Pursuit 输出的基础速度为 `max_linear_speed × 0.8`，再乘以曲率限速因子。

- 实际行驶速度 ≈ `0.8 × 0.5 = 0.4 m/s`（直道，无曲率限速时）
- 转弯时曲率越大，速度越低

| 现象 | 调整 |
|------|------|
| 直道太慢 | ↑ 增大到 0.6~0.7 |
| 转弯失控/侧翻 | ↓ 减小到 0.3~0.4 |
| 电池电压低跑不动 | ↑ 适当增大补偿 |

### `max_reverse_speed`

倒车时的速度上限，建议始终低于前进速度（倒车控制精度较低）。

### `max_angular_speed`

角速度的绝对上限。Pure Pursuit 的角速度 = `curvature × linear_cmd`，最终会被钳位到此值。

- 3.0 rad/s ≈ 172°/s，对阿克曼底盘已经非常激进
- 如果转弯太猛，可以 ↓ 减小到 1.5~2.0

### `max_acceleration`

每个控制周期允许的最大速度变化量：`max_speed_change = max_acceleration × dt`。

以 50Hz 控制频率计算：`0.5 × 0.02 = 0.01 m/s` 每周期，从 0 加速到 0.4 m/s 约需 40 个周期（0.8秒）。

| 现象 | 调整 |
|------|------|
| 起步/停车太突兀 | ↓ 减小到 0.3 |
| 加速太慢、响应迟钝 | ↑ 增大到 0.8 |

---

## 导航容差

```yaml
position_tolerance: 0.05   # m (5cm) 位置容差
angle_tolerance: 0.1       # rad (~5.7°) 角度容差
waypoint_timeout: 10.0     # s 单航点超时时间 (0=不超时)
```

### `position_tolerance` / `angle_tolerance`

这是全局默认值，航点 YAML 中每个航点可独立覆盖。

**注意**：代码中对到达判断使用的是**欧氏距离**，且有下限保护：
```
实际容差 = max(航点 dist_tol, 0.1m)
```
所以即使设成 0.05，实际生效的最小值也是 **0.1m（10cm）**。

### `waypoint_timeout`

单个航点的最大允许耗时。超时后直接跳过，进入下一个航点。

- 设为 0 表示不超时（可能永远卡在一个航点上）
- 建议根据航点间距和速度合理设置：`timeout ≈ 距离 / 速度 × 2`

---

## Pure Pursuit 预瞄控制

这是本功能包的核心控制算法。

```yaml
lookahead_distance: 0.5       # m 默认预瞄距离（绕行模式下锁定值）
min_lookahead_distance: 0.1   # m 最小预瞄距离（防止不稳定）
min_turning_radius: 0.5       # m 最小转弯半径（曲率钳位）
```

### Pure Pursuit 核心公式

```
曲率 κ = 2 × rel_y / L²
角速度 ω = κ × v
```

- `rel_y`：预瞄点在车辆坐标系中的横向偏移
- `L`：预瞄距离（预瞄点到车辆的距离）
- `v`：当前线速度

### `lookahead_distance`

**最关键的参数之一**，决定小车"看多远"来规划路径。

- 仅在**绕行模式**下作为固定值使用
- 正常导航时由[动态预瞄](#动态预瞄距离自适应)系统自动调节

| 值范围 | 效果 |
|--------|------|
| 0.3~0.4m | 跟踪精准，但可能震荡 |
| 0.5~0.7m | 平衡精准与平滑（推荐） |
| 0.8~1.0m | 路径平滑，但转弯切角严重 |

### `min_lookahead_distance`

预瞄距离的安全下限。预瞄点太近会导致角速度计算不稳定（分母趋近于零）。

- 代码中有保护：如果实际预瞄距离 < `min_lookahead_distance`，会按比例放大预瞄点位置
- 建议不要低于 0.1m

### `min_turning_radius`

限制最大曲率：`max_curvature = 1 / min_turning_radius`。

- 0.5m → 最大曲率 2.0 rad/m
- 0.3m → 最大曲率 3.33 rad/m（更急的弯）
- 阿克曼底盘物理上有一个最小转弯半径，设得太小会导致执行不了而偏离

| 现象 | 调整 |
|------|------|
| 转弯切角、走捷径 | ↓ 减小到 0.3~0.4（允许更急的弯） |
| 转弯时车轮打滑/超出底盘能力 | ↑ 增大到 0.6~0.8 |

---

## 动态预瞄（距离自适应）

正常导航时，系统根据**到目标航点的距离**自动调整预瞄距离，实现"远处看远、近处看近"。

```yaml
dynamic_lookahead_max: 0.8    # m 远处最大预瞄（距目标 >= dist_far_threshold 时）
dynamic_lookahead_min: 0.2    # m 近处最小预瞄（距目标 <= dist_near_threshold 时）
dist_far_threshold: 1.5       # m 远距离阈值
dist_near_threshold: 0.3      # m 近距离阈值
lookahead_lpf_alpha: 0.15     # 低通滤波系数(0~1), 越小越平滑
```

### 工作原理

```
距离 >= dist_far_threshold(1.5m)  → 预瞄 = dynamic_lookahead_max(0.8m)
距离 <= dist_near_threshold(0.3m) → 预瞄 = dynamic_lookahead_min(0.2m)
中间距离                           → 线性插值

再经过一阶低通滤波：
  filtered = alpha × target + (1 - alpha) × filtered_prev
```

### `dynamic_lookahead_max` / `dynamic_lookahead_min`

远处用大预瞄 → 路径平滑、减少震荡；近处用小预瞄 → 跟踪精准、到达判断更可靠。

| 现象 | 调整 |
|------|------|
| 远处路径摇摆不定 | ↑ max 增大到 1.0 |
| 接近航点时冲过头 | ↓ min 减小到 0.15 |
| 远处转弯切角严重 | ↓ max 减小到 0.6 |

### `dist_far_threshold` / `dist_near_threshold`

定义"远"和"近"的分界距离。

- 如果航点间距较短（< 1m），应 ↓ `dist_far_threshold` 到 0.8~1.0
- 如果航点间距较长（> 2m），可 ↑ `dist_far_threshold` 到 2.0

### `lookahead_lpf_alpha`

低通滤波系数，范围 0~1：

| 值 | 效果 |
|----|------|
| 0.05~0.10 | 极平滑，响应慢，预瞄变化有延迟 |
| 0.15~0.20 | 平滑优先（推荐） |
| 0.30~0.50 | 响应优先，预瞄变化快但可能跳变 |
| 1.00 | 无滤波，直接使用目标值 |

里程计数据跳动大时应减小此值。

---

## 倒车控制

```yaml
enable_reverse: true          # 启用倒车（目标在后方时）
reverse_angle_threshold: 0.8  # rad (~28.6°) 倒车最大横向偏角
```

### 倒车决策逻辑

```
if (预瞄点在车辆后方 && 横向偏角 < reverse_angle_threshold):
    启用倒车（线速度为负）
else:
    前进
```

其中横向偏角 = `atan2(|rel_y|, |rel_x|)`

### `enable_reverse`

- `true`：当目标在后方时自动倒车（阿克曼底盘的效率更高，省去180°旋转）
- `false`：始终前进，需要旋转180°才能前往后方目标

### `reverse_angle_threshold`

| 值 | 角度 | 行为 |
|----|------|------|
| 0.3 rad | ~17° | 只有几乎正后方才倒车，保守安全 |
| 0.5 rad | ~29° | 平衡安全与效率（推荐起步值） |
| 0.8 rad | ~46° | 激进倒车，允许较大横向偏移 |

当前默认值 0.8 偏激进。如果倒车不稳定，建议 ↓ 减小到 0.5。

---

## PID 角速度控制器

仅在**渐进式姿态混合**（接近航点时调整朝向）中使用。正常路径跟踪由 Pure Pursuit 控制。

```yaml
angular_kp: 3.0               # 比例增益
angular_ki: 0.0               # 积分增益
angular_kd: 0.5               # 微分增益（增大可减少振荡）
```

### 作用时机

当航点指定了目标 yaw（非 -999.0）且距离 < `yaw_blend_start_distance` 时，角度 PID 参与控制。

### 调参建议

| 现象 | 调整 |
|------|------|
| 到达航点时朝向调整太慢 | ↑ kp 增大到 4.0~5.0 |
| 到达航点时来回摆头 | ↑ kd 增大到 0.8~1.0，或 ↓ kp 减小到 2.0 |
| 始终存在稳态角度误差 | ↑ ki 增大到 0.1~0.2（一般不需要） |

---

## 渐进式姿态混合

阿克曼底盘不支持原地旋转，必须在运动中完成朝向调整。此参数控制"边行驶边偏航"的时机。

```yaml
yaw_blend_start_distance: 1.0  # m - 距航点多远开始混合姿态控制
```

### 工作原理

```
距离 > yaw_blend_start_distance → 纯 Pure Pursuit 路径跟踪
距离 < yaw_blend_start_distance → 逐渐混入角度 PID 控制
距离 ≈ position_tolerance       → 完全切换到姿态调整

混合公式:
  final_angular = (1 - blend) × PP_angular + blend × PID_angular
  blend = 1 - (distance - tolerance) / (blend_start - tolerance)
```

| 值 | 效果 |
|----|------|
| 0.5~0.8m | 晚开始调整，适合直线+末端微调 |
| 1.0~1.2m | 适中（推荐） |
| 1.5~2.0m | 早开始调整，适合大角度转弯 |

**注意**：如果值设得太大，远距离就开始偏航，会导致路径跟踪精度下降。

---

## 控制循环

```yaml
control_frequency: 50.0       # Hz 控制循环频率
```

- 50Hz → 每 20ms 一次控制输出，对阿克曼底盘足够
- 不建议低于 20Hz（控制延迟太大，容易震荡）
- 不建议高于 100Hz（里程计更新跟不上，PID 微分项噪声大）

---

## 避障绕行

基于 YOLO 检测结果，在路径前方插入 2 个临时绕行航点。

硬件参考：车 30×20cm，桶 20×20cm 高 30cm，图像 640×480。

```yaml
# --- 触发阈值 ---
avoid_area: 18000             # 像素面积，实际触发值 = (此值 - 4000)
avoid_trigger_bottom: 280     # 桶底部 y 坐标阈值（桶底 y 越大越近）
# --- 绕行几何 ---
avoid_detour_lateral_offset: 0.3   # m 横向闪避距离
avoid_detour_forward_step: 0.5     # m 每个绕行航点的前进步长
# --- 冷却 ---
avoid_detour_cooldown: 3.0    # s 两次绕行之间的最小间隔
# --- 速度 ---
avoid_linear_speed: 0.4       # m/s 绕行时速度
```

### 触发条件

绕行触发需要**同时满足**：
1. `max_barrel_bottom > avoid_trigger_bottom` （桶足够近，在画面下方）
2. `actual_barrel_area >= (avoid_area - 4000)` （桶面积足够大）
3. 冷却时间已过

> `actual_barrel_area` 经过透视校正：图像边缘的桶面积会被打折，推迟触发。

### 绕行轨迹

```
         WP1(侧移 0.3m + 前进 0.5m)
        / 
       /   锥桶
      /    ▢
  小车*─────────★────── 原始路径 ──────★
                WP2(回正，前进 1.0m)    原始WP_K
```

### 参数调优速查

| 现象 | 调整 |
|------|------|
| 撞上锥桶（触发太晚） | `avoid_area` ↓ 到 14000，`avoid_trigger_bottom` ↓ 到 240 |
| 绕得太早/太远 | `avoid_area` ↑ 到 22000，`avoid_trigger_bottom` ↑ 到 320 |
| 擦边撞上 | `avoid_detour_lateral_offset` ↑ 到 0.4~0.5 |
| 转弯太急擦边 | `avoid_detour_forward_step` ↑ 到 0.7~0.8 |
| 反复触发同一个桶 | `avoid_detour_cooldown` ↑ 到 5.0 |
| 遇第二个桶不反应 | `avoid_detour_cooldown` ↓ 到 1.5 |
| 绕行时转向来不及 | `avoid_linear_speed` ↓ 到 0.3 |

### 快速口诀

```
撞上了       → area↓  bottom↓  lateral↑  forward↑  speed↓
绕太早/太远  → area↑  bottom↑  lateral↓  forward↓
转弯擦边     → forward↑  speed↓
反复绕同一个 → cooldown↑
```

---

## 航点序列

```yaml
waypoints: [
  x, y, yaw, yaw_tol, timeout, dist_tol,
  ...
]
```

每个航点 6 个值：

| 字段 | 说明 |
|------|------|
| `x, y` | 目标位置（米，局部坐标系，前方 +X，左侧 +Y） |
| `yaw` | 目标朝向（弧度），**-999.0 表示自动决策** |
| `yaw_tol` | 角度容差（弧度） |
| `timeout` | 超时时间（秒），0 = 不超时 |
| `dist_tol` | 欧氏距离容差（米），下限保护 0.1m |

### yaw 自动决策逻辑

- **中间航点**：自动朝向下一个航点方向
- **最后一个航点**：保持当前朝向（不强制旋转）
- **指定 yaw**：使用精确角度值（如 `1.57` 表示朝左）

### 实际到达判断

```
到达条件 = 欧氏距离 < max(dist_tol, 0.1m)
```

代码中 dist_tol 下限保护为 0.1m，确保 Pure Pursuit 跟踪误差不会导致无法到达。

### 航点规划建议

- 相邻航点间距建议 **0.5~2.0m**（太近会频繁停车，太远会偏离路径）
- 直角转弯处建议在转弯前后各设一个航点
- 如需返回起点，**手动添加 (0.0, 0.0) 作为最后一个航点**

---

## 常见场景调参方案

### 场景 A：直线竞速（速度优先）

```yaml
max_linear_speed: 0.7
max_acceleration: 0.8
dynamic_lookahead_max: 1.0
dynamic_lookahead_min: 0.3
min_turning_radius: 0.4
```

### 场景 B：精确导航（精度优先）

```yaml
max_linear_speed: 0.3
max_acceleration: 0.3
dynamic_lookahead_max: 0.5
dynamic_lookahead_min: 0.15
lookahead_lpf_alpha: 0.1
min_turning_radius: 0.3
```

### 场景 C：直角弯道多

```yaml
max_linear_speed: 0.4
min_turning_radius: 0.35
dynamic_lookahead_min: 0.15
yaw_blend_start_distance: 1.2
angular_kp: 4.0
angular_kd: 0.8
```

### 场景 D：有锥桶障碍物

```yaml
max_linear_speed: 0.5
avoid_area: 16000
avoid_trigger_bottom: 260
avoid_detour_lateral_offset: 0.4
avoid_detour_forward_step: 0.6
avoid_linear_speed: 0.35
```

---

## 调试命令

```bash
# 重启节点使参数生效（无需重新编译）
bash run_inertial_nav.sh

# 查看实时位置
ros2 topic echo /odom_combined | grep position

# 查看控制输出
ros2 topic echo /cmd_vel --once

# 查看 YOLO 检测结果
ros2 topic echo /racing_obstacle_detection --no-arr --once --field targets

# 运行时动态修改参数（无需重启）
ros2 param set /inertial_nav_controller max_linear_speed 0.6
ros2 param set /inertial_nav_controller dynamic_lookahead_max 0.9

# 查看当前所有参数
ros2 param dump /inertial_nav_controller

# 查看控制频率
ros2 topic hz /cmd_vel
```

---

> 修改 `params.yaml` 后重启节点即可生效：
> ```bash
> bash run_inertial_nav.sh
> ```

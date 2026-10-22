# RobotModel4 v0.33 修改记录（预测流程与 CWNA）

> 基于 v0.32。范围：4 板 rmcs 像素 EKF 的**时间更新 / 状态外推拆分**、
> **observable 懒刷新**、**过程噪声模型**，以及 Aimer / Planner / Foxglove
> 的预测接口迁移。初始化、像素观测、anchor 关联和 NIS 门控的数学逻辑未改。

---

## 1. 本版解决什么问题

v0.32 的 `predict()` 同时做四件事：

```text
x 外推
P = F P Fᵀ + Q
observable 刷新（8 灯条投影 + 可见性）
推进时间戳
```

由此带来两个问题：

1. **瞄准/规划外推会顺带改协方差**
   Aimer 的 10 次迭代、Planner 的 101 步轨迹、Foxglove 的预测轨迹都只需要状态，
   但每次调用 `predict()` 都会加 Q、推进 P，协方差被无意义地反复放大。

2. **`full()` / `xyza_list()` 触发灯条投影**
   `full()` 只需要 `RobotSolution::solve_armors()`，但之前会先
   `refresh_observable()`，导致 4 块板的几何解算被 8 条灯条的 `projectPoints`
   拖慢；Aimer 和 Foxglove 恰好大量调用 `xyza_list()`。

v0.33 将“算目标在哪”和“算我们有多不确定”拆开，并让投影只在真正需要时发生。

---

## 2. 接口变化

### 2.1 `RobotModel`

```cpp
// 只做滤波时间更新：x、P += Q；要求 dt > 0
// 不刷新 observable，只标记 observable_dirty_
auto predict(double dt) -> void;

// 只推状态，不碰 P、不做投影；dt 可以为负
auto extrapolate(double dt) const -> State;

// 返回一个“状态已外推”的轻量副本；协方差不变
auto extrapolated(double dt) const -> RobotModel;
```

`predict()` 现在的顺序：

```cpp
state_ = extrapolate_state(state_, dt);
propagate_covariance(dt);
observable_dirty_ = true;
```

### 2.2 `Target`

```cpp
// 只给 tracker 用；基于时间戳推进滤波时间
void predict(std::chrono::steady_clock::time_point t);

// 只给 aimer / planner / Foxglove 用；轻量外推副本
auto at(double dt) const -> Target;

// 当前 posterior 对应的时间
auto time() const -> std::chrono::steady_clock::time_point;
```

`Target::at(dt)`：

```cpp
Target copy = *this;
copy.rmcs_model_ = rmcs_model_.extrapolated(dt);
copy.t_ += dt;
return copy;
```

- 不修改原 Target；
- 不推进原协方差；
- 支持负 `dt`，Planner 的 `fly_time - 0.51s` 回看不再被忽略。

### 2.3 删除的旧接口

| 旧接口 | 替代 |
|---|---|
| `Target::predict(double dt)` | `Target::at(dt)` 或 `Target::predict(time_point)` |
| `Target(double x, double vyaw, double radius, double h)` | 测试用 `Target::synthetic(...)` |
| `q_cwnd` | `process_model: cwna` |
| `q_rate_aware` | `process_model: diag_rate` |

---

## 3. observable 懒刷新

新增：

```cpp
mutable Observable observable_;
mutable bool observable_dirty_ = true;
```

刷新时机：

| 调用 | 是否刷新投影 | 说明 |
|---|---|---|
| `predict(dt)` | 否 | 只置 `observable_dirty_ = true` |
| `extrapolate(dt)` | 否 | 纯状态外推 |
| `extrapolated(dt)` | 否 | 副本置 dirty |
| `full()` | **否** | 只做 `solve_armors()` |
| `diagnostics()` | 是 | 若 dirty 则刷新一次 |
| `correct()` | 是 | 关联前保证 predicted pixels 是当前状态 |
| `update_one()` | 是 | 每更新一根灯条后重投影，保持 rmcs 顺序更新 |

### 3.1 为什么 `full()` 不再刷新

`full()` 只返回 4 块装甲板的世界系位姿：

```cpp
RobotSolution solution;
...
return solution.solve_armors();
```

它不需要 8 条灯条的像素投影，也不需要可见性。去掉 `refresh_observable()` 后：

```text
Target::at()
  → RobotModel::extrapolated()
Target::xyza_list()
  → Target::full()
  → RobotModel::full()
  → solve_armors()
```

Aimer / Planner / Foxglove 的预测路径不再触发 `projectPoints`，外推耗时从
约 18 µs 降到 1 µs 量级。

`correct()` 和 `diagnostics()` 仍会在需要时投影。

---

## 4. Aimer / Planner / Foxglove 迁移

### 4.1 Aimer

不再复制 10 个 Target 后逐个 `predict()`：

```cpp
const auto base = targets.front();
const auto target = base.at(dt0);

for (int iter = 0; iter < 10; ++iter) {
    const auto iter_target = base.at(dt0 + prev_fly_time);
    const auto aim_point = choose_aim_point(iter_target);
    ...
}
```

- 从原始 posterior 状态重新外推，不累积上一轮；
- 每次只做状态外推 + 选板几何，不再改 P/Q；
- 选板逻辑仍保持逐次重新评估。

### 4.2 Planner

`get_trajectory()` 改为从 base Target 按绝对时间偏移外推：

```cpp
auto yaw_pitch_last = aim(target.at(base_dt - DT * (HALF_HORIZON + 1)), ...);
auto yaw_pitch      = aim(target.at(base_dt - DT * HALF_HORIZON), ...);

for (int i = 0; i < HORIZON; ++i) {
    auto yaw_pitch_next =
        aim(target.at(base_dt + (i + 1 - HALF_HORIZON) * DT), ...);
    ...
}
```

`base_dt` 是初始弹道飞行时间。这样：

- 负 `dt` 正常生效；
- 101 步轨迹不会重复推进协方差；
- 轨迹窗口仍是 `fly_time - 0.5s ... fly_time + 0.5s`。

### 4.3 Foxglove

预测轨迹和 ghost 装甲板改为：

```cpp
target.at(step_s * i)
```

不再使用累积的 `predict(step_s)`。

---

## 5. 过程噪声模型

### 5.1 新配置

```yaml
process_model: cwna | dwna | diag_rate
q_accel_xy: 1.0     # (m/s²)²·s
q_accel_z: 1e-4     # (m/s²)²·s
q_alpha: 10.0       # (rad/s²)²·s
```

三份配置（`daedalus.yaml`、`sentry.yaml`、`standard3.yaml`）默认：

```yaml
process_model: cwna
q_accel_xy: 1.0
q_accel_z: 1e-4
q_alpha: 10.0
```

### 5.2 CWNA

```text
Q_pp = q · dt³ / 3
Q_pv = q · dt² / 2
Q_vv = q · dt
```

- `q_accel_xy` 用于 x/y；
- `q_accel_z` 用于 z；
- `q_alpha` 用于 a/w；
- 单位是加速度谱密度：`(m/s²)²·s`、`(rad/s²)²·s`。

CWNA 的优点是换帧率不需要重调；位置和速度协方差通过 `Q_pv` 耦合。

### 5.3 DWNA（sp25 对照）

```text
Q_pp = q · dt⁴ / 4
Q_pv = q · dt³ / 2
Q_vv = q · dt²
```

这是 sp25 原版 `Target::predict()` 的 dt 策略。用于 A/B 时，100 Hz 起点建议：

```yaml
q_accel_xy: 100
q_alpha: 900
```

### 5.4 DIAG_RATE（旧 rmcs）

```text
Q = diag(noise_x, ..., noise_rotation_speed, ...) · dt
```

用于回退和对照；`noise_*` 仅在该模式以及半径/高度的随机游走中使用。

### 5.5 半径/高度

`rf/rl/hl` 没有速度状态，三种模型都保留：

```text
Q_rf = noise_radius_forward · dt
Q_rl = noise_radius_lateral · dt
Q_hl = noise_height_lateral · dt
```

---

## 6. 修改文件

| 文件 | 内容 |
|---|---|
| `tasks/auto_aim/rmcs_ekf/rmcs_ekf.hpp/.cpp` | 拆分 predict/extrapolate、observable dirty、CWNA/DWNA/DIAG_RATE、`full()` 去投影 |
| `tasks/auto_aim/target.hpp/.cpp` | `at()`、`time()`、`Target::synthetic()`、删除 `predict(double)` |
| `tasks/auto_aim/aimer.cpp` | 10 次迭代改用 `at()` |
| `tasks/auto_aim/planner/planner.cpp/.hpp` | 101 步轨迹改用 `at()`，负 dt 生效 |
| `tools/Foxglove/foxglove.cpp` | 预测轨迹 / ghost 改用 `at()` |
| `configs/{daedalus,sentry,standard3}.yaml` | `process_model` / `q_accel_*` / `q_alpha` |
| `tests/planner_test*.cpp` | `Target::synthetic()`、`target = target.at(0.01)` |
| `doc/EKF调试.md` | Q 模型与单位说明 |

---

## 7. 验证

### 7.1 构建

```bash
cmake --build build -j"$(nproc)"
```

`standard`、`sentry_DGE` 通过。

### 7.2 测试

- `planner_test_offline`：完整编译并运行；
- `planner_test`：源文件可编译，但链接仍缺 `io::Gimbal`，原因是
  `io/CMakeLists.txt` 中 `gimbal/gimbal.cpp` 被注释掉，属于既有问题。

### 7.3 仿真重点场景

“假人反复起转和停转”：

| 指标 | 关注点 |
|---|---|
| `ekf_reinits` | 不应持续增长 |
| `vyaw_err` | 起转/停转峰值 |
| `nis` / `nis_fail_rate` | 是否长期超阈值 |
| `ekf_frame_rejected` | 是否长期接近可见灯条数 |
| `converged/diverged` | 是否反复掉目标 |

起始调参建议：

```yaml
process_model: cwna
q_accel_xy: 1.0
q_accel_z: 1e-4
q_alpha: 10.0
```

该组参数是 CWNA 下的均衡起点；若停转收敛慢，优先增大 `q_alpha`；若平移起停抖动大，
再调 `q_accel_xy`。`q_accel_z` 保持小值，避免高度方向被噪声带飞。

---

## 8. 已知限制

1. `diagnostics()` 仍会刷新投影，每帧一次；`full()` 和 `xyza_list()` 不会。
2. `Target::at()` 的协方差仍是 posterior 协方差，未做时间传播；不要用 `at()` 的
   `diagnostics()` 判断未来时刻的不确定性。
3. `planner_test` 的链接依赖 `io/gimbal/gimbal.cpp`，启用测试前需要先把它加入
   `io` 目标。
4. CWNA 参数尚未在真实仿真闭环中完成最终回归；本文的 1.0 / 1e-4 / 10.0 是起点，
   不是最终比赛参数。

# RobotModel4 v0.32 修改记录（观测侧修复）

> 基于 v0.21（`2bc3427 RobotModel4 v0.21`，文档 `RobotModel4_v0.3.md`）。
> 范围：4 板像素 EKF 的**初始化观测源**与**编译/数值正确性**。滤波器结构、Q/R、
> 门控、收敛判据均未改动。新行为都有 YAML 开关，可回退做 A/B。

---

## 1. 修改清单

| # | 类型 | 改动 | 位置 |
|---|---|---|---|
| 1 | 编译修复 | `nis` 由 Eigen 表达式改为 `double`（v0.21 的 `std::max(double, 表达式)` 编译不过） | `rmcs_ekf.cpp` `update_one()` |
| 2 | **Bug 修复** | RobustPnp 重投影 yaw 搜索中 `const auto point = R * Vector3d{...} + t` 是 Eigen 惰性表达式，引用已析构的临时向量（未定义行为）。改为显式 `Eigen::Vector3d` | `rmcs_ekf.cpp` `RobustPnp::solve()` |
| 3 | 健壮性 | `innovation_covariance` / `innovation_covariance_inv` / `kalman_gain` 改为显式类型，避免惰性表达式重复求值（含求逆） | `rmcs_ekf.cpp` `update_one()` |
| 4 | 观测源 | 初始化默认优先使用 Solver 已解算的世界系板位姿，RobustPnp 作兜底（可切换） | `target.cpp` 构造函数 |
| 5 | 诊断 | 记录并发布初始化来源 `ekf_init_source` | `target.hpp`、`foxglove.cpp` |
| 6 | 性能（默认关） | 跟踪阶段 4 板目标可跳过逐板 `Solver::solve` | `tracker.cpp` `update_target()` |
| 7 | 配置 | 三份配置新增 `init_prefer_solver_pose`、`skip_tracking_pnp` | `configs/{daedalus,standard3,sentry}.yaml` |

### 1.1 #2 的影响（本版本最重要的修复）

- 现象：`-O2` 下每个候选 yaw 的重投影误差完全相同，搜索停在窗口左边界，
  **初始化 yaw 恒偏 -30°，旋转中心恒偏 10.35 cm**（= 2 × 0.2 m × sin15°）。
- 波及：所有走 `RobotModel::initialize()` 的路径——v0.21 的默认初始化，以及
  v0.3 新增的 `auto re-init`。v0.3 把 P0 收紧到 σ_a≈18°，-30° 的初值会被“锁”在
  错误解附近，表现为首帧/重初始化后角度偏、收敛慢、反复 reinit。
- 修复后合成测试：RobustPnp 在任意朝向下 yaw 误差 0°、中心误差 0 cm。

### 1.2 #4 为什么默认用 Solver 位姿

Solver（sp_vision）在 PnP 后做 140° 窗口的 yaw 重投影优化，且用的就是 DGE 的
`R/t_camera2gimbal`。合成测试（DGE 真实 Solver + `daedalus.yaml` 外参，每格 200 次）：

| 距离 | 角点噪声 | 中心 RMS（Solver / RobustPnp） | yaw RMS（Solver / RobustPnp） |
|---|---|---|---|
| 2.5 m | 0.5 px | **5.9** / 8.0 cm | **2.4°** / 11.6° |
| 2.5 m | 1.5 px | **11.5** / 14.2 cm | **8.0°** / 17.2° |
| 2.5 m | 3.0 px | 19.1 / 19.9 cm | **14.9°** / 18.8° |
| 5.0 m | 0.5 px | **14.1** / 16.8 cm | **5.8°** / 18.4° |
| 5.0 m | 1.5 px | 36.8 / 37.1 cm | **16.1°** / 20.3° |
| 5.0 m | 3.0 px | 73.2 / 72.7 cm | 23.6° / 22.9° |

（yaw 误差按四板 90° 对称取模；中心误差主要来自单板 PnP 深度噪声，两者相当。）

### 1.3 闭环 A/B（合成，冷启动走 RobustPnp，5 rad/s，1.5 px，100 次随机初始朝向）

| 版本 | 首帧中心误差 | 0.5 s | 2 s | reinit 总数 |
|---|---|---|---|---|
| v0.21 | 19.84 cm | 5.22 cm | 0.87 cm | 28 |
| v0.32 | 16.70 cm | **2.29 cm** | 0.73 cm | **18** |

稳态闭环（Solver 位姿初始化，2 px，300 帧）：中心 RMS 1.05 / 1.12 / 1.24 cm
（静止 / 3 / 8 rad/s），ω RMS ≤ 0.09 rad/s，0 次拒绝、0 次 reinit。

---

## 2. 新 YAML 参数（`ekf:` 节）

```yaml
ekf:
  # v0.32 观测侧（可 A/B 回退）
  init_prefer_solver_pose: true   # 初始化优先用 Solver 世界系板位姿；false = 优先 RobustPnp
  skip_tracking_pnp: false        # true: 跟踪阶段 4 板目标跳过逐板 PnP（省 CPU）
```

- 两个键都可缺省，缺省值如上。
- `skip_tracking_pnp: true` 会让 Foxglove `/dge/scene` 的 detections 方块和
  detector markers 失去世界系位姿（EKF 本身不受影响），所以默认关。
- 回到 v0.21 的初始化行为：`init_prefer_solver_pose: false`（但 #2 的修复仍生效）。

---

## 3. Foxglove 新字段

`/dge/debug`：

```text
ekf_init_source   0 = 未初始化
                  1 = Solver 位姿初始化
                  2 = RobustPnp 初始化（Solver 路径失败时兜底）
```

`ekf_reinits` 的重初始化固定走 RobustPnp，这条路径正是 #2 修复的对象。

---

## 4. 仿真 + Foxglove 测试清单

启动：

```bash
# 终端 1：仿真器
cd ~/桌面/simulation/bevy_robomaster_simulator-master && cargo run --release
# 终端 2：DGE
cd ~/桌面/DGE_vision_26 && ./build/standard configs/daedalus.yaml
# Foxglove：ws://localhost:8765，加载 foxglove_layouts/dge_debug_layout.json
```

建议在 Plot 面板添加：`ekf_init_source`、`ekf_reinits`、`center_err`、`yaw_err`、
`vyaw_err`、`anchor_err_px`、`nis` + `nis_threshold`、`converged`。

| 场景 | 操作 | 看什么 | 通过标准 |
|---|---|---|---|
| T1 首帧 | 静止假人 3 m，反复丢失/重新锁定（Tab 移出视野再移回） | 锁定瞬间 `center_err`、`yaw_err` | `ekf_init_source=1`；首帧 `yaw_err` 不再出现 ±30° 平台；`center_err` < 0.15 m |
| T2 收敛 | 同上 | 锁定到 `converged=1` 的时间 | 明显短于 v0.21（预期 ≤ 0.3 s） |
| T3 重初始化 | 小陀螺 6 rad/s，遮挡/切板 | `ekf_reinits` 增长后的 `center_err`、`anchor_err_px` | reinit 后 0.5 s 内 `center_err` < 0.05 m；`ekf_reinits` 增长比 v0.21 少 |
| T4 稳态 | 小陀螺 0 / 3 / 8 rad/s | `vyaw_err`、`nis` | `vyaw_err` < 0.5 rad/s；`nis` 大多 < 9.488 |
| T5 回退 | `init_prefer_solver_pose: false` 重跑 T1 | `ekf_init_source=2` | 首帧指标与 T1 接近（说明 #2 修复后 RobustPnp 也可用） |
| T6 性能（可选） | `skip_tracking_pnp: true` | 帧率 / 单帧耗时 | 帧率不降；EKF 指标与 T4 一致 |

A/B 时一次只改一个开关。v0.21 对照需要 `git stash` 或切回 `2bc3427`（并手动
套用 #1 才能编译）。

---

## 5. 已知限制 / 本版未做

- 初始化仍只用 `armors.front()` 一块板；单板 PnP 深度噪声远距离较大（见 1.2 表）。
  后续可用整帧多板联合初始化。
- 单灯条观测（`lightbars`）仍未接入：YOLO 路径不输出灯条，`correct()` 的
  `lightbars` 恒为空。
- 关联仍是“锚板 + 按 x 排序连续编号”。合成测试中可见前向板的“机器人左灯条”
  始终在图像左侧，未发现 swap 方向错误，本版未改。
- 角点 R 仍为常数 `noise_observation`；未按灯条像素长度/置信度自适应。

---

## 6. 验证记录

- 构建：`cmake --build build`，`standard`、`sentry_DGE`、标定工具全部通过。
- 合成测试：RobustPnp 多朝向/多俯仰、Solver 位姿初始化、噪声扫描、闭环
  静止/旋转、冷启动 A/B（结果见第 1 节）。
- 仿真冒烟：Daedalus + `standard configs/daedalus.yaml` 运行 30 s，共享内存连接正常，
  无报错/崩溃。目标级指标（T1–T6）需在 Foxglove 下人工驾驶假人测试。

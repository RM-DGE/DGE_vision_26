# RobotModel4 v0.50 修改记录（EKF 初始化几何与跟踪诊断）

> 基于 v0.40（`test` 分支）。只合并 new-sim 工程对 EKF 的改动：
> **4 板像素 EKF 的初始化几何**，以及 **Tracker 的帧间隔 / 空帧诊断清理**。
> 不包含 `rm_aim_sim` 适配层、Foxglove 模型可视化、布局、测试和 `armor.hpp`
> 默认值等改动。
>
> 来源：new-sim 提交 `5f2d9a4`
> （Fix EKF initialization geometry and tracking diagnostics; add pixel replay）
> 中与 EKF 相关的部分。

---

## 1. 本版解决什么问题

### 1.1 初始化几何：模型板和实测板对不上

v0.40 的 `RobotModel::initialize()` 在 RobustPnP 得到 `armor3d` 后，用下面这段
算旋转中心：

```cpp
const auto armor_to_center = Eigen::Vector3d{
  Eigen::AngleAxisd{ -kPredictedOtherArmorPitch, orientation * Eigen::Vector3d::UnitY() }
  * (orientation * Eigen::Vector3d::UnitX()) };
const auto yaw = normalize_angle(std::atan2(-armor_to_center.y(), -armor_to_center.x()));
const auto center = Eigen::Vector3d{ armor3d.translation + 0.2 * armor_to_center };
```

问题在于：

- `RobotSolution` 把四块板放在**水平半径**上（`center ± r·(cos a, sin a)`），
  初始化的 `center` 也必须落在同一水平面上；
- `Solver` 的 yaw 优化只固定 pitch，**保留了原始 IPPE 的 pitch/roll**，这部分
  在远距离/小板上噪声很大；
- 再绕装甲板 y 轴减一个固定的 `15°`（`kPredictedOtherArmorPitch`）并不能把
  姿态“校正”到水平，反而让 `armor_to_center` 带上竖直分量；
- 于是 `center = translation + 0.2·armor_to_center` 被抬出平面，模型里的 0 号板
  与实测板中心不重合。首帧 anchor 误差偏大、NIS 偏大，严重时会触发
  `auto re-init`，表现为“刚锁定就重初始化”或“模型板比实测板高/低一截”。

### 1.2 空帧诊断：没检测到目标时 `/dge/debug` 保留上一帧

v0.40 的 `Tracker::update_target()` 在 `found_count == 0` 时直接 `return false`，
没有调用 `Target::update()`。于是：

- `frame_update_count / frame_rejected / nis / anchor_error / residual_norm`
  保留上一帧的值，Foxglove 看起来像“这一帧还在更新”；
- `ekf_no_update_frames` 的语义被污染，排查“连续无更新”时会误判。

另外，`dt > 0.1` 是硬编码的，仿真/CPU 推理卡顿时无法单独放宽，只能改代码。

---

## 2. 初始化几何修正

`initialize()` 不再自己算 `armor_to_center`，而是直接复用
`initialize_from_pose()`：

```cpp
auto RobotModel::initialize(const CameraFeature & camera,
  const std::vector<Armor2d> & armors) -> bool
{
  // ... 选最宽的板、RobustPnp ...
  if (!pnp.solve(armor3d)) return false;
  return initialize_from_pose(camera, armor3d.translation, armor3d.orientation, large_);
}
```

`initialize_from_pose()` 的新逻辑：

```cpp
if (!translation.allFinite() || !orientation.coeffs().allFinite()
    || orientation.norm() < kEpsilon) return false;

// RobotSolution 的板在水平半径上；Solver 的 yaw 优化固定 pitch，
// 但 IPPE 的 pitch/roll 仍有噪声。先投影到水平面，再归一化。
Eigen::Vector3d armor_to_center = orientation.normalized() * Eigen::Vector3d::UnitX();
armor_to_center.z() = 0.0;
if (armor_to_center.norm() < kEpsilon) return false;
armor_to_center.normalize();

large_ = large;
const auto yaw = normalize_angle(std::atan2(-armor_to_center.y(), -armor_to_center.x()));
const auto center = Eigen::Vector3d{ translation + 0.2 * armor_to_center };
```

要点：

- **去掉固定的 `-15°` 旋转**，改为“取 PnP 的 +x，投影到水平面，归一化”；
- 保留 `0.2 m` 的初始半径（与 v0.40 一致），只修正方向；
- 增加 `translation/orientation` 有限性和零模长检查，避免 NaN 姿态进入
  `finish_initialize()`；
- `initialize_from_pose()` 仍是 fallback 入口：当 RobustPnP 失败时，`Target`
  用 Solver 的 `xyz_in_world/ypr_in_world` 调它，两条路径现在共享同一套几何。

**效果**：首帧 `full()` 的 0 号板与实测板重合，`anchor_err_px` 回到角点噪声量级
（正常 <10 px），不再因为初始化偏置触发重初始化；180° 朝向检查逻辑不变。

---

## 3. Tracker：帧间隔与空帧诊断

### 3.1 `max_frame_gap_s`

`Tracker` 新增配置项，默认 `0.1 s`，替代两处硬编码的 `0.1`：

```yaml
# configs/*.yaml（可选；不写就是 0.1）
max_frame_gap_s: 0.1
```

```cpp
max_frame_gap_s_ = yaml["max_frame_gap_s"].as<double>(0.1);
if (!std::isfinite(max_frame_gap_s_) || max_frame_gap_s_ <= 0)
  throw std::invalid_argument("max_frame_gap_s must be finite and positive");
```

超过阈值时日志改为带原因的格式：

```text
[Tracker] reset reason=frame_gap dt=0.213s limit=0.100s
```

仿真/CPU 推理争用导致偶发 >100 ms 时，可以在仿真配置里单独设 `0.2`，
不影响真机的 0.1 s 判据。

### 3.2 空帧清理

`found_count == 0` 时主动调用一次空更新：

```cpp
if (found_count == 0) {
  // 清空本帧诊断，不把“没检测到”算成“被拒绝的观测”。
  // 恢复仍由 lost 状态机控制。
  target_.update({}, solver_.camera_feature());
  return false;
}
```

`Target::update({}, camera)` 会把 `armors2d` 置空后调用
`rmcs_model_.correct({}, {}, camera)`；EKF 的 `correct()` 在空输入分支只清
`frame_update_count / frame_rejected_count / nis / anchor_error / residual_norm`
等本帧字段，然后返回，不会增加 `no_update_frames`，也不会触发重初始化。

**效果**：

- `/dge/debug.ekf_frame_updates` 在丢检测帧真实归零；
- `ekf_frame_rejected` 不会因为“没检测到”而虚增；
- `nis / anchor_err_px` 不会保留上一帧的值；
- 连续无更新的判断重新可信，恢复仍走 `temp_lost → lost → set_target`。

---

## 4. 改动文件

| 文件 | 改动 |
|---|---|
| `tasks/auto_aim/rmcs_ekf/rmcs_ekf.cpp` | `initialize()` 复用 `initialize_from_pose()`；水平化 + 归一化 + 有限性检查 |
| `tasks/auto_aim/tracker.cpp` | `max_frame_gap_s` 读取/校验/日志；空帧 `target_.update({}, camera)` |
| `tasks/auto_aim/tracker.hpp` | 新增 `max_frame_gap_s_` 成员 |
| `doc/RobotModel4_v0.50.md` | 本文 |

未改 EKF 的预测、观测、关联、NIS、Q/R/P0、门控和收敛/发散判据；
未改 `Target`、`Aimer`、`Solver`、`Shooter` 的接口。

---

## 5. 未合并的内容（有意排除）

以下 new-sim 改动与仿真/可视化相关，不属于本次 EKF 回移：

- `io/rm_aim_sim/*`、`src/auto_aim_sim.cpp`：rm_aim_sim 适配层；
- `tools/Foxglove/ekf_model.*`、`foxglove.cpp/hpp` 的模型/选板可视化、
  `foxglove_layouts/*`；
- `tests/ekf_association_test.cpp`、`tests/ekf_replay.cpp` 及 CMake 测试目标
  （`ekf_replay` 依赖 `io/rm_aim_sim/local_client.hpp`，需要适配层才能编译）；
- `configs/rm_aim_sim.yaml` 的 `max_frame_gap_s: 0.2`（仿真专用）；
- `tasks/auto_aim/armor.hpp` 的 `priority` 默认值；
- `3fb4100` 的 carousel 选板 / aimer / Foxglove 改动。

这些可以后续单独评估；本次只保证 EKF 初始化几何和跟踪诊断在真机链路上一致。

---

## 6. 兼容性与配置

- 接口无变化：`RobotModel` / `Target` / `Tracker` 的公开签名不变；
- `max_frame_gap_s` 可选，缺省 `0.1`，与 v0.40 行为一致；
- 真机建议保持 `0.1`；仿真若因 CPU 推理/渲染争用出现 100–200 ms 帧间隔，
  可单独设 `0.2`，不要为此改真机配置；
- EKF 的 Q/R/P0、NIS 门控、`auto_reinit_after_no_update` 等参数不受本版影响。

---

## 7. 验证建议

1. **构建**：`standard`、`sentry_DGE` 正常编译；没有新增源文件/CMake 目标。
2. **静态目标 2/4/6 m**：
   - 首帧 `anchor_err_px` 应 <10 px；
   - `ekf_reinits` 不应增长；
   - `center_err` 在仿真里应从第一帧就接近 0（不再有“模型板抬高”导致的固定偏差）。
3. **丢检测帧**：
   - `ekf_frame_updates` 归零，`ekf_frame_rejected` 不增加；
   - `nis / anchor_err_px` 归零，不再显示上一帧的值。
4. **大帧间隔**：
   - 超过 `max_frame_gap_s` 时出现 `reset reason=frame_gap` 日志；
   - `state_` 进入 lost，下一帧重新捕获。
5. **A/B**：同一段录制/仿真，v0.40 与 v0.50 各跑一次，比较首帧
   `anchor_err_px`、`center_err` 和 `ekf_reinits`。

---

## 8. 参考

- new-sim 提交 `5f2d9a4`：EKF 初始化几何修复、Tracker 诊断清理、像素回放；
- v0.40 文档：`doc/RobotModel4_v0.40.md`（ROS2 桥接，与本版正交）；
- EKF 调参/字段说明：`doc/EKF调试.md`。

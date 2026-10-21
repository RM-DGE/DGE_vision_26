# RobotModel4 v0.3 改动说明（4 板像素 EKF）

> 范围：只改 `tasks/auto_aim/rmcs_ekf/`（11 维、4 板、像素观测），
> legacy 2/3 板 EKF 和 Target/Tracker 的接口不变。
>
> 目标：先把“首帧刷飞 / 关联坏一次就永久卡死 / Q 和帧率绑定 /
> NIS 口径不对”这四个会直接毁掉版本测试的问题修掉，所有新行为都留了
> YAML 开关，方便和 v0.2 做 A/B。

---

## 1. 改了什么

| # | 改动 | 位置 | 解决的问题 |
|---|---|---|---|
| 1 | P0 从 64/64/0.4/100 收紧到可配置的小值 | `reset_covariance()` | 首帧被单块板的“像面高度”拖飞几米 |
| 2 | Q 按 `dt` 归一（`q_rate_aware`） | `predict()` | 换帧率（60/120/200 fps）行为漂移；w 估计噪声大 |
| 3 | anchor 角点误差门控（绝对值 + 相对板宽） | `correct()` | 错误关联/模型转 90° 时仍把状态拉走 |
| 4 | 连续 0 次接受更新 → 自动重初始化 | `correct()` / `maybe_reinitialize()` | 状态跑到相机后面后永久卡死 |
| 5 | 连续 0 更新 → `diverged()` | `diverged()` | 卡在“中间值”时 tracker 不会丢目标 |
| 6 | NIS 改为“单条最大值”，新增均值 | `correct()` / `diagnostics()` | 帧均值与 χ²(4) 阈值不同口径，调参会误判 |
| 7 | converge 判据加 σ_v、σ_w、最近更新 | `converge()` | 原来 `1.0 m²` + 10 帧几乎恒真 |
| 8 | w 限幅 | `predict()` / `update_one()` | 角速度跑飞 |
| 9 | 新增诊断字段 | `Diagnostics` + Foxglove | 能看到 anchor 误差、连续无更新、重初始化次数 |

---

## 2. 新 YAML 参数（`ekf:` 节）

```yaml
ekf:
  # --- 过程噪声 ---
  q_rate_aware: true          # true: noise_* 按“每秒方差”解释，predict 里乘 dt
                              # false: 回到 rmcs 每帧常量 Q（A/B 用）

  # --- 初始协方差 ---
  initial_cov_position: 0.04       # x, y  (m^2)
  initial_cov_z: 0.01
  initial_cov_velocity: 4.0        # vx, vy ((m/s)^2)
  initial_cov_vz: 0.01
  initial_cov_angle: 0.1           # rad^2
  initial_cov_rotation_speed: 4.0
  initial_cov_radius: 0.001        # rf, rl
  initial_cov_height: 0.001        # hl

  # --- 关联 / 恢复 ---
  anchor_max_corner_error: 40.0    # 平均角点误差绝对门限（px），<=0 关闭
  anchor_max_relative_error: 0.5   # 相对预测板宽的门限（0.5 = 半个板宽）
  auto_reinit_after_no_update: 10  # 连续 10 帧有检测但 0 更新 -> 重初始化；0 关闭
  max_no_update_frames: 30         # 超过 30 帧 -> diverged()；0 关闭

  # --- 收敛 / 发散 ---
  converge_max_cov_xy: 0.04        # σ_xy ≈ 20 cm
  converge_max_cov_v: 1.0          # σ_v ≈ 1 m/s
  converge_max_cov_w: 1.0          # σ_w ≈ 1 rad/s
  converge_max_no_update_frames: 5
  rotation_speed_limit: 25.132741228718345  # 8π rad/s，<=0 关闭

  # 原有参数保持不变
  noise_observation: 100.0
  nis_threshold: 9.488
  nis_gate_enabled: true
  nis_gate_threshold: 13.277
  ...
```

所有键都可缺省，缺省值就是上表的值。

---

## 3. 行为变化

### 3.1 Q 的语义
- `q_rate_aware: true` 时：`Q_used = diag(noise_*) * dt`。
  - 100 Hz、`noise_vx=0.2` → 每帧 `Q_v=0.002`；
  - 120 Hz → `Q_v≈0.00167`；60 Hz → `0.0033`。
  - 物理含义：每秒的过程噪声一致，换帧率不用重调。
- `q_rate_aware: false` 时：和 v0.2 完全一致（每帧 `Q_v=0.2`）。
- 注意：`noise_*` 的数值本身没有改，只是解释方式变了。如果实车
  发现跟机动变慢，优先加 `noise_vx/noise_vy/noise_rotation_speed`
  （按每秒方差加），而不是关掉 `q_rate_aware`。

### 3.2 P0
- 首帧不再被“单块板的像面高度”主导，中心不会被甩出几米。
- `initial_cov_angle` 从 0.4 收到 0.1（σ≈18°）；如果现场 PnP yaw
  初值经常 >20°，可以把它调回 0.2~0.4。
- `initial_cov_radius=1e-3`（σ≈3 cm）：半径只能在 3 cm 内缓慢调整，
  避免沿“中心↔半径”零空间漂移。

### 3.3 关联门控
- 每帧选 anchor（4 角点误差和最小）后，先看平均角点误差：
  `max(40 px, 0.5 × 预测板宽)`；超过就整帧丢弃，不进 EKF。
- 这样错误关联（差一块板/差 90°）不会再把状态拉走；同时近距离
  大板（板宽 150+ px）不会被 40 px 的绝对门限误杀。
- 被丢弃的帧会记 `anchor_err_px`，并让 `no_update_frames+1`。

### 3.4 失锁恢复
- 连续 `auto_reinit_after_no_update` 帧“有检测但 0 条灯条被接受”：
  - 用当前检测重新跑 `RobustPnp` + `finish_initialize`；
  - `reinit_count+1`，日志打 `[rmcs_ekf] auto re-init ...`；
  - 之后 anchor 的 0 号板就是当前观测板，`jumped=false`。
- 重初始化失败会每 10 帧重试一次；超过 `max_no_update_frames`
  则 `diverged()` 返回 true，tracker 丢目标、下一帧重新 `set_target`。

### 3.5 NIS
- `nis` 现在是**当前帧单条灯条的最大 NIS**，和 χ²(4) 的 9.488 同口径；
- `nis_mean` 是帧内均值，只作参考；
- `nis_fail_rate` 仍是滑动窗口内单条失败比例。
- Foxglove 里建议 `nis` 和 `nis_threshold` 一起画，`nis_mean` 放副轴。

### 3.6 converge / diverged
- converged 需要同时满足：≥10 帧、最近 5 帧内有接受更新、
  σ_xy<20 cm、σ_v<1 m/s、σ_w<1 rad/s、初始化后 >0.1 s。
- diverged 先查 NaN，再查“连续 0 更新”，最后才是原来的状态边界。
  这样卡死不会因为状态值“看起来正常”而漏掉。

---

## 4. Foxglove 新字段

`/dge/debug` 新增：

```text
nis_mean            当前帧 NIS 均值（参考）
anchor_err_px       anchor 平均角点误差（像素）
ekf_no_update_frames 连续“有检测但 0 更新”的帧数
ekf_reinits         累计自动重初始化次数
```

原有字段不变；`nis` 的含义从“帧均值”改为“单条最大值”。

建议 Plot：

- `nis` + `nis_threshold`；
- `nis_mean`；
- `anchor_err_px`（正常 <10 px，>40 px 说明关联/状态有问题）；
- `ekf_no_update_frames`（应长期 0；>5 看门控/关联）；
- `ekf_reinits`（稳定后不应增长）；
- `converged` / `diverged`；
- `sigma_x/y/a/w`、`state` vs `gt`（仿真）。

---

## 5. 版本测试清单

### 5.1 仿真（daedalus）

| 场景 | 看什么 | 通过标准 |
|---|---|---|
| 静止 2/4/6 m | 首帧不跳、center_err、sigma | 无 `diverged`，无 reinit，center_err <0.1 m |
| 横移 1/2 m/s | v 估计、预测超前/滞后 | 稳态 v 误差 <0.3 m/s |
| 小陀螺 0→2→6 rad/s | w、vyaw_err、`P(\|w\|>1.5)` | w 误差 <0.5 rad/s，不误判 |
| 切板/遮挡 | `ekf_no_update_frames`、reinit | 短暂遮挡不 reinit；恢复后 3 帧内更新 |
| 前哨站 | 走 legacy，不受本版本影响 | 行为与 v0.2 一致 |

每场景至少跑 3 次（不同噪声/初始角），记录：

- 中心/yaw/vyaw 的 RMSE（`center_err/yaw_err/vyaw_err`）；
- NIS 均值、`<9.488` 覆盖率（目标 90–98%）；
- 首帧最大中心跳变；
- `ekf_reinits`、`diverged` 次数；
- 开火时刻 `aim` 与实际板中心的角度误差（可选）。

### 5.2 实车

1. 先 `calibrate_validate`：世界系板位置 std <10 mm；
2. 静止目标：`anchor_err_px` 应 <10 px；`nis` 应在 4 附近；
3. 小陀螺：看 `sigma_w` 和 `w` 是否跟得上，`ekf_reinits` 是否增长；
4. 如果 `nis` 长期 ≪4：`noise_observation` 偏大；长期 ≫9.488：
   先查标定/关联，再考虑加 R。

### 5.3 A/B 建议

| 变量 | A | B |
|---|---|---|
| Q 语义 | `q_rate_aware: false`（v0.2） | `true`（v0.3） |
| P0 | `initial_cov_position: 64` 等 | 0.04 等 |
| 恢复 | `auto_reinit_after_no_update: 0` | 10 |
| 门控 | `anchor_max_corner_error: 0` | 40 |

一次只动一个，否则出了问题无法归因。

---

## 6. 已知权衡 / 回退

- **P0 收紧后，首帧对大 yaw 误差的修正变慢**。如果实车 PnP yaw
  初值经常 >20°，把 `initial_cov_angle` 调回 0.2~0.4。
- **Q 按 dt 归一后，等效过程噪声比 v0.2 小 50~100 倍**。如果实车
  小陀螺跟不动，按 2~5 倍加 `noise_rotation_speed`，不要直接关
  `q_rate_aware`。
- **anchor 门控在极近距离 + 高速自转时可能误杀**：把
  `anchor_max_relative_error` 从 0.5 提到 0.7~0.8，或把绝对门限
  提到 60 px。
- **自动重初始化会重置 `update_count_` 和协方差**：`converged` 会
  短暂变 false，omni 切换逻辑可能延迟 1~2 帧，属预期。
- 回退：把 `q_rate_aware` 设 false、`initial_cov_*` 设回 64/64/0.4/100、
  `auto_reinit_after_no_update` 设 0、`anchor_max_corner_error` 设 0，
  即回到 v0.2 行为（NIS 最大值口径除外）。

---

## 7. 下一步（不在本版本）

- 接 `Detector` 的未配对灯条（`bars`），70–90° 单灯条也更新；
- 预测角点门控 + 匈牙利关联，替代“按 x 排序连续编号”；
- 目标级（不是灯条级）NIS / NEES 曲线，接仿真 GT 算 NEES；
- aimer 的 `predict` 换成只推状态的 `transition`；
- 前哨站 5 维模型。

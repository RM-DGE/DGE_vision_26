# DGE EKF 调试与可视化

> 目标：把“EKF 估计得准不准、协方差是否可信、滤波器是否一致”变成可观察、可回归的指标。
>
> 融合思路：
>
> - 同济 `sp_vision_25`：3D EKF、重投影验证、NIS/残差曲线、离线视频回放；
> - 南京理工 `rmcs_auto_aim_v2`：2D 像素观测 EKF、合成真值测试、Foxglove 3D 真值对比、协方差收敛/发散判据；
> - DGE：只保留 4 板 rmcs 像素 EKF；2/3 板 legacy 3D EKF 已移除。Foxglove 同时显示状态、协方差、NIS 和真值误差。

---

## 1. 滤波性能分三层

| 层次 | 看什么 | DGE 中的位置 |
|---|---|---|
| 状态层 | 估计轨迹 vs 真值、重投影是否重合 | `/dge/scene`、`/dge/debug.state`、`/sim/ground_truth` |
| 不确定性层 | 协方差 `P`、1σ/2σ、置信椭圆 | `/dge/debug.sigma_*`、`/dge/debug.cov`、3D XY 椭圆 |
| 一致性层 | 新息、NIS、NIS 失败率 | `/dge/debug.nis`、`nis_threshold`、`nis_fail_rate`、`residual_norm` |

只看状态曲线不够：**估计看起来平滑，不代表协方差可信；NIS 看起来小，也不代表没有系统偏差。**

---

## 2. DGE 当前提供的字段

`tools/Foxglove/foxglove.cpp` 的 `publish_debug()` 会发布到 `/dge/debug`：

### 2.1 状态

```text
state[0..10] = [x, y, z, vx, vy, vz, a, w, rf, rl, hl]
```

- `x/y/z`：目标旋转中心（odom 系，m）
- `vx/vy/vz`：平移速度（m/s）
- `a`：朝向角（rad）
- `w`：角速度（rad/s）
- `rf/rl/hl`：前向半径、侧向半径、侧板高度差（m）

### 2.2 不确定性

```text
cov[0..10] = variance 对角线，与 state 同序
sigma_x, sigma_y, sigma_z
sigma_vx, sigma_vy, sigma_vz
sigma_a, sigma_w
sigma_rf, sigma_rl, sigma_hl
```

- `sigma_* = sqrt(max(0, variance))`
- 3D 场景中 `tracked_target` 会画 XY 平面的 1σ/2σ 椭圆：

```text
1σ：洋红色细线
2σ：洋红色半透明线
```

如果椭圆很大但状态很平滑，说明滤波器“过度自信/过度不信”需要检查 Q/R。

### 2.3 一致性

```text
nis              当前帧 4 维像素新息的 NIS
nis_threshold    chi-square(4) 阈值，默认 9.488（95%）
nis_fail_rate    最近 nis_window 次更新的失败比例
residual_norm    当前帧 4 维像素新息的平均范数（像素）
ekf_updates      累计 correct 帧数
ekf_frame_updates 当前帧被接受的灯条数
ekf_frame_rejected 当前帧因 NIS/投影无效被拒绝的灯条数
nis_rejected     累计 NIS 拒绝次数
nis_consecutive_rejects 连续 NIS 拒绝次数
ekf_last_update_accepted 最近一次灯条更新是否被接受
ekf_dt           最近一次 predict 的 dt
converged        协方差/时间收敛标志
diverged         发散标志
last_id          最近观测到的灯条/装甲板 id
uses_rmcs_ekf    固定为 1（DGE 现在只有 4 板 rmcs 像素 EKF）
```

判读建议：

| 字段 | 正常 | 异常/排查 |
|---|---|---|
| `nis` | 稳态大多 < 9.488，切板/机动允许尖峰 | 长期 >> 阈值：Q/R 不匹配、外参/内参有问题、关联错 |
| `nis_fail_rate` | 稳态 < 5%～10% | 持续 > 20%：不要先调 EKF，先查标定/关联 |
| `residual_norm` | 小且平稳（像素） | 持续大：相机外参、PnP、检测角点有问题 |
| `sigma_x/y` | 收敛后逐渐变小 | 不降反升：过程噪声过大或观测被拒绝 |
| `sigma_w` | 静止时小，旋转时适度增大 | 静止时很大：角速度不可观/初始化差 |
| `ekf_frame_updates` | 1～8，随可见灯条变化 | 长期 0：关联失败/可见性模型错 |
| `ekf_frame_rejected` | 偶发 1～2，切板/遮挡时允许 | 长期接近可见灯条数：关联错、外参错或 NIS 门控过紧 |
| `nis_consecutive_rejects` | 稳态接近 0 | 持续 ≥ 3：先查检测角点/关联，再考虑放宽 `nis_gate_threshold` |
| `converged` | 初始化后变为 1 | 长期 0：协方差没降下来或时间不够 |
| `diverged` | 0 | 1：位置/速度/协方差越界，检查 `[rmcs_ekf] div` 日志 |

---

## 3. 真值对比（仿真）

Daedalus 仿真通过 `/sim/ground_truth` 发布敌方真值：

```text
targets[i].x/y/z, yaw, vyaw, is_outpost
```

`publish_debug()` 会在有真值时计算：

```text
center_err   估计中心与真值的水平误差（对齐固定 odom 偏移后）
yaw_err      估计 yaw - 真值 yaw
vyaw_err     估计角速度 - 真值角速度
gt_yaw       真值 yaw
gt_vyaw      真值角速度
```

推荐看：

- `center_err` 稳态应接近 0，换板/机动允许短时尖峰；
- `yaw_err` 稳态小，持续振荡说明观测噪声/换板关联有问题；
- `vyaw_err` 是最干净的指标（无坐标平移歧义）；
- 如果 `center_err` 有一个固定偏移，先查 odom 原点/外参，不要先调 EKF。

> 实车没有真值，不能计算 NEES。实车只看 NIS、残差、重投影和世界系静止性；真值误差只在仿真/录制的已知场景里看。

---

## 4. 3D 场景怎么看

Foxglove 布局 `foxglove_layouts/dge_debug_layout.json`：

| 面板 | 内容 |
|---|---|
| 3D | `/dge/scene` + `/dge/prediction` + `/tf` |
| Image | `/detector/result_img/compressed` |
| Plot | `/dge/debug.nis`、`nis_fail_rate`、`sigma_*`、`center_err`、`yaw_err`、`vyaw_err` |
| RawMessages | `/dge/debug`、`/tracker/target`、`/sim/ground_truth` |

3D 场景里：

- `detections`：识别器原始装甲板（按颜色）；
- `tracked_target`：EKF 估计的四块板、旋转中心、旋转圆、速度/角速度箭头、**XY 协方差椭圆**；
- `target_prediction`：未来预测轨迹 + 幽灵装甲板；
- `aiming_point`：瞄准点；
- `self`：云台光轴/相机光轴。

验证外参的最快方法：

1. 锁定一个静止目标；
2. 缓慢转云台；
3. 看 `detections` 和 `tracked_target` 是否在 odom 系原地不动；
4. 如果随云台画弧 → `R_camera2gimbal` / `t_camera2gimbal` / `R_gimbal2imubody` 有问题；
5. 如果原地不动但 NIS 大 → 检查观测噪声、关联、检测角点。

---

## 5. EKF 实现

DGE 现在只保留 **4 板 rmcs 像素 EKF**：

| 项目 | 内容 |
|---|---|
| 代码 | `tasks/auto_aim/rmcs_ekf/` |
| 观测 | 8 灯条上下端点像素（4D/灯条） |
| 状态 | 11 维，rmcs 顺序 `[x,y,z,vx,vy,vz,a,w,rf,rl,hl]` |
| 半径/高度 | 在线估计 |
| 适用 | 4 板普通机器人/英雄/步兵/哨兵 |
| 诊断 | `RobotModel::diagnostics()` |

2/3 板目标（平衡、前哨站、基地）的 legacy 3D EKF 已删除；`Tracker` 会直接丢弃这些检测，不再创建对应 `Target`。

---

## 6. 推荐调参流程

### 6.1 先标定，再调 EKF

顺序：

1. `calibrate_validate` 通过（世界系板位置 std < 10 mm，姿态 std < 0.5°）；
2. 转云台时 `detections` 世界系不画弧；
3. 再调 `ekf.noise_*` 和 `noise_observation`。

标定误差是系统误差，Q/R 是随机误差模型。**不要用 Q/R 去补外参。**

### 6.2 调 Q/R

- 静止目标抖动大：增大 `noise_observation` 或减小过程噪声；
- 机动跟不上：增大 `q_accel_xy / q_accel_z / q_alpha`；
- 换板时估计跳变：检查关联/可见性模型，不要只调大 R；
- NIS 长期偏大：Q 太小或 R 太小；NIS 长期偏小：Q 太大或 R 太大。

> `process_model: cwna` 时 Q 为连续白噪声加速度的精确离散化：
> `Q_pp = q·dt³/3`、`Q_pv = q·dt²/2`、`Q_vv = q·dt`。
> `q_accel_xy / q_accel_z` 单位是 `(m/s²)²·s`，`q_alpha` 是 `(rad/s²)²·s`。
> `dwna` 保留 sp25 原版 `dt⁴/4, dt³/2, dt²` 做 A/B；`diag_rate` 是旧的
> `diag(noise_*)·dt`。

### 6.3 调收敛/发散

```yaml
converge_min_update: 10
converge_max_cov_xy: 1.0
converge_delay_s: 0.1
diverge_cov: 150.0
diverge_xy: 15.0
diverge_z: 2.0
diverge_w: 31.4159
diverge_v: 5.0
diverge_vz: 1.0
```

- 收敛太慢：检查初始协方差和观测质量；
- 误判发散：先看 `sigma_*` 和原始观测；仿真里可适当放宽 `diverge_v`；
- 真发散：检查外参、关联、检测角点，不要只放宽阈值。

### 6.4 合成真值回归测试

建议后续增加 `tests/ekf_sim_test.cpp`：

- MockMotion：静止、匀速、小陀螺、切板、遮挡、离群点；
- 生成带噪声的像素观测；
- 跑 `RobotModel::init/predict/correct`；
- 输出：位置 RMSE、yaw/vyaw RMSE、NIS 覆盖率、收敛时间、丢目标率；
- 阈值写进 CI/脚本，防止调参回归。

这部分是 rmcs `tool/cxx/test_robot_ekf.cpp` 最有价值的思路，DGE 可以按同样方式补。

---

## 7. 常见误区

1. **只看状态曲线，不看协方差**
   状态平滑不代表滤波器可信；椭圆和 NIS 才能说明不确定性。

2. **把 NIS 大当成 EKF 算法问题**
   先查标定、关联、检测角点。NIS 是“新息是否像噪声”，不是“算法是否高级”。

3. **实车算 NEES**
   没有真值就没有 NEES；实车只看 NIS/残差/重投影。

4. **用 Q/R 补外参**
   外参误差随距离/角度变化，Q/R 补不了；只会让 NIS 看起来正常但估计仍然偏。

5. **忽略 `uses_rmcs_ekf`**
   该字段在 DGE 中固定为 1；2/3 板 legacy EKF 已移除，不要再按 `0` 去查旧路径。

6. **只看单帧 NIS**
   单帧尖峰正常；看 `nis_fail_rate` 和滑动窗口。

---

## 8. 一句话总结

> 状态层看“跟得对不对”，不确定性层看“协方差可不可信”，一致性层看“新息像不像噪声”。
> 先标定，再调 Q/R；先看 NIS/残差，再看 3D 椭圆；实车不看 NEES，仿真才看真值误差。

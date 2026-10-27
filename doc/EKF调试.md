# 用 Foxglove 调 DGE EKF：从跑起来到定位问题

这篇不是 EKF 公式说明，而是一套实际调试流程。目标是你打开 Foxglove 后，知道先看什么、正常是什么样、出现异常时从哪里往下查。

DGE 现在只有 4 板 rmcs 像素 EKF：状态是 `[x,y,z,vx,vy,vz,a,w,rf,rl,hl]`，观测是 8 条灯条的上下端点像素。2/3 板 legacy EKF 已移除。

当前 EKF 基线是 v0.33（预测/协方差拆分、observable 懒刷新、CWNA）。版本细节见 `doc/RobotModel4_v0.33.md`；ROS2 桥接、IMU/相机时间戳和延时测试见 `doc/RobotModel4_v0.40.md`。

当前调试分支已暂时关闭 `sentry_DGE` 的 ROS 接入，不初始化 DDS；这不影响 Foxglove、EKF 曲线和 `standard` 的调试流程。通信链路单独用 `mcu_ros_delay_test` 验证。

调 EKF 时建议先用 `standard`，不要一上来就用 `sentry_DGE` 的全向切换。全向切换会引入目标切换、USB 延迟和队列排序，先把单目标 EKF 调稳，再看全向。

---

## 1. 先把仿真器和 DGE 跑起来

终端 1，启动仿真器：

```bash
cd ~/桌面/simulation/bevy_robomaster_simulator-master
cargo run --release
```

终端 2，启动 DGE：

```bash
cd ~/桌面/DGE_vision_26
./build/standard configs/daedalus.yaml
```

仿真器窗口里按 `F5` 打开自瞄订阅，否则 DGE 的云台指令不会生效。用 `I/J/K/L` 把蓝色假人开进视野，`Tab` 切换假人，`Q/U` 开关小陀螺，`[`/`]` 和 `;`/`'` 让底盘滚转/俯仰。

然后打开 Foxglove Studio，连接：

```text
ws://localhost:8765
```

如果 Foxglove 不在同一台机器上，把 `localhost` 换成 DGE 机器的 IP。

导入布局：

```text
foxglove_layouts/dge_debug_layout.json
```

3D 面板的 `fixedFrame` 保持 `odom`，`followTf` 默认是 `camera_optical_frame`。如果 3D 是空的，先在左侧话题列表确认 `/tf`、`/dge/scene`、`/dge/prediction` 有数据，再检查 fixed frame 是不是 `odom`。

DGE 只有在 Foxglove 有客户端时才真正发送调试数据，所以先连上 Foxglove，再开始看曲线是正常的。连接断了再连上，曲线会从当前时刻重新开始。

---

## 2. 先认清楚 Foxglove 里的几块东西

布局里主要有这些面板：

- **识别结果图像**：`/detector/result_img/compressed`，看原始检测框和角点，不经过 EKF。
- **3D 场景**：`/dge/scene`、`/dge/prediction`、`/detector/marker`、`/tracker/marker`、`/aiming_point`。
- **跟踪抖动**：`/tracker/info.position_diff`、`yaw_diff`。
- **目标状态**：`/tracker/target.v_yaw`、`radius_1`、`radius_2`、`dz` 和 `/sim/ground_truth`。
- **EKF 验证**：`center_err`、`yaw_err`、`vyaw_err`、`nis`、`nis_fail_rate`、`residual_norm`、`converged`、`diverged`。
- **关联/恢复**：`anchor_err_px`、`ekf_no_update_frames`、`ekf_reinits`、`ekf_init_source`。
- **RawMessages**：`/dge/debug`、`/tracker/target`、`/sim/ground_truth`。

3D 场景里的含义：

- `detections`：检测器输出的装甲板，带 Solver 解算出的世界系位姿；
- `tracked_target`：EKF 估计的 4 块板、旋转中心、旋转圆、速度/角速度箭头，以及 XY 平面的 1σ/2σ 协方差椭圆；
- `target_prediction`：未来一段时间的预测轨迹和幽灵装甲板；
- `aiming_point`：Aimer 当前瞄准点；
- `self`：云台/相机光轴。

调试时不要只盯 3D。3D 看起来“差不多”但 NIS 很大，通常说明状态还能用，但协方差或关联有问题；3D 很平滑但 `vyaw_err` 一直偏，通常说明模型或外参有系统误差。

---

## 3. 第一次锁定：先确认 EKF 活着

先把假人放在 2～3 米处，静止，不开小陀螺。

按 `F5`，让 `targets` 从 0 变成 1。然后在 `/dge/debug` 里看这几个字段：

```text
ekf_init_source
converged
ekf_updates
ekf_frame_updates
center_err
yaw_err
nis
```

正常应该是：

- `ekf_init_source` 是 1 或 2。1 表示用 Solver 的世界系板位姿初始化，2 表示 Solver 路径失败后用 RobustPnp 兜底。如果经常出现 2，先看初始化阶段的 PnP 和角点质量。
- `converged` 在锁定后很快变 1，通常 0.1～0.3 秒内。
- `ekf_updates` 持续增加，不再归零。
- `ekf_frame_updates` 每帧 1～8 条，具体取决于能看到几条灯条。
- `center_err` 在 dm 级，静止目标通常小于 0.05～0.1 m。
- `yaw_err` 不持续增大，也不在两个固定值之间来回跳。
- `nis` 大多数时间小于 9.488，偶尔切板/遮挡出现尖峰是正常的。

如果 `targets=1` 但 `converged` 长时间是 0，先看 `ekf_frame_updates`。它长期是 0 说明检测没有真正进 EKF，不是“滤波器收敛慢”。这时去查 `anchor_err_px`、`ekf_frame_rejected` 和 `last_id`。

---

## 4. 先查外参，不要急着调 Q/R

这是最容易走错的一步。EKF 的预测像素依赖：

```text
R_camera2gimbal
t_camera2gimbal
R_gimbal2imubody
camera_matrix / distort_coeffs
```

外参错了，NIS 会长期偏大，`center_err` 会随距离/角度变化。调 Q/R 只能把曲线“抹平”，不能修正系统性偏差。

一个很直接的检查方法：

1. 锁定静止假人；
2. 缓慢左右转云台；
3. 看 3D 里的 `detections` 和 `tracked_target`。

正确时，目标本来没动，`detections` 在世界系里应该基本原地不动，`tracked_target` 也应该稳定在同一个位置。

如果 `detections` 随云台画弧，说明相机外参或手眼标定有问题。如果 `detections` 原地不动，但 `tracked_target` 跟着云台飘，说明 EKF 的观测/关联或状态模型有问题。

如果两者都在动，先回到标定流程，不要继续调 EKF。

标定验收顺序：

```bash
./build/calibrate_validate assets/img_with_q -c=configs/calibration.yaml
```

世界系板位置 std 小于 10 mm、姿态 std 小于 0.5° 再继续。标定误差是系统误差，Q/R 是随机误差模型，两者不能互相替代。

---

## 5. 看 NIS、残差和协方差

`/dge/debug` 里和一致性有关的字段：

```text
nis                 当前帧单条灯条的最大 NIS
nis_mean            当前帧平均 NIS
nis_threshold       9.488，chi-square(4) 95%
nis_gate_threshold  13.277，NIS 门控阈值
nis_fail_rate       最近 nis_window 次的失败比例
residual_norm       当前帧平均像素残差
ekf_frame_updates   当前帧被接受的灯条数
ekf_frame_rejected  当前帧被拒绝的灯条数
nis_rejected        累计 NIS 拒绝次数
nis_consecutive_rejects 连续 NIS 拒绝次数
```

`nis` 是 4 维像素新息的马氏距离，正常稳态大多数应小于 9.488。切板、遮挡、突然加速时允许短时尖峰，`nis_fail_rate` 看的是最近 100 次的比例，不要用单帧尖峰下结论。

我通常按这个顺序判断：

先看 `ekf_frame_updates`。如果是 0，先解决“为什么没有观测进来”，不要看 NIS。

如果 `ekf_frame_updates` 正常，但 `nis_fail_rate` 长期大于 20%，先查外参、角点检测和关联。NIS 大不等于 Q/R 一定错。外参、内参、装甲板尺寸、角点顺序都会让新息持续偏大。

如果 `residual_norm` 一直很大，但 `center_err` 还行，通常是角点或外参有系统误差。像素残差是绝对值，比 NIS 更直观。

再看协方差。`cov[0..10]` 与 `state` 同序：

```text
[ x, y, z, vx, vy, vz, a, w, rf, rl, hl ]
```

`sigma_*` 是对角线开根号。3D 里的洋红色椭圆是 XY 的 1σ/2σ。

正常收敛时，`sigma_x/sigma_y` 应该逐渐下降；静止目标尤其明显。旋转时 `sigma_w` 可以适度变大。如果 `sigma_x` 不降反升，先看 `ekf_frame_updates` 是不是长期为 0，或者 Q 是否过大。

---

## 6. 看真值误差：center_err、yaw_err、vyaw_err

仿真里 `/sim/ground_truth` 会给出目标真值。`publish_debug()` 会计算：

```text
center_err  估计中心与真值的水平误差
yaw_err     估计 yaw 与真值 yaw 的差
vyaw_err    估计角速度与真值角速度的差
gt_yaw      真值 yaw
gt_vyaw     真值角速度
```

三个指标里，`vyaw_err` 最干净，因为它没有 odom 平移偏移的问题。小陀螺场景优先看它。

`center_err` 有一个容易误解的地方：DGE 在第一次拿到真值时，会记录一个固定的 odom 偏移，之后用这个偏移对齐真值。所以 `center_err` 是“相对第一帧对齐后的误差”，不是绝对坐标误差。换目标或仿真重置后，这个偏移可能不再对应，看到固定偏大时先确认是不是这个原因。

`yaw_err` 和 `vyaw_err` 更适合判断滤波器的动态性能：

- 静止时 `vyaw_err` 应在 0 附近小幅波动；
- 起转/停转时允许尖峰，但应很快回落；
- 如果 `vyaw_err` 长期滞后一个固定方向，先查 `q_alpha`、观测噪声和角速度观测；
- 如果 `yaw_err` 在两个值之间周期性跳，先查切板关联和 `last_id`。

实车没有真值，不要算 NEES，也不要拿 `center_err` 调参。实车看 NIS、残差、重投影和世界系静止性。

---

## 7. 小陀螺、切板和遮挡

这部分是 EKF 最容易暴露问题的地方。

把假人放静止，按 `U` 开小陀螺，让 `v_yaw` 从 0 慢慢升到 3、6、8 rad/s，每个速度看十几秒。重点看：

```text
vyaw_err 的峰值和回落时间
nis / nis_fail_rate
ekf_frame_updates / ekf_frame_rejected
anchor_err_px
ekf_reinits
sigma_w
```

正常情况：

- `vyaw_err` 起转时有峰，但 0.5 秒内回落；
- `nis` 在切板瞬间有尖峰，但 `nis_fail_rate` 不持续升高；
- `ekf_frame_updates` 仍然大于 0，不是整帧都被拒绝；
- `anchor_err_px` 偶发升高，但不长期超过 40 px 或预测板宽的 50%；
- `ekf_reinits` 不持续增长。

如果 `ekf_frame_updates` 突然变 0，先看 `anchor_err_px` 和 `last_id`。anchor 对不上时整个关联会失败，表现不是 NIS 大，而是根本没有更新。

如果 `ekf_frame_rejected` 接近可见灯条数，说明每条灯条都被 NIS 或投影有效性拒绝。先查外参和角点，再考虑放宽 `nis_gate_threshold`。把门控放宽只是让坏观测进来，不是修复。

如果 `ekf_no_update_frames` 超过 10，会触发自动重初始化；超过 30 会判发散，Tracker 丢目标。看到 `ekf_reinits` 反复增长，先看是关联失败、遮挡太久，还是 Q 太小导致预测跟不上。

---

## 8. 调 Q/R 的顺序

先标定，再调 Q/R。顺序不要反。

v0.33 的 Q 配置：

```yaml
process_model: cwna
q_accel_xy: 1.0     # (m/s²)²·s
q_accel_z: 1e-4
q_alpha: 10.0       # (rad/s²)²·s
```

`cwna` 是默认模型，Q 为：

```text
Q_pp = q · dt³ / 3
Q_pv = q · dt² / 2
Q_vv = q · dt
```

`q_accel_xy` 管平移，`q_accel_z` 管高度，`q_alpha` 管小陀螺。`q_accel_z` 通常保持很小，避免高度方向被噪声带飞。

要做 sp25 原版对照，用：

```yaml
process_model: dwna
```

`dwna` 的 Q 是 `dt⁴/4, dt³/2, dt²`。100 Hz 起点可以试 `q_accel_xy≈100`、`q_alpha≈900`，但这两个数和 CWNA 不是一个量级，不能直接混填。

调参时一次只改一个参数，至少跑三种场景：

1. 静止；
2. 匀速平移；
3. 反复起转/停转的小陀螺。

每个场景记录：

```text
ekf_reinits
vyaw_err 峰值和回落时间
center_err RMS
nis_fail_rate
ekf_frame_rejected
```

我的经验是先调 `q_alpha`，因为小陀螺最容易暴露预测跟不上。起转/停转时 `vyaw_err` 峰值大、回落慢，先增大 `q_alpha`；静止时抖动明显、`sigma_w` 偏大，再减小 `q_alpha`。

平移起停跟不上再调 `q_accel_xy`。`noise_observation` 是像素观测噪声，降低它会更相信角点，但也更容易被角点抖动带偏；升高它会平滑，但机动会滞后。

NIS 长期偏大时，不要第一反应调大 `nis_gate_threshold`。先确认外参、角点和关联；门控阈值只决定“坏观测要不要拒绝”，不决定“观测本身准不准”。

---

## 9. 帧率和性能

`/dge/debug.fps` 是 DGE 循环帧率，`ekf_dt` 是最近一次 `predict` 的 dt。

如果 `ekf_dt` 经常大于 0.1 秒，Tracker 会直接判丢目标，这不是 EKF 发散。先看 CPU、YOLO 推理、USB 相机和 Foxglove 推流。

v0.33 之后，`Target::at()` 只做状态外推，`full()` 不再刷新 8 灯条投影；Aimer 和 Foxglove 的预测轨迹不会再因为投影变慢。`diagnostics()` 仍然会刷新一次投影，因为它要给调试输出完整的 EKF 状态；所以 Foxglove 调试本身有少量开销，但不会影响 Aimer 的迭代。

如果换帧率做 A/B，CWNA 的 Q 按 dt 自动缩放，原则上不需要重调。`diag_rate` 是旧行为，换帧率需要重新看 NIS 和 `vyaw_err`。

---

## 10. 常见现象怎么查

**3D 里 detections 随云台画弧**

先查外参和手眼标定，不要调 EKF。

**目标锁上了，但 `center_err` 一直有固定偏移**

先确认 `center_err` 的 odom 对齐偏移是否还对应当前目标；换目标或重置仿真后旧偏移可能失效。然后查 odom 原点、外参和真值坐标系。

**`targets=1`，但 `ekf_frame_updates` 长期是 0**

查 `anchor_err_px`、`last_id`、`ekf_frame_rejected`。通常是 anchor 关联失败、预测像素在相机后面、可见性模型不对，或者目标已经被判发散。

**NIS 长期很大，但 3D 看起来还行**

先查角点、内外参和装甲板尺寸。NIS 是统计量，曲线平滑不代表新息像噪声。

**NIS 很小，但 `center_err` 很大**

可能是协方差过度自信，或者存在固定外参偏差。看 `sigma_x/sigma_y` 和椭圆大小；如果椭圆很小但误差很大，不要继续调小 R。

**小陀螺时 `vyaw_err` 峰值很大**

先看 `q_alpha` 是否偏小，再看 `nis_fail_rate` 和 `ekf_frame_rejected`。如果大量观测被拒绝，先解决拒绝原因，再谈 Q。

**`ekf_reinits` 反复增长**

看 `ekf_no_update_frames`、`anchor_err_px` 和 `nis_consecutive_rejects`。重初始化是恢复手段，不是正常状态；频繁重初始化说明关联或模型有问题。

**`diverged=1`**

看状态/协方差是否越界，或者连续无更新超过 30 帧。日志里搜 `[rmcs_ekf]`，结合 `ekf_no_update_frames` 和 `ekf_frame_rejected` 判断是模型发散还是观测断了。

**Foxglove 没有曲线**

先确认有客户端连接；DGE 在没有 Foxglove 客户端时不会发调试数据。再检查话题名和布局是否对应 `/dge/debug`。如果 3D 空但曲线有，检查 fixed frame 和 `/tf`。

---

## 11. 实车要注意什么

实车没有 `/sim/ground_truth`，所以没有 `center_err/yaw_err/vyaw_err`，也不能算 NEES。

实车调试主要看：

- 静止目标时 `nis`、`residual_norm` 是否稳定；
- 转云台时 `detections` 和 `tracked_target` 是否在世界系原地不动；
- 小陀螺时 `nis_fail_rate`、`ekf_frame_rejected`、`ekf_reinits`；
- `sigma_*` 是否收敛，`diverged` 是否误触发；
- 装甲板角点是否稳定，外参是否随距离/角度变化。

实车没有真值，不要用“看起来挺顺”代替一致性检查；NIS、残差和重投影才是可回归的指标。

---

## 12. 一页速查

正常锁定：

```text
targets = 1
converged = 1
ekf_frame_updates = 1..8
nis 多数 < 9.488
nis_fail_rate < 10%
ekf_reinits 不增长
diverged = 0
```

优先排查顺序：

```text
外参/内参
  → 角点质量
  → anchor 关联
  → NIS/残差
  → Q/R
  → 收敛/发散阈值
```

不要做的事：

- 不要用 Q/R 补外参；
- 不要用单帧 NIS 判断滤波器好坏；
- 不要在 `ekf_frame_updates=0` 时先调 NIS 门控；
- 不要把 `center_err` 的固定 odom 偏移当成 EKF 误差；
- 不要只看 3D 平滑，忽略协方差和 NIS。

最后一句话：先把“看得见的 3D”与“看不见的统计量”对齐，再动参数。3D 用来定位现象，NIS/残差/协方差用来判断原因。

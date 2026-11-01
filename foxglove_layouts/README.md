# Foxglove 调试布局

本目录存放 Foxglove Studio 布局文件（Layout → Import layout 导入）。

> 完整启动和测试教程见 [RobotModel4_v1.01](../doc/RobotModel4_v1.01.md)；本文件说明布局和话题。

| 文件 | 说明 |
|---|---|
| `rm_aim_sim_layout.json` | **新仿真器推荐布局**：原始像素/四角、3D、EKF 拒绝与恢复、时间、控制及计分 |
| `dge_debug_layout.json` | **推荐日常调试布局**：识别图像 + 3D 场景（检测/跟踪/预测/瞄准）+ 三组调试曲线 + RawMessages |

Foxglove 连接方式：Studio → Open connection → **Foxglove WebSocket** → `ws://<本机IP>:8765`
（算法进程启动后 `tools/Foxglove` 的 WebSocket 服务即开始监听，无需 ROS。）

## 布局引用的话题（全部由 tools/Foxglove 发布）

| 话题 | 类型 / 编码 | 内容 |
|---|---|---|
| `/detector/result_img/compressed` | `foxglove.CompressedImage` | 带识别框的图像 |
| `/detector/number_img` | `foxglove.CompressedImage` | 数字分类图案拼接图 |
| `/camera/annotations` | `foxglove.ImageAnnotations` | 装甲板四点框叠加 |
| `/dge/scene` | `foxglove.SceneUpdate` | 3D 实体：`detections` / `tracked_target`(估计装甲板+旋转中心+旋转圆+速度箭头+**1σ/2σ XY 协方差椭圆**) / `self`(云台+光轴) / `aiming_point` |
| `/dge/prediction` | `foxglove.SceneUpdate` | 目标未来约 0.6s 预测轨迹 + 幽灵装甲板 |
| `/detector/marker` | `visualization_msgs/MarkerArray` (cdr) | 识别装甲板 CUBE + TEXT |
| `/tracker/marker` | `visualization_msgs/MarkerArray` (cdr) | 跟踪目标 SPHERE + 速度/角速度 ARROW + 装甲板 CUBE |
| `/aiming_point` | `geometry_msgs/PointStamped` (cdr) | 瞄准点（odom 系） |
| `/camera_info` | `foxglove.CameraCalibration` | 相机内参 |
| `/tf` | `foxglove.FrameTransforms` | `odom → gimbal_link → camera_optical_frame` |
| `/tracker/target` | json (`auto_aim_interfaces/msg/Target`) | 跟踪状态：`position` `velocity` `yaw` `v_yaw` `radius_1` `radius_2` `dz` |
| `/tracker/info` | json (`auto_aim_interfaces/msg/TrackerInfo`) | 抖动指标：`position_diff`(m) `yaw_diff`(rad)，每帧差分 |
| `/dge/debug` | json | `fps` `yaw` `pitch` `gimbal_yaw` `gimbal_pitch` `shoot` `armors` `targets` `state[11]` `aim_*`；EKF 一致性：`cov[11]` `sigma_*` `nis` `nis_threshold` `nis_fail_rate` `residual_norm` `ekf_updates` `ekf_frame_updates` `ekf_dt` `converged` `diverged` `uses_rmcs_ekf`；真值误差：`center_err` `yaw_err` `vyaw_err` |
| `/sim/ground_truth` | json | 最新状态的诊断真值：位置已减去云台世界原点，与 odom 共原点；`vyaw=null` 表示上游未提供角速度 |
| `/sim/status` | json | `fresh`、`observation_valid`、`publisher_id/session_id`、`state_sim_time/frame_sim_time`、`observation_age_s`、`paused/ipc_enabled`、`last_seq/publication`、`stats` |

## rm_aim_sim 使用方法

先启动图形仿真 `./run.sh -- --config=res://configs/vision.cfg --remote`，再启动 DGE 的 `./build/auto_aim_sim configs/rm_aim_sim.yaml`。在 Foxglove 中选择 Foxglove WebSocket，连接 `ws://localhost:8765`，通过 Layout → Import layout 导入本目录的 `rm_aim_sim_layout.json`。若 Windows 的 localhost 转发不可用，可用 WSL 的 `hostname -I` 查询地址并连接 `ws://<WSL-IP>:8765`。

新入口每帧发布图像、检测框、分类图、整车状态、抖动、Marker、TF、预测和 `/dge/debug`，使原日常布局的状态与 EKF 曲线也能工作。图像发送限频约 15 FPS，`/dge/debug.fps` 是算法完成帧之间的墙钟频率；它与图像发送频率、仿真物理频率不同。

推荐先看图像中的四角，再看 `/tracker/target.tracking`，随后对照 `/dge/debug.anchor_err_px`、`ekf_no_update_frames`、`ekf_reinits`。`/sim/status.stats.hit_rate` 是 `hit/resolved`，仍在飞行的弹丸不进入分母。恢复或切换 session 时，跟踪和诊断历史会重置；停流时 `observation_valid=false`，暂停/停用时对应状态也会更新。

`/sim/ground_truth` 来自最新状态，通常比取图时刻更晚；`/sim/status` 分别提供状态与帧时间。新入口不把该真值传入旧的自动对齐误差计算，因此 `/dge/debug.center_err/yaw_err/vyaw_err` 不作为新路线的有效指标；上游也没有可直接使用的真值角速度。需要真值误差时，先实现相同时刻和相同装甲编号的配对，不要把这些空曲线解释为零误差。

Foxglove Parameters 面板仅展示/镜像配置；在面板内修改数值不会实时重建算法。算法参数应在 YAML 中修改并重启 DGE。

### 实时连接自检

仿真和 DGE 均运行后，在 DGE 根目录执行：

```bash
python3 tests/foxglove_ws_smoke.py --duration 8
# DGE 仍运行、仿真已停止时：
python3 tests/foxglove_ws_smoke.py --status-only --expect-inactive --duration 3
```

该脚本只依赖 Python 标准库，实际订阅图像、标注、3D、TF、跟踪、EKF 和计分话题，检查是否收到消息及必要字段，不向仿真发送控制。捆绑 SDK 使用 `foxglove.sdk.v1` 子协议；脚本同时提供旧协议候选。旧自制客户端若只发送 `foxglove.websocket.v1` 可能收到 HTTP 400，应使用匹配 SDK 的客户端。脚本通过不等同于已人工验收 Foxglove 应用的每个面板。

坐标系：固定系 `odom`，新布局的 3D 面板跟随 `ekf_target`（EKF 目标中心，仅跟位置），默认视距 1.35 m。

### EKF 装甲板特写（2026-11-01）

更新算法并导入 `rm_aim_sim_3d_fixed.json`（与当前 `rm_aim_sim_layout.json` 内容相同，名称用于区分已导入的旧版）。本次仅参考 `dge_debug_layout.json` 的 3D 配置，复用深色背景、隐藏坐标标签和网格；其他面板及排列与前一版完全一致。3D 主图层为 `/dge/ekf_model`；旧场景、Marker 和预测叠影默认关闭，避免重复绘制。保留目标中心跟随和 1.35 m 视距；环境采用 DGE 的 odom 固定地面、10 m 蓝色网格、每格 1 m，地面高度 z=-0.4 m。

- 四块板独立编号 0–3，姿态和中心直接来自 `Target::full()`。3D 不绘制文字，点击实体可查看板号和尺寸；精确坐标在 `/dge/selection.plates`，坐标系为 odom，单位米。
- 绿色实心板是 Aimer 最后一轮实际选板；橙色为其他板。青色轮廓表示 **最近一次观测关联**，不保证本帧接受了测量。
- 紫色小点是预测命中位置；与当前所选板之间的紫色线段表示提前量。已移除预测装甲板线框重影，保留当前 EKF 四板与预测点。
- 模型宽高使用 EKF 灯条角点矩形：小板 **135 × 56 mm**，大板 **230 × 56 mm**。8 mm 厚度仅为显示辅助，不是物理外壳或裁判命中区域尺寸。
- `r1/r2/dz`、角速度及选板/最近观测编号均放在 `/dge/selection`，与四板位置、四元数和宽高一起查看，不再覆盖在模型上。
- 编号只在当前 EKF 初始化周期内有意义。无有效选板时 `selected_id=-1`；失去目标清除模型，停止发布后实体在 0.5 s 后过期。
- 若视角被手动移走，选择显示坐标系 `ekf_target`，按 `1` 回中，再滚轮调距离。跟随方式见 [Foxglove 3D 文档](https://docs.foxglove.dev/docs/visualization/panels/3d)。

实时验证：`python3 tests/foxglove_ws_smoke.py --duration 8 --require-model`。它检查真实 WebSocket 消息和四板/选板一致性，不能替代应用内视觉验收。

## 3D 视角模式

下面是可手动设置的其他视角；新仿真布局默认使用上面的 EKF 目标特写。在 3D 面板设置中可改：

| 模式 | followTf | 效果 |
|---|---|---|
| 第一人称（默认） | `camera_optical_frame` | 沿光轴看出去，配 `/dge/scene`/`/dge/prediction` 看跟踪与预测最直观 |
| 云台跟随 | `gimbal_link` | 相机绑定云台姿态，云台转视角跟着转（会甩） |
| 只跟位置 | `follow-position` + `gimbal_link` | 相机随云台平移但朝向不变，观察整车运动不晕 |
| 固定全景 | 不跟随（`followTf` 留空）+ 手动调 `distance`/`phi`/`thetaOffset` | 上帝视角看全场 |


## 布局五组曲线

1. **跟踪抖动**：`/tracker/info.position_diff`、`/tracker/info.yaw_diff` —— EKF 状态帧间变化量，衡量跟踪平滑度（丢失后自动清零重新累计）。
2. **目标状态**：`/tracker/target.v_yaw / radius_1 / radius_2 / dz` —— 小陀螺判断与半径估计。4 板目标 `radius_2 = r1 + l`（EKF 的 `x[8]+x[9]`）。
3. **云台指令**：`/dge/debug.yaw / pitch / aim_x`。
4. **EKF 一致性**：`/dge/debug.nis`（4 维像素新息的卡方统计量）、`nis_threshold`（χ²(4) 95% = 9.488）、`nis_fail_rate`（滑动窗口超阈值比例）、`residual_norm`（新息范数）、`sigma_*`（状态 1σ）。稳态 `nis_fail_rate` 应 < 5%～10%，换板/机动瞬间允许尖峰；`sigma_*` 应收敛而不是持续增大。3D 场景里的 1σ/2σ XY 椭圆是同一协方差的几何表示。
5. **性能**：`/dge/debug.fps / armors / targets` —— 端到端帧率、识别板数、锁定目标数；做模型/参数 A/B 对比时以它为准。

## 修改布局时的注意事项

- 面板类型名必须是 Foxglove 的注册名：图片面板是 **`ImageViewPanel`**（不是 `Image`），3D 面板是 `3D`，曲线是 `Plot`，消息查看器是 `RawMessages`。
- Plot 的路径字段必须用 `value` + `enabled` + `timestampMethod` 结构（见本目录 JSON）。
- 3D 面板 `followTf` / `fixedFrame` 必须使用上面 TF 树里存在的 frame，否则面板空白。
- 只能引用 tools/Foxglove 实际注册的话题；引用不存在的话题只会得到空面板。

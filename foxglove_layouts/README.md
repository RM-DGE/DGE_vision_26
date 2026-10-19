# Foxglove 调试布局

本目录存放 Foxglove Studio 布局文件（Layout → Import layout 导入）。

> 完整使用说明（连接、面板判读、调试工作流、FAQ）见
> 完整使用说明见 simulation 工作区的《Foxglove使用说明.md》；本文件只讲布局文件本身与话题对照。

| 文件 | 说明 |
|---|---|
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
| `/sim/ground_truth` | json | 仿真器敌方真值（仅 daedalus 相机）：每目标 `x/y/z/yaw/vyaw/is_outpost`，坐标为仿真器世界系 |

坐标系：固定系 `odom`，3D 面板跟随 `gimbal_link`。

## 3D 视角模式

布局默认**第一人称（枪手视角）**：`followMode=follow-pose` + `followTf=camera_optical_frame`，视线沿相机光轴，目标保持在画面中央，无需手动转视角。在 3D 面板顶部下拉里可改：

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

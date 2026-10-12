# Foxglove 调试布局

本目录存放 Foxglove Studio 布局文件（Layout → Import layout 导入）。

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
| `/dge/scene` | `foxglove.SceneUpdate` | 3D 实体：`detections` / `tracked_target`(估计装甲板+旋转中心+旋转圆+速度箭头) / `self`(云台+光轴) / `aiming_point` |
| `/dge/prediction` | `foxglove.SceneUpdate` | 目标未来约 0.6s 预测轨迹 + 幽灵装甲板 |
| `/detector/marker` | `visualization_msgs/MarkerArray` (cdr) | 识别装甲板 CUBE + TEXT |
| `/tracker/marker` | `visualization_msgs/MarkerArray` (cdr) | 跟踪目标 SPHERE + 速度/角速度 ARROW + 装甲板 CUBE |
| `/aiming_point` | `geometry_msgs/PointStamped` (cdr) | 瞄准点（odom 系） |
| `/camera_info` | `foxglove.CameraCalibration` | 相机内参 |
| `/tf` | `foxglove.FrameTransforms` | `odom → gimbal_link → camera_optical_frame` |
| `/tracker/target` | json (`auto_aim_interfaces/msg/Target`) | 跟踪状态：`position` `velocity` `yaw` `v_yaw` `radius_1` `radius_2` `dz` |
| `/tracker/info` | json (`auto_aim_interfaces/msg/TrackerInfo`) | 抖动指标：`position_diff`(m) `yaw_diff`(rad)，每帧差分 |
| `/dge/debug` | json | `fps` `yaw` `pitch` `shoot` `armors` `targets` `ekf[]` `aim_*` |

坐标系：固定系 `odom`，3D 面板跟随 `gimbal_link`。

## 布局三组曲线

1. **跟踪抖动**：`/tracker/info.position_diff`、`/tracker/info.yaw_diff` —— EKF 状态帧间变化量，衡量跟踪平滑度（丢失后自动清零重新累计）。
2. **目标状态**：`/tracker/target.v_yaw / radius_1 / radius_2 / dz` —— 小陀螺判断与半径估计。4 板目标 `radius_2 = r1 + l`（EKF 的 `x[8]+x[9]`）。
3. **云台指令**：`/dge/debug.yaw / pitch / aim_x`。

## 修改布局时的注意事项

- 面板类型名必须是 Foxglove 的注册名：图片面板是 **`ImageViewPanel`**（不是 `Image`），3D 面板是 `3D`，曲线是 `Plot`，消息查看器是 `RawMessages`。
- Plot 的路径字段必须用 `value` + `enabled` + `timestampMethod` 结构（见本目录 JSON）。
- 3D 面板 `followTf` / `fixedFrame` 必须使用上面 TF 树里存在的 frame，否则面板空白。
- 只能引用 tools/Foxglove 实际注册的话题；引用不存在的话题只会得到空面板。

# Foxglove 布局

本目录包含两类布局：

| 文件 | 说明 |
|---|---|
| `dge_3d_layout.json` | **DGE 原生 3D 布局**：3D 场景（检测 / 目标 / 预测 / 自车 / 瞄准）+ 图像 + 曲线 + RawMessages + Parameters，推荐日常调试使用 |
| `params.json` / `tracker_part.json` / `detector_part.json` | 原始 rm_vision 三份配置（未改动），作为兼容参考 |

在 Foxglove Studio 中直接 **Import layout** 导入即可。

## DGE 原生 3D 布局（dge_3d_layout.json）

布局引用的话题与坐标系，全部由 `tools/Foxglove` 发布：

| 话题 | 类型 / 编码 | 内容 |
|---|---|---|
| `/dge/scene` | `foxglove.SceneUpdate` (protobuf) | 3D 主体：`detections`(识别框) / `tracked_target`(估计装甲板+旋转中心+旋转圆+速度箭头+标签) / `self`(云台与相机光轴) / `aiming_point`(瞄准点+指向箭头) |
| `/dge/prediction` | `foxglove.SceneUpdate` (protobuf) | 跟踪目标未来约 0.6s 的预测轨迹：最近装甲板轨迹(实线) + 旋转中心轨迹(淡线) + 幽灵装甲板 |
| `/dge/debug` | json | 曲线：`yaw` / `pitch` / `shoot` / `fps` / `ekf[]` / `aim_*` |
| `/detector/result_img/compressed` | `foxglove.CompressedImage` (protobuf) | 带识别框的图像 |
| `/detector/marker` | `visualization_msgs/msg/MarkerArray` (cdr) | 识别装甲板 CUBE(ns=armors) + TEXT(ns=classification) |
| `/tracker/marker` | `visualization_msgs/msg/MarkerArray` (cdr) | SPHERE(position) + ARROW(linear_v / angular_v) + CUBE(armors) |
| `/aiming_point` | `geometry_msgs/msg/PointStamped` (cdr) | 瞄准点（odom 系） |
| `/camera_info` | `foxglove.CameraCalibration` (protobuf) | 相机内参 |
| `/tf` | `foxglove.FrameTransforms` (protobuf) | `odom → gimbal_link → camera_optical_frame` |
| `/camera/image` | `foxglove.CompressedImage` (protobuf) | 兼容通道（与 result_img 同图） |
| `/detector/number_img` | `foxglove.CompressedImage` (protobuf) | 装甲板数字图案拼接图 |
| `/camera/annotations` | `foxglove.ImageAnnotations` (protobuf) | 装甲板四点框 |

3D 面板固定坐标系 `odom`，跟随 `gimbal_link`。

## 原始 rm_vision 三份配置

本目录中的另外三份 JSON 是**原始文件，未做任何修改**：

| 文件 | 面板组成 |
|---|---|
| `params.json` | Plot（`/tracker/target.position.x`）+ Parameters |
| `tracker_part.json` | ImageViewPanel + 3D + Plot + 多个 RawMessages |
| `detector_part.json` | ImageViewPanel（result_img / number_img）+ 3D + RawMessages |

在 Foxglove Studio 中直接 **Import layout** 导入即可，无需改动 JSON。

## DGE 发布的话题（与原始布局对应）

| 原始布局引用 | 消息类型 | 编码 | DGE 来源 |
|---|---|---|---|
| `/detector/result_img/compressed` | `foxglove.CompressedImage` | protobuf | 带识别框的图像（原 `publish_image`） |
| `/detector/number_img` | `foxglove.CompressedImage` | protobuf | 装甲板数字图案拼接图 |
| `/camera/image` | `foxglove.CompressedImage` | protobuf | 同 result_img（兼容旧通道） |
| `/camera/annotations` | `foxglove.ImageAnnotations` | protobuf | 装甲板四点框 |
| `/camera_info` | `foxglove.CameraCalibration` | protobuf | yaml 内参，随 `/tf` 重发 |
| `/tf` | `foxglove.FrameTransforms` | protobuf | `odom → gimbal_link → camera_optical_frame` |
| `/detector/marker` | `visualization_msgs/msg/MarkerArray` | **cdr** | 每个装甲板一个 CUBE（ns=`armors`）+ TEXT（ns=`classification`） |
| `/tracker/marker` | `visualization_msgs/msg/MarkerArray` | **cdr** | SPHERE(ns=`position`) + 2×ARROW(ns=`linear_v`/`angular_v`) + 各装甲板 CUBE(ns=`armors`) |
| `/aiming_point` | `geometry_msgs/msg/PointStamped` | **cdr** | 瞄准点（odom 系） |
| `/tracker/target` | `auto_aim_interfaces/msg/Target` | json | 跟踪目标状态（`position/velocity/yaw/v_yaw/...`） |
| `/tracker/info` | `auto_aim_interfaces/msg/TrackerInfo` | json | `position_diff/yaw_diff/position/yaw` |
| `/detector/armors` | `auto_aim_interfaces/msg/Armors` | json | 识别装甲板数组 |
| `/dge/scene` | `foxglove.SceneUpdate` | protobuf | 3D 主体场景（detections / tracked_target / self / aiming_point） |
| `/dge/prediction` | `foxglove.SceneUpdate` | protobuf | 目标未来轨迹预测（最近装甲板轨迹 + 旋转中心轨迹 + 幽灵装甲板） |
| `/dge/debug` | json | json | 附加调试曲线（fps/yaw/pitch/shoot/aim_*） |

CDR 通道使用 `ros2msg` 格式的完整消息定义（与 `foxglove_bridge` 一致），
CDR 为 XCDR1 小端、带 4 字节 RTPS encapsulation 头（`00 01 00 00`，PLAIN_CDR），
对齐以 CDR 数据起点为基准——与 `@foxglove/cdr` / `rosmsg2-serialization` 输出完全一致，
可直接被 Foxglove Studio 的 3D 面板按 ROS MarkerArray 解析（支持
`namespaces: armors/classification` 开关、CUBE / SPHERE / ARROW / TEXT_VIEW_FACING）。

## 已知差异（相对真机 rm_vision 布局）

- `/image_raw`、`/detector/result_img`、`/detector/binary_img`：布局中为不可见或调试用，
  DGE 未发布（不影响显示）。
- `/robot_description`：DGE 无 URDF，3D 面板不会显示机器人模型。
- TF 帧 `camera_link`：DGE 只发布 `camera_optical_frame`，
  布局里 `camera_link` 坐标轴不显示。
- Marker 使用 `odom` 世界系坐标（原工程为相机系 + TF 变换），显示位置等价。

## 启动

```bash
cd ~/桌面/DGE_vision_26
export LD_LIBRARY_PATH=/opt/intel/openvino_2024.6.0/runtime/lib/intel64:$LD_LIBRARY_PATH
./build/standard configs/daedalus.yaml
```

Foxglove Studio → Open connection → Foxglove WebSocket → `ws://localhost:8765`，
然后 Import layout 选择本目录中的 JSON。

## 参数面板

服务器的 `Parameters` 能力会返回 `configs/*.yaml` 的顶层参数（含嵌套 map/数组），
在 Foxglove 的 Parameters 面板中可直接查看。

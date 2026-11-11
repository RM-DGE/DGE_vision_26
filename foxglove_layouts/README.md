# EKF 像素残差布局（当前）

重新导入 `ekf_debug_layout.json`。左侧为窗口、原 3D 和中文状态表，右侧仍然只有 5 个曲线面板：水平像素残差、垂直像素残差、更新前 NIS、半径/高度差、XYZ 位置标准差。云台跟踪布局不变。

- 残差为观测减预测，单位 px。每个模型端点对编号 0–7 独立发布上下端点的 u/v 四个分量；曲线设置中可勾选编号，默认只打开编号 0，避免 16 条曲线遮挡。编号 `2k`、`2k+1` 对应模型装甲板 `k`；重建后编号不保证对应同一实体。
- 记录位置在单条观测更新之前、NIS 门控之前。后续端点对使用顺序更新后的预测，不是统一的整帧初始预测。若一个编号在一帧出现多次，保留最后一次尝试。
- 没有有效观测或投影无效时，对应分量发布 null；没有有效 NIS 时 `pixel_nis_max` 也为 null。零残差只代表本次预测与观测吻合，不代表零世界坐标误差。
- 两个残差图路径：`/dge/debug.pixel_residuals[编号].upper_u`、`lower_u`、`upper_v`、`lower_v`。NIS 图使用 `pixel_nis_max` 和 `nis_threshold`；几何图使用 `state[8..10]`；不确定性图使用 `sigma_x/y/z`。
- 中文状态表通过 Table 面板显示 `/dge/debug.ekf_status`，列为项目、当前、含义。包含目标存在、接受/拒绝计数、关联编号、连续未更新、重建次数、门控配置和收敛状态。它不是原始消息面板。
- `pixel_nis_max` 是各编号最后一次有效尝试的最大值；对单条四维参考线的越界只是诊断提示，不能按整帧 5% 错误率解读。sigma 为滤波器内部不确定性，不是真实误差。

本次只增加诊断记录和发布，不改变 EKF 状态更新、噪声、门控与恢复逻辑。Linux 更新后正常编译：`cmake --build build --target standard sentry_DGE -j2`。小电脑备份：`/home/hhh/foxglove-before-pixel-residual-20261109.tar.gz`。更早布局说明如下，仅作历史参考。

---

# 两份实车调试布局

原 `dge_debug_layout.json` 原样保留作为备份。新增布局均为：左侧窗口 + 原 3D 面板，右侧最多 5 个曲线面板，没有原始消息面板。

| 文件 | 右侧五图（按从上到下、从左到右） |
|---|---|
| `ekf_debug_layout.json` | NIS / 像素误差 / 目标转速 / 半径与高度差 / 每帧接受与拒绝更新次数 |
| `gimbal_tracking_layout.json` | Yaw 指令与反馈 / Pitch 指令与反馈 / 两轴跟随误差 / 目标转速 / 有效状态与开火指令 |

在 Foxglove 的布局菜单导入对应 JSON，连接 `ws://10.0.0.136:8765`。两份布局均采用最近 10 秒并同步图表时间轴。原 3D 配置和场景话题保持不变。

本次发布代码新增 `command_yaw_deg`、`command_pitch_deg`、`gimbal_yaw_deg`、`gimbal_pitch_deg`、`tracking_yaw_error_deg`、`tracking_pitch_error_deg`、`target_valid`、`aim_valid`、`control`，全部为显示用途。更新代码后在 Linux 工程目录执行 `cmake --build build --target standard -j2`。正常运行 `./build/standard`；需要本地窗口时加 `-d`。不修改串口控制行为、滤波计算或参数。

判读注意：
- 当前 EKF 是像素观测：每次更新为一个端点对的四维坐标。像素误差图叠加新息向量范数的帧内平均值和 anchor 关联误差，两者定义不同，不能直接相减。
- NIS 在每条观测更新前计算；顺序更新中后面的观测使用前一次更新后的状态。帧最大 NIS 与单条四维参考阈值对照仅用于报警参考，整帧越界率不能当作单条 5% 显著性检验。统计阈值不表示门控已开启。
- 接受/拒绝计数按端点对计数，拒绝包含 NIS 门控和投影无效。没有有效更新尝试时，现有 NIS/残差字段可能为 0；结合计数图判断，不能认作完美拟合。
- 两组半径和高度差都为 m，可同图观察。位置不确定性通过 3D 的 XY 协方差椭圆辅助观察，本次不另加曲线图。
- 云台误差为环绕至 ±180° 的指令减反馈。指令仅在 control、aim_valid 且目标存在时显示，否则发布 null。绝对 Yaw 跨 ±180° 时可有显示跳变，不一定是控制跳变。
- IMU 反馈按图像时刻对齐，曲线间距不直接代表下位机闭环延迟。target_valid 表示目标列表非空（可能处于暂丢保留期），不是本帧检测成功；shoot 仅为发送的开火指令，不代表实际发射。
- 目标转速通过 Foxglove `.@mul(57.29577951308232)` 从 rad/s 转为 °/s，要求支持该函数的版本（2.35 或更新）。

旧的临时 `.bak` 文件副本已在 v0.60 清理中删除。

以下为旧布局说明，仅作历史参考。

---

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

## 真机远程观测

小电脑和查看画面的电脑连接同一个局域网。在小电脑工程根目录执行以下命令，运行主相机识别、跟踪和预测，并通过 Foxglove 发布数据。

```bash
cmake --build build --target standard -j4
./build/standard configs/sentry.yaml
```

程序默认隐藏本地窗口，通过 Foxglove 查看画面，按 Ctrl+C 退出。需要同时显示小电脑本地窗口时执行 `./build/standard configs/sentry.yaml -d`，窗口中按 q 退出。

在本电脑 Foxglove 中选择 Open connection → Foxglove WebSocket，输入 `ws://10.0.0.136:8765`，导入本目录的 `dge_debug_layout.json`。IP 使用小电脑当前局域网地址。

“窗口”面板订阅 `/detector/result_img/compressed`，接收与本地窗口相同的半分辨率画面，包含装甲板绿框、FPS、Yaw/Pitch 指令和 Shoot 状态。`/camera/image` 发布原始分辨率图像，`/camera/annotations` 的坐标对应原始图像。“窗口”面板使用图像内已绘制的绿框。

3D 场景查看车心、装甲板、预测轨迹和瞄准点。小电脑运行程序后才会监听 8765 端口。

### 截图中的曲线

| 字段 | 单位 | 计算含义 |
|---|---|---|
| `/tracker/info.position_diff` | m/帧 | 相邻发布帧车心估计之差的模长，包含真实运动和估计跳变 |
| `/tracker/info.yaw_diff` | rad/帧 | 相邻发布帧目标旋转角的最短有符号角差，包含真实旋转和估计跳变 |
| `/tracker/target.v_yaw` | rad/s | EKF 估计的目标旋转角速度，正负方向按世界系 z 轴右手定则 |
| `/tracker/target.radius_1`、`radius_2` | m | 两组相对装甲板到车心的半径，编号对应滤波器板组 |
| `/tracker/target.dz` | m | 另一板组相对参考板组的高度差，可正可负 |
| `/dge/debug.sigma_x`、`sigma_y` | m | 车心 x、y 的估计标准差，等于对应协方差对角线的平方根 |
| `/dge/debug.yaw`、`pitch` | rad | 算法输出的云台角度指令，包含瞄准预测、弹道与角度偏置 |
| `/dge/debug.aim_x` | m | 世界系中瞄准点的 x 坐标 |
| `/sim/ground_truth.targets[0].vyaw` | rad/s | 仿真器中目标 0 的真实旋转角速度，仅仿真数据流提供 |

位置与角度帧间变化量随真实运动和发布间隔变化。对运动目标应结合速度、角速度和帧间时间解释尖峰。标准差表示滤波器自身的不确定度估计，实际精度还需独立测量核验。

`/tracker/target.tracking` 为 false 时，当前发布器将目标状态数值置零，`/tracker/info` 的两个帧间变化量也清零。`sigma_x/y` 仅在存在跟踪目标时发布，`aim_x` 仅在瞄准点有效时发布。

图表红色感叹号需要悬停查看具体提示。若 Raw Messages 中 `/dge/debug`、`/tracker/target` 同时等待消息，先检查程序运行、连接和播放状态。若已收到消息但某条曲线报错，核对消息中是否存在对应字段。真机运行时仿真真值路径没有数据。

# RobotModel4 v1.01：rm_aim_sim 测试记录与使用教程

日期：2026-11-01。开发分支：`new-sim-test`。

本版将独立运行的 `rm_aim_sim` 作为 DGE 的仿真调试环境，新增 `auto_aim_sim` 入口。图像检测、PnP、跟踪、瞄准与射击仍由 DGE 执行，调试器负责三维渲染、云台响应、靶车运动、弹道及命中统计。测试使用真实渲染像素；理想角点和目标真值只用于诊断。

## 1. 分支和版本关系

```text
test：feaba15（RobotModel4 v0.40）
└─ new-sim-test
   ├─ 411192f：新调试器适配、移除旧 Daedalus 代码
   └─ 7b42c8f：记录首次像素闭环测试和动态场景限制
```

以上为本次测试的代码基线，后续 Foxglove 适配与本文在该分支上继续提交。

- DGE 仓库：[RM-DGE/DGE_vision_26 的 new-sim-test 分支](https://github.com/RM-DGE/DGE_vision_26/tree/new-sim-test)。
- 调试器：[ForgetWW520/rm_aim_sim](https://github.com/ForgetWW520/rm_aim_sim)。本文核对版本为 `3895971a25e7712286395840b3c4cea9dffbc27c`，协议 `rm_aim_sim/2`。
- 旧 `io/Daedalus/` 的 7 个代码文件和 `configs/daedalus.yaml` 已在新分支逐文件移除；旧路线及版本记录可从 Git 历史查看。本文编写期间按维护者确认保留工作区的旧文档删除状态。
- 真机继续使用 `standard`、`sentry_DGE`；仿真使用 `auto_aim_sim`。
- EKF 数学基线仍为 v0.33，本版没有重写其预测、观测、关联或 NIS 门控算法。

## 2. 数据如何流动

```text
独立 rm_aim_sim（Godot）
  │ state.json：会话、云台反馈、弹速、标定、统计
  ↑ request.json：DGE 申请一帧图像
  ↓ frame.json + frame-<request_id>.raw：像素及同帧姿态
auto_aim_sim（DGE）
  BGR 图像 → YOLO → Solver/PnP → Tracker/EKF → Aimer → Shooter
  │ command.json：绝对 yaw/pitch、control、fire
  ↓
调试器云台与发射 → 弹道/碰撞 → state.json 统计

DGE 调试输出 → Foxglove WebSocket → 图像、3D、曲线和统计
```

两个进程必须在同一个 Linux/WSL 发行版、同一用户下运行。默认 IPC 目录是 `/dev/shm/rm_aim_sim_default/`。图像是无压缩 RGB/RGBA，适配层复制并转换为自有 BGR 图像后，才申请下一帧。

每帧使用自己的四元数和仿真采样时间，不用较新的状态姿态替换。会话或发布者变化后重新建立时间 epoch 并清空旧跟踪。Aimer 的仿真入口显式使用观测年龄，避免仿真时间落后于墙钟时不断增加预测提前量。

坐标采用 x 前、y 左、z 上；yaw 向左为正、pitch 向下为正，协议角度单位为 rad。不要继续套用旧 Daedalus 的反号、零位 trim 或轴变换。

## 3. 已完成的测试

### 3.1 环境与自动检查

实测环境为 WSL Ubuntu 22.04 x86_64、WSLg、Godot 4.7.2、OpenGL Compatibility；图形输出使用 Intel UHD 的 D3D12/Mesa 驱动。DGE 使用 OpenVINO 2024.6 和 CPU 推理。

| 检查 | 已有结果 | 能证明什么 |
|---|---|---|
| DGE 全部默认目标构建 | 通过 | 真机与仿真目标均能编译 |
| DGE `rm_aim_sim_test` | CTest 1/1 通过 | 协议、时间映射、锁、RGB/RGBA、文件长度、序号、watchdog |
| 调试器核心测试 | 382 项通过 | 运动、云台、弹道和计分核心 |
| 调试器 IPC 集成 | 通过 | 指令回读、延迟出膛、计分、失联停火、拒绝旧会话 |
| 调试器图形相机测试 | 178 项通过 | 渲染、相机读回及相关像素行为 |

调试器内置辅助直接读取真值，它的测试成绩不能作为 DGE 图像自瞄的命中率。下面的结果才是 DGE 像素闭环样本。

### 3.2 首次像素闭环结果

| 场景 | 配置 | 样本结果 | 说明 |
|---|---|---|---|
| 静止靶 | `vision.cfg` | 19/20 已结算命中，95%；350 伤害，击毁 1 次 | 真实像素检测与开火闭环成立 |
| 平移靶 | `translate.cfg`，1.5 m/s | 12 s 样本：4/32 已结算命中，12.5%；采样时另有 3 发在飞行 | 换向附近出现估计超调和 EKF 重初始化 |
| 旋转靶 | `spin.cfg`，4 rad/s | 12 s、156 帧；19/90 已结算命中，21.1%；350 伤害，击毁 1 次；另有 2 发在飞行 | 动态性能仍需优化 |

平移场景观测年龄约 21–38 ms，估计速度约 ±1.2–1.65 m/s；旋转场景观测年龄约 29–42 ms。以上是本机单轮日志样本，未做多轮统计或统一置信区间，不代表最终性能，也不能直接外推到实车。

首次联调还修复了一个兼容问题：Godot 可能把整型尺寸输出为 `1440.0`。解析器允许有限且数值为整数的 JSON 数字，仍拒绝小数尺寸、非法范围和长度不符的帧。

## 4. 第一次安装与准备

### 4.1 使用 WSL 原生文件系统

以下命令都在 WSL Ubuntu 终端执行。推荐目录为：

```text
~/Downloads/
├─ hhh-rm-aim-sim/       DGE，new-sim-test 分支
└─ sim/rm_aim_sim/       独立 Godot 调试器
```

不要在 `/mnt/c` 下做日常构建：本机此前在该挂载路径遇到 `p9_client_rpc` 长时间等待。现有目录有改动时先检查 `git status`，不要覆盖或删除后重克隆。

全新环境可以这样取得两份代码：

```bash
mkdir -p ~/Downloads/sim
cd ~/Downloads
git clone --branch new-sim-test https://github.com/RM-DGE/DGE_vision_26.git hhh-rm-aim-sim
cd ~/Downloads/sim
git clone https://github.com/ForgetWW520/rm_aim_sim.git
cd rm_aim_sim
git checkout 3895971a25e7712286395840b3c4cea9dffbc27c
```

### 4.2 准备 Godot 和图形环境

```bash
cd ~/Downloads/sim/rm_aim_sim
python3 tools/setup_godot.py
./run.sh --headless --editor --import --quit
echo "$DISPLAY"
echo "$WAYLAND_DISPLAY"
```

下载脚本会取得固定版本 Godot 并校验 SHA256。资源导入命令的 `--headless` 仅用于首次导入；正式像素调试必须启动图形窗口。若没有 WSLg/显示会话，先解决图形环境，不能通过给正式仿真加 `--headless` 来替代。

### 4.3 准备 DGE 依赖和构建

新机器先参考仓库主 [README](../readme.md) 安装 OpenVINO 2024.6、OpenCV、Eigen、fmt、spdlog、yaml-cpp、nlohmann-json、libserial 和 libusb 等依赖。调试器本身不依赖 OpenVINO，但 DGE 检测网络需要它。

```bash
cd ~/Downloads/hhh-rm-aim-sim
git branch --show-current
source /opt/intel/openvino_2024/setupvars.sh
cmake -S . -B build
cmake --build build -j4
ctest --test-dir build --output-on-failure
```

本机安装路径为 `/opt/intel/openvino_2024`，项目 CMake 使用 `/opt/intel/openvino_2024.6.0/runtime/cmake/`。自定义安装位置时需同步检查 CMake 与运行库环境。运行程序时保持工作目录为 DGE 仓库根目录，模型路径 `assets/...` 相对此目录解析。

## 5. 日常启动：两个终端

### 5.1 终端一：调试器

```bash
cd ~/Downloads/sim/rm_aim_sim
./run.sh -- --config=res://configs/vision.cfg --remote
```

确认窗口出现，并看到 `Aim Lab ready`、`remote=true`、`local_ipc=/dev/shm/rm_aim_sim_default`。`--remote` 表示启用外部控制。

### 5.2 终端二：DGE 自瞄

```bash
cd ~/Downloads/hhh-rm-aim-sim
source /opt/intel/openvino_2024/setupvars.sh
./build/auto_aim_sim configs/rm_aim_sim.yaml
```

有限时长测试可以使用第二个位置参数，例如：

```bash
./build/auto_aim_sim configs/rm_aim_sim.yaml 30
```

启动后查看 `rm_aim_sim pixel closed loop` 及每秒 JSON 日志。确认先有 `frames` 增长，再有 `armors/targets`，最后检查 `control/fire`、`stats.fired/hit/resolved`。程序本身不要求打开 OpenCV 图像窗口，图像通过调试器预览或 Foxglove 查看。

### 5.3 先分离检查通信和图像

首次运行有问题时，先退出 DGE，在调试器仓库依次执行：

```bash
python3 tools/control_demo.py --yaw=5 --pitch=3 --duration=3
python3 tools/camera_demo.py --no-display --frames=30 --duration=10
```

第一个示例检查固定角控制，不主动开火；第二个只收图。示例角度参数单位为度，协议内部为弧度。同一通道只允许一个控制端，不能让这些示例与 `auto_aim_sim` 同时占锁；示例结束后再启动 DGE。

## 6. 界面操作与场景切换

| 操作 | 用途 |
|---|---|
| `V` | 第一/第三人称切换；第三人称可右键拖动、滚轮缩放 |
| `P` | 暂停/恢复；会话改变，DGE 清空旧跟踪后重新获取观测 |
| `R` | 重置本轮、恢复血量、清空统计，开始新会话 |
| 平移/自转开关与速度输入 | 设置运动场景，参数即时生效 |
| 车型/阵营 | 改变目标外观；阵营需与 DGE 的 `enemy_color` 配套 |
| 相机预览 | 查看曝光后的自瞄输入；第一人称主画面不等于曝光后图像 |

本地模式下，方向键调角、按住 F 进行真值辅助跟踪、Space 发射。外部控制开启后，以 DGE 指令为准。不要用本地真值辅助的表现替代图像算法验收。

重启调试器前先 Ctrl+C 退出 DGE；每轮测试重新启动两边或按 R 重置，并确认新会话后再计数。下面的配置覆盖默认值，相机仍采用默认 1440×1080、45° 垂直视场角：

```bash
# 仅平移：1.5 m/s
./run.sh -- --config=res://configs/translate.cfg --remote
# 仅旋转：4 rad/s
./run.sh -- --config=res://configs/spin.cfg --remote
# 平移加旋转
./run.sh -- --config=res://configs/combined.cfg --remote
```

一次只启动一个默认通道仿真器。建议按“静止 → 平移 → 旋转 → 组合”的顺序测试，每次只改变一个变量。目标击毁后会停在死亡位置，继续运行不再是存活运动靶测试；应按 R 开始下一轮。

## 7. 两边配置如何配套

DGE 配置文件为 `configs/rm_aim_sim.yaml`：

```yaml
rm_aim_sim:
  channel: default
  ipc_root: /dev/shm
  state_timeout_ms: 200
  frame_timeout_ms: 5000
  publish_truth: true
```

| 参数 | 当前值与要求 |
|---|---|
| 通道 | 仿真 `[ipc].channel` 与 DGE `rm_aim_sim.channel` 一致，默认 `default` |
| 图像 | 1440×1080，垂直 FOV 45°，零畸变 |
| 内参 | fx=fy≈1303.675324，cx=720，cy=540 |
| 相机外参 | optical→gimbal 为 `[0,0,1; -1,0,0; 0,-1,0]`，安装平移 `[0.08,0,0.04]` m |
| IMU 变换 | `R_gimbal2imubody` 为单位阵 |
| 发射延迟 | DGE 高/低速 delay 均为 0.015 s，与调试器 `weapon.delay` 一致 |
| 弹速 | 从状态反馈读取，不能沿用旧真机强制弹速 |
| 阵营 | 默认靶为蓝方，DGE `enemy_color: blue` |
| 开火 | `auto_fire: true` 才允许 Shooter 给出开火许可 |

运行时会逐项检查内外参。改变相机分辨率、FOV 或安装偏移后，必须同步 DGE 配置并重启；当前入口不会自动重新标定。改变发射延迟时也要同步 DGE 的预测延迟，不能只改调试器界面。

## 8. Foxglove 调试教程

### 8.1 连接

1. 启动调试器和 DGE，确认 DGE 输出 `WebSocket server on ws://0.0.0.0:8765`。
2. 在 Foxglove 中选择 Open connection → Foxglove WebSocket，输入 `ws://localhost:8765`。
3. 选择 Layout → Import layout，导入 `foxglove_layouts/rm_aim_sim_layout.json`。完整话题字段见 [布局说明](../foxglove_layouts/README.md)。
4. 若 Windows 无法通过 localhost 连接，先在 WSL 执行 `hostname -I` 查看地址，使用 `ws://<WSL地址>:8765`，并检查 Windows/WSL 防火墙与端口转发。

该 WebSocket 来自 DGE，不是 Godot，也不是 ROS bridge。8765 已占用时应结束重复的 DGE 进程；仅启动 Godot 不会产生这些 Foxglove 话题。

自行编写 WebSocket 客户端时，要与仓库使用的官方 SDK 协商匹配的子协议。验证脚本会提供 `foxglove.sdk.v1` 等候选项；仅提供旧的 `foxglove.websocket.v1` 可能收到 HTTP 400。直接使用支持该 SDK 的 Foxglove 客户端连接即可，无需在界面手工填写子协议。

启动两端后，在 DGE 根目录可用标准库脚本验证消息传输：

```bash
python3 tests/foxglove_ws_smoke.py --duration=6
```

本次 6 s 实测收到 `/camera/image` 71 条，以及跟踪、调试、仿真状态各约 142 条；TF、3D 场景和预测消息也均收到，`fresh=true`、`observation_valid=true`，观测年龄约 20.8 ms。该检查证明协议握手与话题传输成功；布局 JSON 通过解析与字段核对，但没有把这些检查等同于 Foxglove 桌面界面的人工逐面板验收。

保持 DGE 运行、退出调试器后，还可以验证失联状态：

```bash
python3 tests/foxglove_ws_smoke.py --status-only --expect-inactive --duration=3
```

本次负向检查收到 254 条状态消息，并断言 `fresh=false`、`observation_valid=false`；帧时间和观测年龄为 null。恢复测试时重新启动调试器，必要时重启 DGE，以重新打开通道并获取新会话。

### 8.2 推荐观察顺序

| 层次 | 面板/话题 | 先判断什么 |
|---|---|---|
| 图像 | `/camera/image`、`/camera/annotations` | 灯条和编号是否清晰、四角是否稳定 |
| 检测/跟踪 | `/tracker/target`、`/tracker/info`、3D 场景 | 是否锁定、板位姿是否跳动、换板是否合理 |
| 预测/开火 | `/dge/debug`、`/dge/prediction` | 指令角、实际云台角、瞄准点和射击许可 |
| EKF | `/dge/debug` 的 NIS、更新数、协方差字段 | 是否长期拒绝观测、协方差是否增大、是否反复重初始化 |
| 仿真 | `/sim/status` 与布局中的统计视图 | `observation_age_s`、会话、控制状态、`stats.hit/resolved/in_flight` |

3D 固定坐标系使用 `odom`，TF 为 `odom → gimbal_link → camera_optical_frame`。DGE 的 odom 原点在云台旋转中心；上游世界原点在地面，直接拿原始 truth 世界坐标与 DGE 位置相减会引入固定高度偏差。

真值仅用于对照。上游状态没有目标角速度测量，不能用填零的 `vyaw` 计算“角速度误差”；当前时刻真值与历史图像估计也需要先对齐采样时刻，不能把时间偏差当成 EKF 位置误差。

本版补齐 `auto_aim_sim` 对分类图、目标状态、抖动、Marker 和 `/dge/debug` 的发布，并新增 `/sim/status`。`fresh=false` 表示状态超时，`observation_valid=false` 表示当前没有可用于控制的有效观测；先检查这两个字段，再判读历史曲线。`state_sim_time` 与 `frame_sim_time` 分别是反馈状态和图像采样的仿真时间。

`/sim/ground_truth` 的位置已减去云台世界原点，与 odom 共原点；角速度缺失时输出 null。新入口不会把较新的状态真值传入旧误差计算，因此 `center_err/yaw_err/vyaw_err` 不作为本版有效指标，也不应将空曲线理解为零误差。

### 8.3 动态性能排查

当识别持续有板但命中率低时，按顺序检查：

1. 比较观测年龄和实际帧间隔，区分推理耗时、仿真时间和墙钟时间。
2. 看目标位置/速度是否在换向附近超调，再看 NIS、接受更新数和重初始化日志。
3. 检查当前实际云台姿态是否追上指令，避免只看目标角。
4. 检查 15 ms 发射延迟与弹速反馈，再检查 Shooter 许可。
5. 保存同一场景的配置、DGE/调试器提交和日志后，才进行单参数 A/B 对比。

“10 帧没有接受更新后重新初始化”说明观测连续未通过更新路径，不等于适配层重启了 Tracker。目前没有证据支持直接翻转装甲倾角或 yaw/pitch 符号；应先分析预测角点与观测角点残差。

## 9. 统计口径与复现实验

命中率使用 `stats.hit / stats.resolved`；在飞弹丸不计入分母。上游满足 `fired = resolved + in_flight`，`pending` 是待出膛数量，不计入 fired。伤害按实际扣血累计，目标归零后几何命中仍可能增加，不能据此重复统计击毁。

建议每轮固定 30 s 墙钟运行时长，并同时记录仿真时间。下例在 DGE 根目录保存控制台输出：

```bash
mkdir -p logs
set -o pipefail
./build/auto_aim_sim configs/rm_aim_sim.yaml 30 2>&1 | tee logs/sim-static.log
```

将 static 换成 translate/spin 标记对应场景；不要用同一文件名覆盖需要保留的实验。记录：

- 两个仓库各自的 `git rev-parse HEAD`、DGE 分支名。
- 场景配置、目标速度/角速度、相机参数、阵营、血量。
- 帧数、观测年龄、fired/hit/resolved/in_flight/damage/kills。
- 是否发生暂停、重置、击毁、卡顿或重新初始化。

上游可导出本轮配置、累计统计和近期事件到 `captures/`。近期事件为有长度限制的环形记录，完整结果优先以累计 stats 和及时保存的日志为准。

## 10. 常见问题

| 现象 | 检查与处理 |
|---|---|
| 找不到 IPC 目录 | 先启动调试器；检查两进程同 WSL、同用户、同 channel |
| `Cannot acquire client.lock` | 退出其他 DGE/control_demo/camera_demo；不要删除活进程的锁文件 |
| 缺图或 frame timeout | 正式仿真必须图形模式；检查 WSLg、OpenGL 和是否启用了 IPC |
| `Calibration mismatch` | 还原默认相机或同步 YAML，重启 DGE；不要靠经验 trim 掩盖 |
| frames 增长但无 armors | 看曝光后图像、阵营、YOLO 模型、阈值及数字贴图；先排查识别 |
| targets 有但不发射 | 检查 auto_fire、控制使能、跟踪确认、瞄准有效性和 Shooter 容差 |
| 击毁后图像里灯条熄灭 | 属于正常死亡表现，按 R 恢复血量并开始新一轮 |
| Foxglove 图像有但曲线空 | 确认运行的是本版 auto_aim_sim，导入本版布局并检查话题列表 |
| 曲线停留在旧值 | 查看状态有效性与 session；失联期间不得把最后一条消息当成实时反馈 |
| OpenVINO 动态库找不到 | source `/opt/intel/openvino_2024/setupvars.sh`，检查安装位置 |
| 从 Windows 目录编译很慢 | 切换到 WSL 原生目录构建；不要用旧 Bevy 启动脚本 |

## 11. 文件速查与后续工作

### 11.1 EKF 四板特写与选板

2026-11-01 更新后的 `foxglove_layouts/rm_aim_sim_layout.json` 默认跟随 `ekf_target`，视距 1.7 m，显示 `/dge/ekf_model`。更新程序后需要重新导入布局；仅重连不会替换 Foxglove 保存的旧布局。

四板中心和完整姿态直接来自 EKF `Target::full()`，以 `#0`～`#3` 标识。绿色实心板是 Aimer 最终选择的板，青色轮廓是最近一次观测关联的板，橙色为其他板。浅绿色板及 `AIM` 标注显示预计命中时刻的位置，与当前选板之间的线段显示提前量。目标中心上方展示双半径、高度差、角速度及板号。

尺寸是算法所用的灯条角点模型：小板 135 × 56 mm，大板 230 × 56 mm；8 mm 显示厚度不是实际板厚，也不代表裁判命中区域。右下 `/dge/selection` 可展开四板 odom 坐标、四元数、尺寸和选板标志。`selected_id=-1` 表示没有有效选板；最近观测编号不代表本帧一定更新成功。编号只在本次 EKF 初始化周期内有效。

视角移走时选择显示坐标系 `ekf_target`、跟随位置，并按 `1` 回中。默认隐藏整场 `/dge/scene` 和未来多帧 `/dge/prediction`，需要时在主题里打开。无目标会清除模型，发布停止后实体在 0.5 秒后过期。

回归命令：

```bash
ctest --test-dir build --output-on-failure
python3 tests/foxglove_ws_smoke.py --duration 8 --require-model
```

本次验证：WSL 下 `auto_aim_sim`、`standard`、`sentry_DGE` 构建通过，CTest 2/2 通过（协议与模型几何/选板/序列化）。独立通道、Intel + Wayland、CPU 推理、`gimbal.max_speed=0.1`、`gimbal.response_hz=0.1`、`auto_fire=false` 的诊断运行中，8 秒收到 188 帧有效模型与选板消息。该低速配置用于验证可视化，不是默认闭环命中验收；另一轮默认云台、NVIDIA 渲染运行只收到 2 帧有效选板后失去检测，不能据此宣称默认闭环已稳定。尚未在 Foxglove 应用中人工确认新版视角的最终显示效果。

### 11.2 分清三种帧率再优化

- 仿真窗口 FPS：Godot 渲染帧率；不能用它代替 DGE 算法处理频率。
- `/dge/debug.fps`：DGE 完成帧之间的墙钟频率，受取图、推理和发布等影响。
- Foxglove 图像：当前 `publish_image()` 主动限制约 15 FPS；3D 和状态随算法帧发布。

本机 WSL 默认 OpenGL 为 Intel UHD，也能选择 RTX 4060 Laptop。2026-11-01 在相同相机/物理配置、无 DGE 负载、关闭 IPC 的短时对照中，各组运行 18 秒，丢弃最初两个 FPS 采样：Intel 默认路径中位数 72 FPS，RTX 4060 解除限帧 51.5 FPS，Intel + Wayland 解除限帧 77 FPS。该结果只代表本机此次空载窗口测试，不是闭环性能保证；不要假定独显一定更快。

本机推荐先尝试以下命令，DGE 仍使用 CPU，不更改物理步长或相机配置：

```bash
MESA_D3D12_DEFAULT_ADAPTER_NAME=Intel \
./run.sh --display-driver wayland --disable-vsync --max-fps 0 -- --config=res://configs/vision.cfg --remote
```

环境变量依据 [WSLg GPU 选择说明](https://github.com/microsoft/wslg/wiki/GPU-selection-in-WSLg)，引擎参数依据 [Godot 命令行说明](https://docs.godotengine.org/en/stable/tutorials/editor/command_line_tutorial.html)。先比较同一场景、同一控制端下的帧率与延迟；取消上限不保证一定更快，也可能让渲染占用更多资源。

本工程为贴近真实部署，明确保留 `device: CPU`。后续仿真器优化应优先测量主窗口渲染、两级相机视口渲染及 GPU 像素读回开销。上游原始相机和曝光处理视口目前均为 `UPDATE_ALWAYS`；可研究将主窗口刷新与相机请求采样独立调度，但必须验证图像与姿态仍为同帧，不能直接跳帧后复用旧像素。当前没有修改上游源码或采样策略。

Godot 默认 X11 路径在本机报告无法切换 VSync；Wayland 本次没有该警告。窗口真正显示的流畅度还受 WSLg 合成器及显示器刷新率限制，参见 [WSLg 帧率说明](https://github.com/microsoft/wslg/wiki/Controlling-WSLg-frame-rate)。降低相机分辨率需要同步内参、元数据校验和检测精度验证，本次保持 1440×1080。

### 11.3 文件速查

| 文件 | 作用 |
|---|---|
| `src/auto_aim_sim.cpp` | 仿真主循环、标定检查、像素算法调用、Foxglove 输出 |
| `io/rm_aim_sim/protocol.*` | 协议类型、严格解析、命令序列化 |
| `io/rm_aim_sim/local_client.*` | 独占锁、原子 JSON、请求握手、像素复制、watchdog |
| `io/rm_aim_sim/clock_mapper.*` | 仿真时间 epoch 映射 |
| `configs/rm_aim_sim.yaml` | 模型、算法、相机及通道配置 |
| `tests/rm_aim_sim_test.cpp` | 不需要 Godot 的协议与 IPC 回归测试 |
| `tools/Foxglove/`、`foxglove_layouts/` | WebSocket 发布与调试布局 |

后续优先工作是多轮动态场景基线、换向时观测拒绝原因、检测角点稳定性及预测/火控延迟预算。仿真尚不等同完整实车：没有我方底盘运动、完整车体遮挡、空气阻力、相机噪声和运动模糊。参数调整应先在固定配置下比较，再回到实车验证。

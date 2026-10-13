# sp25_learn + bevy_robomaster_sim 接入 Foxglove 技术文档

> 目标：把桌面上的 `sp25_learn`（同济 sp_vision_25 自瞄算法）与 `bevy_robomaster_sim-master`（Daedalus 仿真器）接入 Foxglove，实现 **图像、控制曲线、3D** 三类显示，并**最小化主循环改动**。
>
> 本文所有关于两个项目的结论都基于源码核对（文件路径与行号见第 12 节）。Foxglove SDK 相关代码是**接口骨架或待验证设计**，其头文件名 / 命名空间 / 返回值请以你实际安装的 SDK 版本为准（见 1.4 版本锁定表）。

---

## 0. 摘要与结论

经过源码核对，得到几个与"初步思路"不同的关键事实：

1. **仿真器 Daedalus 已经把接入 Foxglove 所需的全部数据写进了 Talos 共享内存**：图像、5 路姿态、相机内参、底盘观测、**场景真值（敌方机器人位置/朝向/角速度、能量机关）**、运行状态。因此**在主推方案下仿真器不需要改动**（见 Phase 8 的例外说明）。
2. **sp25_learn 已有一个 UDP JSON 曲线通道**：`tools::Plotter` 会把 `nlohmann::json` 通过 UDP 发到 `127.0.0.1:9870`（`tools/plotter.hpp:15`）。`standard.cpp:40` 已构造 `plotter`，只是 `plot()` 调用被注释（`standard.cpp:183-196`）。**注意：这是通往 PlotJuggler 的独立通道，不等于"已进入 Foxglove"**（见 2.1）。
3. **两个项目里都不存在任何 Foxglove / MCAP / WebSocket 代码**。参考说明文档里 Hero 项目的 `simulation/foxglove_bridge.py` 在这两个项目里没有对应物，需要新建。
4. 推荐路线是 **在算法进程内新增一个只读的 `FoxglovePublisher` 模块**，由 `standard.cpp` 每帧调用一次；`io::Talos` 增加**返回自持快照**的只读访问器（不改动任何写入与控制逻辑）。
   - `standard.cpp` 预计 **约 3 处**改动；
   - **整个项目**还需要：新增发布模块、扩展 `io::Talos` 接口、修改根 `CMakeLists.txt`、新增 SDK 依赖、新增独立测试程序。**不要**把"3 处改动"理解成整个工程只有 3 个修改点。
   - **架构风险较低，但接口实现、共享内存快照一致性、坐标方向和性能风险均待验证。**

**为什么必须在算法进程内发布图像**：Talos 图像是**单生产者-单消费者**协议，仿真器只有在图像和 4 路姿态都被消费后才发布下一帧（`crates/talos-ipc/src/publisher.rs:123-134`）。若另起进程读共享内存图像，会成为第二个消费者，**破坏仿真器的同步门控**。

---

### 0.1 实现状态（2026-10-13）

方案 A 已在本机落地并通过端到端验证：

| 项 | 状态 | 证据 |
|---|---|---|
| Foxglove SDK 集成（CMake） | ✅ | `find_package` + `foxglove_sdk_add_cpp_library`，仅链接 `standard` |
| `ENABLE_FOXGLOVE=OFF` 编译隔离 | ✅ | OFF 版二进制无任何 foxglove 符号/话题串 |
| `io::Talos` 只读快照 | ✅ | `read_poses` / `read_ground_truth` / `last_image_timestamp_ns` |
| `FoxglovePublisher` | ✅ | 曲线 + 图像 + TF + 3D 四条通道 |
| `standard.cpp` 最小改动 | ✅ | 3 处（include / 构造 / 每帧 `publishFrame`） |
| 端到端 | ✅ | daedalus + standard 运行 485+ 帧；服务端 advertise：`/sp25/telemetry`(json)、`/sp25/image/compressed`(CompressedImage)、`/sp25/scene`(SceneUpdate)、`/tf`(FrameTransform) |

构建/运行（本机实测）：
```bash
cmake -S . -B build-talos-foxglove -DENABLE_TALOS_MODE=ON -DENABLE_SIM_MODE=OFF \
  -DENABLE_FOXGLOVE=ON -DOpenVINO_DIR=/home/ljy/opt/openvino_2024.6.0/runtime/cmake
cmake --build build-talos-foxglove --target standard -j"$(nproc)"
# 先起 daedalus，再起算法，然后 Foxglove 连 ws://127.0.0.1:8765
```
> `/sp25/telemetry` 已带 JSON Schema（`sp25.Telemetry` / `jsonschema`），Plot 可直接选字段；图像经**从实时流提取验证**为有效 1440×1080 JPEG。3D 已加**参考场景**（`/sp25/reference_scene`：地面网格 + 带标签 XYZ 三轴）与**瞄准点/预测装甲**（`/sp25/scene` 的 `aim` 实体）；`gimbal→camera` 已改用**配置外参**并与配置**误差 0.00mm**。剩余：性能测量、回归测试、MCAP、`[TALOS-DBG]` 降频。

---

## 1. 现状核对（以源码为准）

### 1.1 仿真器 Daedalus 的输出

**共享内存文件**
- `/tmp/talos_ipc_meta`：3712 字节元数据（`crates/talos-ipc/src/layout.rs:290`）。
- `/tmp/talos_ipc_image_pool`：3 × 1440×1080×3 的 RGB 图像池（`layout.rs:3-12`）。

**元数据布局**（`layout.rs`，与 C++ 侧 `io/talos/talos.cpp:81-106` 的编译期断言一致）：

| 偏移 | 内容 | 写入方式 | 读取安全性 |
|---|---|---|---|
| 0 | `ShmHeader` | 心跳（`update_heartbeat`） | 只读字段，安全 |
| 64 | `ImageTripleBuffer` | 三缓冲 + CAS 提交 | 安全（有握手） |
| 256 | `poses[5]` | 三缓冲 + CAS 提交 | 安全（有握手，见 1.2 消费约束） |
| 1536 | `GimbalTripleBuffer` | 三缓冲 | 算法写入，仿真读取 |
| 1728 | `CameraInfo` | **一次性** `set_camera_info`（`publisher.rs:183-188`） | 构造后不变，安全 |
| 1856 | `ChassisObservation` | **普通整块拷贝**（`publisher.rs:190-195`） | ⚠️ 无同步，可能撕裂 |
| 1984 | `GroundTruthBatch` | **普通整块拷贝**（`publisher.rs:197-202`） | ⚠️ 无同步，可能撕裂 |
| 3648 | `RuntimeState` | **普通整块拷贝**（`publisher.rs:204-209`） | ⚠️ 无同步，可能撕裂 |

**5 路姿态的语义**（`src/talos/capture.rs:332-373`，索引见 `layout.rs:296-306`）：

| PoseIndex | 位置 | 姿态 | 用途 |
|---|---|---|---|
| `Gimbal`=0 | `[0,0,0]` | 云台朝向四元数（枪口 FLU） | 云台姿态 |
| `Odom`=1 | 云台**世界位置**（ROS/FLU） | 单位四元数 | 世界定位 |
| `Muzzle`=2 | 相对云台的枪口平移 | 单位四元数 | 枪口外参 |
| `Camera`=3 | 相对云台的相机平移 | 单位四元数 | 相机外参 |
| `ChassisObservation`=4 | 兼容通道（旧） | — | 建议读 `chassis_observation` |

> 位置和姿态被拆开了：`Gimbal` 只有姿态（位置为零），世界位置在 `Odom`。做 TF 时需组合，且**每个字段的坐标系与旋转方向必须逐项验证**（见第 7 节）。

**场景真值**（`src/talos/ground_truth.rs`）：步兵的 ROS/FLU 世界坐标 `position`、`yaw`、`vyaw`、`team`、`armor_label`；能量机关的世界坐标、当前角度、转速、方向、激活状态。真值与图像是**两条独立发布路径**（`src/talos/plugin.rs:99-103`，主世界 `Last` 阶段），**没有图像那样的四通道握手**。

**坐标映射**：Bevy 是 Y-up，通过 `M_ALIGN_MAT3` 映射到 ROS/FLU（`plugin.rs:180-204`）：`x_ros=-z_bevy, y_ros=-x_bevy, z_ros=y_bevy`。Foxglove 3D 同为 FLU，**在确认真值/姿态确实经过该转换后**才可直接使用（见第 7 节）。

**关键事实：仿真器没有 Foxglove / UDP / MCAP 代码**；`src/telemetry/` 已定义 `TelemetryExporter` 抽象，但 `telemetry_dispatch` 目前只发空 `FrameData`（`telemetry/mod.rs:39-52`），且 `main.rs` **未注册** `TelemetryPlugin`。

### 1.2 算法 sp25_learn 的输入

**`io::Talos` 接口**（`io/talos/talos.hpp`，PIMPL）：
```cpp
class Talos {
public:
  bool read_image(cv::Mat & image, std::chrono::steady_clock::time_point & timestamp);
  Eigen::Quaterniond get_gimbal_quaternion(std::chrono::steady_clock::time_point timestamp) const;
  void send_gimbal_command(bool control, bool fire, float yaw, float yaw_vel, float yaw_acc,
                           float pitch, float pitch_vel, float pitch_acc, float distance_m);
};
```
- `read_image` 内部消费 **1 个图像槽 + poses[0..3]**（`talos.cpp:232-269`），RGB→BGR。
- **只对外暴露云台四元数**；`camera_info / chassis_observation / ground_truth / runtime_state` 已映射进内存但无对外接口。
- 时间戳：图像时间由 shm 的 Unix ns 经一次性 `clock_offset` 转成 **`steady_clock`**（`talos.cpp:220-230`）。**原始 Unix ns 目前不对外返回**（见第 5 节设计任务）。

**`standard.cpp` 主循环**（共 200 行）：

| 行号 | 动作 |
|---|---|
| 39-41 | `tools::Exiter exiter; tools::Plotter plotter;` |
| 43-49 | `#ifdef ENABLE_TALOS_MODE` → `io::Talos talos;`（否则实机 camera/gimbal/cboard） |
| 51-55 | `detector / solver / tracker / aimer / shooter` 构造 |
| 68 | `talos.read_image(img, t)` |
| 87 | `q = talos.get_gimbal_quaternion(t - 1ms)` |
| 95-96 | `armors = detector.detect(img); targets = tracker.track(armors, t);` |
| 102 | `command = aimer.aim(targets, t, bullet_speed)` |
| 106-107 | `command.shoot = shooter.shoot(...)` |
| 111-117 | 计算 `target_distance`，`talos.send_gimbal_command(...)` |
| 126-129 | 在 `img` 上画绿色检测框 |
| 150 | `cv::resize(img, img, {}, 0.5, 0.5)` |
| 157-169 | 计算 `fps`，`putText` 画 FPS / Yaw / Pitch / Shoot |
| 171-172 | `cv::imshow("Standard Test", img); cv::waitKey(1)` |
| 183-196 | **被注释掉的** `nlohmann::json` 曲线上报（`plotter.plot(...)`） |

**`tools::Plotter`**（`tools/plotter.hpp/cpp`）：UDP `SOCK_DGRAM` → `127.0.0.1:9870`，`plot(json)` 用 mutex 保护后 `sendto`。

### 1.3 构建系统

- 根 `CMakeLists.txt:21-25`：`ENABLE_SIM_MODE`（ROS2）与 `ENABLE_TALOS_MODE` 互斥。
- 目标 `standard`（`CMakeLists.txt:45,52-58`）：链接 `${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io`，仅当 `ENABLE_TALOS_MODE` 时加宏 `ENABLE_TALOS_MODE`。
- `io` 是 STATIC 库（`io/CMakeLists.txt:28`），`tools` 是 OBJECT 库（`tools/CMakeLists.txt:3`）。**OBJECT 库的坑**：往 `tools` 里加依赖 Foxglove SDK 的源文件，会让**所有**链接 `tools` 的可执行文件都需要 SDK 符号 → 链接失败。所以 Foxglove 源文件应**直接加到 `standard` 目标**。

### 1.4 版本锁定表（实现前先填实并冻结）

| 项目 | 本机当前值 | 备注 |
|---|---|---|
| 操作系统 | Ubuntu 22.04.5 LTS (Jammy) | |
| 编译器 | GCC 12.3.0 | SDK 要求 C++17+，本项目已是 C++17 |
| CMake | 3.22.1 | SDK 的 CMake helper 需 ≥ 你选定 SDK 版本要求 |
| OpenCV | 4.5.4 | |
| Rust | 1.99.0 | 仿真器 edition 2024 |
| Foxglove Studio | 3.3.0（`/opt/Foxglove`） | 客户端 |
| **Foxglove C++ SDK** | **sdk/v0.29.1**（已下载，解压到 `sp25_learn/third_party/foxglove`） | 非 apt 包；C++ 包装源码随工程编译，链接预编译 `libfoxglove.a` |
| Talos 仿真器 | 待填：git commit | |
| sp25_learn | 待填：git commit | |

---

## 2. 目标架构与设计原则

### 2.1 数据流（注意两条独立的观察通道）

```
┌──────────────────────────┐        Talos 共享内存          ┌──────────────────────────┐
│  bevy_robomaster_sim     │  /tmp/talos_ipc_meta           │  sp25_learn              │
│  (Daedalus, Rust/Bevy)   │  /tmp/talos_ipc_image_pool     │  (standard.cpp)          │
│  渲染/采集/物理           │ ────────────────────────────▶  │  检测/跟踪/瞄准/开火       │
│                          │  图像 + 5 路姿态 + 真值         │                          │
│  云台 PID / 弹丸物理      │  ◀──────────────────────────── │  send_gimbal_command     │
└──────────────────────────┘        gimbal_cmd             └────────────┬─────────────┘
                                                                         │ 只读快照(自持存储)
                              [通道 A: 独立，不进入 Foxglove]             ▼
   tools::Plotter(UDP 9870) ──▶ PlotJuggler                              ┌──────────────────────────┐
                                                                         │  FoxglovePublisher(新)    │
                                                                         │  读 io::Talos + command   │
                              [通道 B: 本方案]                            └────────────┬─────────────┘
                                                                                      │ WebSocket (log_time)
                                                                                      ▼
                                                                       ┌──────────────────────────┐
                                                                       │ Foxglove Studio          │
                                                                       │ ws://127.0.0.1:8765      │
                                                                       │ Image / Plot / 3D / MCAP │
                                                                       └──────────────────────────┘
```

- **通道 A**：`tools::Plotter` → UDP 9870 → PlotJuggler。既有实现，**数据不会自动进入 Foxglove**；除非另写桥接，否则不能把它当作"已接入 Foxglove"。
- **通道 B**：`FoxglovePublisher` 进程内 WebSocket 直发。本方案主线。
- 两条通道可并存，互不干扰（端口、协议都不同）。

### 2.2 设计原则

1. **只读**：`FoxglovePublisher` 只接收**自持存储的快照**，绝不操作云台、目标状态或共享内存写入路径。
2. **失败隔离**：Foxglove 初始化/发布失败只记录并降级，**绝不能让 `standard` 退出**；但 `io::Talos` 自身的失败仍沿用原有异常策略。
3. **进程内**：图像必须由算法进程发布，避免成为 Talos 图像的第二消费者。
4. **编译隔离**：新增独立开关 `ENABLE_FOXGLOVE`，默认 **OFF**；关闭时不引入任何 Foxglove 依赖。
5. **快照一致性**：只读 API 必须返回**拥有自身存储空间**的快照；**不得**返回指向共享内存的裸指针或引用（见第 6 节）。
6. **性能可测**：图像编码与消息发布耗时必须单独测量；性能隔离与异常隔离是**两件不同的事**，不能互相证明（见第 8 节）。
7. **可复现**：锁定 SDK / 工具链版本，先过最小测试程序，再接入主项目（见 Phase 0）。

### 2.3 方案对比

| 方案 | `standard.cpp` 改动 | 新增依赖 | 图像 | 3D | 风险 |
|---|---|---|---|---|---|
| **A. 进程内 C++ Foxglove SDK（推荐）** | 约 3 处 | 仅 `standard` 目标链接 SDK | 直接编码内存中的 `img` | 直接读 shm 真值/姿态 | **架构风险较低；接口实现与性能风险待验证** |
| B. 复用 `Plotter` UDP + Python 桥 | 约 3 处 + 新 UDP 图像发送器 | Python `foxglove-sdk` | 需新增分片 JPEG UDP | 需把真值再走 UDP | 中（多一套分片/重组，易出错） |

本文以 **方案 A** 为主线；仿真器在所有方案下都零改动。

### 2.4 代码状态标注（重要）

本文代码分三类，后续实现与验收必须对号入座：

- **可直接编译的示例**：已在明确记录的 SDK/编译器/系统环境下构建验证。目前 `Phase 0` 三个冒烟测试与 **`FoxglovePublisher` 及其 `standard.cpp` 接入**均已在本机 sdk/v0.29.1 上编译并端到端运行通过（见 0.1）。
- **接口骨架**：展示类结构、数据流或调用位置，仍需补充具体消息字段与错误处理（如 `FoxglovePublisher` 的通道创建与 `log()` 调用）。
- **待验证设计**：共享内存快照一致性（第 6 节）、坐标变换方向（第 7 节）、性能开销（第 8 节），必须在集成测试中验证。

**验收时不得把 `...` 或注释掉的发布语句当作已实现功能。** 正确做法：先做 Phase 0 的三个独立最小测试。

---

## 3. 数据映射表（Talos/算法字段 → Foxglove）

| 面板 | 话题 | 消息类型 | 数据来源 | 状态 |
|---|---|---|---|---|
| Image | `/sp25/image/compressed` | `foxglove.CompressedImage` | `standard.cpp` 的 `img`（BGR，含绿框） | 待实现 |
| Raw Messages | `/sp25/telemetry` | JSON（RawChannel） | 见下表 | 待实现 |
| Plot | `/sp25/telemetry` | 同上 | 同上 | **需单独验证 schema（见 Phase 5）** |
| 3D | `/sp25/scene` | `foxglove.SceneUpdate` | `GroundTruthBatch` | 待实现 |
| 3D TF | `/tf` | `foxglove.FrameTransform(s)` | `io::Talos` poses | 待实现 |
| 3D 参考 | `/sp25/reference_scene` | `foxglove.SceneUpdate` | 静态米制网格/坐标轴 | 待实现 |

**曲线字段**（`io::Command` 字段名见 `io/command.hpp`）：

| 字段 | 含义 | 来源 | 单位 |
|---|---|---|---|
| `fps` | 处理帧率 | `standard.cpp` | Hz |
| `cmd_yaw` / `cmd_pitch` | 云台指令 | `command.yaw/pitch` | rad |
| `cmd_yaw_vel` / `cmd_yaw_acc` | 前馈 | `command.yaw_vel/_acc` | rad/s, rad/s² |
| `cmd_pitch_vel` / `cmd_pitch_acc` | 前馈 | `command.pitch_vel/_acc` | rad/s, rad/s² |
| `target_distance` | 目标距离 | `standard.cpp:113` | m |
| `control` / `shoot` | 控制有效/开火 | `command.control/shoot` | 0/1 |
| `ekf_cx` / `ekf_cy` / `ekf_cz` | EKF 中心 | `target.ekf_x()[0,2,4]` | m |
| `ekf_vx` / `ekf_vy` / `ekf_vz` | EKF 速度 | `ekf_x()[1,3,5]` | m/s |
| `ekf_yaw` / `ekf_vyaw` | 相位/角速度 | `ekf_x()[6,7]` | rad, rad/s |
| `ekf_r` / `ekf_dr` / `ekf_h` | 半径/半径差/高差 | `ekf_x()[8,9,10]` | m |

> **EKF 状态向量定义**（核对自 `tasks/auto_aim/target.cpp:39`）：
> `x = [cx₀, vx₁, cy₂, vy₃, cz₄, vz₅, yaw₆, vyaw₇, r₈, Δr₉, Δh₁₀]`。
> **务必注意**：`standard.cpp:189-190` 被注释的示例写的是 `debug_data["kf_y"] = target.ekf_x()[2]` 且注释为 "EKF预测的X"，但 **index 2 实际是 `cy`（Y 方向中心），index 3 是 `vy`**。该处字段名是错的，接入时按上表索引，不要照抄旧注释命名。
> EKF 所在坐标系与求解器一致（水平中心 + 高度），具体轴语义应在实现时对照 `solver` 一次性确认并在字段表注明。

---

## 4. 分阶段实施

> 建议顺序：**Phase 0 → 1 → 2 → 3 → 4（先跑通 JSON + 图像）→ 5 → 6 → 7 → 8**。不要一次把 CMake、SDK 序列化、图像编码、坐标变换一起调。

### Phase 0：先做三个独立的 SDK 最小测试程序（必做）

**在主项目接入前**，先用官方 SDK 单独编译并运行三个小程序（放在项目外或 `tests/` 下，不链接本项目）：

| 程序 | 验证能力 | 通过标准 |
|---|---|---|
| `test_foxglove_json` | WebSocket 服务 + `RawChannel` 发 JSON | Studio 连接后 Raw Messages 能看到 JSON |
| `test_foxglove_image` | 发一张固定 JPEG（`CompressedImage`） | Image 面板显示该图 |
| `test_foxglove_scene` | 发一个固定立方体 + 一个 `FrameTransform` | 3D 面板显示立方体，父子坐标系正确 |

三者都通过，再把对应逻辑搬进 `FoxglovePublisher`。这样当图像不显示时，可先排除 SDK/schema/连接问题，再查 OpenCV 数据。

> **本机已验证（2026-10-13）**：三个最小程序均已在本机（Ubuntu 22.04 / GCC 12.3 / CMake 3.22.1 + sdk/v0.29.1）**编译并运行**，并用客户端成功连接、读到服务端 `advertise`：
> - `/test/image/compressed` → schema `foxglove.CompressedImage`（protobuf）
> - `/test/scene` → schema `foxglove.SceneUpdate`；`/tf` → schema `foxglove.FrameTransform`
> - `serverInfo.metadata["fg-library"] = "foxglove-sdk-cpp/0.29.1"`
>
> **协议细节**：Foxglove WebSocket 的**子协议名是 `foxglove.sdk.v1`**（不是 `foxglove.websocket.v1`；用错会握手 HTTP 400）。Foxglove Studio 会自动使用正确值；**自写客户端必须显式指定该子协议**。
> 最小 `CMakeLists.txt` 见 Phase 1；`find_package` 的 HINTS 指向 `third_party/foxglove` 即可。

### Phase 1：CMake 增加可选的 Foxglove 模块

根 `CMakeLists.txt` 选项区（`:21-25` 附近）新增：

```cmake
option(ENABLE_FOXGLOVE "Enable Foxglove visualization (Talos sim only)" OFF)
```

`standard` 目标定义之后（`:52-58` 之后）新增：

```cmake
if(ENABLE_FOXGLOVE)
  if(NOT ENABLE_TALOS_MODE)
    message(FATAL_ERROR "ENABLE_FOXGLOVE requires ENABLE_TALOS_MODE")
  endif()

  # HINTS 指向解压根目录（本仓库为 third_party/foxglove）；
  # 也可用 -Dfoxglove-sdk_DIR=<prefix>/lib/cmake/foxglove-sdk 指定。
  find_package(foxglove-sdk CONFIG REQUIRED
    HINTS "${CMAKE_SOURCE_DIR}/third_party/foxglove")
  foxglove_sdk_add_cpp_library(foxglove_cpp TYPE STATIC)

  # 只加到 standard，避免污染 tools(OBJECT) / io(STATIC) 的其他使用者。
  target_sources(standard PRIVATE tools/foxglove_publisher.cpp)
  target_compile_definitions(standard PRIVATE ENABLE_FOXGLOVE)
  target_link_libraries(standard PRIVATE foxglove_cpp)
endif()
```

```bash
cmake -S . -B build-talos -DENABLE_TALOS_MODE=ON -DENABLE_SIM_MODE=OFF -DENABLE_FOXGLOVE=ON
cmake --build build-talos --target standard -j"$(nproc)"
```

验收：`ENABLE_FOXGLOVE=OFF` 时，`standard` 的编译/链接命令里**不出现**任何 foxglove 路径或库。

> 实测（sdk/v0.29.1，`foxglove-v0.29.1-cpp-x86_64-unknown-linux-gnu.zip`）：
> - 该发行包**无需系统安装**，自带 `include/`、`src/`（C++ 包装源码）、`lib/libfoxglove.a` + `libfoxglove.so`、`lib/cmake/foxglove-sdk/`。
> - `foxglove_sdk_add_cpp_library(foxglove_cpp TYPE STATIC)` 会把 SDK 的 C++ 包装源码（`channel.cpp`/`websocket.cpp`/`messages.cpp` 等）**编进本工程**，故首次构建较慢；`STATIC` 仅支持 `REMOTE_ACCESS=OFF`（默认），本方案不需要远程访问。
> - 官方示例用 `cmake_minimum_required(VERSION 3.20)`；本机 CMake 3.22.1 满足，但项目根目前是 `3.16.3`，接入时建议提到 3.20。
> - 平台链接库（`pthread dl m`）由 SDK 的 CMake config 自动附加。

### Phase 2：扩展 `io::Talos`，返回自持快照（3D 与时间戳需要）

**用命名结构体消除"参数顺序 vs `PoseIndex` 顺序"的歧义**（`PoseIndex` 顺序是 Gimbal=0, Odom=1, Muzzle=2, Camera=3）。

`io/talos/talos.hpp` 增加：

```cpp
#include <vector>

namespace io {

struct Pose {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();      // ROS/FLU, m
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
};

// 字段名与 PoseIndex 一一对应，避免位置参数顺序写错。
struct Poses {
  Pose gimbal;   // PoseIndex::Gimbal = 0
  Pose odom;     // PoseIndex::Odom   = 1
  Pose muzzle;   // PoseIndex::Muzzle = 2
  Pose camera;   // PoseIndex::Camera = 3
};

struct GroundTruthTarget {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();      // ROS/FLU, m
  double yaw = 0.0;
  double vyaw = 0.0;
  int team = 0;                                            // 0=Red, 1=Blue
  int armor_label = 0;
};

class Talos {
public:
  // ... 原有接口不变 ...

  // 返回自持存储的快照（内部拷贝），绝不返回指向共享内存的指针/引用。
  bool read_poses(Poses & out) const;
  std::vector<GroundTruthTarget> read_ground_truth() const;

  // 新增：最近一帧图像的原始 Unix 纳秒时间戳（用于 Foxglove log_time，见第 5 节）。
  // 返回 0 表示尚无有效帧。
  std::uint64_t last_image_timestamp_ns() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace io
```

`io/talos/talos.cpp` 的 `Impl`：
- 增加缓存 `Poses poses_;` 与 `std::uint64_t last_image_timestamp_ns_ = 0;`；
- `consume_frame` 中把 `poses[0..3]` 分别填入 `poses_.gimbal/odom/muzzle/camera`（**保持原有消费顺序与返回值不变**），并记录 `image_meta.timestamp_ns`（`talos.cpp:259` 处已有该值）；
- `read_ground_truth()` 读取 `meta->ground_truth`（`talos.cpp:89` 已映射），把前 `target_count` 个 `targets[i]` 转成 `GroundTruthTarget`。

> **接口语义（必须遵守）**：
> - 两个访问器都返回**拥有自身存储**的值类型；不返回 `const&`/指针进 shm。
> - **只读 API 不等于底层读取线程安全**。`GroundTruthBatch` 是普通整块拷贝、无握手（见 1.1、第 6 节），`read_ground_truth()` 需采用第 6 节的快照策略。
> - 不得新增对任何姿态的 `consume`（否则破坏"四路消费后才发下一帧"的门控，`publisher.rs:123-134`）；真值不是握手的一部分，读它不影响门控。

### Phase 3：新增 `FoxglovePublisher` 模块

新增 `tools/foxglove_publisher.hpp` / `.cpp`（**只在 standard 目标编译**）。

**生命周期顺序（重要）**：先创建 `WebSocketServer`，再创建各通道；析构时**先释放通道、再释放 server**。用 `Impl` 成员声明顺序控制析构逆序即可（`server` 声明在最前 → 最后析构）。

`tools/foxglove_publisher.hpp`：

```cpp
#ifndef TOOLS__FOXGLOVE_PUBLISHER_HPP
#define TOOLS__FOXGLOVE_PUBLISHER_HPP

#ifdef ENABLE_FOXGLOVE

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace io { class Talos; struct Command; }

namespace tools
{

class FoxglovePublisher
{
public:
  FoxglovePublisher(std::string host = "127.0.0.1", uint16_t port = 8765);
  ~FoxglovePublisher();

  FoxglovePublisher(const FoxglovePublisher &) = delete;
  FoxglovePublisher & operator=(const FoxglovePublisher &) = delete;

  // 每帧调用一次。只读：img / command / talos 都不会被修改。
  // 返回 false 表示本帧发布失败（内部已记录错误）。
  bool publishFrame(
    const cv::Mat & img,
    const io::Command & command,
    double target_distance,
    double fps,
    const io::Talos & talos);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace tools

#endif  // ENABLE_FOXGLOVE
#endif  // TOOLS__FOXGLOVE_PUBLISHER_HPP
```

`tools/foxglove_publisher.cpp`（**接口骨架**，头文件/命名空间/返回类型以你的 SDK 版本为准）：

```cpp
#ifdef ENABLE_FOXGLOVE
#include "tools/foxglove_publisher.hpp"

#include <foxglove/foxglove.hpp>
#include <foxglove/websocket.hpp>  // v0.29.1：WebSocketServer/Options 在此；server.hpp 已废弃
#include <foxglove/channel.hpp>    // RawChannel
#include <foxglove/messages.hpp>   // 结构体通道：CompressedImage / SceneUpdate / FrameTransform
#include <cmath>

#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <stdexcept>
#include <vector>

#include "io/talos/talos.hpp"

namespace tools
{

// Unix 纳秒 → Foxglove Timestamp{sec, nsec}
inline foxglove::messages::Timestamp ts(std::uint64_t ns)
{
  return foxglove::messages::Timestamp{
    static_cast<uint32_t>(ns / 1000000000ull), static_cast<uint32_t>(ns % 1000000000ull)};
}

struct FoxglovePublisher::Impl
{
  // 声明顺序 = 构造顺序；析构逆序 => 通道先释放，server 最后释放。
  std::unique_ptr<foxglove::WebSocketServer> server;
  std::unique_ptr<foxglove::RawChannel> telemetry;                     // JSON 标量
  std::unique_ptr<foxglove::messages::CompressedImageChannel> image;   // 图像
  std::unique_ptr<foxglove::messages::SceneUpdateChannel> scene;       // 3D 场景
  std::unique_ptr<foxglove::messages::FrameTransformChannel> tf;       // 坐标变换

  int frame_counter = 0;
  int image_period = 3;   // 每 3 帧发一次图
};

FoxglovePublisher::FoxglovePublisher(std::string host, uint16_t port)
: impl_(std::make_unique<Impl>())
{
  foxglove::WebSocketServerOptions options;
  options.name = "sp25_learn";
  options.host = std::move(host);
  options.port = port;

  auto server = foxglove::WebSocketServer::create(std::move(options));
  if (!server.has_value()) {
    throw std::runtime_error(
      std::string("Foxglove server failed: ") + foxglove::strerror(server.error()));
  }
  impl_->server = std::make_unique<foxglove::WebSocketServer>(std::move(server.value()));

  // TODO(骨架): 依次创建并保存各通道（失败需抛异常或降级，不要忽略）：
  // impl_->telemetry = foxglove::RawChannel::create("/sp25/telemetry", "json").value();
  // impl_->image     = foxglove::messages::CompressedImageChannel::create("/sp25/image/compressed").value();
  // impl_->scene     = foxglove::messages::SceneUpdateChannel::create("/sp25/scene").value();
  // impl_->tf        = foxglove::messages::FrameTransformChannel::create("/tf").value();
}

FoxglovePublisher::~FoxglovePublisher() = default;

bool FoxglovePublisher::publishFrame(
  const cv::Mat & img, const io::Command & command, double target_distance, double fps,
  const io::Talos & talos)
{
  // 本帧源时间戳：优先用图像采集时间（Unix ns），拿不到时为 0（见第 5 节）。
  const std::uint64_t source_ns = talos.last_image_timestamp_ns();

  // ---------- 1) 控制曲线（JSON 标量） ----------
  // TODO(骨架): 用 nlohmann::json 组装；log() 返回 FoxgloveError（非 Result）。
  // auto err = impl_->telemetry->log(
  //   reinterpret_cast<const std::byte*>(json.data()), json.size(), source_ns /* log_time */);
  // if (err != foxglove::FoxgloveError::Ok) { std::cerr << foxglove::strerror(err); }

  // ---------- 2) 图像（限流 + JPEG 编码） ----------
  if (++impl_->frame_counter % impl_->image_period == 0 && !img.empty()) {
    std::vector<unsigned char> jpeg;
    if (cv::imencode(".jpg", img, jpeg, {cv::IMWRITE_JPEG_QUALITY, 80})) {
      // TODO(骨架): 填 CompressedImage 并 log（data 为 std::vector<std::byte>）：
      // foxglove::messages::CompressedImage msg;
      // msg.timestamp = ts(source_ns);          // 数据自身时间
      // msg.frame_id  = "camera";
      // msg.format    = "jpeg";
      // msg.data.assign(reinterpret_cast<const std::byte*>(jpeg.data()),
      //                 reinterpret_cast<const std::byte*>(jpeg.data()) + jpeg.size());
      // auto err = impl_->image->log(msg, source_ns);
      // if (err != foxglove::FoxgloveError::Ok) { std::cerr << foxglove::strerror(err); }
    }
  }

  // ---------- 3) 3D 场景 + TF ----------
  foxglove::messages::SceneUpdate scene;
  foxglove::messages::SceneEntity entity;
  entity.timestamp = ts(source_ns);
  entity.frame_id = "world";
  entity.id = "ground_truth";

  io::Poses poses;
  if (talos.read_poses(poses)) {
    // world → gimbal：位置取 odom.position，姿态取 gimbal.orientation（方向见第 7 节验证）
    foxglove::messages::FrameTransform tfg;
    tfg.timestamp = ts(source_ns);
    tfg.parent_frame_id = "world";
    tfg.child_frame_id = "gimbal";
    tfg.translation = foxglove::messages::Vector3{
      poses.odom.position.x(), poses.odom.position.y(), poses.odom.position.z()};
    const auto & q = poses.gimbal.orientation;   // Eigen(w,x,y,z) → Foxglove(x,y,z,w)
    tfg.rotation = foxglove::messages::Quaternion{q.x(), q.y(), q.z(), q.w()};
    // gimbal → camera / muzzle 同理（平移取 poses.camera/muzzle.position），略。
    // impl_->tf->log(tfg, source_ns);
  }

  std::vector<io::GroundTruthTarget> targets = talos.read_ground_truth();
  for (const auto & t : targets) {
    foxglove::messages::SpherePrimitive s;
    s.pose = foxglove::messages::Pose{};
    s.pose->position = foxglove::messages::Vector3{
      t.position.x(), t.position.y(), t.position.z()};
    s.size = foxglove::messages::Vector3{0.2, 0.2, 0.2};
    s.color = (t.team == 0) ? foxglove::messages::Color{1, 0, 0, 1}
                            : foxglove::messages::Color{0, 0, 1, 1};
    entity.spheres.push_back(s);

    foxglove::messages::ArrowPrimitive a;
    a.pose = foxglove::messages::Pose{};
    a.pose->position = s.pose->position;
    a.pose->orientation = foxglove::messages::Quaternion{
      0, 0, std::sin(t.yaw / 2), std::cos(t.yaw / 2)};   // 箭头默认 +x，绕 z 转 yaw
    a.shaft_length = 0.5; a.shaft_diameter = 0.02;
    a.head_length = 0.1;  a.head_diameter = 0.05;
    a.color = s.color;
    entity.arrows.push_back(a);
  }
  scene.entities.push_back(std::move(entity));
  // impl_->scene->log(scene, source_ns);

  return true;
}

}  // namespace tools
#endif  // ENABLE_FOXGLOVE
```

> **SDK 注意事项（已按 sdk/v0.29.1 头文件核实）**：
> - 头文件：`WebSocketServer`/`WebSocketServerOptions` 在 `<foxglove/websocket.hpp>`；`server.hpp` 已废弃（会打印 `#pragma message`）。`RawChannel` 在 `<foxglove/channel.hpp>`；结构体通道与消息在 `<foxglove/messages.hpp>`。
> - `WebSocketServer::create(WebSocketServerOptions&&)` 与各 `Channel::create(...)` 返回 `FoxgloveResult<T>`（即 `tl::expected<T, FoxgloveError>`），用 `.has_value()`/`.value()`/`.error()`，错误串用 `foxglove::strerror(err)`。
> - `Channel::log(...)` 返回 **`FoxgloveError`**（不是 Result），判断 `!= foxglove::FoxgloveError::Ok`；`log` 可传第三个参数 `log_time`（Unix ns），不传则用当前时间。
> - `WebSocketServerOptions` 默认 `host="127.0.0.1"`、`port=8765`，另有 `name` 字段。
> - 结构体：`Timestamp{sec,nsec}`、`Vector3{x,y,z}`、`Quaternion{x,y,z,w}`、`Color{r,g,b,a∈[0,1]}`、`Pose{optional position, optional orientation}`。
> - 错误请记录并按类型降级，不要只靠 `try/catch`。
> - 通道生命周期：**先建 server，再建通道；析构时通道先于 server**。

### Phase 4：`standard.cpp` 最小改动（约 3 处）

**改动点 1 —— 顶部 include**（`:21` 附近）：

```diff
 #include "tools/plotter.hpp"
 #include "tools/recorder.hpp"
+#ifdef ENABLE_FOXGLOVE
+#include "tools/foxglove_publisher.hpp"
+#endif
```

**改动点 2 —— 构造**（`:39-41`，用 try/catch 只降级可视化）：

```diff
   tools::Exiter exiter;
   tools::Plotter plotter;
   // tools::Recorder recorder;
+
+#ifdef ENABLE_FOXGLOVE
+  std::unique_ptr<tools::FoxglovePublisher> foxglove;
+  try {
+    foxglove = std::make_unique<tools::FoxglovePublisher>("127.0.0.1", 8765);
+  } catch (const std::exception & e) {
+    std::cerr << "[FOXGLOVE] disabled: " << e.what() << std::endl;
+  }
+#endif
```

**改动点 3 —— 每帧发布**（插在画完绿框后、`cv::resize` 前，即 `:129` 与 `:150` 之间）：

```diff
     for(const auto& armor : armors) {
         for(int i=0; i<4; i++)
             cv::line(img, armor.points[i], armor.points[(i+1)%4], {0,255,0}, 2);
     }
 
+#ifdef ENABLE_FOXGLOVE
+    if (foxglove && !foxglove->publishFrame(img, command, target_distance, fps, talos)) {
+      std::cerr << "[FOXGLOVE] publish failed" << std::endl;
+    }
+#endif
+
     cv::resize(img, img, {}, 0.5, 0.5);
```

要点：
- 放在 `cv::resize` 前 → 发的是**全分辨率 1440×1080**（带绿框）；若要连 FPS/Yaw/Shoot 文字一起发，可改到 `putText` 之后（但那是 0.5 缩放图）。
- `img` 在 `resize/imshow` 前数据有效；Publisher 内部**先 `imencode`（生成自持 `jpeg`）再 log**，无悬垂引用。
- `fps` 在此处是**上一帧**的值（本帧在 `:159` 才更新），对曲线观察无影响。
- 该行不读写任何控制状态；`talos` 只通过只读访问器读取。
- `#ifdef ENABLE_FOXGLOVE` 只在 `standard` 目标生效，`mt_standard` 不受影响。

### Phase 5：控制曲线（Plot）——两级验收

- **第一级**：Raw Messages 能看到合法 JSON（确认通道与发布成功）。
- **第二级**：Plot 能识别并选择预期数值字段（确认消息格式/schema 与当前 Foxglove 版本兼容）。

只有第一级通过、第二级失败时，才考虑改用受支持的 **JSON Schema 或 Protobuf** 消息定义，并使用 SDK 的结构化消息接口。**不要**在未验证前就断言问题一定是"缺 schema"——还要检查消息编码、字段路径与客户端解析行为。无 schema 的 `RawChannel` 与带 schema 的通道，其可视化能力不能混为一谈。

字段见第 3 节；**不要度数/弧度混用**：算法内部是 rad；发往仿真的 `GimbalCmd.yaw_deg` 才是 degree（`talos.cpp:277-283`）。

### Phase 6：图像（Image）

- 对 `img`（BGR）做 `cv::imencode(".jpg", ...)`，填 `foxglove.CompressedImage`（`format="jpeg"`, `frame_id="camera"`）。
- **限流**：每 N 帧发一次（10–15 FPS 起步）。
- **不要**只建一个 `RawChannel` 名为 `/camera/image` 塞 JPEG 字节 —— Image 面板不会识别。
- 生命周期：编码在 `publishFrame` 内完成，安全。

### Phase 7：3D（SceneUpdate + FrameTransform）与坐标验证

**先独立验证坐标，再画真实场景。**

**TF 树建议**：
```
world
 └── gimbal      (位置 = odom.position；姿态 = gimbal.orientation)
      ├── camera (平移 = camera.position；姿态需叠加 configs 的 R_camera2gimbal)
      └── muzzle (平移 = muzzle.position)
```

**场景元素**（`GroundTruthBatch`）：
- 每个真值目标：`frame_id="world"`，球体（按 `team` 红/蓝）+ 箭头表示 `yaw`（方向 = `(cos yaw, sin yaw, 0)`）。
- 能量机关：`r_center_odom` + `current_angle`/`direction`。
- 参考场景：`/sp25/reference_scene` 发米制网格 + 三轴。

**坐标验证清单（实现前逐项确认，不能仅凭轴名推断）**：
- [ ] `Odom.position` 是否确为云台世界位置。
- [ ] `Gimbal.quaternion` 是"云台→世界"还是反向；四元数分量顺序（w,x,y,z）、旋转方向、右手约定。
- [ ] `Camera`/`Muzzle.position` 属于哪个父坐标系（是否相对云台原点）。
- [ ] `R_camera2gimbal`（`configs/standard_sim.yaml:76`）的旋转方向、矩阵/四元数乘法顺序。
- [ ] 角度单位。

**必做单元测试**：设定一个已知云台位置、已知 yaw、已知相机相对平移，**手工计算**预期相机世界位置，再与 Foxglove 中显示的位置比较，排除旋转顺序与轴向错误。先用单位姿态与已知平移测试，再叠真实外参。

> **验证结果（2026-10-13，用 `tf_check.py` 解码 `/tf`）**：
> - `world→gimbal`：|q|=1.0，位置 z 朝上 → ✅ 正确，`world` 系确为 FLU。
> - `gimbal→camera`：**已改用配置外参** `R_camera2gimbal`/`t_camera2gimbal`（gimbal_link 约定）。实测平移 =(0.1374, 0.0020, 0.1526) m，与配置**误差 0.00 mm** ✅。
> - 说明：仿真器 `Camera`/`Muzzle` 的平移在**原始云台系**（`camera.reparented_to(gimbal_transform)`），而 `Gimbal` 四元数在 gimbal_link 系（多乘 `SHOT_DIRECTION*Rz90`），直接拼会自洽性错误（实测差 88.9mm，模长均 0.2053m）。故 `gimbal→camera` 采用算法配置外参；`muzzle` 无独立配置外参，已从 TF 树移除。

### Phase 8：仿真器侧接入（主推方案：零代码改动）

- Daedalus 已把图像、5 路姿态、相机内参、底盘观测、场景真值、运行状态全部写入 `/tmp/talos_ipc_*`（1.1）。算法进程的 `FoxglovePublisher` 读同一块内存即可，**仿真器不需要改动**。
- **若"不启动算法也要看仿真"**，需额外工作（二选一，均超出最小改动范围）：
  1. **原生 Rust 发布器**：在 `src/talos/plugin.rs` 的 `Last` 阶段加系统，用 Rust Foxglove SDK 发 TF/图像/真值。图像可在 `capture.rs:90-124` 的异步回调里复用同一张 RGB 数据。会新增 Rust 依赖，改动量比算法侧大。
  2. **复用 `src/telemetry/`**：该模块已定义 `TelemetryExporter` 抽象，但 `telemetry_dispatch` 目前发空数据、且未在 `main.rs` 注册。需先让 `FrameData` 填充真实 `armors/poses`。
- **需要改仿真器的例外**：把预测框/检测框回传到仿真窗口（参考项目的 UDP 9871 + `prediction_overlay.rs`），或接入发射/命中事件统计（`src/statistic.rs`）——**当前仿真器尚未实现**，属额外功能。

### Phase 9：MCAP 记录（可选）

- SDK 若支持 `McapWriter`，可在同一 `publishFrame` 路径实时 WebSocket + 落盘 MCAP 共用一份消息；否则让 Foxglove Studio 连接时 Record。
- MCAP 与实时分享同一 `log_time` 策略（第 5 节），便于离线复现。

---

## 5. 时间戳与数据同步

**核心原则：发布时间 ≠ 采集时间。** "在同一 `publishFrame()` 内发送"不代表这些消息属于同一采集时刻。

**统一时间戳策略**
1. 优先保留各数据源**自身的源时间戳**（图像 Unix ns、真值 Unix ns 等）。
2. 同一帧的算法结果，以**该帧采集时间**作为关联时间；若某消息表达的是"计算完成时刻"，单独记录计算时间。
3. **不要**用 `system_clock::now()` 冒充采集时间。
4. `steady_clock` 与 Unix 是两个时间基准。若要从算法时间恢复采集时间，必须通过 `io::Talos` **保留的原始 Unix 纳秒时间戳**，或用明确记录的时钟偏移换算。
5. Foxglove 消息的 `log_time`（记录时间）与消息体内的 `timestamp`（数据自身时间）语义不同，不能混用；**在 SDK 支持时显式设置 `log_time`**，否则退化为服务器接收时间。
6. 若某数据源确实拿不到源时间戳，明确标注为"发布时间/近似时间"，**不得**宣称它与图像严格同步。

**落地设计任务（已并入 Phase 2）**：
- 检查 `io::Talos::read_image()` 能否同时提供原始图像 Unix ns。当前它只返回 `steady_clock::time_point`（`talos.cpp:259-266`）。
- 建议新增**不破坏旧接口**的快照访问器 `last_image_timestamp_ns()`（`Impl` 在 `consume_frame` 里缓存 `image_meta.timestamp_ns`），而不是用 `system_clock::now()` 顶替。

**注意**：真值与图像是两条独立发布路径（`ground_truth` 在 `Last` 阶段、无握手；图像有握手），二者可能**不属于同一帧**。若需严格同帧，应以图像的 `frame_seq` 对齐，并接受真值可能来自相邻帧。

---

## 6. 共享内存一致性与快照

**只读 ≠ 安全。** `GroundTruthBatch` 不参与图像握手，不代表读取天然具有快照一致性：生产端是普通整块拷贝 `meta.ground_truth = *batch`（`publisher.rs:197-202`），**没有 seqlock / 版本号 / 双缓冲**，并发读取可能撕裂，导致 3D 出现瞬移或异常坐标。

**实现要求**
1. 访问器**必须返回自持存储的快照**（`std::vector` / 值类型），不返回指向 shm 的裸指针或引用。
2. **核对生产端写入方式**：目前 `GroundTruthBatch` / `ChassisObservation` / `RuntimeState` 都是无同步整块拷贝；`CameraInfo` 一次性写入；`ImageTripleBuffer`、`PoseTripleBuffer` 有三缓冲握手（安全）。
3. **优先方案（best-effort）**：读取真值前先后读 `frame_seq`，两次一致才采用。但生产端没有配对的自增/屏障，这只**降低**而非消除撕裂概率。
4. **若要保证一致性**：需在仿真器侧补充轻量级保护（真值的双缓冲或 seqlock，改 `publisher.rs` 对应函数），这属于**仿真器改动**，会打破"仿真器零改动"。取舍由你决定；否则采用第 3 条并在文档注明限制。
5. 所有改动必须**保持现有图像及姿态消费握手不变**（`publisher.rs:123-134`）。

---

## 7. 坐标系与姿态组合（与"坐标轴名称"分开对待）

本方案以 Talos 的 ROS/FLU 数据作为坐标输入，但**"名字叫 FLU"不等于"可以直接拼"**。发布前逐项确认（清单见 Phase 7），尤其是：`Odom` 位置、`Gimbal` 四元数方向、`Camera`/`Muzzle` 的父坐标系、`R_camera2gimbal` 的乘法顺序、四元数分量顺序与单位。只有约定一致后才能构造 `world → gimbal → camera/muzzle` 树，并用 Phase 7 的单元测试验证。

**不要**在算法侧再做一次 Bevy→ROS 转换（那已在仿真器内完成，`plugin.rs:180-204`），否则会出现 −z/−x 镜像式错位。

---

## 8. 性能与故障隔离（分开验收）

1. **失败隔离**：Foxglove 初始化失败不得改变原有 Talos IO 与控制逻辑；发布错误应被记录并按类型降级，**不能只依赖 `try/catch`**。
2. **异常隔离 ≠ 性能隔离**：能捕获异常不代表不阻塞。`cv::imencode()` 与消息序列化/网络发送都可能增加主循环耗时。
3. **必须单独测量**：图像编码耗时、消息发布耗时、算法帧耗时、实际 FPS。
4. **首阶段限流**：图像按 10–15 FPS 发布。
5. **仅在必要时异步化**：若发布耗时明显影响控制帧率，再将可视化改为**独立工作线程 + 有界队列**，传递自持快照；**队列满时丢弃旧可视化帧，绝不阻塞控制路径**。**不建议一开始就引入线程**（增加同步与生命周期复杂度），先测量再决定。
6. 标量与图像可用不同发布频率，但必须明确各自时间戳与数据对应关系。

---

## 9. 验收清单

- [ ] **构建隔离**：`ENABLE_FOXGLOVE=OFF` 时仿真/自瞄照常构建运行，且不链接 SDK。
- [ ] **Phase 0 三个最小测试**全部通过。
- [ ] **连接成功**：启动 `daedalus` → 启动 `standard` → Foxglove 连 `ws://127.0.0.1:8765`。
- [ ] **数值消息**：Raw Messages 能在 `/sp25/telemetry` 看到合法 JSON。
- [ ] **曲线显示**：Plot 能选到预期字段（第二级验收），时间轴单调递增。
- [ ] **图像显示**：Image 面板显示 1440×1080 画面，绿框位置正确。
- [ ] **3D 场景**：`/sp25/scene` 显示真值目标（红/蓝）与 yaw 箭头；`/tf` 父子关系正确；坐标单元测试通过。
- [ ] **时间戳**：同一帧各路消息使用同一源时间基准；`log_time` 与消息 `timestamp` 语义正确。
- [ ] **一致性**：真值读取采用自持快照；已评估撕裂风险并记录限制。
- [ ] **性能**：单独记录编码耗时、发布耗时、算法帧耗时、FPS（开关 Foxglove 前后对比）。
- [ ] **控制不变**：开/关 Foxglove、PC 端 Foxglove 断开，都不改变 `send_gimbal_command` 内容，也不阻塞主循环。

**本机环境示例**（路径需按机器替换，或改用环境变量 `SIM_DIR` / `ALGO_DIR`）：
```bash
# 终端 1：仿真器
source "$HOME/.cargo/env"
cd "$SIM_DIR" && ./target/release/daedalus

# 终端 2：算法（先 source OpenVINO）
source /opt/intel/openvino_2024.6.0/setupvars.sh    # 或设置 OpenVINO_DIR
"$ALGO_DIR/build-talos/standard" "$ALGO_DIR/configs/standard_sim.yaml"
# Foxglove Studio：Open connection → Foxglove WebSocket → ws://127.0.0.1:8765
```
本机实测路径示例：仿真器 `/home/ljy/桌面/bevy_robomaster_sim-master`，算法 `/home/ljy/桌面/sp25_learn`。

---

## 10. 可视化与控制解耦的回归测试

不只比 FPS。保存同一段仿真的控制输出并比较（可视化开 vs 关）：
- `command.yaw`、`command.pitch`；
- `control` 与 `shoot` 状态；
- 图像处理耗时与帧间隔；
- 可视化丢帧次数（若引入异步线程）。

不要求两次运行每个浮点数完全一致（仿真调度/输入时序可能有差异），重点是确认：**可视化没有额外写入控制状态，没有改变控制路径，没有引入不可接受的延迟**。

---

## 11. 风险与边界

1. **单消费者**：绝不能让第二个进程读 Talos 图像；图像发布必须在算法进程内。
2. **真值非握手**：`GroundTruthBatch` 无同步，读取需自持快照并评估撕裂；**绝不**用真值参与控制。
3. **姿态拆分与方向**：位置在 `Odom`、姿态在 `Gimbal`；相机姿态需叠 `R_camera2gimbal`；方向/顺序必须逐项验证。
4. **时间戳域**：见第 5 节，区分采集时间与发布时间、`log_time` 与消息 `timestamp`。
5. **SDK 版本差异**：所有 SDK 片段是骨架；先过 Phase 0。
6. **图像开销**：全帧率 JPEG 编码可能影响控制帧率，必须限流并实测。
7. **失败隔离**：任何 Foxglove 错误都记录并降级，不能让 `standard` 退出。
8. **改动范围表述**：`standard.cpp` 约 3 处；**整个项目**还包括新增发布模块、`io::Talos` 扩展、根 CMake 选项、SDK 依赖与测试程序。

---

## 12. 附录：文件与行号索引

**仿真器 `bevy_robomaster_sim-master/`**
- `crates/talos-ipc/src/layout.rs`：布局、`PoseIndex`(296-306)、`GroundTruthBatch`、`CameraInfo`、`ChassisObservation`、`RuntimeState`。
- `crates/talos-ipc/src/publisher.rs:97-134`：`try_publish_synchronized_image` 同步门控；`:183-209` 一次性内参 / 无同步真值等。
- `crates/talos-ipc/src/subscriber.rs`：`recv_gimbal_cmd` / `chassis_observation`。
- `src/talos/plugin.rs:49-110`：Talos 插件与发布系统；`:180-204`：`M_ALIGN_MAT3`（Bevy→ROS/FLU）。
- `src/talos/capture.rs:276-393`：`captured_pose_data` / `publish_pose_data`（5 路姿态语义）。
- `src/talos/ground_truth.rs:58-181`：场景真值发布。
- `src/main.rs:253-264`：Talos 插件启用；`src/telemetry/` 未注册。
- `Cargo.toml:18-22`：features（`default=["talos"]`）。

**算法 `sp25_learn/`**
- `src/standard.cpp`：主循环（行号见 1.2 表）。
- `io/talos/talos.hpp` / `talos.cpp`：接口、布局断言、`publish_command` 的 pitch 偏置、时间戳转换（220-230）、消费逻辑（232-269）。
- `io/command.hpp`：`io::Command`。
- `tools/plotter.hpp` / `plotter.cpp`：UDP 9870 曲线通道（通道 A）。
- `tasks/auto_aim/target.cpp:39`：EKF 状态向量定义（`ekf_x` 顺序）。
- `CMakeLists.txt:21-25,45,52-58`：选项与 `standard` 目标。
- `io/CMakeLists.txt` / `tools/CMakeLists.txt`：STATIC / OBJECT 库结构。
- `configs/standard_sim.yaml:70-79`：仿真内外参（`R_camera2gimbal`、`t_camera2gimbal`）。

**外部**
- Foxglove Studio 3.3.0：`/opt/Foxglove/foxglove-studio`。
- Foxglove C++ SDK：需另行安装并锁定版本（1.4）。
- 参考数据流（说明文档图 1）：Daedalus → Talos shm → 算法 → UDP 9870/9871 → Foxglove 桥 → WS 8765 → MCAP。本方案以"进程内直发"替代其中的 UDP 桥；`Plotter` 仍作为**独立**的可选曲线通道保留。

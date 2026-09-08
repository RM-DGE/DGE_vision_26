# sp_vision_25 哨兵适配与算法学习指南

> 目标：先在不改变核心算法的前提下，将项目安全、可观测地运行在自己的哨兵机器人上；再沿真实数据流深入理解代码，建立离线回放与量化评测体系；最后基于数据逐项完善算法。

## 1. 阅读范围与重要结论

本文结合以下资料整理：

- [项目原始 README](readme.md)
- [DeepWiki 项目总览](https://deepwiki.com/TongjiSuperPower/sp_vision_25/)
- [DeepWiki 哨兵平台说明](https://deepwiki.com/TongjiSuperPower/sp_vision_25/4.2-sentry-platform)
- [DeepWiki 自瞄系统说明](https://deepwiki.com/TongjiSuperPower/sp_vision_25/5.1-auto-aim-system)
- [DeepWiki 硬件接口说明](https://deepwiki.com/TongjiSuperPower/sp_vision_25/6-hardware-interfaces)
- [DeepWiki 配置系统说明](https://deepwiki.com/TongjiSuperPower/sp_vision_25/7-configuration-system)
- 本地源码，尤其是 `src/sentry.cpp`、`src/sentry_multithread.cpp`、`configs/sentry.yaml`、`io/` 和 `tasks/auto_aim/`

项目并不是一个需要从步兵版重新改造成哨兵版的框架。仓库已经提供：

- 哨兵主程序源码：`src/sentry.cpp`、`src/sentry_bp.cpp`、`src/sentry_debug.cpp`、`src/sentry_multithread.cpp`；这些目标只有在 ROS2 依赖全部找到时才会加入构建
- 主工业相机自瞄与多相机全向感知
- CAN 下位机通信与图像、IMU 时间对齐
- ROS2 导航信息交互
- 装甲板检测、位姿解算、EKF 整车估计、瞄准及开火决策

因此，第一目标应是**适配硬件边界和参数，而不是立即修改算法**。推荐顺序为：

1. 编译及离线回放；
2. 单独验证相机、IMU、CAN 和坐标方向；
3. 禁火跑通主相机闭环；
4. 接入全向感知与 ROS2；
5. 完成安全实弹基线；
6. 再开始算法实验。

## 2. 项目地图

### 2.1 四层架构

| 层次 | 目录 | 职责 | 学习重点 |
| --- | --- | --- | --- |
| 应用层 | `src/` | 按兵种组装功能并形成唯一主循环 | 数据如何流动、模式如何切换 |
| 功能层 | `tasks/auto_aim/`、`tasks/omniperception/` | 检测、解算、跟踪、瞄准、射击与全向感知 | 算法输入输出及状态机 |
| 硬件层 | `io/` | 相机、USB 相机、CAN、串口、ROS2 | 自己机器人最先需要适配的部分 |
| 工具层 | `tools/` | EKF、弹道、日志、绘图、录制、线程工具 | 调试、评测与复现手段 |

### 2.2 哨兵单帧主链路

以 `src/sentry.cpp` 为主线：

```text
工业相机图像与时间戳
  -> CBoard::imu_at() 获取对应时刻四元数
  -> Solver::set_R_gimbal2world()
  -> YOLO::detect() 检测装甲板
  -> Decider::armor_filter()/set_priority()
  -> Tracker::track() 维护目标和 EKF 状态
  -> 若目标正常：Aimer::aim()
     若目标丢失：Decider::decide() 调用侧后相机搜索
  -> Shooter::shoot() 最终决定是否开火
  -> CBoard::send() 向下位机发送控制帧
  -> ROS2::publish() 向导航发布目标信息
```

`io::Command` 是视觉侧最终输出，包含：

- `control`：是否由视觉控制云台；
- `shoot`：是否开火；
- `yaw`、`pitch`：目标角度，代码内部使用弧度；
- `horizon_distance`：无人机专用字段，哨兵通常不使用。

### 2.3 关键算法链路

| 阶段 | 主要代码 | 输出 |
| --- | --- | --- |
| 检测 | `tasks/auto_aim/yolo.*`、`detector.*`、`classifier.*` | 装甲板类别、颜色、置信度、四角点 |
| 几何解算 | `tasks/auto_aim/solver.*` | 相机、云台、世界坐标下的装甲板位姿 |
| 状态估计 | `tasks/auto_aim/tracker.*`、`target.*` | 目标中心、速度、旋转角速度、半径和高度差等状态 |
| 瞄准 | `tasks/auto_aim/aimer.*`、`tools/trajectory.*` | 考虑弹道和延迟后的 yaw、pitch |
| 轨迹规划 | `tasks/auto_aim/planner/` | 受云台加速度约束的平滑瞄准轨迹 |
| 开火决策 | `tasks/auto_aim/shooter.*` | 最终 `shoot` 标志 |
| 全向感知 | `tasks/omniperception/` | 丢失目标后的搜索、筛选和切换命令 |

## 3. 上机前必须建立的适配清单

不要直接复制 `configs/sentry.yaml` 使用。它记录的是原队哨兵的硬件、安装误差和调参结果。建议先复制为自己的配置，例如：

```bash
cp configs/sentry.yaml configs/my_sentry.yaml
```

随后主要修改 `configs/my_sentry.yaml`，保留原始文件作为对照；但注意 `src/sentry.cpp` 的后置工业相机仍固定读取 `configs/camera.yaml`，该相机参数需要单独维护，或先将代码改为可配置路径。

### 3.1 环境和依赖

原项目 README 的基准环境为 Ubuntu 22.04、C++17、OpenCV、Eigen、yaml-cpp、spdlog、fmt、OpenVINO 2024.6.0 和 Ceres；当前本地 `CMakeLists.txt` 实际显式查找的是 OpenCV、fmt、Eigen3、spdlog、yaml-cpp、nlohmann_json 和 OpenVINO。它把 OpenVINO 路径固定为：

```cmake
/opt/intel/openvino_2024.6.0/runtime/cmake/
```

如果本机安装位置或版本不同，应调整 CMake 配置，而不是创建同名伪目录。

本次在仓库当前工作环境（Windows + MinGW）执行 `cmake -S . -B build` 未完成配置：CMake 找到了 OpenCV 5.0.0，但在 `find_package(fmt)` 处同时看到 Anaconda 的 fmt 9.1.0 和 MSYS2/UCRT 的 fmt 12.2.0，均未被当前生成器接受。该项目 README 的目标环境是 Ubuntu 22.04，因此正式上机应优先使用匹配的 Linux 环境；若只是要在 Windows 上做离线开发，应统一编译器、架构和依赖来源，不要混用 Anaconda、MSYS2/UCRT 和其他工具链的 CMake package。

基础构建命令（Linux）：

```bash
cmake -B build
cmake --build build -j"$(nproc)"
```

注意：哨兵程序只有在 CMake 找到 `ament_cmake`、`rclcpp`、`std_msgs`、`rosidl_typesupport_cpp` 和自定义消息包 `sp_msgs` 时才会编译。配置阶段必须检查输出中是否出现：

```text
ROS2 environment not found, skipping ROS2-related code.
```

若出现该信息，普通程序仍可能构建成功，但 `build/sentry*` 不会生成。应先 source 正确的 ROS2 工作空间，再重新配置构建目录。当前检出还存在独立的构建完整性问题：顶层 `CMakeLists.txt` 注册了 `src/auto_aim_debug_mpc.cpp`、`src/uav.cpp` 和 `src/uav_debug.cpp`，但这些文件不在本地目录中；因此第一次配置可能在到达 ROS2 检查前就失败。先核对检出版本或暂时移除这些失效目标后再构建，不要把失败归因于算法代码。

### 3.2 CAN 与下位机协议

`io/cboard.cpp` 当前协议不是抽象描述，而是固定的 8 字节帧布局。适配前必须与电控逐字节确认。

接收帧：

| CAN ID | 字节 | 当前含义 | 缩放/编码 |
| --- | --- | --- | --- |
| `quaternion_canid` | 0..7 | `x,y,z,w` 四个有符号 16 位整数 | 除以 `1e4` |
| `bullet_speed_canid` | 0..1 | 弹速 | 有符号 16 位，除以 `1e2` |
| 同上 | 2 | `Mode` | `idle/auto_aim/small_buff/big_buff/outpost` |
| 同上 | 3 | `ShootMode` | 左枪/右枪/双枪 |
| 同上 | 4..5 | `ft_angle` | 有符号 16 位，除以 `1e4` |

发送帧 `send_canid`：

| 字节 | 当前含义 |
| --- | --- |
| 0 | `control` |
| 1 | `shoot` |
| 2..3 | `yaw * 1e4` |
| 4..5 | `pitch * 1e4` |
| 6..7 | `horizon_distance * 1e4` |

必须确认以下约定：

- 大小端和有符号整数编码一致；
- 四元数顺序与坐标定义一致；
- yaw、pitch 正方向一致，单位均为弧度；
- `control=false` 时下位机行为安全；
- `shoot` 必须经过实体急停、裁判系统和下位机安全条件再次门控；
- 弹速为 0、IMU 数据中断或视觉进程退出时不会继续发射。

先执行（目标程序支持的配置位置参数以 `tests/cboard_test.cpp` 为准）。不要省略最后的配置参数：该测试默认读取 `configs/standard.yaml`，而当前检出不一定提供此文件；显式传入自己的配置可避免因默认路径缺失而误判为 CAN 故障。

```bash
ip -details link show can0
candump can0
./build/cboard_test configs/my_sentry.yaml
```

验收：静止时四元数模长接近 1；手动按正方向转动云台时欧拉角方向正确且连续；弹速与下位机显示一致；模式及枪口选择无越界值。

### 3.3 主相机及侧后相机

配置中至少需要重新确认：

- `camera_name` 与实际相机 SDK；
- `vid_pid`；
- 工业相机的曝光和增益；
- USB 相机设备节点、分辨率、帧率、曝光、增益和 FOV；
- `src/sentry.cpp` 中写死的 `video0`、`video2`；
- `src/sentry.cpp` 的后置工业相机固定读取 `configs/camera.yaml`，不会随位置参数切换到 `configs/my_sentry.yaml`；若后相机也要使用自有参数，必须同步维护该文件或先把路径改为可配置项；
- `src/sentry_multithread.cpp` 中写死的 `video0/2/4/6`。

Linux 的 `/dev/videoN` 编号可能在重启或插拔后变化，正式部署应使用稳定的 udev 链接。单项测试建议按下列顺序：

```bash
./build/camera_test --help
./build/camera_test --config-path=configs/camera.yaml
./build/camera_detect_test configs/my_sentry.yaml
./build/usbcamera_test --name=video0 configs/my_sentry.yaml
./build/usbcamera_detect_test configs/my_sentry.yaml
./build/multi_usbcamera_test configs/my_sentry.yaml
```

具体参数先通过各程序的 `--help` 核对，不应凭文档猜测位置参数。

验收：连续运行至少 10 分钟无断流；时间戳单调；目标运动时无明显拖影；主相机稳定达到预期帧率；每个 USB 相机身份与安装方向一一对应。

### 3.4 相机内参与手眼标定

以下参数不可沿用原哨兵结果：

- `camera_matrix`
- `distort_coeffs`
- `R_camera2gimbal`
- `t_camera2gimbal`
- `R_gimbal2imubody`

相关程序：

- `calibration/capture.cpp`
- `calibration/calibrate_camera.cpp`
- `calibration/calibrate_handeye.cpp`
- `calibration/calibrate_robotworld_handeye.cpp`
- `tests/handeye_test.cpp`

推荐流程：

1. 固定镜头焦距、光圈、分辨率后再采集；
2. 采集覆盖画面中心、四角、远近和多姿态的数据；
3. 求内参并记录重投影误差；
4. 在云台多个 yaw/pitch 姿态采集手眼数据；
5. 将结果写入自己的 YAML；
6. 使用 `handeye_test` 和装甲板重投影做独立验证。

重投影误差低不等于手眼标定一定正确。最终验收应是：云台转动后，同一静止目标在世界坐标中的位置基本稳定；不同距离下瞄准偏差没有明显随距离增长的系统趋势。

### 3.5 全向感知安装角和 FOV

`tasks/omniperception/decider.cpp` 当前包含原车安装的固定角度：左侧约 `+62°`、右侧约 `-62°`、后方约 `170°`，后相机 FOV 也直接写成 `54.2° × 44.5°`。这些不是通用参数。

上机初期可暂时关闭全向感知，仅跑主相机。主链路稳定后，再将以下内容参数化并标定：

- 每个相机相对云台的 yaw/pitch 安装角；
- 实际水平、垂直 FOV；
- 图像裁剪后的有效 FOV；
- 相机设备身份；
- 重叠视场中的切换方向和误差。

验收：把目标依次放在每个相机视场中心和边缘，系统给出的旋转方向始终正确；转入主相机视场后，跟踪器能够稳定接管且不会来回振荡。

### 3.6 ROS2 与导航边界

哨兵入口通过 `io/ros2/` 发布目标信息并接收敌方状态。开始阶段把它看成独立外部接口：

1. 无导航时先验证自瞄本体；
2. 再验证消息类型、话题名、QoS、坐标系和单位；
3. 最后启用无敌状态过滤及导航目标指定。

特别注意：原项目 README 明确指出当前算法没有完整考虑自身移动。若哨兵会持续平移或旋转底盘，不能把相对相机观测直接解释为敌方世界运动。后续需要引入底盘里程计/定位和严格的时间同步，在统一世界坐标系中补偿自运动。

## 4. 分阶段实施路线

### 阶段 0：建立可复现环境

**任务**

- 记录 Ubuntu、编译器、OpenVINO、OpenCV、相机 SDK 和 ROS2 版本；
- 完成构建，保存 CMake 配置日志；
- 运行所有相关程序的 `--help`；
- 不连接机器人，跑通 `auto_aim_test` 离线样例。

```bash
./build/auto_aim_test --help
./build/auto_aim_test assets/demo/demo --config-path=configs/demo.yaml
```

**验收**

- 程序能读完视频和对应四元数文本；
- 可看到检测框、重投影、目标状态及命令；
- 日志中能分别看到 YOLO、Tracker 和 Aimer 耗时；
- 将一段异常帧号、现象及对应日志记录下来，证明已经会定位离线问题。

### 阶段 1：硬件单元测试

按“相机 -> CAN/IMU -> 标定 -> 云台命令”的顺序测试，每次只接通一个变量。此阶段禁止发弹。

**验收表**

| 项目 | 最低验收条件 |
| --- | --- |
| 主相机 | 连续 10 分钟无断流，曝光适合高速装甲板 |
| 多相机 | 设备身份稳定，方向/FOV 测量完成 |
| CAN | ID、端序、缩放和频率全部有记录 |
| IMU | 四元数有效，旋转方向正确，时间序列连续 |
| 标定 | 保存数据集、参数、重投影误差和实物复核结果 |
| 云台命令 | 小角度限速测试方向正确，断联自动退出视觉控制 |

### 阶段 2：禁火主相机闭环

把 `auto_fire` 设为 `false`，并在下位机硬件层及实体急停链路彻底禁止发射。两道禁火措施必须同时具备，不能用其中一道替代另一道。先运行最简单的单线程 `sentry`，不要一开始就使用多线程版本。

```bash
./build/sentry --help
./build/sentry configs/my_sentry.yaml
```

依次验证：

1. 静止装甲板、静止云台；
2. 静止装甲板、缓慢转动云台；
3. 目标匀速平移；
4. 低速小陀螺；
5. 短时遮挡和重新出现。

**验收**

- 目标类别和敌我颜色稳定；
- 重投影框与装甲板基本重合；
- EKF 状态没有明显发散或不合理跳变；
- yaw/pitch 指令方向正确，无持续振荡；
- 丢失目标后 `control` 能按约定释放；
- 每种场景均保存录像、四元数和绘图数据。

### 阶段 3：接入全向感知和 ROS2

先使用单线程轮询版本验证逻辑，再考虑 `sentry_multithread`。多线程版会提高重捕获速度，但同时增加算力、相机带宽、线程同步和 OpenVINO 多实例压力。

**验收**

- 主相机丢失目标后，侧后相机能给出正确转向；
- 目标进入主相机后，主跟踪链路稳定接管；
- 无敌状态和目标优先级符合当前赛季规则；
- ROS2 中断时视觉本体保持安全；
- CPU/GPU、内存、USB 带宽和各阶段 P95 延迟均有记录。

### 阶段 4：安全实弹基线

实弹必须从静止、近距离、低射频开始。先确认弹道和固定偏置，再调延迟，最后才调整开火容差。

推荐顺序：

1. 固定装甲板，校准 `yaw_offset`、`pitch_offset`；
2. 多距离固定目标，验证弹道解算和真实弹速；
3. 低速横移目标，调整低速延迟；
4. 低速小陀螺，检查装甲板切换；
5. 高速小陀螺，调整高速延迟与控制能力边界；
6. 测量从发射命令到子弹出膛的 `t_fire` 分布，而不只记录平均值。

**验收指标**

- 命中率：命中发数 / 总发数；
- 射击窗口占比：允许发射的时间 / 总跟踪时间；
- 击杀时间：固定血量、距离、转速和射频下的完成时间；
- 跟踪误差：`yaw_command - yaw_gimbal` 的均值、P95、最大值；
- 重捕获时间：目标丢失至稳定跟踪的时间；
- 端到端延迟：曝光时刻至控制命令送达下位机。

不要只追求命中率。极窄射击窗口可以得到高命中率但显著增加击杀时间；项目的核心思想是同时提高射击轨迹与云台轨迹的重合度。

### 阶段 5：部署与故障恢复

最后再配置开机启动、日志轮转和 watchdog。当前 `autostart.sh` 中路径固定为 `~/Desktop/sp_vision_25/`，并调用本地仓库中未见的 `watchdog.sh`，直接使用前必须补齐或改造。

部署验收至少包括：

- 冷启动、热重启均能拉起正确程序和配置；
- 相机拔插、CAN 中断、ROS2 中断和模型加载失败均不产生危险命令；
- 日志不会无限占满磁盘；
- 进程重启后默认禁火，直到输入状态全部有效；
- 断电前后的配置、模型和二进制版本可追溯。

## 5. 深入学习代码的推荐顺序

不要按目录字母顺序读。每读一个模块，都用“输入、状态、输出、频率、坐标系、失败行为”六个问题做笔记。

### 第 1 轮：先看一帧如何跑完

1. `tests/auto_aim_test.cpp`：最容易观察的离线主链路；
2. `src/sentry.cpp`：哨兵真实单线程主循环；
3. `io/command.hpp` 与 `io/cboard.cpp`：系统输入输出边界；
4. `configs/sentry.yaml`：参数如何进入各模块。

产出：手绘一张从图像到 CAN 帧的数据流图，并标注每个变量的单位和坐标系。

### 第 2 轮：检测和几何

1. `armor.*`：装甲板数据结构；
2. `yolo.*` 及 `yolos/`：不同模型的预处理、推理和后处理；
3. `detector.*`、`classifier.*`：传统检测与分类；
4. `solver.*`：PnP、坐标变换、yaw 优化和重投影。

产出：选择 100 至 500 帧，保存检测漏检、误检、类别错误、角点误差及 PnP 异常样例，避免仅凭实时画面判断模型好坏。

### 第 3 轮：跟踪和 EKF

1. `tracker.*`：跟踪状态机、目标创建、匹配与丢失；
2. `target.*`：11 维状态、状态转移、观测函数及装甲板切换；
3. `tools/extended_kalman_filter.hpp`：预测、更新与协方差传播；
4. `auto_aim_test.cpp` 中的残差、NIS、NEES 绘图字段。

产出：解释状态向量 `x,vx,y,vy,z,vz,a,w,r,l,h` 的物理意义；在静止、平移、低速及高速旋转数据上画出观测、估计和残差。

### 第 4 轮：弹道、瞄准和开火

1. `tools/trajectory.*`：弹道飞行时间与 pitch 解算；
2. `aimer.*`：候选装甲板、预测时间和瞄准点；
3. `shooter.*`：距离分段容差与发射门控；
4. `planner/`：射击轨迹、云台约束和提前减速。

需要真正掌握三个轨迹：

```text
目标轨迹 yaw_target(t)
射击轨迹 yaw_shoot(t) = yaw_target(t + t_fly)
实际云台轨迹 yaw_gimbal(t)
```

学习重点不是会调 Q/R，而是理解为什么装甲板切换造成轨迹不连续、为什么云台最大加速度限制决定可跟踪性，以及开火延迟为什么必须用未来时刻误差判断。

### 第 5 轮：并发和全向感知

1. `io` 中各相机和 CAN 的采集线程、队列与重连；
2. `src/sentry_multithread.cpp`；
3. `tasks/omniperception/perceptron.*`；
4. `tasks/auto_aim/multithread/`；
5. `io/ros2/`。

产出：列出所有线程的生产者、消费者、共享数据、队列上限、阻塞点和退出条件；测量的应是端到端延迟分布，而不只是平均 FPS。

## 6. 算法优化路线

优化应遵循“建立基线 -> 一次只改一个因素 -> 离线复现 -> 禁火上车 -> 实弹 A/B”的闭环。

### 6.1 第一优先级：可观测性和数据集

在修改核心算法前，先确保每一帧可关联以下信息：

- 图像和曝光/接收时间戳；
- 对齐后的 IMU 四元数；
- 检测框、四点、类别和置信度；
- PnP 位姿与重投影误差；
- EKF 状态、协方差、残差、NIS/NEES；
- 瞄准点、弹道飞行时间、预测时间；
- 云台命令、实际角度、是否开火；
- 下位机模式、弹速和枪口；
- 导航位姿或里程计（移动哨兵）。

建议建立固定回放集：近/中/远距离、逆光/暗光、静止/平移/小陀螺、遮挡、快速出入画面及车体移动。所有优化都在同一回放集上比较。

### 6.2 检测优化

指标：Precision、Recall、分类准确率、关键点像素误差、不同距离分桶结果、推理 P50/P95。

实验方向：

- 针对本队相机、曝光和赛场采集数据微调模型；
- 比较 YOLO v5/v8/v11，不只比较 FPS；
- 检查置信度阈值对漏检、误检和跟踪稳定性的联合影响；
- 先修复数据和角点质量，再调整 EKF 噪声吸收检测缺陷。

### 6.3 位姿和坐标优化

指标：重投影误差、静止目标世界坐标抖动、不同距离的系统偏差、云台转动前后一致性。

实验方向：

- 标定数据覆盖与异常图像剔除；
- 大小装甲板几何尺寸核对；
- solvePnP 方法和 yaw 优化的消融实验；
- 主相机时间戳与 IMU 插值偏移扫描。

### 6.4 EKF 与目标管理优化

指标：位置/角速度 RMSE、NIS/NEES 一致性、错误切板率、丢失率、重捕获时间。

实验方向：

- 从真实残差估计测量噪声，而非凭感觉调矩阵；
- 按距离、装甲板倾角或置信度建立异方差观测噪声；
- 分析加减速、急停和碰撞等匀速模型失配；
- 将跟踪状态机与估计器数值问题分开评测。

### 6.5 轨迹规划和开火优化

指标：轨迹重合度、射击窗口占比、yaw 误差 P95、切板超调、击杀时间和命中率。

实验方向：

- 先系统辨识哨兵云台最大加速度和延迟，再设置规划约束；
- 比较无规划、TinyMPC 隐式规划和五次多项式显式过渡；
- 测量 `t_fire` 的均值与方差，评估提前开火是否可靠；
- 按弹速和距离在线更新飞行时间；
- 对双枪分别建模固定偏置、弹速和发射延迟，不能假设完全相同。

原项目说明指出轨迹规划器当时主要在步兵上实战验证，哨兵具有更大惯量，因此不能直接照搬步兵的最大加速度、Q/R 或开火阈值。

### 6.6 移动哨兵的自运动补偿

这是高价值但后置的系统级优化。基础版本假设视觉观测变化主要来自敌方运动；当自身底盘运动时，该假设会污染目标速度估计和预测。

建议数据流：

```text
导航/里程计位姿 + 时间戳
  -> 插值到图像曝光时刻
  -> 相机/云台观测变换至统一世界坐标
  -> EKF 估计敌方世界状态
  -> 预测命中时刻双方相对位姿
  -> 弹道、瞄准和开火决策
```

必须先解决：时钟来源、坐标系树、外参、底盘振动、定位跳变及失效降级。否则添加里程计可能比不添加更差。

## 7. 建议的实验记录模板

每次实验复制以下模板：

```markdown
### 实验名称
- 日期/代码版本：
- 机器人/配置文件/模型：
- 唯一改动：
- 场景：距离、光照、目标速度、底盘状态、弹速、射频
- 数据集或录像编号：
- 假设：
- 指标：基线值 -> 实验值
- 异常样例：
- 结论：接受 / 拒绝 / 数据不足
- 下一步：
```

任何参数调整都应能回答三个问题：它修复了哪个可复现问题？改善了哪个指标？是否破坏了其他距离、速度或光照场景？

## 8. 已发现的仓库注意事项

1. 顶层 `CMakeLists.txt` 注册了 `src/auto_aim_debug_mpc.cpp`、`src/uav.cpp` 和 `src/uav_debug.cpp`，但当前本地目录没有这些文件；首次 CMake 配置可能因此失败。
2. 当前本地目录没有 `watchdog.sh`，但 `autostart.sh` 会调用它；部署前必须补齐脚本或改成实际的进程管理方式。
3. `src/sentry*.cpp` 中 USB 摄像头节点和 `tasks/omniperception/decider.cpp` 中安装角/FOV 存在硬编码。
4. `configs/sentry.yaml` 的 `auto_fire` 默认值是 `true`，第一次上机必须显式改为 `false`，并使用下位机/实体急停做第二道禁火保护。
5. `io/cboard.cpp` 将 CAN 原始模式字节直接转换为 `Mode`、`ShootMode`，这些值后续会参与分支或索引；真实部署前应增加合法值校验，并让非法帧进入禁火、释放控制的安全状态。
6. `tests/cboard_test.cpp` 默认读取 `configs/standard.yaml`，当前检出不保证存在该文件；运行时应显式传入自有配置。
7. 原始 README 列出的 `tasks/auto_buff/` 和 `tests/auto_buff_test.cpp` 当前检出不存在；不要根据 README 假定打符功能已经可构建。
8. DeepWiki 是基于特定提交生成的辅助说明，可能包含推断或与本地检出不一致的内容；实际行为以本地源码、构建结果和实测为准。

## 9. 推荐的近期里程碑

### 里程碑 A：离线可理解

- 能从 `auto_aim_test.cpp` 解释完整单帧链路；
- 能在录制数据中定位一次检测、PnP、跟踪或瞄准异常；
- 有一张标注单位、坐标系和时间戳的数据流图。

### 里程碑 B：哨兵禁火运行

- 自有 YAML、相机内参和手眼标定完成；
- CAN 与下位机协议逐字节验证；
- 主相机闭环稳定，异常时可靠释放控制；
- 每次运行能保存足够数据供离线复现。

### 里程碑 C：全向感知与实弹基线

- 侧后相机安装角/FOV 完成标定；
- 目标切换和 ROS2 交互稳定；
- 固定标准场景下获得命中率、击杀时间、跟踪误差和延迟基线。

### 里程碑 D：第一个算法改进

- 只选择一个瓶颈，例如关键点误差、时间同步、EKF 失配或轨迹切换；
- 使用固定数据集做消融和 A/B；
- 离线指标、禁火上车和实弹指标方向一致；
- 保留能够一键回退的基线配置。

完成里程碑 C 之前，最有价值的工作通常不是换模型或重写 EKF，而是把硬件协议、坐标系、时间同步、标定和数据记录做正确。它们既决定项目能否安全运行，也决定后续算法优化是否可信。

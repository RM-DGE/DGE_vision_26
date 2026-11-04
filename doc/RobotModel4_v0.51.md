# RobotModel4_v0.51 使用说明

## 1、本次改了什么

- 将固定减 1ms 改为配置 `imu_delay_ms`，支持正数、负数和小数毫秒。
- IMU 缓存支持重复查询，检查缺样和插值间隔；对齐失败时跳过视觉更新，关闭控制与发射使能。
- 增加图像和 IMU 独立自动采集，以及棋盘格延时搜索、独立数据复验。
- 修复串口断线重连处理；仿真按相同帧号匹配图像与姿态。
- 测试合并为 3 份，构建统一在根 `CMakeLists.txt`。

真机当前仍用 **1ms 初值**，仿真用 **0ms**。真实延时需要下面的上车采集才能确定。软件三组测试已通过。

## 2、新增了哪些文件

| 文件 | 用途 |
|---|---|
| `calibration/capture_time_sync.cpp` | 自动采集图像与 IMU |
| `calibration/calibrate_time_offset.cpp` | 离线计算和复验延时 |
| `tools/imu_time_buffer.hpp/.cpp` | IMU 时间缓存与插值 |
| `tools/time_sync_config.hpp/.cpp` | 延时配置读取 |
| `tools/time_sync_recorder.hpp/.cpp` | 采集数据写盘 |
| `tools/time_sync_session.hpp/.cpp` | 配置快照指纹 |
| `tools/time_offset_calibration.hpp/.cpp` | 延时搜索与评分 |
| `io/serial_port.hpp` | Linux 串口收发与安全关闭 |
| `tests/time_sync_core_test.cpp` | 核心算法测试 |
| `tests/time_sync_capture_test.cpp` | 采集、仿真与串口测试 |
| `tests/time_sync_offline_test.py` | 离线完整流程测试 |

同时修改了根 `CMakeLists.txt`、相机/CBoard 接口、`standard`、`sentry_DGE`、`handeye_test` 和对应配置。原有内参、手眼标定程序继续使用。

## 3、标定和延时测试怎么运行

以下命令均在 **Linux 工程根目录**执行。示例使用 **9×6 内角点、25mm 单格边长**，必须按实物修改。已有可靠内外参时，可直接从第④步开始。

### ① 编译

使用工程原有的海康 SDK、OpenVINO 等依赖：

```bash
cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build -j4
ctest --test-dir build --output-on-failure
mkdir -p data/calibration data/time_sync
```

Python 测试需要 `cv2`、`numpy`、`yaml`。各可执行文件都在 `build` 下。

### ② 标定相机内参

修改 `configs/calibration.yaml`：填实际相机、串口和棋盘格参数；此步设置：

```yaml
pattern_cols: 9
pattern_rows: 6
square_size_mm: 25.0
capture_imu: false
```

```bash
./build/capture configs/calibration.yaml -o=data/calibration/intrinsic_01
```

移动棋盘格，覆盖画面各处及不同距离、倾角。按 **s** 保存清晰图片，建议 20～30 张；按 **q** 结束。

```bash
./build/calibrate_camera data/calibration/intrinsic_01 -c=configs/calibration.yaml
```

将终端输出的 **`camera_matrix`、`distort_coeffs`** 手动写回 `configs/calibration.yaml`。程序不会自动回填。

### ③ 标定手眼外参

在同一配置中设 `capture_imu: true`，按机械安装填写 **`R_gimbal2imubody`**。固定底盘和棋盘格，改变云台 Yaw/Pitch；每次停稳后按 s，保存约 20～30 组不同姿态。

```bash
./build/capture configs/calibration.yaml -o=data/calibration/handeye_01
./build/calibrate_robotworld_handeye data/calibration/handeye_01 -c=configs/calibration.yaml
```

将输出的 **`R_camera2gimbal`、`t_camera2gimbal`** 手动写回配置，平移单位为米。然后检查：

```bash
./build/calibrate_validate data/calibration/handeye_01 -c=configs/calibration.yaml
```

查看输出的重投影误差、世界坐标波动和 PASS/FAIL。另采一组不同姿态的数据复验更可靠。

把确认后的 **`camera_matrix`、`distort_coeffs`、`R_gimbal2imubody`、`R_camera2gimbal`、`t_camera2gimbal`** 同步到 `configs/handeye.yaml` 和实际运行配置（如 `configs/standard3.yaml`）。相机分辨率、曝光、翻转等设置应与运行时一致。

### ④ 自动采集延时数据

固定底盘和棋盘格，保持相机与串口正常工作。操作者移动云台，加入 **Yaw/Pitch 加减速、反向和组合运动**，让棋盘格持续清晰可见。程序只记录数据，云台由操作者控制。

```bash
./build/capture_time_sync --config configs/handeye.yaml --output data/time_sync/train_01 --duration 45 --image-fps 30 --cols 9 --rows 6 --square-m 0.025
./build/capture_time_sync --config configs/handeye.yaml --output data/time_sync/validate_01 --duration 45 --image-fps 30 --cols 9 --rows 6 --square-m 0.025
```

两次使用不同运动节奏，配置保持一致。**此处格长单位是米**。目录必须不存在；重录时换成 `train_02`、`validate_02` 等新名字。

结束后确认终端显示 **`complete=true`**。若为 false，查看该目录的 `events.csv`，处理断线、丢帧或写盘问题后重录。

### ⑤ 计算并复验延时

```bash
./build/calibrate_time_offset --session data/time_sync/train_01 --validation data/time_sync/validate_01 --output data/time_sync/result_01
```

打开 `data/time_sync/result_01/summary.yaml`，检查：

- `status: candidate`：得到候选值。
- `candidate_imu_delay_ms`：候选延时，单位 ms。
- `offline_validation_passed: true`：独立数据复验通过。

`inconclusive` 表示本次数据无法确定可靠延时；`invalid_data` 表示数据或参数不合格，按 `reason` 处理。`scan.csv` 可查看延时评分曲线。软件不会自动覆盖运行配置。

### ⑥ 回填延时并实测

离线复验通过后，将候选值填入 **`configs/handeye.yaml` 和实际运行 YAML**：

```yaml
imu_delay_ms: 1.0   # 将 1.0 替换为 summary.yaml 中的候选值
imu_max_gap_ms: 20.0
imu_wait_ms: 12.0
```

正延时表示查询更早的 IMU 姿态：`t_query = t_image - imu_delay_ms`。

```bash
./build/handeye_test -c=configs/handeye.yaml -d
./build/standard configs/standard3.yaml
```

手眼投影前，按实际场地设置 `handeye.yaml` 的 `height`、`grid_num`、`grid_size`，对照真实网格观察转动云台时的漂移。运行自瞄时检查日志中 `ok` 比例、失败次数和 `wait_max_ms`，比较修改前后的画面稳定性。离线复验通过后仍需这一步在线验收。

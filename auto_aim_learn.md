# RoboMaster 自瞄系统完全解析指南

## 目录

1. [自瞄系统总体架构](#1-自瞄系统总体架构)
2. [核心数据结构](#2-核心数据结构)
3. [模块详解](#3-模块详解)
4. [完整数据流](#4-完整数据流)
5. [依赖关系图](#5-依赖关系图)
6. [关键算法讲解](#6-关键算法讲解)
7. [代码执行流程](#7-代码执行流程)

---

## 1. 自瞄系统总体架构

### 1.1 系统分层

```
┌─────────────────────────────────────────────────┐
│               主程序入口 (main)                  │
│           例如: standard.cpp, sentry.cpp        │
└─────────────────────────────────────────────────┘
                       ↓
┌─────────────────────────────────────────────────┐
│         应用逻辑层 (Application Layer)          │
│    负责整体流程控制和模块间通信                 │
└─────────────────────────────────────────────────┘
       ↓              ↓              ↓
┌──────────────┐ ┌──────────────┐ ┌──────────────┐
│   检测层     │ │   追踪层     │ │   决策层     │
│ (Detector)   │ │ (Tracker)    │ │ (Aimer)      │
│    YOLO      │ │ EKF滤波器    │ │ 弹道解算     │
│ 装甲板识别   │ │ 目标预测     │ │ 瞄准计算     │
└──────────────┘ └──────────────┘ └──────────────┘
       ↓              ↓              ↓
┌──────────────┐ ┌──────────────┐ ┌──────────────┐
│   解算层     │ │  射击层      │ │  驱动层      │
│ (Solver)     │ │ (Shooter)    │ │ (Gimbal/    │
│ 坐标变换     │ │ 开火判断     │ │  Camera)    │
│ PnP求解     │ │             │ │ 硬件控制     │
└──────────────┘ └──────────────┘ └──────────────┘
```

### 1.2 核心模块清单

自瞄系统的核心模块存放在 `tasks/auto_aim/` 下：

| 模块名 | 文件 | 功能 | 输入 | 输出 |
|--------|------|------|------|------|
| **Detector** | detector.cpp/hpp | 从图像检测装甲板 | BGR图像 | 装甲板列表 |
| **Solver** | solver.cpp/hpp | 3D位姿求解与坐标变换 | 图像坐标+IMU姿态 | 世界坐标位置 |
| **Tracker** | tracker.cpp/hpp | 目标跟踪与状态估计 | 装甲板列表 | 目标列表(含预测) |
| **Target** | target.cpp/hpp | 单个目标的EKF滤波 | 装甲板观测 | 预测位置和速度 |
| **Aimer** | aimer.cpp/hpp | 瞄准位置计算 | 目标信息+弹速 | 瞄准命令 |
| **Shooter** | shooter.cpp/hpp | 开火决策 | 瞄准信息 | 是否发射信号 |
| **Classifier** | classifier.cpp/hpp | 装甲板分类(颜色/编号) | 装甲图案 | 装甲类型 |
| **Planner** | planner/planner.cpp/hpp | 轨迹规划(MPC) | 目标轨迹 | 优化轨迹 |

---

## 2. 核心数据结构

### 2.1 Armor（装甲板）

装甲板是自瞄的基本单位，代表机器人上的一块装甲。

```cpp
struct Armor {
  // ========== 检测阶段获得的信息 ==========
  Color color;                    // 装甲颜色：red/blue/extinguish/purple
  Lightbar left, right;           // 左右灯条（光条）
  std::vector<cv::Point2f> points; // 4个角点的图像坐标（像素坐标）
  cv::Rect box;                   // 检测框
  cv::Mat pattern;                // 装甲图案ROI
  double confidence;              // 神经网络置信度
  
  // ========== 分类阶段确定的属性 ==========
  ArmorType type;                 // 装甲类型：big/small
  ArmorName name;                 // 装甲编号：one/two/three/four/five/sentry/outpost
  ArmorPriority priority;         // 优先级
  int class_id;                   // 分类ID
  
  // ========== 几何特征 ==========
  double ratio;                   // 中点连线与长灯条的比例
  double side_ratio;              // 长短灯条的长度比
  double rectangular_error;       // 矩形拟合误差
  
  // ========== 解算阶段获得的3D信息（重要）==========
  Eigen::Vector3d xyz_in_gimbal;  // 相对于云台的3D坐标（单位：米）
  Eigen::Vector3d xyz_in_world;   // 相对于世界坐标系的3D坐标
  Eigen::Vector3d ypr_in_gimbal;  // 相对于云台的欧拉角(Yaw, Pitch, Roll)
  Eigen::Vector3d ypr_in_world;   // 相对于世界的欧拉角
  Eigen::Vector3d ypd_in_world;   // 球坐标系(Yaw, Pitch, Distance)
  double yaw_raw;                 // PnP求解的原始Yaw角
};
```

**说人话**: Armor 是一个"信息容器"，从图像中的像素坐标，经过一系列计算，逐步变成 3D 世界坐标。

### 2.2 Lightbar（灯条）

灯条是装甲板的组成部分。RoboMaster 机器人的装甲板由两根竖灯条组成。

```cpp
struct Lightbar {
  std::size_t id;                 // 灯条ID
  Color color;                    // 灯条颜色
  cv::Point2f center, top, bottom; // 灯条的中心、顶点、底点
  std::vector<cv::Point2f> points; // 轮廓上的采样点
  double angle;                   // 灯条的倾角（与水平线的夹角）
  double length, width;           // 灯条的长宽
  double ratio;                   // 长宽比（用来筛选异常灯条）
  cv::RotatedRect rotated_rect;   // 旋转矩形
};
```

### 2.3 Target（目标）

Target 代表一个敌方单位（如步兵、英雄等）。它包含EKF滤波器，用于预测目标的运动状态。

```cpp
class Target {
public:
  // ========== 装甲信息 ==========
  ArmorName name;                 // 当前跟踪的装甲编号
  ArmorType armor_type;           // 装甲类型
  ArmorPriority priority;         // 优先级
  bool jumped;                    // 是否发生装甲切换
  
  // ========== EKF状态向量（11维） ==========
  // x[0:2] = [center_x, center_y, center_z] - 旋转中心位置
  // x[1:4] = [vx, vy, vz] - 速度
  // x[6:7] = [yaw_angle, yaw_angular_velocity] - 偏航角和角速度
  // x[8] = radius - 装甲板到旋转中心的距离
  // x[9:10] = [dRadius, dHeight] - 装甲板间的相对差异
  
private:
  tools::ExtendedKalmanFilter ekf_; // 扩展卡尔曼滤波器
  int armor_num_;                   // 装甲板数量（4或8）
  std::chrono::steady_clock::time_point t_; // 上次更新时间
};
```

**说人话**: 每个敌方目标用一个 Target 来表示。Target 内部维护一个滤波器，根据历次观测到的装甲板位置，逐步学习目标的运动规律（旋转中心、速度、角速度等）。

### 2.4 Color, ArmorType, ArmorName（枚举）

```cpp
enum Color { red, blue, extinguish, purple };      // 装甲颜色
enum ArmorType { big, small };                     // 大小装甲
enum ArmorName { one, two, three, four, five, sentry, outpost, base, not_armor };
```

---

## 3. 模块详解

### 3.1 Detector（检测器）

**职责**: 从原始 BGR 图像中检测出所有装甲板

**输入**: 
- `cv::Mat bgr_img` - OpenCV 的 BGR 彩色图像

**输出**: 
- `std::list<Armor> armors` - 检测到的装甲板列表

**工作流程**:

```
原始图像
   ↓
【第1步】神经网络检测（YOLO v8）
   ├─ 输入: RGB 图像
   ├─ 输出: 装甲框 + 4个角点（keypoints）
   ↓
【第2步】构造 Armor 对象
   ├─ 从 YOLO 的 4 个角点和类别 ID 构造
   ├─ 记录: class_id, confidence, points, box
   ↓
【第3步】几何有效性检查
   ├─ 检查灯条长宽比是否合理
   ├─ 检查装甲板长宽比是否合理
   ├─ 检查两灯条夹角是否接近 90°
   ↓
【第4步】装甲分类（Classifier）
   ├─ 输入: 装甲图案(pattern)
   ├─ 确定: 颜色(color) + 编号(name) + 类型(big/small)
   ↓
【第5步】去重和排序
   ├─ 移除重复检测
   ├─ 按优先级排序
   ↓
装甲板列表输出
```

**关键代码**:

```cpp
std::list<Armor> Detector::detect(const cv::Mat & bgr_img, int frame_count) {
  // 1. YOLO 检测
  auto armors = yolo_.detect(bgr_img);  
  
  // 2. 几何检查
  armors.remove_if([this](const Armor& a) { 
    return !check_geometry(a) || !check_type(a); 
  });
  
  // 3. 分类器
  for (auto& armor : armors) {
    if (!check_name(armor)) {
      classifier_.classify(armor);  // 分类
    }
  }
  
  return armors;
}
```

---

### 3.2 Solver（解算器）

**职责**: 将 2D 图像坐标转换为 3D 世界坐标

**关键概念**:

```
世界坐标系（World Frame）
    ↑ Z（向上）
    │
    │     X（前进方向）
    │    /
    │   /
    └─────→ Y（右方）

云台坐标系（Gimbal Frame）
    与世界坐标系的旋转关系由 IMU 四元数提供

相机坐标系（Camera Frame）
    通过手眼标定的外参 R_camera2gimbal, t_camera2gimbal 与云台坐标系关联
```

**输入**:
- 装甲板的 4 个角点（像素坐标）
- 当前 IMU 四元数（定义世界坐标系方向）

**输出**:
- 装甲板在世界坐标系中的 3D 位置 `xyz_in_world`
- 装甲板的姿态（欧拉角）`ypr_in_world`
- 球坐标表示 `ypd_in_world` (Yaw, Pitch, Distance)

**工作流程**:

```
【第1步】PnP 求解（Perspective-n-Point）
   ├─ 输入: 装甲板 4 个 3D 模板点（相机标定得到）
   ├─ 输入: 图像上对应的 4 个 2D 像素点
   ├─ 算法: OpenCV solvePnP (IPPE方法)
   └─ 输出: rvec(旋转向量), tvec(平移向量) 相对于【相机坐标系】
   
【第2步】坐标变换 - 相机 → 云台
   ├─ 应用: Armor_in_gimbal = R_camera2gimbal * Armor_in_camera + t_camera2gimbal
   └─ 结果: xyz_in_gimbal, ypr_in_gimbal
   
【第3步】坐标变换 - 云台 → 世界
   ├─ 应用: Armor_in_world = R_gimbal2world * Armor_in_gimbal
   └─ 结果: xyz_in_world, ypr_in_world
   
【第4步】Yaw 角优化（只在非平衡步兵上执行）
   ├─ 问题: PnP 求解的 Yaw 角常有误差
   ├─ 方法: 在 [-70°, +70°] 范围内扫描，选择重投影误差最小的 Yaw
   ├─ 步长: 1°
   └─ 结果: 更精准的 yaw_in_world
   
【第5步】转球坐标
   ├─ 应用: (yaw, pitch, distance) = xyz2ypd(xyz_in_world)
   └─ 结果: ypd_in_world（便于后续弹道计算）
```

**核心函数**:

```cpp
void Solver::solve(Armor & armor) const {
  // 1. PnP 求解
  cv::Vec3d rvec, tvec;
  cv::solvePnP(BIG_ARMOR_POINTS, armor.points, 
               camera_matrix_, distort_coeffs_, 
               rvec, tvec, false, cv::SOLVEPNP_IPPE);
  
  // 2. 转换到云台坐标系
  Eigen::Vector3d xyz_in_camera;
  cv::cv2eigen(tvec, xyz_in_camera);
  armor.xyz_in_gimbal = R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_;
  
  // 3. 转换到世界坐标系
  armor.xyz_in_world = R_gimbal2world_ * armor.xyz_in_gimbal;
  
  // 4. Yaw 优化
  optimize_yaw(armor);
}
```

**重投影误差计算**:

Yaw 优化时，通过计算重投影误差来衡量当前 Yaw 是否合理：

```
候选Yaw → 投影回图像 → 与实际检测点比较 → 计算像素距离和
```

误差越小，说明该 Yaw 越可靠。

---

### 3.3 Tracker（追踪器）

**职责**: 将单帧的装甲板检测结果关联到目标，并维护目标状态

**输入**:
- 当前帧检测到的装甲板列表
- 时间戳

**输出**:
- 追踪到的目标列表（通常只有 1 个目标）
- 目标的预测状态

**工作流程**:

```
【第1步】过滤装甲板
   ├─ 去除非敌方颜色的装甲
   ├─ 按距离图像中心排序（靠近中心优先）
   ├─ 按优先级排序（优先级高的装甲更容易跟踪）
   
【第2步】状态机判断
   ├─ 状态 "lost": 未追踪到目标
   ├─ 状态 "detecting": 可能检测到目标，等待确认
   ├─ 状态 "tracking": 成功追踪中
   └─ 逻辑: lost → detecting (满足 min_detect_count) → tracking
              tracking → detecting (丢失但未超时) → lost (超时)
   
【第3步】关键决策
   IF 状态 == "lost":
       尝试 set_target()  // 用检测到的装甲初始化新目标
   ELSE:
       尝试 update_target()  // 用新装甲更新现有目标
   
【第4步】合法性检查
   ├─ 检查 EKF 是否发散（协方差过大）
   ├─ 检查 EKF 收敛质量（NIS检验）
   └─ 若发散或质量差，回到 "lost" 状态
   
输出: 目标列表（或空列表）
```

**关键函数**:

```cpp
std::list<Target> Tracker::track(
    std::list<Armor> & armors, 
    std::chrono::steady_clock::time_point t,
    bool use_enemy_color) {
  
  // 1. 过滤装甲
  armors.remove_if([&](const Armor& a) { 
    return a.color != enemy_color_; 
  });
  armors.sort([](const Armor& a, const Armor& b) { 
    return a.priority < b.priority;  // 按优先级排序
  });
  
  // 2. 状态机
  bool found;
  if (state_ == "lost") {
    found = set_target(armors, t);
  } else {
    found = update_target(armors, t);
  }
  
  state_machine(found);
  
  // 3. 检查合法性
  if (target_.diverged()) {
    state_ = "lost";
    return {};
  }
  
  if (state_ == "lost") return {};
  
  return {target_};
}
```

---

### 3.4 Target（目标状态估计）

**职责**: 用 EKF 维护单个目标的运动状态，并进行预测

**状态向量** (11维):

```cpp
x = [
  x, vx,              // 旋转中心 X 坐标和速度
  y, vy,              // 旋转中心 Y 坐标和速度
  z, vz,              // 旋转中心 Z 坐标和速度
  yaw, yaw_vel,       // Yaw角和角速度
  radius,             // 装甲板到旋转中心的距离
  dRadius, dHeight    // 装甲板间相对差异
]
```

**预测过程**:

```
时刻 t0，已知状态 x(t0)
   │
   ├─ 根据恒速模型，计算时刻 t 的预测: x_pred(t) = F*x(t0) + w
   │  其中 F 是状态转移矩阵，w 是过程噪声
   │
时刻 t，收到新观测（新装甲）
   │
   ├─ 计算观测向量: z = [yaw, pitch, distance] 对应的装甲位置
   │
   ├─ 比较预测与观测的差异，更新状态: x(t) = x_pred(t) + K*(z - h(x_pred))
   │  其中 K 是卡尔曼增益，h 是观测模型
   │
更新后状态 x(t)
```

**关键函数**:

```cpp
void Target::predict(std::chrono::steady_clock::time_point t) {
  auto dt = delta_time(t, t_);
  
  // 构造状态转移矩阵 F（每个分量独立）
  Eigen::MatrixXd F = /* 11x11矩阵 */;
  // F[i][i+1] = dt 表示用速度预测位置变化
  
  ekf_.predict(F);  // EKF 预测步
}

void Target::update(const Armor & armor) {
  // 用新观测更新 EKF
  ekf_.update(z, R);  // z是观测，R是观测噪声协方差
}
```

**优势**:

- 即使短时间内没有检测到装甲（丢失），EKF 仍能预测其位置
- 通过噪声协方差自适应地调整对新观测的信任度
- 能够学习目标的速度、角速度等运动特性

---

### 3.5 Aimer（瞄准器）

**职责**: 根据目标的预测位置计算最优瞄准点，并求解所需的云台角度

**输入**:
- 追踪到的目标列表
- 当前时间戳
- 子弹速度

**输出**:
- `io::Command` - 包含 Yaw、Pitch 控制指令

**核心挑战**:

```
现在（t=0）: 检测到敌方装甲在位置 A，可以看到
  │
  ├─ 通讯延迟: 5ms
  ├─ 图像处理延迟: 10ms
  ├─ 云台响应延迟: 20ms
  ├─ 开火延迟: 100ms
  │
时刻 t ≈ 135ms: 子弹出膛
  │
  ├─ 子弹飞行时间: 150ms
  │
时刻 t ≈ 285ms: 子弹应该到达敌方装甲
  │
问题: 285ms 后敌方会在哪里？
```

**解决方案 - 迭代预测**:

```cpp
io::Command Aimer::aim(
    std::list<Target> targets,
    std::chrono::steady_clock::time_point timestamp,
    double bullet_speed) {
  
  auto target = targets.front();
  
  // 1. 计算总延迟（通讯+处理+云台响应+开火）
  double total_delay = 0.135;  // 135ms
  auto future_time = timestamp + total_delay;
  
  // 2. 初始弹道求解
  auto aim_point = choose_aim_point(target);
  auto trajectory = Trajectory(bullet_speed, distance, height);
  double fly_time = trajectory.fly_time;
  
  // 3. 迭代：根据子弹飞行时间细化预测
  for (int iter = 0; iter < 10; iter++) {
    auto predict_time = future_time + fly_time;
    target.predict(predict_time);
    
    auto aim_point_new = choose_aim_point(target);
    auto trajectory_new = Trajectory(bullet_speed, distance, height);
    
    // 检查收敛
    if (abs(trajectory_new.fly_time - fly_time) < 0.001) break;
    
    fly_time = trajectory_new.fly_time;
  }
  
  return Command(yaw, pitch);
}
```

**弹道模型**:

```
子弹运动方程（忽略空气阻力的简化模型）:
  
  水平距离 d (米)
  竖直距离 h (米)
  子弹速度 v (m/s)
  
  首先确定出膛时的仰角 alpha（pitch）:
    h = d * tan(alpha) - 0.5 * g * (d/v/cos(alpha))^2
  
  求解出 alpha 后，计算飞行时间:
    t_fly = d / (v * cos(alpha))
```

---

### 3.6 Shooter（射击控制）

**职责**: 判断是否应该发射子弹

**输入**:
- 瞄准命令（是否有有效的瞄准点）
- 当前云台角度
- 目标信息

**输出**:
- `bool shoot` - 是否发射

**判断逻辑**:

```
IF 没有有效的瞄准点:
    RETURN false  // 不发射
    
IF 当前云台角度与瞄准角度偏差 > tolerance:
    RETURN false  // 瞄准还不够精准，稍等
    
IF 目标距离 < judge_distance 且 角度精度OK:
    RETURN true   // 距离合适且瞄准精准，发射！
    
ELSE:
    RETURN false  // 距离太远或其他条件不满足
```

---

## 4. 完整数据流

### 4.1 单帧处理流程图

```
├─────────────────────────────────────────────────────────────────┤
│                    【开始】新的一帧                               │
├─────────────────────────────────────────────────────────────────┤
                           ↓
┌─────────────────────────────────────────────────────────────────┐
│ 【步骤1】相机采集                                               │
│  camera.read(img, t)                                            │
│  输出: BGR 图像 + 时间戳                                         │
└─────────────────────────────────────────────────────────────────┘
                           ↓
┌─────────────────────────────────────────────────────────────────┐
│ 【步骤2】IMU 姿态获取                                           │
│  Eigen::Quaterniond q = gimbal.q(t - 1ms)                       │
│  获取当前云台的四元数姿态（用于建立世界坐标系）                 │
│  solver.set_R_gimbal2world(q)                                   │
│  更新坐标变换矩阵                                               │
└─────────────────────────────────────────────────────────────────┘
                           ↓
┌─────────────────────────────────────────────────────────────────┐
│ 【步骤3】装甲板检测                                             │
│  armors = detector.detect(img)                                  │
│  ├─ YOLO 神经网络检测                                           │
│  ├─ 几何有效性检查                                             │
│  ├─ 装甲板分类（颜色、编号、大小）                             │
│  ├─ 去重                                                       │
│  └─ 输出: std::list<Armor> 检测到的装甲板列表                 │
└─────────────────────────────────────────────────────────────────┘
                           ↓
┌─────────────────────────────────────────────────────────────────┐
│ 【步骤4】装甲板坐标解算                                         │
│  for (auto& armor : armors) {                                   │
│    solver.solve(armor);  // PnP 求解                           │
│  }                                                              │
│  ├─ PnP 求解 (相机坐标系)                                       │
│  ├─ 坐标变换 (相机 → 云台 → 世界)                              │
│  ├─ Yaw 角优化                                                 │
│  └─ 装甲板现在有了 xyz_in_world, ypr_in_world 等属性           │
└─────────────────────────────────────────────────────────────────┘
                           ↓
┌─────────────────────────────────────────────────────────────────┐
│ 【步骤5】目标追踪与预测                                         │
│  targets = tracker.track(armors, t)                             │
│  ├─ 过滤装甲（敌方颜色、优先级排序）                           │
│  ├─ 状态机判断（lost/detecting/tracking）                      │
│  ├─ set_target() 或 update_target()                            │
│  ├─ EKF 预测与更新                                             │
│  ├─ 收敛性检查                                                 │
│  └─ 输出: std::list<Target> 当前追踪的目标（通常 1 个）        │
└─────────────────────────────────────────────────────────────────┘
                           ↓
                    targets 是否为空？
                    /                 \
                 YES (丢失)            NO (有目标)
                /                        \
               ↓                          ↓
    返回空指令，         ┌──────────────────────────────────────┐
    不动作            │ 【步骤6】瞄准计算                       │
                      │  command = aimer.aim(targets, t, v)   │
                      │ ├─ 预测延迟补偿                        │
                      │ ├─ 迭代子弹飞行时间                    │
                      │ ├─ 计算瞄准点 (xyza)                   │
                      │ ├─ 弹道求解                           │
                      │ └─ 输出: io::Command                   │
                      │    含 yaw, pitch, yaw_vel, pitch_vel  │
                      └──────────────────────────────────────┘
                                 ↓
                      ┌──────────────────────────────────────┐
                      │ 【步骤7】射击决策                    │
                      │  command.shoot = shooter.shoot(...)  │
                      │ ├─ 检查瞄准精度                      │
                      │ ├─ 检查距离                          │
                      │ └─ 输出: bool                        │
                      │    true = 发射，false = 不发射       │
                      └──────────────────────────────────────┘
                                 ↓
                      ┌──────────────────────────────────────┐
                      │ 【步骤8】下发控制命令                │
                      │  gimbal.send(command)                │
                      │ ├─ 发送 Yaw, Pitch 角度              │
                      │ ├─ 发送速度、加速度前馈              │
                      │ ├─ 发送射击信号                      │
                      │ └─ 下位机执行控制                    │
                      └──────────────────────────────────────┘
                                 ↓
                      ┌──────────────────────────────────────┐
                      │ 【步骤9】可视化与调试                │
                      │ ├─ 绘制检测框（绿）                  │
                      │ ├─ 绘制预测框（红）                  │
                      │ ├─ 显示 FPS、角度、发射状态          │
                      │ └─ imshow() 显示                     │
                      └──────────────────────────────────────┘
                                 ↓
├─────────────────────────────────────────────────────────────────┤
│                 【循环】跳回开始，处理下一帧                      │
├─────────────────────────────────────────────────────────────────┤
```

### 4.2 数据流示意

```
时间线：
─────────────────────────────────────────────────────→ 时间

Frame 0:
  ├─ t=0ms: 相机采集图像 I0
  ├─ t=5ms: 图像传输完成
  ├─ t=5ms: 开始 YOLO 推理
  ├─ t=15ms: YOLO 完成，得到 armors0
  ├─ t=16ms: PnP 求解，坐标变换
  ├─ t=20ms: 目标追踪
  ├─ t=25ms: 瞄准计算
  ├─ t=30ms: 控制指令下发
  │
Frame 1:
  ├─ t=33ms: 相机采集图像 I1
  ├─ ...
  │
Frame N (当前):
  ├─ t=Nms: 现在时刻
  ├─ 云台当前姿态: 已知
  ├─ 当前检测到的装甲: 已知
  ├─ 已追踪的目标状态: 已知
  │
瞄准计算需要预测：
  ├─ 当前时刻 + 总延迟 (135ms) + 飞行时间 (150ms) = 285ms 后
  ├─ 目标会在哪里？
  │
回答：
  ├─ 用 EKF 根据当前状态和运动模型外推
  ├─ 得到 285ms 后的预测位置
  ├─ 以该位置为瞄准点
  └─ 保证子弹能击中
```

---

## 5. 核心函数调用关系

### 5.1 文件依赖与函数调用关系

\\\
main (standard.cpp / sentry.cpp)
│
├─ detector.hpp / detector.cpp
│   ├─ 调用: classifier.classify(armor)
│   └─ 返回: std::list<Armor> armors
│
├─ tracker.hpp / tracker.cpp
│   ├─ 依赖: solver.hpp (保持引用)
│   │
│   ├─ Tracker::track(armors, t)
│   │   ├─ set_target(armors, t)
│   │   │   └─ 调用: solver_.solve(armor)        【第1次】
│   │   │       ├─ 文件: solver.hpp/solver.cpp
│   │   │       └─ 返回: 无(原地修改armor)
│   │   │
│   │   └─ update_target(armors, t)
│   │       ├─ 调用: target_.predict(t)
│   │       │   └─ 文件: target.hpp/target.cpp
│   │       │       └─ 内部: ekf_.predict(F)
│   │       │
│   │       ├─ 调用: solver_.solve(armor) ×N    【第2～Nth次】
│   │       │   └─ 文件: solver.hpp/solver.cpp
│   │       │
│   │       └─ 调用: target_.update(matched_armor)
│   │           └─ 文件: target.hpp/target.cpp
│   │               └─ 内部: ekf_.update(z, H, R)
│   │
│   └─ 返回: std::list<Target> targets
│
├─ aimer.hpp / aimer.cpp
│   ├─ Aimer::aim(targets, t, bullet_speed)
│   │   ├─ 调用: target.predict(future)
│   │   │   └─ 文件: target.hpp/target.cpp
│   │   │
│   │   ├─ 调用: choose_aim_point(target)
│   │   │   └─ 文件: aimer.hpp/aimer.cpp (私有函数)
│   │   │
│   │   ├─ 调用: Trajectory(bullet_speed, d, h)
│   │   │   └─ 文件: tools/trajectory.hpp
│   │   │
│   │   └─ for 迭代 ×10:
│   │       ├─ 调用: target.predict()
│   │       ├─ 调用: choose_aim_point()
│   │       └─ 调用: Trajectory()
│   │
│   └─ 返回: io::Command command
│
├─ shooter.hpp / shooter.cpp
│   ├─ Shooter::shoot(command, aimer, targets, gimbal_pos)
│   │   ├─ 读取: aimer.debug_aim_point
│   │   ├─ 读取: targets[0].ekf_x()
│   │   ├─ 读取: gimbal_pos (欧拉角)
│   │   └─ 逻辑判决(无新函数调用)
│   │
│   └─ 返回: bool shoot
│
└─ gimbal.send(command, shoot)
    └─ 文件: io/gimbal.hpp/gimbal.cpp
\\\

---

### 5.2 关键模块文件对应关系

#### Detector (detector.hpp / detector.cpp)
**被调用处**: main() 主循环
**调用内部函数**:
- \classifier_.classify(armor)\ → 返回修改后的armor(颜色/编号)

**依赖文件**:
- armor.hpp - 数据结构
- classifier.hpp/classifier.cpp - 分类子模块

#### Solver (solver.hpp / solver.cpp)
**被调用处**: 
- tracker.cpp 的 set_target() 和 update_target()

**重要函数**: \solve(armor)\
- 无返回值，原地修改 armor 的:
  - xyz_in_gimbal
  - xyz_in_world
  - ypr_in_gimbal
  - ypr_in_world
  - ypd_in_world

**依赖文件**:
- armor.hpp - 数据结构
- YAML配置 - 读取相机标定参数
- Eigen库 - 矩阵运算

#### Tracker (tracker.hpp / tracker.cpp)
**被调用处**: main() 主循环

**重要函数**: \	rack(armors, t)\
**内部调用链**:
1. \set_target(armors, t)\ (当 state_ == "lost")
   ├─ 调用: \solver_.solve(armor)\
   ├─ 创建: \Target(armor, ...)\
   └─ 返回: bool

2. \update_target(armors, t)\ (当 state_ != "lost")
   ├─ 调用: \	arget_.predict(t)\
   ├─ 调用: \solver_.solve(armor)\ ×N
   ├─ 调用: \	arget_.update(matched_armor)\
   └─ 返回: bool

**依赖文件**:
- target.hpp/target.cpp - Target对象和EKF
- solver.hpp - 保存引用，调用solve()
- armor.hpp - 数据结构

#### Target (target.hpp / target.cpp)
**被调用处**:
- tracker.cpp 中 set_target() 和 update_target()
- aimer.cpp 中 aim() 和 choose_aim_point()

**重要函数**:
1. \predict(t)\ - EKF预测
   ├─ 构造状态转移矩阵F
   └─ 调用: \ekf_.predict(F)\

2. \update(armor)\ - EKF更新
   ├─ 提取观测z
   └─ 调用: \ekf_.update(z, H, R)\

**依赖文件**:
- armor.hpp - 数据结构
- tools/extended_kalman_filter.hpp - EKF实现

#### Aimer (aimer.hpp / aimer.cpp)
**被调用处**: main() 主循环

**重要函数**: \aim(targets, t, bullet_speed)\
**内部调用链**:
1. \	arget.predict(future)\ - 延迟补偿预测
2. \choose_aim_point(target)\ - 装甲选择(私有)
3. \Trajectory(v, d, h)\ - 弹道求解
4. 迭代×10:
   ├─ \	arget.predict()\
   ├─ \choose_aim_point()\
   └─ \Trajectory()\

**依赖文件**:
- target.hpp - Target对象和predict()
- tools/trajectory.hpp - 弹道计算

#### Shooter (shooter.hpp / shooter.cpp)
**被调用处**: main() 主循环

**重要函数**: \shoot(command, aimer, targets, gimbal_pos)\
**操作**:
- 读取: \aimer.debug_aim_point\ (检查有效性)
- 读取: \	argets[0].ekf_x()\ (获取角速度和距离)
- 读取: \gimbal_pos\ (当前欧拉角)
- 逻辑计算: 容差判决

**依赖文件**:
- aimer.hpp - 读取debug_aim_point
- target.hpp - 读取ekf_x()

---

### 5.3 函数调用时序

\\\
主循环 (main)
│
├─【第1步】 detector.detect(img)
│   └─ classifier.classify() ×M 装甲板数
│   └─ 返回: armors
│
├─【第2步】 tracker.track(armors, t)
│   ├─ 首次: solver.solve() ×1
│   ├─ 后续: solver.solve() ×N (N=检测到的装甲数)
│   └─ 返回: targets
│
├─【第3步】 aimer.aim(targets, t, bullet_speed)
│   ├─ target.predict() ×11 (1+10次迭代)
│   ├─ choose_aim_point() ×11
│   └─ Trajectory() ×11
│   └─ 返回: command
│
├─【第4步】 shooter.shoot(command, aimer, targets, gimbal_pos)
│   └─ 返回: shoot(bool)
│
└─【第5步】 gimbal.send(command, shoot)
    └─ 下发到硬件
\\\

---
## 6. 关键算法讲解

### 6.1 PnP 求解（Perspective-n-Point）

**问题描述**:
```
已知: 世界坐标系中 4 个点的 3D 坐标
     这 4 个点在摄像机中对应的 2D 像素坐标
     摄像机内参矩阵 K（焦距、主点等）

求: 摄像机相对于世界的位置(t) 和方向(R)
    即 R 和 t，使得 3D 点投影到图像上后与实际像素点重合
```

**数学模型**:
```
世界坐标 P_w → 相机坐标 P_c = R*P_w + t
相机坐标 → 图像坐标 p = K * P_c / P_c.z
```

**OpenCV 实现**:
```cpp
cv::solvePnP(
  object_points,      // 4个 3D 点（装甲板四角在标定时的已知坐标）
  image_points,       // 4个对应的 2D 像素点
  camera_matrix,      // 相机内参 K
  distort_coeffs,     // 畸变系数
  rvec,              // 输出：旋转向量（轴角表示）
  tvec,              // 输出：平移向量
  useExtrinsicGuess, // 是否有初始估计
  flags              // 算法选择 (IPPE, EPNP, etc.)
);
```

**装甲板模板坐标**:
```cpp
// 大装甲板：230mm 宽，56mm 高
const std::vector<cv::Point3f> BIG_ARMOR_POINTS{
  {0, 0.115, 0.028},    // 右上角（X=0在装甲平面，Y轴向右，Z向上）
  {0, -0.115, 0.028},   // 左上角
  {0, -0.115, -0.028},  // 左下角
  {0, 0.115, -0.028}    // 右下角
};
```

### 6.2 扩展卡尔曼滤波（EKF）

**为什么需要 EKF**:

```
没有 EKF 的情况：
  ├─ 只用检测到的装甲板位置
  ├─ 每一帧都重新从零开始计算
  ├─ 如果某帧漏检，就无法预测目标位置
  └─ 结果：低鲁棒性

有 EKF 的情况：
  ├─ 用历史观测逐步学习目标运动规律
  ├─ 即使短时间漏检，也能预测
  ├─ 自动调节对观测的信任度（通过协方差）
  └─ 结果：更鲁棒的追踪
```

**EKF 工作流程** (两个步骤循环):

```
预测步（无观测时）:
  x_pred = F * x_prev      // 用运动模型预测
  P_pred = F * P * F^T + Q // 不确定性增加

更新步（有观测时）:
  innovation = z - h(x_pred)  // 预测与观测差异
  K = P * H^T / (H*P*H^T + R) // 卡尔曼增益（决定信任谁）
  x_new = x_pred + K * innovation // 融合
  P_new = (I - K*H) * P_pred       // 不确定性减少
```

**参数含义**:

```
x: 状态向量（位置、速度、角度等）
F: 状态转移矩阵（运动学方程）
Q: 过程噪声协方差（对"模型有多准确"的猜测）
z: 观测（检测到的装甲位置）
h: 观测模型（如何从状态得到观测）
R: 观测噪声协方差（对"检测有多准确"的猜测）
P: 状态估计的不确定性
```

**自适应例子**:

```
情况1：Q 很大（过程噪声大，模型不可信）
  ├─ K 会变小
  └─ EKF 更信任观测，跟踪更紧

情况2：R 很大（观测噪声大，检测不可信）
  ├─ K 会变小
  └─ EKF 更信任模型，预测更平稳

情况3：观测突然变异（可能是误检）
  ├─ innovation 很大
  ├─ EKF 会只部分接受这个观测
  └─ 结果：滤波器"平滑"了噪声
```

### 6.3 弹道求解

**输入参数**:
```
bullet_speed: 子弹速度（m/s），通常 23~30
distance_xy: 水平距离（米）
height_z: 竖直高度差（米）
gravity: 重力加速度 9.8 m/s²
```

**输出结果**:
```
pitch_angle: 所需仰角（弧度）
fly_time: 子弹飞行时间（秒）
```

**物理模型**:

```
竖直方程: z = y*tan(pitch) - 0.5*g*(t^2)
水平方程: xy = v*cos(pitch)*t

消去时间 t，得：
  z = xy*tan(pitch) - 0.5*g*(xy/(v*cos(pitch)))^2
  z = xy*tan(pitch) - g*xy^2 / (2*v^2*cos^2(pitch))

令 tan(pitch) = u，则 cos^2(pitch) = 1/(1+u^2)，可得关于 u 的二次方程：
  a*u^2 + b*u + c = 0

求解后得两个解（弧线击中、直线击中），选合理的一个。
```

**代码骨架**:
```cpp
struct Trajectory {
  bool unsolvable;
  double pitch;
  double fly_time;
  
  Trajectory(double v, double xy, double z) {
    // 求解二次方程
    // 检查判别式，若 < 0 则无解（距离太远或高度差太大）
    // 选择更小的仰角（通常更稳定）
    // 计算飞行时间: t = xy / (v * cos(pitch))
  }
};
```

### 6.4 坐标系转换

**相机 → 云台**:
```
P_gimbal = R_camera2gimbal * P_camera + t_camera2gimbal

其中 R 是 3×3 旋转矩阵，t 是 3×1 平移向量
这些参数由手眼标定得到（在项目部署前一次性标定）
```

**云台 → 世界**:
```
P_world = R_gimbal2world * P_gimbal

R_gimbal2world 由 IMU 四元数 q 实时计算：
  q = (qw, qx, qy, qz)
  R_gimbal2world = q.toRotationMatrix()
```

**逆变换示例**（用于重投影）:
```
世界坐标回投到相机：
  P_camera = R_camera2gimbal^T * (R_gimbal2world^T * P_world - t_camera2gimbal)
  
然后用相机内参投影到图像：
  p_pixel = K * P_camera / P_camera.z
```

---

## 7. 代码执行流程

### 7.1 初始化阶段

```cpp
int main(int argc, char * argv[]) {
  // 1. 解析命令行参数
  std::string config_path = "configs/standard3.yaml";
  
  // 2. 初始化各模块（从YAML配置文件读参数）
  io::CBoard cboard(config_path);              // 云台通信
  io::Camera camera(config_path);              // 相机
  
  auto_aim::Detector detector(config_path);    // 检测器
  auto_aim::Solver solver(config_path);        // 解算器
    // 读取相机内参、畸变参数、手眼标定外参
  
  auto_aim::Tracker tracker(config_path, solver);  // 追踪器
    // solver 作为引用传入，追踪时会调用 solver.solve()
  
  auto_aim::Aimer aimer(config_path);          // 瞄准器
    // 读取偏移量、延迟参数
  
  auto_aim::Shooter shooter(config_path);      // 射击控制
  
  // 3. 准备变量
  cv::Mat img;
  std::chrono::steady_clock::time_point t;
  
  // ========== 主循环 ==========
  while (!exiter.exit()) {
    // 见下一小节
  }
}
```

### 7.2 主循环（每帧 30-40ms）

```cpp
// ========== 单帧处理 ==========

// 【第1步】采集图像与时间戳
camera.read(img, t);
if (img.empty()) {
  std::this_thread::sleep_for(10ms);
  continue;
}

// 【第2步】获取 IMU 四元数，建立世界坐标系
Eigen::Quaterniond q = gimbal.q(t - 1ms);  // 回溯 1ms 补偿延迟
solver.set_R_gimbal2world(q);

// 【第3步】检测装甲板
auto armors = detector.detect(img);

// 【第4步】追踪（内部自动调用 solver.solve()）
auto targets = tracker.track(armors, t);

// 【第5步】瞄准计算
auto gimbal_state = gimbal.state();
auto command = aimer.aim(targets, t, gimbal_state.bullet_speed);

// 【第6步】射击决策
Eigen::Vector3d current_ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);
command.shoot = shooter.shoot(command, aimer, targets, current_ypr);

// 【第7步】下发控制命令
gimbal.send(command.control, command.shoot, command.yaw,
            command.yaw_vel, command.yaw_acc, command.pitch,
            command.pitch_vel, command.pitch_acc);

// 【第8步】可视化
for (const auto& armor : armors) {
  for (int i = 0; i < 4; i++) {
    cv::line(img, armor.points[i], armor.points[(i+1)%4], {0, 255, 0}, 2);
  }
}
cv::imshow("Auto Aim", img);
if (cv::waitKey(1) == 'q') break;
```

### 7.3 Tracker 内部流程

```cpp
std::list<Target> Tracker::track(
    std::list<Armor> & armors,
    std::chrono::steady_clock::time_point t) {
  
  // 1. 时间检查
  auto dt = delta_time(t, last_timestamp_);
  if (dt > 0.1 && state_ != "lost") {
    state_ = "lost";  // 相机掉线
  }
  
  // 2. 过滤装甲
  armors.remove_if([&](const Armor& a) {
    return a.color != enemy_color_;  // 只保留敌方颜色
  });
  
  // 3. 排序
  armors.sort([](const Armor& a, const Armor& b) {
    return a.priority < b.priority;  // 按优先级排序
  });
  
  // 4. 状态机与关键决策
  bool found;
  if (state_ == "lost") {
    // 尝试初始化新目标
    found = set_target(armors, t);
    if (found) {
      // solver.solve() 在 set_target() 内部被调用
      // 现在 target_ 已有 xyz_in_world, ypr_in_world
    }
  } else {
    // 更新现有目标
    found = update_target(armors, t);
  }
  
  // 5. 状态转移
  state_machine(found);
  
  // 6. 合法性检查
  if (target_.diverged()) {
    state_ = "lost";
    return {};
  }
  
  // 7. 返回结果
  if (state_ == "lost") {
    return {};
  }
  return {target_};
}
```

### 7.4 set_target() - 初始化新目标

```cpp
bool Tracker::set_target(std::list<Armor> & armors,
                         std::chrono::steady_clock::time_point t) {
  if (armors.empty()) return false;
  
  auto& armor = armors.front();  // 取优先级最高的
  
  // 【关键】在这里调用 solver.solve()
  solver_.solve(armor);
  // 现在 armor 已有:
  //   - xyz_in_gimbal
  //   - xyz_in_world
  //   - ypr_in_world
  //   - ypd_in_world
  
  // 根据装甲创建新 Target
  target_ = Target(
    armor,
    t,
    estimated_radius,  // 估计的旋转半径
    armor_num,         // 装甲数量 4 或 8
    P0_dig             // EKF初始协方差
  );
  
  detect_count_ = 1;
  return true;
}
```

### 7.5 update_target() - 更新目标

```cpp
bool Tracker::update_target(std::list<Armor> & armors,
                            std::chrono::steady_clock::time_point t) {
  // 1. 预测：根据上一时刻的状态和运动模型预测当前位置
  target_.predict(t);
  
  // 2. 寻找最匹配的装甲
  double best_error = 1e9;
  int best_idx = -1;
  int idx = 0;
  
  for (auto& armor : armors) {
    solver_.solve(armor);  // 求解装甲的 3D 位置
    
    // 计算预测位置与观测位置的差异
    auto error = /* 某种距离度量 */;
    
    if (error < best_error) {
      best_error = error;
      best_idx = idx;
    }
    idx++;
  }
  
  if (best_idx == -1 || best_error > MAX_ERROR) {
    return false;  // 没找到匹配的装甲
  }
  
  // 3. 用最匹配的装甲更新 EKF
  auto& matched_armor = /* 取出第 best_idx 个 */;
  target_.update(matched_armor);
  
  detect_count_++;
  temp_lost_count_ = 0;
  return true;
}
```

### 7.6 Aimer 内部流程

```cpp
io::Command Aimer::aim(
    std::list<Target> targets,
    std::chrono::steady_clock::time_point timestamp,
    double bullet_speed) {
  
  if (targets.empty()) {
    return {false, false, 0, 0};  // 无效指令
  }
  
  auto target = targets.front();
  
  // 1. 计算总延迟
  // 通讯(5ms) + 图像处理(10ms) + 云台响应(20ms) + 开火(100ms) ≈ 135ms
  double total_delay = 0.135;
  auto future_time = timestamp + total_delay;
  
  // 2. 预测目标在 future_time 时的位置
  target.predict(future_time);
  
  // 3. 选择瞄准点（选择离云台最近的装甲）
  auto aim_point = choose_aim_point(target);
  
  // 4. 根据距离求解弹道
  Eigen::Vector3d xyz = aim_point.xyza.head(3);
  double distance_xy = sqrt(xyz[0]^2 + xyz[1]^2);
  double height = xyz[2];
  
  tools::Trajectory trajectory(bullet_speed, distance_xy, height);
  if (trajectory.unsolvable) {
    return {false, false, 0, 0};  // 无法击中
  }
  
  double fly_time = trajectory.fly_time;
  double pitch = trajectory.pitch;
  
  // 5. 迭代精化（考虑子弹飞行时间）
  for (int iter = 0; iter < 10; iter++) {
    auto predict_time_refined = future_time + fly_time;
    target.predict(predict_time_refined);
    
    auto aim_point_new = choose_aim_point(target);
    auto trajectory_new = trajectory(bullet_speed, ...);
    
    if (abs(trajectory_new.fly_time - fly_time) < 0.001) {
      break;  // 收敛
    }
    fly_time = trajectory_new.fly_time;
    pitch = trajectory_new.pitch;
  }
  
  // 6. 计算目标 Yaw 角
  double yaw = atan2(xyz[1], xyz[0]);
  
  // 7. 加上标定偏移
  yaw += yaw_offset_;
  pitch += pitch_offset_;
  
  // 8. 返回指令
  return {
    true,                           // 有效
    false,                          // 不区分左右炮（新版本）
    yaw,                            // Yaw 角
    pitch,                          // Pitch 角
    /* yaw速度、加速度等前馈 */
  };
}
```

---

## 8. 常见问题与调试建议

### Q1: 为什么需要 Yaw 优化？

**A**: PnP 求解的结果对几何变化很敏感。当装甲板斜向摄像机时，Yaw 角误差会较大。通过在 ±70° 范围内扫描，选择重投影误差最小的 Yaw，可以校正这个误差。

### Q2: 为什么有时目标会"丢失"？

**A**: 可能的原因：
1. **漏检**: 装甲板被遮挡，YOLO 没检测到
2. **误检**: YOLO 检测了错误的目标，EKF 发散
3. **快速切换**: 装甲切换过快，Tracker 反应不及
4. **通讯中断**: IMU 或相机掉线

**调试**: 调高 `max_temp_lost_count_` 参数，容许更长的丢失时间。

### Q3: 为什么打不准？

**A**: 可能的原因：
1. **标定不准**: 相机内参或手眼标定有误
2. **延迟预测错误**: 总延迟时间估计不准
3. **EKF 参数**: Q、R 参数设置不合理
4. **弹速不对**: 实际子弹速度与配置不符

**调试**: 
- 用 `reproject_armor()` 在图像上绘制预测框，与实际比对
- 通过视频回放测量实际延迟，调整参数
- 参考 `aimer.cpp` 中的 `high_speed_delay_time_` 等参数

### Q4: EKF 什么时候会发散？

**A**: 当：
1. 观测噪声 R 设置过小，EKF 过度信任错误观测
2. 过程噪声 Q 设置过小，EKF 对模型过度自信
3. 发生大角度装甲切换，模型假设失效

**调试**: 查看 `target_.ekf().recent_nis_failures` 检验 NIS（Normalized Innovation Squared），质量应该 < 40%。

---

## 9. 总结

### 自瞄系统的本质

```
检测 → 解算 → 追踪 → 瞄准 → 射击
  │      │      │      │      │
图像坐标  3D坐标  运动   最优点  控制
  →      →      →      →      →
世界坐标  滤波   预测   精化    执行
```

### 核心设计思想

1. **分离关注**: 各模块各司其职，通过清晰的接口相连
2. **概率框架**: 用 EKF 进行观测融合和预测
3. **前馈补偿**: 计算速度和加速度前馈，改善控制响应
4. **延迟补偿**: 迭代预测，准确计算子弹飞行时间
5. **鲁棒性**: 即使短时间漏检，仍能追踪

### 学习建议

1. 从 `Armor` 数据结构入手，理解坐标系的层级变换
2. 阅读 `Solver` 代码，掌握 PnP 和坐标变换
3. 学习 `Target` 和 EKF 的原理
4. 在实验中调整参数，感受各参数对性能的影响
5. 用调试工具（PlotJuggler）绘制关键变量曲线，观察系统动态

---

**文档完成时间**: 2025年9月26日  
**项目来源**: 同济大学 SuperPower 战队 2025 赛季自瞄开源
**维护者**: 视觉组

**[返回项目主页](README.md)**

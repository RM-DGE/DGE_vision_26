# RoboMaster 自瞄完整系统技术手册

> **目标读者**: 有C++基础、想理解完整自瞄系统（包括硬件通信）的工程师
> **阅读时间**: 60-90分钟
> **前置知识**: C++基础、auto_aim_learn.md 中的算法概念

---

## 目录

1. [系统总体架构](#1-系统总体架构)
2. [完整数据流](#2-完整数据流)
3. [感知层详解](#3-感知层详解)
4. [估计层详解](#4-估计层详解)
5. [决策层详解](#5-决策层详解)
6. [执行层详解](#6-执行层详解)
7. [硬件通信协议](#7-硬件通信协议)
8. [配置与参数](#8-配置与参数)
9. [工程实践](#9-工程实践)

---

## 1. 系统总体架构

### 1.1 系统分层

```
┌──────────────────────────────────────────────┐
│         应用层 (standard.cpp / sentry.cpp)   │
│              主循环 while(true)              │
└──────────────────────────────────────────────┘
            ↓           ↓           ↓
┌──────────────────┐ ┌──────────────────┐ ┌──────────────────┐
│   【感知层】     │ │   【估计层】     │ │   【决策层】     │
│   Detector       │ │   Tracker        │ │   Aimer          │
│   Classifier     │ │   Solver (PnP)   │ │   Shooter        │
│   输出: Armor[]  │ │   Target (EKF)   │ │   输出: Command  │
└──────────────────┘ └──────────────────┘ └──────────────────┘
            ↓           ↓           ↓
┌──────────────────────────────────────────────┐
│         【执行层】Gimbal / Camera             │
│    输出到下位机(C板) 或采集图像              │
└──────────────────────────────────────────────┘
            ↓
┌──────────────────────────────────────────────┐
│    【通信层】CBoard - USB串口/CAN            │
│  发送: 云台角度(yaw/pitch) + 开火信号        │
│  接收: 云台反馈(四元数/欧拉角) + 状态        │
└──────────────────────────────────────────────┘
```

**总结**: 整个系统就是一个流水线：相机拍照 → 检测装甲 → 追踪敌人 → 计算瞄准 → 判断开火 → 发送给下位机 → 下位机控制云台射击。

### 1.2 核心模块清单

| 模块 | 文件位置 | 【输入】 | 【输出】 | 职责 |
|------|---------|---------|---------|------|
| **Camera** | `io/camera.hpp` | - | `cv::Mat img + double ts` | 采集实时图像 |
| **Detector** | `tasks/auto_aim/detector.cpp` | `cv::Mat img` | `std::list<Armor>` | 从图像检测装甲 |
| **Classifier** | `tasks/auto_aim/classifier.cpp` | `Armor& armor` | 修改 `armor.color/type` | 识别装甲颜色和编号 |
| **Solver** | `tasks/auto_aim/solver.cpp` | `Armor& armor (含像素坐标)` | 修改 `armor.xyz_in_world` | PnP求解，像素→3D坐标 |
| **Tracker** | `tasks/auto_aim/tracker.cpp` | `Armor[] + timestamp` | `std::list<Target>` | 多目标追踪 |
| **Target** | `tasks/auto_aim/target.cpp` | `Armor 观测值` | EKF预测位置 | 单目标EKF状态估计 |
| **Aimer** | `tasks/auto_aim/aimer.cpp` | `Target[] + 子弹速度` | `io::Command` | 瞄准计算 + 前馈 |
| **Shooter** | `tasks/auto_aim/shooter.cpp` | `Command + 目标状态` | `bool shoot` | 开火决策 |
| **Gimbal** | `io/gimbal.cpp` | `io::Command` | - | 发送控制指令给云台 |
| **CBoard** | `io/cboard.cpp` | `io::Command` | `Feedback` | USB串口与下位机通信 |

---

## 2. 完整数据流

### 2.1 单帧处理流程（详细时序）

```
【T=0ms】 Camera::getFrame()
   输入: -
   处理: USB传输图像 (0.5ms)
   输出: Frame{cv::Mat img, double ts=0}
   ├─ img: 640×480 BGR彩色图像
   └─ ts: 相机时间戳
   
   ↓
   
【T=0.5ms】 Detector::detect(img)
   输入: cv::Mat img
   处理: 
      ① YOLO推理 (8-10ms)
      ② 后处理 (2ms)
      ③ 几何检查
   输出: std::list<Armor> armors
   ├─ armor[0].points[4]: 像素坐标
   ├─ armor[0].color: RED/BLUE
   ├─ armor[0].type: BIG/SMALL
   └─ 共N个装甲 (N=检测到的数量)
   
   ↓
   
【T=10.5ms】 Classifier::classify(armor) ×N
   输入: Armor& armor (含ROI图案)
   处理: 神经网络分类 (1ms/个)
   输出: 修改 armor.number (装甲编号)
   
   ↓
   
【T=11.5ms】 Solver::solve(armor) ×N
   输入: Armor& armor (含像素坐标 pts[4])
   处理: 
      ① PnP求解 (1ms)
      ② 坐标变换: 相机→云台→世界 (1ms)
      ③ Yaw角优化 ±70° (1ms)
   输出: 修改 armor
      ├─ armor.xyz_in_world [x, y, z]  (3D位置)
      ├─ armor.ypr_in_world [yaw, pitch, roll]
      └─ armor.ypd_in_world [yaw, pitch, distance]
   
   ↓
   
【T=14.5ms】 Tracker::track(armors, ts)
   输入: std::list<Armor> armors, double ts
   处理:
      IF state == LOST:
         ① 新建Target
         ② solver.solve(armor) 一次 (已在上面做过)
         ③ 初始化EKF状态向量 (11维)
      ELSE:
         ① target.predict(ts)      → EKF预测
         ② 数据关联 (匹配检测到的装甲)
         ③ solver.solve(matched_armor) ×N
         ④ target.update(matched_armor) → EKF更新
   输出: std::list<Target> targets
   └─ Target包含:
      ├─ 位置 (x, y, z)
      ├─ 速度 (vx, vy, vz)
      ├─ yaw角 + ω_yaw (角速度)
      └─ 装甲半径r1, r2
   
   ↓
   
【T=20ms】 Aimer::aim(targets, ts, bullet_speed)
   输入: std::list<Target> targets, double ts, double bullet_speed (m/s)
   处理:
      ① 计算系统延迟: delay = 135ms (调试参数)
      ② 迭代 ×11:
         FOR i = 1 to 10:
            a) target.predict(ts + delay + i*t_fly)
            b) choose_aim_point(target)
            c) Trajectory::solve(bullet_speed, distance, height)
            d) 检查收敛性
      ③ 输出前馈信息
   输出: io::Command
      ├─ yaw:      目标偏航角 (弧度)
      ├─ pitch:    目标俯仰角 (弧度)
      ├─ yaw_vel:  角速度前馈 (rad/s)
      └─ yaw_acc:  加速度前馈 (rad/s²)
   
   ↓
   
【T=135ms】 Shooter::shoot(command, aimer, targets, gimbal_pos)
   输入: io::Command cmd, Aimer& aimer, Target[] targets, Quaternion gimbal_pos
   处理:
      ① 获取当前云台位置
      ② 计算动态容差:
         tolerance = 0.04 * (1 + 0.4 * max(|ω|-1.5, 0))
      ③ 判决:
         IF |ω| > 1.5:  // 小陀螺
            shoot = (cmd_diff < 18°) AND (yaw_err < tolerance)
         ELSE:           // 平移
            shoot = (cmd_diff < 2*tolerance) AND (yaw_err < tolerance)
   输出: bool shoot
   
   ↓
   
【T=140ms】 Gimbal::send(command, shoot)
   输入: io::Command cmd, bool shoot
   处理: 
      ① 打包数据
      ② CRC16校验
      ③ USB/CAN发送 (2ms)
   输出: 云台电机转动 + 摩擦轮旋转
   
   ↓
   
【T=142ms】 云台响应时间 (~100ms)
   
【T=242ms】 弹丸出膛
   
【T=246ms】 弹丸命中 (飞行时间 ~4ms)
```

### 2.2 时间延迟分解

```
【关键公式】
   最终预测时刻 = 当前时刻 + 系统延迟 + 子弹飞行时间
   ≈ T + 135ms + 4ms = T + 139ms

【系统延迟 135ms 的组成】
   ├─ 图像传输延迟:    0.5ms  (USB)
   ├─ 图像处理延迟:   13.5ms  (检测+分类+求解)
   ├─ 通信延迟:         2ms   (USB/CAN)
   ├─ 云台响应延迟:   100ms   (电机启动+加速)
   ├─ 调试参数补偿:    15ms   (通过拍视频调整)
   └─ 总计:          135ms   (经验值，需要实车校准)

【调整方法】
   拍240fps慢动作视频 → 对比相机捕获到射击之间的帧差
   → delay_ms = frame_diff / 240 * 1000
   → 与当前参数对比，微调
```

---

## 3. 感知层详解

### 3.1 Detector（装甲检测）

**【职责】** 从原始图像中找到所有装甲板

**【输入】** `cv::Mat img` (BGR彩色图像)  
**【输出】** `std::list<Armor> armors`

**【工作流程】**

```
原始图像 (BGR 640×480)
   ↓
【YOLO推理】
   ① YOLO v8 神经网络检测
   ② 输出: Bounding Box + 4个角点(keypoints)
   ③ 置信度过滤
   ↓
【构造Armor对象】
   ① 从YOLO的4个角点创建 Armor
   ② 记录像素坐标 armor.points[4]
   ③ 记录置信度
   ↓
【几何检查】
   ① 灯条长宽比检查
   ② 装甲长宽比检查 (0.5～3.0)
   ③ 两灯条夹角检查 (接近90°)
   ④ 面积检查 (不能太小)
   ↓
【输出】
   std::list<Armor> armors (已通过检查的装甲)
```

**【关键代码】**
```cpp
class Detector {
public:
    std::list<Armor> detect(const cv::Mat& img) {
        // YOLO推理
        auto detections = yolo_.inference(img);
        
        std::list<Armor> valid_armors;
        for (auto& det : detections) {
            Armor armor;
            armor.points = det.keypoints;  // 4个角点
            armor.confidence = det.conf;
            
            // 几何检查
            if (isValidGeometry(armor)) {
                valid_armors.push_back(armor);
            }
        }
        return valid_armors;
    }
};
```

### 3.2 Classifier（装甲分类）

**【职责】** 识别装甲的颜色和编号

**【输入】** `Armor& armor` (已包含ROI图案)  
**【输出】** 修改 `armor.color` 和 `armor.number`

**【流程】**
```
Armor ROI 图案 (85×55 图像)
   ↓
【神经网络推理】
   ① 提取Armor的ROI区域
   ② 分类网络推理 (输出: 类别概率)
   ③ 取最大概率的类别
   ↓
【结果】
   armor.color ∈ {RED, BLUE}
   armor.number ∈ {ONE, TWO, THREE, FOUR, FIVE, SENTRY, OUTPOST}
```

**【关键代码】**
```cpp
void Classifier::classify(Armor& armor) {
    // 截取装甲ROI
    cv::Mat roi = armor.pattern;  // 已在detector中截取
    
    // 神经网络推理
    auto output = classifier_net_.infer(roi);
    
    // 解析结果
    int color_id = argmax(output.color_logits);
    int number_id = argmax(output.number_logits);
    
    armor.color = (Color)color_id;
    armor.number = (ArmorName)number_id;
}
```

---

## 4. 估计层详解

### 4.1 Solver（PnP求解）

**【职责】** 将2D像素坐标变换为3D世界坐标

**【输入】** `Armor& armor` (含像素坐标 `armor.points[4]`)  
**【输出】** 修改 `armor.xyz_in_world` 和 `armor.ypr_in_world`

**【工作流程】**

```
【PnP求解】
   已知:
   • 装甲板的4个3D点 (世界坐标系)
   • 这4个点在图像上的投影 (像素坐标)
   • 相机内参矩阵K
   
   求:
   • 旋转矩阵R和平移向量t
   • 使得: p_i = K [R | t] P_i
   
   ↓
   
【坐标变换链】
   像素坐标 (u, v)
      ↓ (相机内参 K^-1)
   图像坐标 (x, y, 1)
      ↓ (PnP求解: R, t)
   相机坐标系 (X_c, Y_c, Z_c)
      ↓ (手眼标定矩阵)
   云台坐标系 (X_g, Y_g, Z_g)
      ↓ (IMU四元数)
   世界坐标系 (X_w, Y_w, Z_w)
   
   ↓
   
【Yaw角优化】
   PnP有 ±180° 歧义
   → 通过扫描 [-70°, 70°] 检查
   → 选择重投影误差最小的角度
```

**【关键代码】**
```cpp
void Solver::solve(Armor& armor) {
    // 装甲板的4个3D点 (CAD尺寸, 相对于装甲中心)
    std::vector<cv::Point3f> obj_pts = {
        {-0.1125, -0.0635, 0},  // 左上
        { 0.1125, -0.0635, 0},  // 右上
        { 0.1125,  0.0635, 0},  // 右下
        {-0.1125,  0.0635, 0}   // 左下
    };
    
    // 这些3D点在图像上的投影 (像素坐标)
    std::vector<cv::Point2f> img_pts = armor.points;
    
    // PnP求解
    cv::Mat rvec, tvec;
    cv::solvePnP(obj_pts, img_pts, camera_matrix_, dist_coeffs_, 
                 rvec, tvec, false, cv::SOLVEPNP_EPNP);
    
    // 旋转向量 → 旋转矩阵
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    
    // 转换到世界坐标系
    Eigen::Vector3d pos_camera(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
    Eigen::Vector3d pos_world = cam_to_world_R_ * pos_camera + cam_to_world_t_;
    
    armor.xyz_in_world = pos_world;
    
    // 提取欧拉角 (并优化Yaw)
    armor.ypr_in_world = rotationMatrixToEuler(R);
    optimizeYaw(armor);  // Yaw角优化
}
```

### 4.2 Tracker（目标追踪）

**【职责】** 维护多个Target，进行数据关联和状态估计

**【输入】** `std::list<Armor> armors`, `double t`  
**【输出】** `std::list<Target> targets`

**【工作流程】**

```
【状态机】
   ┌─────────┐
   │  LOST   │ ← 初始状态或丢失
   └────┬────┘
        │ (检测到装甲)
        ↓
   ┌─────────────────────┐
   │ set_target()        │
   │ ① solve(armor)      │
   │ ② 初始化 EKF       │
   │ ③ 转为 TRACKING    │
   └────┬────────────────┘
        ↓
   ┌──────────────┐
   │  TRACKING    │
   └────┬─────────┘
        │ (每帧)
        ↓
   ┌──────────────────────────┐
   │ update_target()          │
   │ ① predict(t)  → EKF预测  │
   │ ② 数据关联     → 找最近装甲│
   │ ③ solve(armor) → 求解3D  │
   │ ④ update(a)   → EKF更新  │
   └────┬─────────────────────┘
        │
        ├─ (检测失败) → LOST
        └─ (继续) → 重复
```

**【关键代码】**
```cpp
class Tracker {
public:
    std::list<Target> track(const std::list<Armor>& armors, double t) {
        if (state_ == LOST && !armors.empty()) {
            // 新建Target
            Target target;
            target.predict(t);
            state_ = TRACKING;
            targets_.push_back(target);
        }
        else if (state_ == TRACKING && !targets_.empty()) {
            // 更新现有Target
            for (auto& target : targets_) {
                // 预测
                target.predict(t);
                
                // 数据关联 (找最接近的装甲)
                Armor* best_armor = nullptr;
                double min_dist = 1e9;
                for (auto& armor : armors) {
                    double dist = calcDistance(target.predictedPos(), armor.xyz_in_world);
                    if (dist < min_dist) {
                        min_dist = dist;
                        best_armor = &armor;
                    }
                }
                
                // 更新
                if (best_armor && min_dist < MAX_ASSOC_DIST) {
                    target.update(*best_armor);
                }
            }
        }
        
        return targets_;
    }
};
```

### 4.3 Target（单目标EKF）

**【职责】** 存储单个目标的运动状态，提供EKF预测/更新

**【输入】** `Armor 观测值`  
**【输出】** EKF预测的位置/速度/角速度

**【11维状态向量】**
```
x = [x, y, z,              // 位置 (旋转中心)
     vx, vy, vz,           // 速度
     yaw,                  // 旋转角
     ω_yaw,                // 旋转角速度
     r1, r2,               // 装甲半径
     h]                    // 装甲高度差
```

**【关键代码】**
```cpp
class Target {
private:
    ExtendedKalmanFilter ekf_;  // 11×11 EKF
    
public:
    void predict(double dt) {
        // 状态转移矩阵 F (11×11)
        Eigen::MatrixXd F = Eigen::MatrixXd::Identity(11, 11);
        F(0, 3) = dt; F(1, 4) = dt; F(2, 5) = dt;  // 位置 += 速度*dt
        F(6, 7) = dt;                              // yaw += ω_yaw*dt
        
        // EKF预测步
        ekf_.predict(F, Q);  // Q: 过程噪声协方差
    }
    
    void update(const Armor& armor) {
        // 观测向量 z = [yaw, pitch, distance]
        Eigen::Vector3d z;
        z << armor.ypd_in_world[0],
             armor.ypd_in_world[1],
             armor.ypd_in_world[2];
        
        // 观测矩阵 H (3×11)
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 11);
        H(0, 6) = 1;  // 观测yaw
        H(1, 6) = 1;  // 观测pitch
        H(2, 0) = 1;  // 观测距离
        
        // EKF更新步
        ekf_.update(z, H, R);  // R: 观测噪声协方差
    }
};
```

---

## 5. 决策层详解

### 5.1 Aimer（瞄准决策）

**【职责】** 计算最优瞄准点，生成云台指令

**【输入】** `std::list<Target> targets`, `double t`, `double bullet_speed`  
**【输出】** `io::Command` (含yaw, pitch, 前馈)

**【核心算法】**

```
【第0步】计算预测窗口
   总延迟 delay = 135ms (配置参数)
   t_predict = t_now + delay

【第1～10步】迭代优化
   初始化: t_fly ≈ distance / bullet_speed
   
   FOR i = 1 to 10:
      ① target.predict(t_predict + i * t_fly)
         → 获得第 i 次迭代的目标位置
      
      ② choose_aim_point(target)
         → 选择要瞄准的装甲 (通常选最近的)
      
      ③ Trajectory(bullet_speed, distance, height)
         → 计算弹道，获得新的飞行时间
      
      ④ 检查收敛: |t_fly_new - t_fly_old| < 0.001?
         → 如果收敛则跳出

【第11步】输出指令
   cmd.yaw = final_yaw
   cmd.pitch = final_pitch
   cmd.yaw_vel = target.ω_yaw      (前馈角速度)
   cmd.yaw_acc = 0 (可选)
```

**【关键代码】**
```cpp
class Aimer {
public:
    io::Command aim(const std::list<Target>& targets, double t, double bullet_speed) {
        io::Command cmd;
        
        if (targets.empty()) return cmd;
        
        auto& target = targets.front();  // 追踪第一个目标
        
        // 参数
        double delay_offset = 0.135;  // 135ms系统延迟
        double t_predict = t + delay_offset;
        
        // 迭代优化
        double t_fly = 0.01;  // 初始飞行时间估计
        
        for (int iter = 0; iter < 10; iter++) {
            // 预测目标位置
            target.predict(t_predict + t_fly);
            Eigen::Vector3d pred_pos = target.getState().segment(0, 3);
            
            // 弹道计算
            double distance = pred_pos.norm();
            double height_diff = pred_pos.z();
            Trajectory traj;
            auto res = traj.solve(bullet_speed, distance, height_diff);
            
            // 更新飞行时间
            double t_fly_new = res.flight_time;
            if (std::abs(t_fly_new - t_fly) < 0.001) break;  // 收敛
            t_fly = t_fly_new;
        }
        
        // 输出指令
        cmd.yaw = atan2(pred_pos.y(), pred_pos.x());
        cmd.pitch = atan2(pred_pos.z(), 
                         std::hypot(pred_pos.x(), pred_pos.y()));
        cmd.yaw_vel = target.getState()(7);  // ω_yaw
        
        return cmd;
    }
};
```

### 5.2 Shooter（开火决策）

**【职责】** 判断是否开火

**【输入】** `io::Command cmd`, `Target[] targets`, `Quaternion gimbal_pos`  
**【输出】** `bool shoot`

**【开火判决逻辑】**

```
【获取信息】
   command_diff = |cmd.yaw - gimbal_current_yaw|
   yaw_error = ...
   ω_yaw = target.state[7]
   
【计算容差】
   IF ω_yaw > 1.5 rad/s:  // 小陀螺
      tolerance = base_tolerance * (1 + 0.4 * (ω_yaw - 1.5))
   ELSE:                   // 平移目标
      tolerance = base_tolerance

【判决】
   IF ω_yaw > 1.5:
      shoot = (command_diff < 18°) AND (yaw_error < tolerance)
   ELSE:
      shoot = (command_diff < 2*tolerance) AND (yaw_error < tolerance)
```

**【关键代码】**
```cpp
class Shooter {
public:
    bool shoot(const io::Command& cmd, const Target& target, 
               const Quaternion& gimbal_pos) {
        // 获取当前位置
        auto euler = quaternionToEuler(gimbal_pos);
        double gimbal_yaw = euler[0];
        
        // 计算误差
        double cmd_diff = std::abs(cmd.yaw - gimbal_yaw);
        if (cmd_diff > M_PI) cmd_diff = 2*M_PI - cmd_diff;
        
        // 计算容差
        double base_tol = 0.04;
        double omega_yaw = target.getState()(7);
        double tolerance = base_tol;
        
        if (omega_yaw > 1.5) {
            // 小陀螺: 容差随角速度增大
            tolerance = base_tol * (1 + 0.4 * (omega_yaw - 1.5));
            return cmd_diff < 0.314 && cmd_diff < tolerance;  // 18° ≈ 0.314 rad
        } else {
            // 平移: 容差更严格
            return cmd_diff < 2*tolerance && cmd_diff < tolerance;
        }
    }
};
```

---

## 6. 执行层详解

### 6.1 Gimbal（云台接口）

**【职责】** 发送控制指令给下位机

**【输入】** `io::Command cmd`  
**【输出】** 无 (通过CBoard发送数据)

**【关键代码】**
```cpp
class Gimbal {
public:
    void send(const io::Command& cmd) {
        // 打包数据
        uint8_t packet[32];
        packet[0] = 0xA5;  // SOF (Start of Frame)
        
        // 填充 yaw, pitch
        float* yaw_ptr = (float*)&packet[1];
        float* pitch_ptr = (float*)&packet[5];
        *yaw_ptr = (float)cmd.yaw;
        *pitch_ptr = (float)cmd.pitch;
        
        // 填充前馈信息
        float* yaw_vel_ptr = (float*)&packet[9];
        *yaw_vel_ptr = (float)cmd.yaw_vel;
        
        // 填充开火信号
        packet[13] = cmd.shoot ? 1 : 0;
        
        // CRC校验
        uint16_t crc = calculateCRC(packet, 14);
        packet[14] = crc & 0xFF;
        packet[15] = (crc >> 8) & 0xFF;
        
        // 发送
        cboard_.sendData(packet, 16);
    }
};
```

### 6.2 Camera（图像采集）

**【职责】** 采集实时图像和时间戳

**【输入】** 无  
**【输出】** `Frame{cv::Mat img, double ts}`

**【关键代码】**
```cpp
class Camera {
public:
    struct Frame {
        cv::Mat image;
        double timestamp;
    };
    
    Frame getFrame() {
        Frame frame;
        camera_.grab();  // 触发采集
        camera_.retrieve(frame.image);
        frame.timestamp = getSystemTime();  // 时间戳
        return frame;
    }
};
```

---

## 7. 硬件通信协议

### 7.1 协议概述

| 特性 | 说明 |
|------|------|
| **通信方式** | USB虚拟串口 或 CAN总线 |
| **波特率** | 115200 bps (USB) / 1Mbps (CAN) |
| **周期** | 50Hz (20ms) |
| **同步** | 帧同步 (SOF) + CRC |
| **超时** | 100ms无数据则超时 |

### 7.2 发送包格式 (上位机 → C板)

```
┌──────┬──────┬─────────────────────────────────┬──────┐
│ SOF  │ CTRL │         DATA (20 bytes)         │ CRC  │
│0xA5  │ 0x08 │                                 │ 2B   │
└──────┴──────┴─────────────────────────────────┴──────┘

【DATA字段详解】
Byte   0-3:  yaw (float, 弧度)
Byte   4-7:  pitch (float, 弧度)  
Byte   8-11: yaw_vel (float, rad/s, 前馈)
Byte  12-15: yaw_acc (float, rad/s², 前馈)
Byte     16: shoot (uint8, 0=不射 1=射)
Byte  17-19: 保留
```

**例子**: 发送yaw=0.5, pitch=0.2, shoot=1
```
A5 08 | 00 00 00 3F (yaw=0.5) | CD CC 4C 3E (pitch=0.2) | 
      00 00 00 00 | 00 00 00 00 | 01 00 00 00 | 
      [CRC16]
```

### 7.3 接收包格式 (C板 → 上位机)

```
┌──────┬──────┬─────────────────────────────────┬──────┐
│ SOF  │ CTRL │       DATA (16 bytes)           │ CRC  │
│0xA5  │ 0x09 │                                 │ 2B   │
└──────┴──────┴─────────────────────────────────┴──────┘

【DATA字段】
Byte   0-3:  gimbal_qw (float, 四元数w)
Byte   4-7:  gimbal_qx (float, 四元数x)
Byte   8-11: gimbal_qy (float, 四元数y)
Byte  12-15: gimbal_qz (float, 四元数z)
```

### 7.4 CRC16校验

```cpp
uint16_t calculateCRC16(const uint8_t* data, int len) {
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}
```

### 7.5 通信可靠性

```cpp
class CBoard {
private:
    bool sendWithRetry(const uint8_t* data, int len, int retry_times = 3) {
        for (int i = 0; i < retry_times; i++) {
            if (serial_.write(data, len) && waitAck(100)) {
                return true;
            }
        }
        return false;
    }
    
    bool receiveWithTimeout(uint8_t* data, int len, int timeout_ms = 50) {
        auto start = std::chrono::system_clock::now();
        while (true) {
            if (serial_.read(data, len)) return true;
            
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now() - start).count();
            if (elapsed > timeout_ms) return false;
        }
    }
};
```

---

## 8. 配置与参数

### 8.1 YAML配置文件

**路径**: `configs/standard.yaml`

```yaml
# ===== 相机参数 =====
camera:
  model: "HikRobot"
  fx: 320.5
  fy: 320.5
  cx: 320.0
  cy: 240.0
  k1: 0.1
  k2: -0.05

# ===== 装甲板尺寸 =====
armor:
  normal:
    width: 0.225    # m
    height: 0.127
  small:
    width: 0.135
    height: 0.055

# ===== 子弹参数 =====
bullet:
  speed: 30          # m/s
  drag_coefficient: 0.001

# ===== 控制参数 =====
controller:
  delay_offset: 0.015      # 15ms 系统延迟
  tolerance_base: 0.04     # 40mrad 开火容差
  max_gimbal_acc: 20       # rad/s²

# ===== EKF参数 =====
ekf:
  q_pos: 0.01
  q_vel: 0.01
  q_yaw: 0.01
  r_meas: 0.1
```

### 8.2 关键参数调整

| 参数 | 含义 | 调试方法 |
|------|------|--------|
| `delay_offset` | 系统延迟 | 拍240fps视频,计算帧差 |
| `tolerance_base` | 开火容差 | 离线测试,记录命中率 |
| `ekf.q_*` | 过程噪声 | 实车调试,观察追踪抖动 |
| `ekf.r_*` | 观测噪声 | 实车调试,观察预测误差 |

---

## 9. 工程实践

### 9.1 完整主循环代码

```cpp
int main() {
    // 初始化
    Camera camera;
    Detector detector;
    Classifier classifier;
    Solver solver;
    Tracker tracker;
    Aimer aimer;
    Shooter shooter;
    Gimbal gimbal;
    
    Config::load("configs/standard.yaml");
    
    // 主循环 50Hz
    while (true) {
        // 【感知】
        auto frame = camera.getFrame();
        auto armors = detector.detect(frame.image);
        
        // 【分类】
        for (auto& armor : armors) {
            classifier.classify(armor);
        }
        
        // 【求解】
        for (auto& armor : armors) {
            solver.solve(armor);
        }
        
        // 【追踪】
        auto targets = tracker.track(armors, frame.timestamp);
        
        // 【瞄准】
        auto cmd = aimer.aim(targets, frame.timestamp, 30.0);
        
        // 【开火】
        cmd.shoot = shooter.shoot(cmd, targets.front(), gimbal.getPose());
        
        // 【执行】
        gimbal.send(cmd);
        
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    
    return 0;
}
```

### 9.2 快速问题排查

| 问题 | 原因 | 解决方案 |
|------|------|--------|
| **识别率低** | 相机曝光、YOLO置信度低 | 调整camera.exposure / yolo.conf_threshold |
| **追踪漂移** | EKF参数不合适 | 增大ekf.r_meas (降低观测信任) |
| **瞄准不准** | delay_offset不准 | 拍视频校准延迟 |
| **击杀时间长** | 容差过严格 | 增大tolerance_base |
| **小陀螺打不中** | 云台跟不上 | 检查max_gimbal_acc参数 |

### 9.3 离线测试

```bash
# 编译
cmake -B build && make -C build/ -j$(nproc)

# 运行离线测试
./build/auto_aim_test --video assets/test.mp4

# 输出: 
# - Detected: 45 armors
# - Tracked: 3 targets
# - Hit rate: 82.5%
```

---

## 参考

**详细算法**: 参考 [auto_aim_learn.md](./auto_aim_learn.md)
- § 4: 轨迹规划理论
- § 6: 关键算法推导
- § 7: 代码执行流程

**源代码**: `tasks/auto_aim/` 和 `io/`

---

**文档更新**: 2026年9月26日

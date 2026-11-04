# EKF技术文档：SP25、南理、武科、中南

本工程以同济 SP25 为基石，融合南理 `rmcs_auto_aim_v2` 的像素观测方案。本文沿用“状态与观测 → 过程模型 → P/Q/R → 关联 → NIS → 恢复”的结构，解释各实现的实际算法及本工程的优化空间。

核对日期：2026-11-03。

## 0、统一符号与基本公式

### 0.1、状态、观测函数与雅可比的区别

| 符号 | 含义 | 本文中的典型例子 |
|---|---|---|
| $\mathbf x$ | 待估计的状态向量 | 车心、速度、角度、角速度、半径、高差 |
| $\mathbf z$ | 本帧实测观测向量 | 装甲位姿、球坐标，或灯条像素 |
| $f(\mathbf x,dt)$ | 状态转移函数 | 三轴匀速、匀角速 |
| $F=\partial f/\partial\mathbf x$ | 状态转移雅可比 | 位置对速度的导数为 $dt$ |
| $h(\mathbf x)$ | 将状态映射到观测空间的函数 | 三维灯条端点投影到像素 |
| $H=\partial h/\partial\mathbf x$ | 观测雅可比 | 像素对车心、角度、半径的导数 |
| $P$ | 状态估计误差协方差 | 车心、速度等状态的不确定度 |
| $Q$ | 一次预测引入的过程噪声协方差 | 未建模加速度导致的不确定度 |
| $R$ | 观测噪声协方差 | 像素误差或PnP位姿误差 |

11维或13维描述的是状态维数，不能称为“11维观测”。$H$ 是矩阵，$h(\mathbf x)$ 才是非线性观测函数。$P,Q,R$ 的对角元素是方差，不是标准差；不同观测空间中的数值不能直接比较大小。

### 0.2、EKF预测与更新

$$
\mathbf x^-_k=f(\mathbf x^+_{k-1},dt),\qquad
P^-_k=F_kP^+_{k-1}F_k^T+Q_k
$$

$$
\nu_k=\mathbf z_k-h(\mathbf x^-_k),\quad
S_k=H_kP^-_kH_k^T+R_k,\quad
K_k=P^-_kH_k^TS_k^{-1}
$$

$$
\mathbf x^+_k=\mathbf x^-_k\boxplus K_k\nu_k
$$

其中 $\boxplus$ 表示按状态定义修正：普通状态相加，角度需归一化，SO(3)姿态需通过旋转群运算注入误差。角度观测的残差也应取最短角差。

Joseph形式的协方差更新为：

$$
P^+=(I-KH)P^-(I-KH)^T+KRK^T
$$

标准NIS使用更新前创新与协方差：

$$
\operatorname{NIS}=\nu^TS^{-1}\nu
$$

四维观测对应的卡方95%、99%分位数约为9.488、13.277。只有噪声模型、关联与线性化等假设基本成立时，才能按该分布解释统计量；同帧多条NIS的最大值也不等于一次四维检验。

### 0.3、两种加速度噪声离散化

后文对一个位置—速度对使用以下简写：

$$
B_D(q,dt)=q\begin{bmatrix}dt^4/4&dt^3/2\\dt^3/2&dt^2\end{bmatrix}
$$

$$
B_C(q,dt)=q\begin{bmatrix}dt^3/3&dt^2/2\\dt^2/2&dt\end{bmatrix}
$$

$B_D$ 表示每个时间段内随机恒定加速度模型（DWNA）；$B_C$ 表示连续白噪声加速度模型（CWNA）的离散协方差。两者的 $q$ 单位与含义不同，不能直接复制参数做公平比较。

## 1、SP25

### 1.1、设计状态变量、观测函数h、雅可比H与观测Z

观测流程：

```text
检测四角点
→ 每块装甲板执行IPPE PnP
→ camera → gimbal → world坐标变换
→ 固定位置、按板pitch先验搜索最优装甲yaw
→ 得到世界系视线yaw / pitch / distance / 装甲yaw
→ 预测整车状态并关联预测装甲板ID
→ 每块观测板执行一次4维EKF更新
```

PnP利用已知物点及其图像对应点估计位姿。普通装甲默认pitch先验为+15°，前哨为−15°；平衡目标不执行这项yaw优化。搜索范围为云台yaw附近±70°，步长1°，实际成本为四角重投影距离之和。

状态为：

$$
\mathbf x=[c_x,v_x,c_y,v_y,c_z,v_z,a,\omega,r,l,h]^T\in\mathbb R^{11}
$$

$r$ 是参考板组的半径，不保证一定为短半径；$l=r_2-r_1$ 可以有符号，$h=z_2-z_1$ 是另一板组的高度差。对 $N=4$ 的奇数板使用 $r+l,c_z+h$；其他板使用 $r,c_z$。

$$
a_i=\operatorname{wrap}(a+i2\pi/N),\qquad
\mathbf p_i=(c_x-r_i\cos a_i,\ c_y-r_i\sin a_i,\ z_i)^T
$$

定义 $g(\mathbf p)=[\operatorname{atan2}(p_y,p_x),\operatorname{atan2}(p_z,\sqrt{p_x^2+p_y^2}),\|\mathbf p\|]^T$，则：

$$
h_i(\mathbf x)=[g(\mathbf p_i)^T,a_i]^T,\qquad
\mathbf z=[yaw_{LOS},pitch_{LOS},d,yaw_{armor}]^T
$$

这里的视线yaw与装甲朝向yaw是不同物理量。H为4×11解析链式雅可比：

$$
H_i=\begin{bmatrix}J_g&0\\0&1\end{bmatrix}
\frac{\partial[\mathbf p_i^T,a_i]^T}{\partial\mathbf x}
$$

例如 $\partial p_x/\partial a=r_i\sin a_i$，$\partial p_y/\partial a=-r_i\cos a_i$；对r的导数为 $(-\cos a_i,-\sin a_i,0)^T$，奇数板对l有同样导数、对h的z导数为1。

依据：[Tracker观测入口](<D:/1RM_vision/sp25/同济-2025/tasks/auto_aim/tracker.cpp:266>)、[PnP与坐标变换](<D:/1RM_vision/sp25/同济-2025/tasks/auto_aim/solver.cpp:55>)、[h与H](<D:/1RM_vision/sp25/同济-2025/tasks/auto_aim/target.cpp:187>)。

### 1.2、设计过程模型F

三轴匀速、车体匀角速，半径和高差保持不变：

$$
\mathbf c'=\mathbf c+\mathbf vdt,\qquad a'=\operatorname{wrap}(a+\omega dt)
$$

F为11×11单位阵加 $F_{0,1}=F_{2,3}=F_{4,5}=F_{6,7}=dt$（索引从0开始）。预测和状态修正后都归一化a；观测残差的第0、1、3维取最短角差。

前哨特殊处理：更新数大于10、半径合法且 $|\omega|>2$ 时，在预测前将角速度设为同符号的2.51 rad/s。

### 1.3、设计P、R、Q

普通四板车初始化：

$$
P_0=\operatorname{diag}(1,64,1,64,1,64,0.4,100,1,1,1)
$$

初始半径0.2m，中心由板位置加半径方向反推；速度、角速度、半径差、高差为0。平衡为2板；前哨为3板、半径0.2765m，半径P为 $10^{-4}$、l/h的P为0，z速度P为81。

$$
R=\operatorname{diag}\left(0.004,0.004,
\log(|\delta|+1)+1,\frac{\log(|d|+1)}{200}+0.09\right)
$$

$\delta=\operatorname{wrap}(yaw_{armor}-yaw_{LOS})$。距离通道方差随斜视角变化；装甲yaw方差随距离变化。该R是经验几何调度，没有将检测像素协方差通过PnP传播，也没有使用检测置信度。

Q中三组位置—速度块用 $B_D(100,dt)$，角度—角速块用 $B_D(400,dt)$，r/l/h噪声为0。前哨对应参数为10和0.1。P更新使用Joseph形式。

依据：[F与Q](<D:/1RM_vision/sp25/同济-2025/tasks/auto_aim/target.cpp:75>)、[初始化分型](<D:/1RM_vision/sp25/同济-2025/tasks/auto_aim/tracker.cpp:231>)、[R构造](<D:/1RM_vision/sp25/同济-2025/tasks/auto_aim/target.cpp:192>)。

### 1.4、门控与数据关联

将预测板按距离排序，从最近三板中最小化：

$$
|\operatorname{wrap}(yaw_{armor}^{obs}-yaw_{armor}^{pred})|
+|\operatorname{wrap}(yaw_{LOS}^{obs}-yaw_{LOS}^{pred})|
$$

没有关联误差上限，也没有更新前马氏距离门。所有同name/type观测板依次更新一个EKF，不强制不同观测占用不同板ID。后面的观测使用前一次更新后的状态重新关联。

代码固定遍历前三板，但平衡模型只有两板，存在访问第三元素的静态越界路径；这是源码风险，本文未运行触发。

### 1.5、NIS与拒绝观测

代码在完成更新后才计算：

$$
r=z-h(x^+),\qquad S=HP^+H^T+R,\qquad nis=r^TS^{-1}r
$$

因此它是后验残差统计，不是标准更新前NIS，也不会拒绝当前观测。阈值为0.711，与注释中的“四维95%”不一致。普通 `track()` 在最近最多100次更新中累计失败达到40次时置lost，不要求先填满窗口；另一个全向感知重载没有同样检查。

代码中名为NEES的量使用本次状态修正差，不含真值，也不能当作标准状态估计NEES解释。依据：[滤波与统计实现](<D:/1RM_vision/sp25/同济-2025/tools/extended_kalman_filter.cpp:44>)。

### 1.6、时间、收敛与重建

dt来自图像时间戳差；Tracker在帧间隔大于0.1s时置lost，可在同帧重新建模。无同类板时先预测，再进入temp_lost；超过配置丢失帧数才lost。发散主要检查r及r+l是否都在0.05～0.5m。普通目标更新数>3且未发散后标收敛，前哨要求>10；不是协方差收敛判据。

## 2、南理rmcs_auto_aim_v2

### 2.1、设计状态变量、h/H/Z

普通机器人观测流程：

```text
检测完整装甲四角点
→ 首次用Robust PnP初始化中心和朝向
→ 预测整车状态
→ 生成4板、8灯条的3D端点
→ world/odom → camera → 像素投影
→ 完整板作锚、分配灯条ID
→ 每灯条一次4维像素EKF更新
```

$$
\mathbf x=[c_x,c_y,c_z,v_x,v_y,v_z,a,\omega,r_f,r_l,h_l]^T
$$

令 $u=(\cos a,\sin a,0)^T$，$v=(-\sin a,\cos a,0)^T$，板中心为：

$$
p_0=c+r_fu,\quad p_1=c+r_lv+h_le_z,\quad
p_2=c-r_fu,\quad p_3=c-r_lv+h_le_z
$$

每板按板姿态、宽度和0.056m灯条高度生成上下端点，普通板pitch固定+15°。状态a表示中心到参考板方向，与SP25同名a的几何方向约定不能直接混用。

$$
z_j=[u_{top},v_{top},u_{bottom},v_{bottom}]^T
$$

$h_j(x)$ 为端点经相机外参和带畸变投影得到的四维像素。H为4×11：中心、半径、高差导数解析；角度的三维端点导数采用 $10^{-4}$ 中心差分，再乘相机旋转和针孔投影导数。速度、角速度对应H列为0，通过P中的互协方差被间接修正。

投影h含畸变，H的投影部分没有畸变导数；非零畸变下属于近似线性化。依据：[状态与几何](<D:/1RM_vision/南理/rmcs_auto_aim_v2/src/module/tracker/model/robot.cpp:47>)、[观测投影](<D:/1RM_vision/南理/rmcs_auto_aim_v2/src/module/tracker/model/robot.cpp:123>)、[H实现](<D:/1RM_vision/南理/rmcs_auto_aim_v2/src/module/tracker/model/robot.cpp:234>)。

### 2.2、设计过程模型F

中心三轴匀速，a匀角速；F为单位阵加四个位置—速度、角度—角速的dt项。Q在每次predict时直接相加，不随dt缩放。

### 2.3、设计P、R、Q

南理原始普通模型的初始P为：

$$
P_0=\operatorname{diag}(64,64,64,64,64,64,0.4,100,0.01,0.01,0.01)
$$

该P不同于hhh。初始化选择水平像素宽度最大的板PnP，参考半径均设0.2m，高差、速度、角速度为0；比较a与a+π的完整可见板数，选择更多的一支。

$$
Q=\operatorname{diag}(10^{-4},10^{-4},10^{-8},0.2,0.2,10^{-8},10^{-3},1,10^{-8},10^{-8},10^{-8})
$$

$$
R=40I_4\quad(\mathrm{pixel}^2)
$$

40是方差，对应单分量标准差约6.32px。固定Q按预测调用次数累加，因此不同帧率下每秒注入的噪声不同。P更新采用Joseph形式并强制对称化；每次更新后重新投影，供下一灯条使用。

依据：[P/Q/R](<D:/1RM_vision/南理/rmcs_auto_aim_v2/src/module/tracker/model/robot.cpp:71>)、[预测](<D:/1RM_vision/南理/rmcs_auto_aim_v2/src/module/tracker/model/robot.cpp:418>)、[逐灯条更新](<D:/1RM_vision/南理/rmcs_auto_aim_v2/src/module/tracker/model/robot.cpp:444>)。

### 2.4、门控与数据关联

朝向相机夹角小于配置70°的板两灯可见；70°～90°只标一灯可见；≥90°不可见。遍历检测板与完整可见预测板，选择四角平方像素误差最小的一对作anchor，无最大误差门。

观测板拆成灯条并加入额外灯条，按上端点x排序，从anchor向两侧以±1模8传播ID，再按序更新。接口支持散灯；当前检测主链的普通散灯提取被注释，不能把接口能力视为已启用观测。

依据：[关联实现](<D:/1RM_vision/南理/rmcs_auto_aim_v2/src/module/tracker/model/robot.cpp:525>)。

### 2.5、NIS与拒绝观测

普通RobotModel没有计算NIS或使用卡方门。H、S逆矩阵或后验状态数值非法时跳过该灯条；半径夹在[0.1,0.4]m、高差夹在[−0.2,0.2]m，vx/vy夹在[−10,10]m/s。错误关联但数值合法的观测仍可进入更新。

### 2.6、时间、收敛与重建

按设备类别保存模型；只有本帧看到该类目标时才predict/correct，dt覆盖距该类上次seen的间隔。超过1.5s未seen则销毁。收敛要求correct流程数≥10、Pxx/Pyy≤1、初始化墙钟时间≥0.1s。状态/速度/角速或位置协方差越界则销毁，后续重新初始化。

南理另有前哨专用5维模型 $[c_x,c_y,c_z,\omega,\theta]^T$：由PnP生成中心xyz和参考板方向观测，通过固定参考板与三板高差补偿处理换板。它不是这里的11维像素RobotModel。依据：[前哨模型](<D:/1RM_vision/南理/rmcs_auto_aim_v2/src/module/tracker/model/outpost.cpp:208>)。

## 3、武科

### 3.1、设计状态变量、h/H/Z

实际采用13维误差状态迭代滤波器 `ErrorStateEKF`，没有调用同目录中的AdaptiveEKF或UKF。

```text
检测灯条端点
→ 转成灯条方向、中点、长度
→ 预测并投影候选板灯条
→ 一对一关联检测板与预测板，补充散灯
→ 堆叠同帧全部观测
→ 迭代误差状态更新
```

$$
x=[c_x,v_x,c_y,v_y,c_z,v_z,\rho_z,\omega,\log r_1,\log r_2,h,\rho_y,\rho_x]^T
$$

$\rho$ 是SO(3)旋转向量，不是三个欧拉角。普通车支持整车倾斜；前哨和基地仅取绕z旋转。半径采用指数映射保持正值。

每灯条观测为：

$$
z=\left[\operatorname{atan2}(u_t-u_b,v_t-v_b),
\frac{u_t+u_b}{2},\frac{v_t+v_b}{2},
\sqrt{(u_t-u_b)^2+(v_t-v_b)^2}\right]^T
$$

状态→三维板姿态→灯条端点→带畸变像素投影→上述四维变换，构成h。当前 `MEASURE_NORMALIZED=false`，中点和长度单位为像素，方向单位为弧度；角残差归一化。

完整板贡献两根灯条8维；恰好只匹配一块板时，还通过IPPE构造左右灯中心的相机深度差1维辅助观测。该辅助量与端点来自同一组角点，存在相关性，不能视为独立新增传感器信息。

依据：[状态与模型](<D:/1RM_vision/武科/awakening-main/src/tasks/auto_aim/armor_track/motion_model.hpp:20>)、[观测定义](<D:/1RM_vision/武科/awakening-main/src/tasks/auto_aim/armor_track/motion_model.hpp:269>)、[实际update入口](<D:/1RM_vision/武科/awakening-main/src/tasks/auto_aim/armor_track/armor_target.cpp:345>)。

### 3.2、设计过程模型F

中心匀速，整车姿态右乘 $\operatorname{Exp}([0,0,\omega dt]^T)$。误差注入采用 $R^+=R\operatorname{Exp}(\delta\rho)$，状态差采用 $\operatorname{Log}(R_{nom}^TR_{value})$。

F对“误差注入→状态预测→状态差”使用Ceres Jet自动微分。实际 `update_multi` 的H采用误差空间中心差分，步长 $10^{-6}$，每个扰动重新注入姿态并投影；不能将它概括成所有雅可比均自动微分。

### 3.3、设计P、R、Q

P0按状态分组：位置=1、速度=10、旋转向量=1、log半径及高差=1、角速度=100，全对角。普通目标初始半径0.26m，PnP板姿态乘板—车体逆变换得到中心与整车姿态。

车体系加速度噪声先变换到odom：

$$
A=R_{car}\operatorname{diag}(q_x,q_y,q_z)R_{car}^T
$$

$$
Q_{pp}=dt^4A/4,\quad Q_{pv}=dt^3A/2,\quad Q_{vv}=dt^2A
$$

绕z角度—角速使用DWNA块；倾斜姿态另有随机游走。log半径噪声为 $q_r/r^2$、高度为 $q_h$，这些几何噪声代码中不乘dt。

设实测灯长为L：

$$
R=\tfrac12\operatorname{diag}(\sigma_\alpha^2,(k_cL)^2,(k_cL)^2,(k_lL)^2)
$$

例如 `wit.yaml`：$q_{xyz}=[30,30,1]$、$q_{yaw}=30$、$k_c=0.2$、$k_l=0.3$、$\sigma_\alpha=0.1$，迭代5轮；其他车辆配置不同。这是几何/尺度调度，不是按创新统计在线估计Q/R。

同帧观测堆叠、R取块对角；迭代重算h/H，通过LDLT求K，最终用Joseph更新并对称化P。实际迭代式及SO(3)注入后的协方差处理仍需独立审查，不能直接认定为标准IEKF实现。

依据：[Q](<D:/1RM_vision/武科/awakening-main/src/tasks/auto_aim/armor_track/armor_target.cpp:280>)、[R](<D:/1RM_vision/武科/awakening-main/src/tasks/auto_aim/armor_track/armor_target.cpp:371>)、[迭代核心](<D:/1RM_vision/武科/awakening-main/3rdparty/KalmanHyLib/error_state_extended_kalman_filter.hpp:333>)、[配置例](<D:/1RM_vision/武科/awakening-main/config/wit.yaml:91>)。

### 3.4、门控与数据关联

候选板ID按正对相机程度取前三个；代价为中心像素距离、四边方向差及周长相对差的加权和。一对一贪心匹配，未成熟和成熟目标用不同门限。散灯另检查灯长比例、方向和端点位置门；要求已经匹配到完整板，不能仅靠散灯独立重获。

这与hhh的anchor后横向递推不同，但它的几何cost也没有使用P。依据：[整板关联](<D:/1RM_vision/武科/awakening-main/src/tasks/auto_aim/armor_track/armor_target.cpp:508>)、[散灯关联](<D:/1RM_vision/武科/awakening-main/src/tasks/auto_aim/armor_track/armor_target.cpp:642>)。

### 3.5、NIS与拒绝观测

实际更新链未见NIS卡方门；主要通过关联前的几何门拒绝。预测中限制半径范围，部分状态越界时直接设零。源码还存在静态风险：`DIFFZ_N=1`，深度差R回调却访问 `r(1,1)`；R回调引用捕获局部角点数据也有生命周期风险。这些尚未运行复现，不能当成可直接搬用的成熟实现。

### 3.6、时间、收敛与恢复

按实际秒数判定暂丢超时，TEMP_LOST期间继续预测；当前目标暂丢时维护备用target，备用进入TRACKING后替换当前；当前恢复则放弃备用。普通换板通过固定板ID表达，不手工交换半径或重建车心。

这一备用假设机制适合参考；更复杂的13维状态与迭代次数是否有益，取决于目标倾斜程度、观测质量和算力。依据：[双target恢复](<D:/1RM_vision/武科/awakening-main/src/tasks/auto_aim/armor_track/armor_tracker.cpp:22>)。

## 4、FYT2024

### 4.1、设计状态变量、h/H/Z

```text
检测/PnP并可选优化板yaw
→ 按图像时间TF至odom
→ 预测整车状态
→ 同ID位置最近关联及位置/yaw双门
→ 每帧最多一次装甲xyz+yaw更新
```

实际为10维，不能据构造函数中的临时9维向量判断模型维数：

$$
x=[c_x,v_x,c_y,v_y,c_z,v_z,\psi,\omega,r,dz_c]^T
$$

另一半径 `another_r`、板组高度差在滤波器外维护。

$$
h(x)=[c_x-r\cos\psi,c_y-r\sin\psi,c_z+dz_c,\psi]^T,\quad
z=[x_a,y_a,z_a,yaw_a]^T
$$

F和H由Ceres Jet自动微分生成。$c_z$ 与 $dz_c$ 在单板观测中只以和出现，不能仅凭一个瞬间观测将二者独立确定。

依据：[状态和h](<D:/1RM_vision/FYT2024_vision-main/FYT2024_vision-main/rm_auto_aim/armor_solver/include/armor_solver/motion_model.hpp:32>)、[Tracker](<D:/1RM_vision/FYT2024_vision-main/FYT2024_vision-main/rm_auto_aim/armor_solver/src/armor_tracker.cpp:78>)。

### 4.2、设计过程模型F

普通车匀速+匀角速；前哨切换为只旋转，平移速度设0、中心保持。图像时间戳差提供dt。普通模型F包含位置—速度及yaw—角速的dt项。

### 4.3、设计P、R、Q

构造EKF时 $P_0=I_{10}$；初始化半径0.26m，速度和高差设0。`setState`只写x，不重新初始化P。

Q按DWNA块构造，r和高差对角用 $qdt^4/4$。配置平移参数均0.05，yaw为1，r为80；高差参数未配置时用节点默认800。代码中z及yaw交叉项存在误用x噪声参数的问题。

$$
R=\operatorname{diag}(k_x|x_a|,k_y|y_a|,k_z|z_a|,r_\psi)
$$

配置系数为 $4\times10^{-4},4\times10^{-4},9\times10^{-4},5\times10^{-3}$。这只是世界坐标绝对值缩放；某坐标为0时该方向R趋零，不能视为可靠的距离/观测质量模型。

P采用简化 $(I-KH)P^-$ 更新。另有重要实现限制：predict更新 `P_pri` 后没有回写 `P_post`；连续漏测时状态逐帧前推，而P未同步逐帧累积传播。

依据：[Q/R构造](<D:/1RM_vision/FYT2024_vision-main/FYT2024_vision-main/rm_auto_aim/armor_solver/src/armor_solver_node.cpp:53>)、[EKF实现](<D:/1RM_vision/FYT2024_vision-main/FYT2024_vision-main/rm_utils/include/rm_utils/math/extended_kalman_filter.hpp:69>)、[参数](<D:/1RM_vision/FYT2024_vision-main/FYT2024_vision-main/rm_bringup/config/node_params/armor_solver_params.yaml:8>)。

### 4.4、门控与数据关联

仅同ID候选，取预测板三维位置最近者，再要求位置差<0.5m、yaw差<1rad（配置例）。只有一块同ID且yaw差大时进入换板：更新参考yaw、交换内外两半径、处理高度组；若位置仍严重不符则重设中心并清平移速度。

换板只修改x而不变换P；该分支也未将matched设true，所以仍可能进入TEMP_LOST或在DETECTING阶段LOST。依据：[换板实现](<D:/1RM_vision/FYT2024_vision-main/FYT2024_vision-main/rm_auto_aim/armor_solver/src/armor_tracker.cpp:206>)。

### 4.5、NIS与拒绝观测

没有NIS门，以位置/yaw硬门决定是否更新。yaw通过连续解包处理跨π，但解包函数修改历史yaw，候选遍历也调用它，存在候选顺序影响。

### 4.6、时间与恢复

暂丢继续预测，匹配恢复后返回TRACKING；超时转LOST。失联帧阈值由当前dt反推 `lost_time/dt`，不是累计真实失联时间。初始化、换板、半径限幅只修改x，旧P可能与新几何状态不一致。

可参考自动微分接口与显式换板事件，不应直接复用其P传播、R公式或重置逻辑。


## 5、本工程hhh

### 5.1、设计状态变量、h/H/Z

本工程继承SP25工程框架和Solver，普通目标滤波核心改为南理式11维像素模型：

$$
x=[c_x,c_y,c_z,v_x,v_y,v_z,a,\omega,r_f,r_l,h_l]^T
$$

```text
检测四角点
→ 首次优先用原Solver世界系板位姿初始化，失败回退Robust PnP
→ 按图像时间预测整车状态与P
→ 四板/八灯条投影
→ anchor关联及左右交换检查
→ 整帧粗像素门
→ 灯条ID排序传播
→ 每灯条标准NIS门及EKF顺序更新
→ 连续无接受更新达到阈值时PnP重初始化
```

实际 $z_j=[u_t,v_t,u_b,v_b]^T$。虽然跟踪默认仍运行Solver PnP，但correct只消费像素角点，不消费板三维位姿；实际调用的独立散灯列表为空。

H的结构与南理相同：部分解析、角度三维几何中心差分、针孔投影导数。h含畸变而H未含畸变导数；零畸变仿真不受该差异影响，真机非零畸变需核验。

依据：[初始化与实际输入](C:/Users/hhh/Desktop/hhh/tasks/auto_aim/target.cpp:29)、[Tracker](C:/Users/hhh/Desktop/hhh/tasks/auto_aim/tracker.cpp:259)、[H](C:/Users/hhh/Desktop/hhh/tasks/auto_aim/rmcs_ekf/rmcs_ekf.cpp:1119)。

### 5.2、设计过程模型F

中心匀速、a匀角速，F四个dt项。只接受有限正dt；状态未来查询通过副本外推，不修改原滤波器时间或P。普通滤波预测正确累计传播P。

### 5.3、设计P、R、Q

以standard3/sentry配置为例：

$$
P_0=\operatorname{diag}(0.04,0.04,0.01,4,4,0.01,0.1,4,0.001,0.001,0.001)
$$

$$
R=40I_4
$$

默认CWNA：x/y块用 $B_C(1,dt)$，z块用 $B_C(10^{-4},dt)$，角度块用 $B_C(10,dt)$；半径、高差采用 $qdt$ 随机游走。还支持DWNA和DIAG_RATE作对照，但后者是对角噪声乘dt，与南理原始“每预测固定加Q”不同。

初始中心来自水平板法线偏移0.2m，速度/角速度为0，两半径为0.2m，高差为0。P比南理更紧。更新使用Joseph形式加对称化，保持已有数值稳定性结构。

依据：[配置](C:/Users/hhh/Desktop/hhh/configs/standard3.yaml:39)、[P/Q构造](C:/Users/hhh/Desktop/hhh/tasks/auto_aim/rmcs_ekf/rmcs_ekf.cpp:454)、[Q传播](C:/Users/hhh/Desktop/hhh/tasks/auto_aim/rmcs_ekf/rmcs_ekf.cpp:614)。

### 5.4、门控与数据关联

相对南理增加左右灯条交换尝试，以适应图像左右与机器人灯条ID方向差异。选最小四角误差anchor，再检查：

$$
e=\sqrt{\tfrac14\sum_{k=1}^4\|p_k^{obs}-p_k^{pred}\|^2},\qquad
T=\max(40\mathrm{px},0.5\,w_{pred})
$$

若 $e>T$ 则整帧提前返回。e是四角欧氏误差的RMS，不是简单误差均值。通过后按上端点x排序，递推灯条ID并逐条更新。

该粗门不依赖P，且在NIS之前执行。只要有同name/type检测，外层Tracker仍可视为found，即使EKF全拒绝；因此tracking状态不能证明EKF本帧有有效更新。

依据：[correct与粗门](C:/Users/hhh/Desktop/hhh/tasks/auto_aim/rmcs_ekf/rmcs_ekf.cpp:716)。

### 5.5、NIS与拒绝观测

每根灯条用更新前 $\nu,S$ 计算标准四维NIS。配置预热10次correct后启用门控：监控阈值9.488，拒绝阈值13.277；连续5次NIS拒绝，在位置、速度、角度和角速的P对角各加0.01。

三个语义细节：

1. 粗门提前返回时NIS记0，表示未执行检验，不能解释为残差良好。
2. `update_count_`在通过anchor阶段后增加，即使所有灯条均拒绝；它不是成功融合帧数。
3. 帧NIS输出是本帧有效尝试的最大单灯条NIS；另有均值字段。最大值超过阈值的频率不能直接按单次四维卡方失败率解释。

依据：[NIS与更新](C:/Users/hhh/Desktop/hhh/tasks/auto_aim/rmcs_ekf/rmcs_ekf.cpp:943)。

### 5.6、时间、收敛与重建

有检测但无anchor、粗门拒绝或所有灯条拒绝时，增加 `no_update_frames`；无检测时不增加该计数，由Tracker的temp_lost处理。连续10帧无接受更新，重新调用当前单帧Robust PnP，重设全部运动状态、P及NIS预热；速度和角速度再次为0。

收敛还检查最小correct数、连续无更新数、位置/速度/角速协方差和初始化时长。数值非法、状态范围越界或长期无更新可触发发散。计数与参数均应按实际加载配置解释。

源码可形成以下失配循环：运动突变→旧速度外推→粗门拒绝→无法接受修正→单帧重建清速度→运动目标再次失配。是否为某段数据的实际原因，需要同输入回放确认。

实验worktree另有两帧拟合、第三帧确认的窗口恢复，其状态与验证结果独立于本文主工程。依据：[重初始化](C:/Users/hhh/Desktop/hhh/tasks/auto_aim/rmcs_ekf/rmcs_ekf.cpp:918)。

## 6、横向对比与算法含义

| 维度 | SP25 | 南理普通车 | 武科 | FYT2024 | hhh |
|---|---|---|---|---|---|
| 状态维数 | 11 | 11 | 13 | 10 | 11 |
| 观测 | 球坐标+板yaw | 灯条端点像素 | 灯条角度/中点/长度，另有单板深度差 | 板xyz+yaw | 灯条端点像素 |
| PnP作用 | 每次观测 | 初始化 | 初始化及单板辅助 | 每次观测 | 初始化，跟踪结果不入EKF |
| F/H | 解析 | 解析+差分，H未含畸变导数 | F自动微分，实际H误差空间差分 | 自动微分 | 同南理结构 |
| Q主要形式 | DWNA | 固定每预测对角Q | 车体系各向异性DWNA | DWNA，有参数误用 | 默认CWNA |
| R调度 | 斜视角/距离 | 固定40I | 灯长及物理分量 | 世界坐标绝对值 | 固定40I配置例 |
| 标准更新前NIS | 否，后验统计 | 无 | 无 | 无 | 有 |
| 多观测更新 | 逐板 | 逐灯条 | 堆叠迭代 | 单板 | 逐灯条 |
| P数值更新 | Joseph | Joseph+对称 | Joseph+对称 | 简化形式 | Joseph+对称 |
| 关联 | 最近三板角差 | anchor+灯条排序 | 一对一几何代价 | 最近板+位置/yaw门 | anchor+粗门+灯条NIS |
| 恢复特点 | lost后重建 | 删除后重建 | 暂丢保留+备用假设 | 换板改状态 | 10帧拒绝后重建 |

像素域方案避免将每帧PnP点估计直接当测量，但仍受标定、角点噪声、可观测性和关联影响。PnP域方案结构较直观，但PnP深度/姿态噪声及相关性必须体现在R中。两者不能只凭状态维数或滤波器名称判断精度高低。


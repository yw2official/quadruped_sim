# Quadruped Sim — Codebase Quick Reference

> 本文档供 Claude 快速查阅，避免每次重新读取全部源文件。  
> 项目：两个 rh1 双足机器人首尾硬拼接 → 四足机器人，MuJoCo 仿真 + ESP32 双机控制。

---

## 1. 目录结构

```
/home/yanwang/quadruped_sim/
├── rh1/urdf/rh1.urdf              # 双足机器人 URDF（764 行）
├── SF_serveo_control/src/
│   ├── main.cpp                   # 主控固件（797 行）Device1
│   ├── bipedal_data.h             # 数据结构（132 行）
│   └── config.h
├── SF_serveo_control_device2/src/
│   ├── device2.cpp                # 从控固件（153 行）Device2
│   └── config.h
└── quadruped_sim/
    ├── convert_and_run.py         # MuJoCo 仿真主脚本（256 行）
    ├── quadruped_final.xml        # 自动生成的四足 XML（~200 行）
    └── meshes/visual|collision/   # STL 网格（11 个零件）
```

---

## 2. 硬件架构

```
Device1 (ESP32-S3, main.cpp) — 主控
├── PPM 遥控接收（8 通道，Pin 40）
│   ch1=转向  ch2=前后速度  ch3=腿高  ch4=横滚
│   ch5=模式切换  ch6=速度缩放  ch7=步态类型
├── IMU MPU6050 → pitch/roll/yaw
├── PCA9685 I2C → 8 路舵机（4 腿×2 关节）
├── Serial2 → 后轮 BLDC（M0左后, M1右后）
└── CAN (TX=35, RX=41) → Device2

Device2 (ESP32-S3, device2.cpp) — 从控
├── CAN 接收 ID=0x01 → 解压 16-bit → FOC 电机目标
└── DengFOC + MT6701 SSI 编码器 → 前轮（M0左前, M1右前）
```

---

## 3. 机器人模型 (rh1 URDF)

### 关节树（单侧双足，两份组成四足）
```
base_link
├── joint1  → Link1  (revolute, 左前大腿旋转)
├── joint2  → Link2  (revolute, 左前大腿延伸)
├── joint4  → Link4  (continuous, 左前轮)
├── joint5  → Link5  (revolute, 左前5-bar平行杆)
├── joint6  → Link6  (revolute, 左前小腿)
├── joint7  → Link7  (revolute, 右前大腿旋转)
├── joint8  → Link8  (revolute, 右前大腿延伸)
├── joint10 → Link10 (continuous, 右前轮)
├── joint11 → Link11 (revolute, 右前5-bar平行杆)
└── joint12 → Link12 (revolute, 右前小腿)
```
> Link6_3 / Link12_9（5-bar 末端约束链接）已在 URDF 中注释掉，仅 Gazebo plugin 中保留 joint6_3 / joint12_9 定义。

### 5-bar Linkage 参数（固件与仿真一致）
| 参数 | 值 (mm) | 含义 |
|------|---------|------|
| L1 | 60  | 大腿主杆 |
| L2 | 100 | 小腿主杆 |
| L3 | 100 | 平行返回杆 |
| L4 | 60  | 平行大腿杆 |
| L5 | 40  | 偏置距离 |

---

## 4. 逆运动学 (5-bar IK)

**文件：** `main.cpp:278-417` / `convert_and_run.py` `solve_5bar_ik()`

**输入：** 足端目标 `(x, y)` mm  
**输出：** `(α, β)` 舵机角度（度）

### 核心公式
```
# 前向链 (joint1 → Link1 → Link2)
aRight = 2*x*L1
bRight = 2*y*L1
cRight = x²+y² + L1²-L2²
α = 2*atan2(b + √(a²+b²-c²), a+c)   # 取+号解

# 后向链 (joint5 → Link5 → Link6)
# x4 = x - L1*cos(α),  y4 = y - L1*sin(α)
d = 2*x4*L3,  e = 2*y4*L3
f = x4²+y4² + L3²-L4²
β = 2*atan2(e - √(d²+e²-f²), d+f)   # 取-号解
```

### 舵机角度映射
```cpp
// 左腿：  servoAngle = 90  + ik_alpha_deg
// 右腿：  servoAngle = 270 - ik_alpha_deg
// 后腿：  x3 = -x3,  x4 = -x4  (x 取反后再算 IK)
```

---

## 5. 步态生成 (Trot Gait)

**文件：** `main.cpp:134-242` / `convert_and_run.py` `trot_gait()`

### 关键参数
| 参数 | 值 | 说明 |
|------|----|------|
| Ts   | 1.0 s | 步态周期 |
| faai | 0.5   | 占空比（摆相 50%）|
| h    | 60 mm | 抬腿高度 |
| height | 120 mm | 站立腿长 |
| xf   | -45 mm | 前伸量 |
| xs   | 45 mm  | 后缩量 |

### Cycloid 摆相轨迹公式
```
摆相条件：t_mod > faai*Ts
σ = 2π*(t - faai*Ts) / ((1-faai)*Ts)   ← 注意：Bug B3 当前分母写错为 faai*Ts
z_ep = h*(1 - cos(σ)) / 2               # 垂直抬起
x_ep = (xf-xs)*((σ - sin(σ))/(2π)) + xs # 前后移动
```

### 步态相位分配

**motionMode=0（双足模式，对角腿同步）**
```
t_mod < faai*Ts:  左前+右后 摆相，右前+左后 支撑
t_mod ≥ faai*Ts:  右前+左后 摆相，左前+右后 支撑
```

**motionMode=1（四足 trot，同侧腿同步）**
```
t_mod < faai*Ts:  左前+左后 抬起，右前+右后 支撑
t_mod ≥ faai*Ts:  右前+右后 抬起，左前+左后 支撑
```

---

## 6. 控制模式

### 任务1：轮足整体前进（Wheel-Leg Forward）
```
controlmode = 0
├── 四轮提供驱动力
│   rear: motor = 0.35*(fwd + 0.32*speed_fb) ± steering/3
│   front (CAN): 同 rear，转向反号
├── 腿高根据 ch3 调节（range 70-150 mm）
└── steadyState=1 时：PID pitch/roll 自动补偿腿高
```

### 任务2：原地交叉抬腿（Cross-Leg Lifting In Place）
```
controlmode = 1,  motionMode = 1,  steadyState = 1
├── 同侧腿同步抬起（四足 trot，xf=xs=0 不前进）
├── IMU PID 维持身体水平
└── ch6 调节步态速度缩放
```

---

## 7. MuJoCo 仿真 (convert_and_run.py)

### 模型构建流程 `prepare_model()` (行 12-88)
1. 读取 `rh1.urdf`，URDF→MuJoCo XML 转换
2. 复制为两份：前缀 `F_`（前组）和 `R_`（后组）
3. R 组在 X 轴偏移 `body_offset_x = 0.07 m`
4. 添加 4 个等式约束（5-bar 闭链）：
   - `eq_F_L2_L6`, `eq_F_L8_L12`, `eq_R_L2_L6`, `eq_R_L8_L12`
5. 添加 8 个位置舵机执行器 + 4 个速度轮执行器

### 等式约束锚点 `fix_anchors_dynamically()` (行 181-211)
```python
LOCAL_OFFSETS = {
    'L2_L6':  np.array([-0.091798, -0.039663, -0.0097823]),  # 来自 URDF joint6_3
    'L8_L12': np.array([-0.091798,  0.039663,  0.0024116]),  # 来自 URDF joint12_9
}
```
- **作用：** 防止 freejoint 撕裂，从 body 全局变换动态计算局部锚点

### 控制时序
```
t ∈ [0,  2s]: 蹲起过渡（smooth startup，progress = 0.5*(1-cos(π*t/2))）
t ∈ [2,  3s]: 静立保持
t ∈ [3s, ∞ ): 步态控制（xf=-45, xs=45）
```

### 执行器映射
```
位置舵机 (kp=30): F_joint1, F_joint5, F_joint7, F_joint11,
                   R_joint1, R_joint5, R_joint7, R_joint11
速度轮   (kv=5):  F_joint4, F_joint10, R_joint4, R_joint10
```

---

## 8. 主控固件关键参数

| 参数 | 值 | 位置 |
|------|----|------|
| `servo_off[8]` | {3,5,-5,-7,3,-5,-8,8} | main.cpp:38，机械零偏（度）|
| PID pitch | P=0.2, I=0.02, D=0.001 | main.cpp 俯仰稳定 |
| PID roll  | P=0.1, I=0.01, D=0.001 | main.cpp 横滚稳定 |
| 低通滤波 α | 0.02 | PPM 信号平滑 |
| PPM 范围 | 1000-2000 µs | 死区约 1430-1500 µs |

---

## 9. 已知 Bug 列表

| # | 文件:行号 | 问题描述 | 严重度 |
|---|-----------|---------|--------|
| **B1** | main.cpp:276 | `alphaBackLefToAngle` → 应为 `alphaBackLeftToAngle`（拼写） | 低 |
| **B2** | main.cpp:351-376 | 后腿 IK 使用了全局变量 `alphaRight`，会覆盖前腿计算结果 | **高** |
| **B3** | main.cpp:193 | 摆相 σ 分母写为 `faai*Ts`，应为 `(1-faai)*Ts`，导致轨迹时间压缩 | 中 |
| **B4** | main.cpp:738,748 | `sendmotor1target = sendmotor1target`（无意义赋值，方向校正逻辑缺失）| 低 |

---

## 10. 文件速查索引

| 文件 | 行数 | 核心职责 | 重要函数/类 |
|------|------|---------|------------|
| `convert_and_run.py` | 256 | MuJoCo 仿真主循环 | `prepare_model()`, `fix_anchors_dynamically()`, `QuadrupedController` |
| `main.cpp` | 797 | 主控固件 | `trot()`, `inverseKinematics()`, `can_control()` |
| `device2.cpp` | 153 | 从控固件（前轮 FOC）| CAN 接收 → `MITController()` |
| `rh1.urdf` | 764 | 双足机器人定义 | joint1-12，Link1-12 |
| `quadruped_final.xml` | ~200 | 自动生成四足 XML | — |
| `bipedal_data.h` | 132 | 共享数据结构 | `IKParam`, `MotorTarget` |
| `pid.h` | 37 | PID 控制器 | `PIDController` |

---

## 11. 注意事项 / 调试提示

1. **仿真 vs 固件 IK 一致性：** `convert_and_run.py` 的 `solve_5bar_ik()` 是固件 `inverseKinematics()` 的 Python 版，修 bug 时需同步两处。
2. **前后腿偏移：** 仿真中前后组间距 0.07m，对应实物需匹配安装尺寸。
3. **等式约束稳定性：** 若 MuJoCo 仿真崩溃/爆炸，首先检查 `fix_anchors_dynamically()` 的锚点偏移量是否与最新 URDF 关节位置一致。
4. **舵机零位校准：** 实物调试时 `servo_off[8]` 需重新标定，仿真中无需此项。
5. **Link6_3/Link12_9：** 若要在 MuJoCo 中精确模拟 5-bar 闭链，需恢复这两个链接并添加对应等式约束；当前方案用 `equality weld` 近似。

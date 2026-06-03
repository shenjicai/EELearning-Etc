# Clark 变换与 Park 变换 Simulink 仿真模型

基于三相坐标变换理论，搭建完整的 Simulink 仿真模型，包含 Clark/Park/反 Park/反 Clark 四组变换，并通过误差分析和理论推导验证模型正确性。

---

## 1. 数学推导

### 1.1 Clark 变换（等幅值变换）

在三相平衡系统中（ia + ib + ic = 0），将三相静止坐标系映射到两相静止坐标系。

**核心约束**：变换前后电流矢量的幅值保持不变。

设三相电流为：
- ia = Im · cos(ωt)
- ib = Im · cos(ωt − 2π/3)
- ic = Im · cos(ωt + 2π/3)

等幅值 Clark 变换矩阵：

```
[iα]   [ 1      0        0    ] [ia]
[iβ] = [ 0   1/√3    -1/√3   ] [ib]
[i0]   [1/3   1/3      1/3    ] [ic]
```

在三相对称条件下（ia + ib + ic = 0），零序分量 i0 = 0，简化为：
- **iα = ia**
- **iβ = (ib − ic) / √3**
- **i0 = (ia + ib + ic) / 3**

**推导验证**：代入三相电流，利用和差化积公式：
- iα = Im · cos(ωt)
- iβ = Im · sin(ωt)

合成矢量幅值 √(iα² + iβ²) = Im，验证等幅值特性。

### 1.2 Park 变换

将两相静止坐标系（αβ）映射到两相旋转坐标系（dq），旋转坐标系以角速度 ω 与电流矢量同步旋转。

变换矩阵：

```
[id]   [ cosθ   sinθ ] [iα]
[iq] = [-sinθ   cosθ ] [iβ]
```

展开得：
- **id = iα · cosθ + iβ · sinθ**
- **iq = −iα · sinθ + iβ · cosθ**

**稳态分析**：当 θ = ωt 时（与电流矢量同步旋转）：
- id = Im, iq = 0

在稳态下，id 和 iq 均为**直流量**——这是 Park 变换的核心价值：将交流量转换为直流量，便于控制器设计。

### 1.3 反 Park 变换

```
[iα]   [ cosθ  −sinθ ] [id]
[iβ] = [ sinθ   cosθ ] [iq]
```

### 1.4 反 Clark 变换（等幅值）

```
ia = iα
ib = −0.5·iα + (√3/2)·iβ
ic = −0.5·iα − (√3/2)·iβ
```

---

## 2. 关键理论问题

### 2.1 Park 变换作用于导数时为什么出现耦合项

对 \( $X_{dq} = T(\theta) \cdot X_{\alpha\beta}$ \) 求导：

$
\frac{dX_{dq}}{dt} = T(\theta) \cdot \frac{dX_{\alpha\beta}}{dt} \;+\; \frac{dT(\theta)}{dt} \cdot X_{\alpha\beta}
$

**第二项** \( $\dot{T} \cdot X_{\alpha\beta} $\) 就是耦合项的来源。因为 Park 变换矩阵 T(θ) 本身是时变的（θ = ωt），对其求导时链式法则产生额外项：

$
\frac{dT}{dt} = \frac{\partial T}{\partial \theta} \cdot \frac{d\theta}{dt} = \omega \cdot \begin{bmatrix} -\sin\theta & \cos\theta \\ -\cos\theta & -\sin\theta \end{bmatrix}
$

**物理含义**：耦合项反映了坐标系本身在旋转。设想站在旋转转盘上观察静止矢量——即使矢量本身不动，在旋转坐标系看来它也在变化。\( $\dot{T} \cdot X $\) 捕捉的正是这种相对运动效应。

在电机控制中，这正是 dq 电压方程中出现旋转电动势耦合项的原因：

$
u_d = R i_d + L\frac{di_d}{dt} - \omega L i_q, \quad u_q = R i_q + L\frac{di_q}{dt} + \omega L i_d
$

其中 ±ωLi 项是前馈解耦控制需要处理的对象。

> **核心**：耦合项来自导数链式法则 \( $\frac{d}{dt}[T(\theta)X] = T\dot{X} + \dot{T}X$ \)，其中 \( $\dot{T} \propto \omega$ \)。它不是被变换系统本身的特性，而是旋转观测者带来的"视在"效应。

### 2.2 PI 控制器跟踪交流信号为什么存在稳态误差

**内模原理（Internal Model Principle）**：控制器必须在其传递函数的分母中包含参考信号的模型，才能实现零稳态误差。

PI 控制器传递函数：

$
G_c(s) = K_p + \frac{K_i}{s} = \frac{K_p s + K_i}{s}
$

- 分母中的 1/s 使控制器在 **DC（ω = 0）** 处增益无穷大 → 对直流信号零稳态误差
- 对频率为 ω₀ 的正弦信号：\( $|G_c(j\omega_0)| = \sqrt{K_p^2 + (K_i/\omega_0)^2} < \infty $\)，增益有限 → 存在稳态误差

**解决路径**：

| 方案 | 坐标系 | 控制信号 | 控制器 |
|------|--------|----------|--------|
| abc/αβ 静止坐标 | 交流（50Hz） | 正弦波 | PR 或 PI（有误差）|
| dq 旋转坐标 | 直流 | 常数 | PI（零误差）|

**这正是 Clark/Park 变换的核心动机之一**：将交流转为直流，就能用简单的 PI 控制器实现零稳态误差跟踪。

> **核心**：PI 只有 1/s，内含的是阶跃信号模型（DC），对正弦交流信号增益有限 → 必有稳态误差。消除误差需要 PR 控制器的谐振项 1/(s²+ω₀²)，或通过 Park 变换将交流变为直流再用 PI。

### 2.3 Park 变换中 θ 的含义

θ = ωt 是旋转坐标系 d 轴与静止坐标系 α 轴之间的**瞬时电角度**。

- d 轴以角速度 ω 旋转，与 α 轴的夹角随时间线性增长
- 当 ω 等于电流矢量的角频率时，dq 坐标系与电流矢量**同步旋转**
- 此时在 dq 坐标系中"看"电流矢量是静止的 → 交流量变为直流量

### 2.4 同步旋转如何将交流变为直流

在三相系统中，电流矢量以角速度 ω 匀速旋转。Park 变换相当于**将观测者放到一个同样以 ω 旋转的平台上**去观察这个矢量。在静止坐标系中，矢量投影到 α/β 轴上是随时间正弦变化的分量；在同步旋转的 dq 坐标系中，矢量相对观测者静止，投影到 d/q 轴上的分量就不再随时间变化——变成了直流量。

---

## 3. Simulink 模型结构

### 3.1 整体信号流

```
三相电流源 → Clark_Transform → Park_Transform → InvPark_Transform → InvClark_Transform
                  ↑                   ↑                ↑                    ↑
              原始电流           Iα,Iβ,I0          Id,Iq            重建的Iα,Iβ        重建的三相
```

### 3.2 子系统实现

| 子系统 | 输入 | 输出 | 核心模块 |
|--------|------|------|---------|
| Clark_Transform | Ia, Ib, Ic | Iα, Iβ, I0 | Subtract, Gain, Add |
| Park_Transform | Iα, Iβ, θ | Id, Iq | Trigonometric Function, Product, Add |
| InvPark_Transform | Id, Iq, θ | Iα_inv, Iβ_inv | Trigonometric Function, Product, Add, Gain(−1) |
| InvClark_Transform | Iα_inv, Iβ_inv | Ia_inv, Ib_inv, Ic_inv | Gain(−0.5, √3/2), Add |

### 3.3 角度信号生成

使用 `Ramp` 模块（斜率 = 2πf）生成线性增长的 θ，通过 `mod 2π` 限制在 [0, 2π) 区间，避免数值溢出。

### 3.4 信号路由（From/Goto）

所有共用信号使用 From/Goto 模块连接，标签包括：
Ia_sig, Ib_sig, Ic_sig, Theta_sig, Ialpha_sig, Ibeta_sig, I0_sig, Id_sig, Iq_sig, Ialpha_inv_sig, Ibeta_inv_sig, Ia_inv_sig, Ib_inv_sig, Ic_inv_sig

### 3.5 模块配色与原理标注

| 子系统 | 颜色 | 标注公式 |
|--------|------|---------|
| Clark | 绿色 `[0.7 1.0 0.7]` | iα=ia  iβ=(ib−ic)/√3  i0=(ia+ib+ic)/3 |
| Park | 橙色 `[1.0 0.8 0.5]` | id=iα·cosθ+iβ·sinθ  iq=−iα·sinθ+iβ·cosθ |
| InvPark | 紫色 `[0.9 0.7 1.0]` | iα=id·cosθ−iq·sinθ  iβ=id·sinθ+iq·cosθ |
| InvClark | 粉色 `[1.0 0.7 0.8]` | ia=iα  ib=−0.5iα+√3/2·iβ  ic=−0.5iα−√3/2·iβ |

---

## 4. 关键问题与解决方案

### 问题 1：`-batch` 模式下 To Workspace 模块失效

**根因**：To Workspace 模块依赖 JVM/图形系统，在 `-batch` 模式下被禁用后无法正常工作。

**解决方案**：使用 `Outport` + `Mux` 替代 `To Workspace`。
- 将需要导出的信号通过 Mux 合并后接入 Outport
- `sim` 命令返回的 `Simulink.SimulationOutput` 对象通过 `out.yout.getElement(n).Values.Data` 提取数据

### 问题 2：Park 变换输出与理论预期偏差 90°

**根因**：Simulink `Sine Wave` 模块输出公式为 `Amplitude * sin(Frequency * time + Phase)`，而电机控制理论中通常以 `cos(ωt)` 作为 A 相参考。设 A 相为 sin(ωt)，Clark 变换后 iα = Im·sin(ωt), iβ = −Im·cos(ωt)，代入 Park 变换得 id=0, iq=−Im。

**解决方案**：将三相电流的初始相位统一偏移 +π/2：
- A 相：π/2 → 输出 sin(ωt + π/2) = cos(ωt)
- B 相：π/2 − 2π/3
- C 相：π/2 + 2π/3

---

## 5. 仿真结果

### 参数设置

| 参数 | 值 |
|------|-----|
| 频率 f | 50 Hz |
| 幅值 A | 10 A |
| 仿真步长 Ts | 1 μs |
| 仿真时长 Tstop | 0.1 s |

### 5.1 误差评估

| 指标 | A相 | B相 | C相 |
|------|-----|-----|-----|
| 误差 RMS | 8.6e−16 A | 3.0e−05 A | 3.0e−05 A |
| 误差最大值 | 3.6e−15 A | 4.2e−05 A | 4.2e−05 A |

- A 相误差在机器精度级别（~1e−15），因为反 Clark 中 ia = iα 是直接连线，无运算累积
- B/C 相误差 ~3e−05 A，源于多步浮点运算的累积及 1μs 仿真步长的离散化误差

### 5.2 Park 稳态输出

| 指标 | 值 | 理论值 |
|------|-----|--------|
| Id 平均值 | 9.999986 A | 10 A |
| Iq 平均值 | 0.000507 A | 0 A |
| Id 标准差 | 1.0e−05 A | 0 A |
| Iq 标准差 | 1.1e−04 A | 0 A |

- Id/Iq 已稳定为直流量，标准差极小
- 微小偏差来自离散仿真步长的数值误差，可通过减小 Ts 进一步降低

---

## 6. 文件清单

| 文件 | 说明 |
|------|------|
| `Clark_Park_Transform.slx` | Simulink 模型文件（含 Clark/Park/反 Park/反 Clark 四个子系统） |
| `clark_park_model.m` | 主脚本：一键生成模型 + 自动运行仿真 + 生成 8 子图分析 + 打印误差统计 |
| `Clark_Park_Analysis.png` | 8 子图分析结果（三相电流、Clark/Park/反变换输出、原始 vs 重建对比、三相重建误差） |
| `README.md` | 本文件：数学推导、理论问题解答、模型结构、问题排查与仿真分析 |

---

## 7. 使用方式

在 MATLAB 中运行：

```matlab
cd('..\Clark_Park_Transform_Project');
clark_park_model;
```

脚本将自动：
1. 创建/覆盖 Simulink 模型
2. 运行 0.1s 仿真
3. 生成 8 子图对比分析
4. 打印误差统计
5. 将分析图保存为 PNG
6. 将所有数据变量写入 Workspace

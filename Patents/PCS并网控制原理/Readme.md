# PCS 并网控制原理 — 从物理本质到 Simulink 建模

## 1. 核心物理原理

### 1.1 本质：两个电压源 + 一个电感

PCS（储能变流器）是一个**可控交流电压源**，电网是**固定交流电压源**。两者通过等效电感 L（滤波电感/并网电抗器/线路电感）连接。

```
L × di/dt = U_pcs − U_grid − R×i
```

PCS 不直接控制电流，而是通过调节自身输出电压的**幅值和相位**，在电感上建立压差，将并网电流"逼"成目标状态。

### 1.2 四象限运行

电流相位在 360° 范围内连续可调，实现有功和无功的独立控制：

| 电流相位 (相对电网电压) | 有功功率 | 无功功率 |
|---|---|---|
| 0°（同相） | P > 0（输出/放电） | — |
| 180°（反相） | P < 0（吸收/充电） | — |
| 超前 0°~90° | 输出有功 | 输出无功（容性） |
| 滞后 0°~90° | 输出有功 | 吸收无功（感性） |
| 超前 90°~180° | 吸收有功 | 输出无功（容性） |
| 滞后 90°~180° | 吸收有功 | 吸收无功（感性） |

---

## 2. 模型概览

项目包含两个独立模型，从不同角度验证同一物理原理：

| 模型 | 目录 | 说明 |
|---|---|---|
| **单相模型** | `单相仿真/` | 手动搭建 RL 积分器，演示核心微分方程 |
| **三相模型** | `三相仿真/` | 使用 Simscape Power Systems 原生块，工业级建模 |

### 2.1 单相模型

使用基础 Simulink 块直接实现 `L×di/dt = Vdiff − R×i`：

- Grid / PCS：Sine Wave 电压源（Frequency 单位 rad/s，50Hz = 314.16）
- Vdiff = PCS − Grid（Subtract 块）
- Sum_RL = Vdiff − R×i（扣除电阻压降）
- Gain(1/L) → Integrator → i(t)
- Gain_R 负反馈提供阻尼，τ = L/R = 50ms
- Grid_90（Phase=+π/2，同幅同频）与电流相乘得无功分量

### 2.2 三相模型

使用 Simscape Power Systems 原生块：

- **Grid / PCS**：Three-Phase Source（Yg，NonIdealSource=off）
  - Voltage 参数 = **线电压 RMS**（理论计算需 `/√3` 转换为相电压）
- **VI_Meas**：Three-Phase V-I Measurement（提取 Vabc、Iabc 信号）
  - 电流正方向 = 电压端子 → 电流端子（模型中 = Grid → PCS 方向）
- **RLC**：Three-Phase Series RLC Branch（BranchType=RL）
- **powergui**：sps 仿真必需
- 信号路由全部使用 **Goto/From** 块（TagVisibility=local），零交叉连线
- Q 计算使用 Transport Delay（5ms = T/4）+ Gain(−1) = +90° 超前

**三相核心优势**：P_total 和 Q_total 为**常数**（0% 100Hz 纹波），因三相 120° 相位偏移完美互消。

---

## 3. 关键参数

| 参数 | 数值 | 说明 |
|---|---|---|
| 电网电压 | 220 Vrms（线电压） | 三相模型/√3 ≈ 127V 相电压 |
| PCS 电压 | 220~230 Vrms（可调） | 可控幅值与相位 |
| 频率 | 50 Hz | 工频交流 |
| 等效电感 L | 5 mH | ωL = 1.571 Ω @ 50Hz |
| 等效电阻 R | 0.1 Ω | τ = L/R = 50ms |
| 阻抗角 ∠Z | 86.4° | atan(ωL/R)，电感特性极强 |
| 求解器 | ode4（固定步长） | 步长 1e-5s, StopTime 0.5s |

---

## 4. 调试历程（单相模型）

| 版本 | 做法 | 问题 | 根因 |
|---|---|---|---|
| v1 | 理想积分器，无电阻 | P/Q 发散至百万级 | 无阻尼，直流偏置无法衰减 |
| v2 | Transfer Fcn `1/(Ls+R)` | P/Q 偏大 6 倍，Q 全负 | Frequency 填 50（当作 Hz，实际是 rad/s），Transport Delay 接线错误 |
| v3/v4 | 物理积分器 + 负反馈 + ode4 | 与理论完全吻合 | Frequency=314.16 rad/s，Grid_90 直出正交信号，稳态段直接取均值 |

---

## 5. Q（无功功率）符号约定深度分析

### 5.1 为什么单相模型 Q 不需要翻转？

单相模型用 `Grid_90`（独立 Sine Wave，Phase=+90°）与电流相乘：

```
Q = mean(Grid_90 × I)
  = mean(V_peak·cos(ωt) × I_peak·sin(ωt+θ))
  = +V_rms × I_rms × sin(θ)
```

理论：`Q_th = −imag(V × conj(I)) = +V_rms × I_rms × sin(θ)`

两者公式天然一致，**Q 的符号由电流相角 θ 直接决定，与电流测量方向无关**。

### 5.2 为什么三相模型 Q 需要 Transport Delay + Gain(−1)？

| 移相方式 | 公式 | Q 有无自带负号 |
|---|---|---|
| 独立 Sine Wave (Phase=+90°) | `mean(V_90 × I) = +Vrms·Irms·sin(θ)` | 无 |
| Transport Delay (5ms) | `mean(V(t−T/4) × I) = −Vrms·Irms·sin(θ)` | **有**（−cos 引入） |
| Delay + Gain(−1) | `mean(−V(t−T/4) × I) = +Vrms·Irms·sin(θ)` | 无（与 Sine Wave 等效） |

Delay 路径自带负号（V(t−T/4) = −cos(ωt)），加 Gain(−1) 后消除，恢复为 +90° 超前。

### 5.3 P 和 Q 对电流方向反转的不同响应

VI_Meas 测量电流方向 = Grid → PCS，理论电流方向 = PCS → Grid（差 180°）：

| | cos(θ+180°) = −cosθ | sin(θ+180°) = −sinθ |
|---|---|---|
| **P** = Vrms·Irms·cos(θ) | **直接取反** → 需手动翻转 | — |
| **Q** (Delay 方式) = **−**Vrms·Irms·sin(θ) | — | (−1)×(−sinθ) = sinθ → **负负得正，不需翻转** |
| **Q** (Gain(−1) 方式) = **+**Vrms·Irms·sin(θ) | — | sin(θ+180°) = −sinθ → **需手动翻转** |

**结论**：P 和 Q 对电流方向同样敏感。旧拓扑中 Q 不需翻转是"公式自带负号"与"电流方向反转"互相抵消的巧合。

### 5.4 为什么绝大多数工况 Q > 0（容性）？

阻抗角 ∠Z = atan(ωL/R) = 86.4°，电感特性极强。Vdiff = Vpcs − Vgrid 经电感后电流被大幅偏转：

- 当 |Vpcs| = |Vgrid| 时，电流角度 θ ≈ φ/2 + 3.6°，感性窗口仅 φ ∈ (−7.3°, 0°)
- 要实现 Q < 0（感性），需 Vpcs 幅值高于 Vgrid（如 230V）且相位接近同相

---

## 6. 三相模型布局设计

信号处理采用严格列对齐，每种模块独占一列，列间距 30~40px：

| 列 | x | 功能 | 模块 |
|---|---|---|---|
| C1 | 640 | 信号分离 | Demux_V, Demux_I |
| C2 | 710 | Goto 发射 | Goto_Va/b/c, Goto_Ia/b/c |
| C3 | 790 | From 接收 | From_Va/b/c（→ 移相链） |
| C4 | 860 | 移相延迟 | Transport Delay (5ms) |
| C5 | 950 | 符号修正 | Sign90 (Gain=−1) |
| C6 | 1030 | Goto 移相 | Goto_Va90/b90/c90 |
| C7 | 1110 | From 输入 | From_V/I → Prod (每相2个) |
| C8 | 1170 | 乘积 | Prod_Pa/b/c, Prod_Qa/b/c |
| C9 | 1250 | Goto 功率 | Goto_Pa/b/c, Goto_Qa/b/c |
| C10 | 1330 | From 求和 | From_P/Q → Sum |
| C11 | 1390 | 三相求和 | Sum_P, Sum_Q, Mux_I |
| C12 | 1480 | Goto 总线 | Goto_Psum, Goto_Qsum |
| C13 | 1570 | From 输出 | → To Workspace / LPF |
| C14 | 1630 | 数据存储 | P_total, Q_total, I_abc_out, LPF |
| C15 | 1700 | 符号统一 | Gain_Pinv (Gain=−1) |
| C16 | 1770 | 实时显示 | Disp_P, Disp_Q, Disp_I |
| C17 | 1880 | From 观测 | → Scopes |
| C18 | 1930 | 波形 | Scope_V_I_A相, Scope_三相电流, Scope_总功率 |

Goto Tag 清单（全部 `TagVisibility='local'`）：
`Va,Vb,Vc`, `Ia,Ib,Ic`, `Va90,Vb90,Vc90`, `Pa,Pb,Pc`, `Qa,Qb,Qc`, `Psum`, `Qsum`

---

## 7. 目录结构

```
PCS并网控制原理/
├── Readme.md                          ← 本文件（综合文档）
├── 视频/
│   └── 逆变器并网时PCS有功无功和功率流动控制.mp4
├── 单相仿真/
│   ├── pcs_run_v3.m                   ← 单相模型构建+批量仿真脚本
│   └── PCS_Grid_Connection_v3.slx     ← 单相 Simulink 模型（脚本生成）
└── 三相仿真/
    ├── pcs_run_3phase.m               ← 三相模型构建+批量仿真脚本（sps 原生块）
    ├── PCS_3Phase_Grid.slx            ← 三相 Simulink 模型（脚本自动生成）
    └── PCS_3Phase_Grid_Manual.slx     ← 三相 Simulink 模型（手动修改版本）
```

---

## 8. 快速运行

### 单相模型
```matlab
cd('单相仿真');
run('pcs_run_v3.m');
```

### 三相模型
```matlab
cd('三相仿真');
run('pcs_run_3phase.m');
```

两个脚本均自动完成：清理旧模型 → 创建新模型 → 批量仿真 9 种工况 → 打印 P/Q/I 对比表 → 更新 Display 名称。

仿真结果汇总表保存在模型 `Description` 属性中（File > Model Properties > Description）。

---

## 9. 仿真结果（三相模型，9 工况）

| PCS 相位 | P (W) | Q (var) | I_rms (A) | 状态 |
|---|---|---|---|---|
| 0° (同幅) | 0.0 | 0.0 | 0.00 | 无功率流动 |
| 10° | +5299 | −806 | 14.07 | 输出有功，吸收无功(感性) |
| 45° | +21128 | −10370 | 61.76 | 输出有功，吸收无功(感性) |
| 90° | +28734 | −32642 | 114.12 | 输出有功，吸收无功(感性) |
| 135° | +18365 | −53769 | 149.11 | 输出有功，吸收无功(感性) |
| 180° | −3907 | −61376 | 161.40 | 吸收有功，吸收无功(感性) |
| −45° | −22272 | −7607 | 61.76 | 吸收有功，吸收无功(感性) |
| −90° | −32642 | −28734 | 114.12 | 吸收有功，吸收无功(感性) |
| 230V, 0° | +89 | +1395 | 3.67 | 输出有功，输出无功(容性) |

> **注**：Q 符号当前为 +90° 超前（Gain=−1）测量方式，需翻转一次才能匹配理论约定。详见第 5 节。

---

## 10. 核心结论

> **控制 PCS 输出电压的幅值与相位 → 在电感上建立压差 → 控制并网电流的大小与相位 → 独立、连续地控制有功 P 和无功 Q 的流动方向。**

无论 PQ 控制、VSG 下垂控制还是构网型控制，最终在物理层都在做同一件事。

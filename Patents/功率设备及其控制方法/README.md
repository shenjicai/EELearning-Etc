# 功率设备及其控制方法 — 项目目录

> 基于专利 CN 121984334 A（阳光电源股份有限公司）的 MATLAB/Simulink 仿真实现

---

## 目录结构

```
功率设备及其控制方法/
│
├── README.md                          ← 本文件（目录导航）
├── 项目综合说明.md                     ← ★ 核心文档：原理、实现、校准、讨论
├── 功率设备及其控制方法.md              ← 专利全文（权利要求 + 说明书）
├── 功率设备及其控制方法.pdf             ← 专利原始 PDF
│
├── 仿真代码/                           ← MATLAB/Simulink 仿真文件
│   ├── anti_condensation_control.slx  # Simulink 闭环控制模型
│   ├── build_simulink_model.m         # Simulink 模型构建脚本
│   ├── simulate_anti_condensation.m   # 主仿真（有控制 vs 无控制）
│   ├── compare_strategies.m           # 三策略对比仿真
│   ├── validate_simulink_vs_script.m  # Simulink/脚本交叉验证
│   ├── anti_condensation_ctrl.m       # 控制器独立函数
│   └── plot_simulink_comparison.m     # Simulink 结果对比绘图
│
├── 专利附图/                           ← 专利说明书附图 (JPEG)
│   ├── 2025120155959_fig1.jpeg        # 功率设备结构示意图
│   ├── 2025120155959_fig2.jpeg        # 控制方法步骤流程图
│   └── 2025120155959_fig3.jpeg        # 含散热风扇的功率设备结构
│
└── 架构图/                             ← 项目架构/流程/状态图 (draw.io)
    ├── 01_系统结构图.drawio            # 系统整体结构
    ├── 02_控制方法流程图.drawio        # 控制方法流程
    ├── 03_Simulink模型架构图.drawio    # Simulink 模型架构
    ├── 04_分级控制决策状态机.drawio    # 控制器状态机
    ├── 2025120155959_fig1.drawio      # 专利图1重绘
    ├── 2025120155959_fig2.drawio      # 专利图2重绘
    └── 2025120155959_fig3.drawio      # 专利图3重绘
```

---

## 快速开始

```matlab
cd('F:\Practices\Claude\功率设备及其控制方法\仿真代码');

% 运行 MATLAB 主仿真
run('simulate_anti_condensation.m');

% 构建并运行 Simulink 模型
run('build_simulink_model.m');
sim('anti_condensation_control');
```

---

## 文件说明

### 核心文档

| 文件 | 内容 |
|------|------|
| `项目综合说明.md` | **完整项目文档**：专利原理、热模型方程、控制策略、Simulink 实现、参数校准历史（7 轮）、仿真结果、关键设计决策（T_amb 简化、一阶 RC 适用性、风扇双面性、多模块局限）、运行方法、创新点总结 |
| `功率设备及其控制方法.md` | 专利全文：权利要求 1-11、说明书 [0001]-[0086]、技术领域、背景技术、发明内容、具体实施方式 |

### 仿真脚本

| 文件 | 运行时间 | 输出 |
|------|:------:|------|
| `simulate_anti_condensation.m` | ~2s | 4 面板对比图（有/无控制） |
| `compare_strategies.m` | ~2s | 6 子图三策略对比 |
| `build_simulink_model.m` | ~30s | 构建 `anti_condensation_control.slx` |
| `validate_simulink_vs_script.m` | ~5s | 交叉验证对比图 |

### 架构图（draw.io 可编辑）

| 文件 | 说明 |
|------|------|
| `01_系统结构图.drawio` | 系统级：壳体、模块、T_amb/T_mod 传感器、控制器、母线电压 |
| `02_控制方法流程图.drawio` | 控制流程：S201(获取温度) → S202(判断ΔT → 升压) |
| `03_Simulink模型架构图.drawio` | Simulink 块图：Goto/From、热动态、控制器、电压平滑 |
| `04_分级控制决策状态机.drawio` | 状态机：模式0(安全)/模式1(轻度)/模式2(高风险) |

---

## 关键参数

| 参数 | 值 | 说明 |
|------|:--:|------|
| dt | 0.5 s | 仿真步长 |
| T_sim | 5400 s | 总仿真时间（90 min） |
| C_th | 1000 J/K | 模块热容 |
| R_th0 | 0.48 K/W | 额定热阻 |
| P_loss0 | 12 W | 基础功率损耗 |
| dT_threshold | 5 °C | 凝露温差阈值 |
| dT_second | 7 °C | 第二温差（高风险） |
| dV_step | 40 V | 每周期电压增量 |
| T_sample | 20 s | 温度采样周期 |
| V_bus | 580~880 V | 母线电压范围 |
| fan | 50~100% | 风扇转速范围 |

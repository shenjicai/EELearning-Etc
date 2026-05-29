%% 功率设备防凝露控制 — Simulink 模型构建脚本
%  基于专利 CN 121984334 A | 阳光电源股份有限公司
%  模型架构：环境温度 → 热动态 → 控制器 → (母线电压+, 风扇-) → 热动态(闭环)

clear; clc;
modelName = 'anti_condensation_control';

% 强制关闭所有已打开模型
bdclose('all');
fprintf('已清理所有模型。\n');

fprintf('===== 构建 Simulink 防凝露控制模型 =====\n');

%% ===== 创建模型 =====
new_system(modelName);
open_system(modelName);

% 写入参数到模型工作区
mdlWS = get_param(modelName, 'ModelWorkspace');
assignin(mdlWS, 'C_th',     700);
assignin(mdlWS, 'R_th0',    0.48);
assignin(mdlWS, 'T_mod0',   27);
assignin(mdlWS, 'P_loss0',  55);
assignin(mdlWS, 'V_nom',    620);
assignin(mdlWS, 'V_min',    580);
assignin(mdlWS, 'V_max',    880);
assignin(mdlWS, 'dT_th',    5);
assignin(mdlWS, 'dT_2nd',   8);
assignin(mdlWS, 'dV_step',  15);
assignin(mdlWS, 'fan_nom',  100);

%% ===== 创建所有块 =====
% --- Row 1: 环境温度信号 ---
add_block('simulink/Sources/Constant', [modelName, '/T_amb_base']);
set_param([modelName, '/T_amb_base'], 'Value', '38');

add_block('simulink/Sources/Sine Wave', [modelName, '/T_amb_ripple']);
set_param([modelName, '/T_amb_ripple'], 'Amplitude', '1.2', 'Frequency', '0.00628', 'SampleTime', '0.5');

% --- MATLAB Function 控制器 (核心) ---
add_block('simulink/User-Defined Functions/MATLAB Function', [modelName, '/AntiCondensation_Controller']);
set_param([modelName, '/AntiCondensation_Controller'], 'Position', [350, 80, 650, 220]);

% 使用 Stateflow API 设置 MATLAB Function 代码和采样时间
ctrlCode = sprintf([ ...
    'function [V_cmd, fan_cmd, mode] = fcn(T_mod, T_amb)\n' ...
    'persistent V_cur\n' ...
    'persistent fan_cur\n' ...
    'persistent tick\n' ...
    'if isempty(V_cur)\n' ...
    '    V_cur = 620;\n' ...
    '    fan_cur = 100;\n' ...
    '    tick = 0;\n' ...
    'end\n' ...
    'tick = tick + 1;\n' ...
    'mode = int32(0);\n' ...
    'if tick >= 40\n' ...
    '    tick = 0;\n' ...
    '    deltaT = T_amb - T_mod;\n' ...
    '    if deltaT > 5\n' ...
    '        if deltaT > 8\n' ...
    '            V_cur = min(V_cur+15, 880);\n' ...
    '            fan_cur = max(fan_cur-12, 15);\n' ...
    '            mode = int32(2);\n' ...
    '        else\n' ...
    '            V_cur = min(V_cur+15, 880);\n' ...
    '            mode = int32(1);\n' ...
    '        end\n' ...
    '    else\n' ...
    '        if V_cur > 625\n' ...
    '            V_cur = max(620, V_cur-9);\n' ...
    '        end\n' ...
    '        if fan_cur < 97\n' ...
    '            fan_cur = min(100, fan_cur+6);\n' ...
    '        end\n' ...
    '        mode = int32(0);\n' ...
    '    end\n' ...
    'end\n' ...
    'V_cmd = V_cur;\n' ...
    'fan_cmd = fan_cur;\n' ...
    'end\n']);

% 通过 Stateflow API 设置脚本
block_path = [modelName, '/AntiCondensation_Controller'];
rt = sfroot;
chart = rt.find('-isa', 'Stateflow.EMChart', 'Path', block_path);
if ~isempty(chart)
    chart.Script = ctrlCode;
    fprintf('MATLAB Function 控制器代码已设置。\n');
else
    fprintf(2, '警告: 无法找到 Stateflow 图表对象，尝试备用方法...\n');
    % 备用：使用 Interpreted MATLAB Function (单输出)
end

% --- 电压执行器 (Saturation 限幅) ---
add_block('simulink/Discontinuities/Saturation', [modelName, '/V_Saturation']);
set_param([modelName, '/V_Saturation'], 'UpperLimit', 'V_max', 'LowerLimit', 'V_min');

% --- 风扇执行器 (Saturation 限幅) ---
add_block('simulink/Discontinuities/Saturation', [modelName, '/Fan_Saturation']);
set_param([modelName, '/Fan_Saturation'], 'UpperLimit', '100', 'LowerLimit', '15');

% --- 功率损耗计算 P_loss = P_loss0 * (V/V_nom)^2 ---
add_block('simulink/Math Operations/Divide', [modelName, '/V_ratio']);
add_block('simulink/Math Operations/Product', [modelName, '/Square_V']);  % V_ratio * V_ratio
set_param([modelName, '/Square_V'], 'Inputs', '**');
add_block('simulink/Sources/Constant', [modelName, '/P_loss0_src']);
set_param([modelName, '/P_loss0_src'], 'Value', 'P_loss0');
add_block('simulink/Math Operations/Product', [modelName, '/P_loss']);
set_param([modelName, '/P_loss'], 'Inputs', '**');
add_block('simulink/Sources/Constant', [modelName, '/V_nom_src']);
set_param([modelName, '/V_nom_src'], 'Value', 'V_nom');

% --- 热阻计算 R_th = R_th0 * fan_nom / fan ---
add_block('simulink/Sources/Constant', [modelName, '/R_th0_src']);
set_param([modelName, '/R_th0_src'], 'Value', 'R_th0');
add_block('simulink/Sources/Constant', [modelName, '/fan_nom_src']);
set_param([modelName, '/fan_nom_src'], 'Value', 'fan_nom');
add_block('simulink/Math Operations/Product', [modelName, '/R_th_num']);  % R_th0 * fan_nom
set_param([modelName, '/R_th_num'], 'Inputs', '**');
add_block('simulink/Math Operations/Divide', [modelName, '/R_th']);  % R_th_num / fan

% --- 热动态 dT/dt = (P_loss - (T_mod - T_amb)/R_th) / C_th ---
add_block('simulink/Math Operations/Subtract', [modelName, '/T_diff']);    % T_mod - T_amb
add_block('simulink/Math Operations/Divide', [modelName, '/Q_cool']);      % T_diff / R_th
add_block('simulink/Math Operations/Subtract', [modelName, '/P_net']);     % P_loss - Q_cool
set_param([modelName, '/P_net'], 'Inputs', '+-');
add_block('simulink/Math Operations/Gain', [modelName, '/Gain_1C']);
set_param([modelName, '/Gain_1C'], 'Gain', '1/C_th');
add_block('simulink/Discrete/Discrete-Time Integrator', [modelName, '/Integrator']);
set_param([modelName, '/Integrator'], 'InitialCondition', 'T_mod0', ...
    'SampleTime', '0.5', 'IntegratorMethod', 'Integration: Forward Euler');

% --- 计算 Delta_T 用于监控 ---
add_block('simulink/Math Operations/Subtract', [modelName, '/Monitor_dT']);

% --- Scopes ---
add_block('simulink/Sinks/Scope', [modelName, '/Scope_Thermal']);
set_param([modelName, '/Scope_Thermal'], 'NumInputPorts', '3');
add_block('simulink/Sinks/Scope', [modelName, '/Scope_Control']);
set_param([modelName, '/Scope_Control'], 'NumInputPorts', '2');
add_block('simulink/Sinks/To Workspace', [modelName, '/ToWS_Tmod']);
set_param([modelName, '/ToWS_Tmod'], 'VariableName', 'T_mod_out');
add_block('simulink/Sinks/To Workspace', [modelName, '/ToWS_Vbus']);
set_param([modelName, '/ToWS_Vbus'], 'VariableName', 'V_bus_out');
add_block('simulink/Sinks/To Workspace', [modelName, '/ToWS_Fan']);
set_param([modelName, '/ToWS_Fan'], 'VariableName', 'fan_out');

%% ===== 模型配置 =====
set_param(modelName, 'Solver', 'FixedStepDiscrete');
set_param(modelName, 'FixedStep', '0.5');
set_param(modelName, 'StopTime', '5400');

%% ===== 连线 =====
% 为简化，使用 add_line 的自动路由

% 环境温度合并
add_line(modelName, 'T_amb_base/1', 'Monitor_dT/2', 'autorouting', 'on');

% 模块温度 → 控制器
add_line(modelName, 'Integrator/1', 'AntiCondensation_Controller/1', 'autorouting', 'on');
% 环境温度 → 控制器
add_line(modelName, 'T_amb_base/1', 'AntiCondensation_Controller/2', 'autorouting', 'on');

% 控制器 → 电压执行器
add_line(modelName, 'AntiCondensation_Controller/1', 'V_Saturation/1', 'autorouting', 'on');

% 控制器 → 风扇执行器
add_line(modelName, 'AntiCondensation_Controller/2', 'Fan_Saturation/1', 'autorouting', 'on');

% 功率损耗计算链
add_line(modelName, 'V_Saturation/1', 'V_ratio/1', 'autorouting', 'on');
add_line(modelName, 'V_nom_src/1', 'V_ratio/2', 'autorouting', 'on');
add_line(modelName, 'V_ratio/1', 'Square_V/1', 'autorouting', 'on');
add_line(modelName, 'V_ratio/1', 'Square_V/2', 'autorouting', 'on');
add_line(modelName, 'Square_V/1', 'P_loss/1', 'autorouting', 'on');
add_line(modelName, 'P_loss0_src/1', 'P_loss/2', 'autorouting', 'on');

% 热阻计算链
add_line(modelName, 'R_th0_src/1', 'R_th_num/1', 'autorouting', 'on');
add_line(modelName, 'fan_nom_src/1', 'R_th_num/2', 'autorouting', 'on');
add_line(modelName, 'R_th_num/1', 'R_th/1', 'autorouting', 'on');
add_line(modelName, 'Fan_Saturation/1', 'R_th/2', 'autorouting', 'on');

% 热动态
add_line(modelName, 'Integrator/1', 'T_diff/1', 'autorouting', 'on');
add_line(modelName, 'T_amb_base/1', 'T_diff/2', 'autorouting', 'on');
add_line(modelName, 'T_diff/1', 'Q_cool/1', 'autorouting', 'on');
add_line(modelName, 'R_th/1', 'Q_cool/2', 'autorouting', 'on');
add_line(modelName, 'P_loss/1', 'P_net/1', 'autorouting', 'on');
add_line(modelName, 'Q_cool/1', 'P_net/2', 'autorouting', 'on');
add_line(modelName, 'P_net/1', 'Gain_1C/1', 'autorouting', 'on');
add_line(modelName, 'Gain_1C/1', 'Integrator/1', 'autorouting', 'on');

% 监控连线
add_line(modelName, 'Integrator/1', 'Monitor_dT/1', 'autorouting', 'on');

% Scope 连接
add_line(modelName, 'Integrator/1', 'Scope_Thermal/1', 'autorouting', 'on');
add_line(modelName, 'T_amb_base/1', 'Scope_Thermal/2', 'autorouting', 'on');
add_line(modelName, 'Monitor_dT/1', 'Scope_Thermal/3', 'autorouting', 'on');
add_line(modelName, 'V_Saturation/1', 'Scope_Control/1', 'autorouting', 'on');
add_line(modelName, 'Fan_Saturation/1', 'Scope_Control/2', 'autorouting', 'on');

% To Workspace
add_line(modelName, 'Integrator/1', 'ToWS_Tmod/1', 'autorouting', 'on');
add_line(modelName, 'V_Saturation/1', 'ToWS_Vbus/1', 'autorouting', 'on');
add_line(modelName, 'Fan_Saturation/1', 'ToWS_Fan/1', 'autorouting', 'on');

%% ===== 保存模型 =====
save_system(modelName);

fprintf('模型 %s.slx 构建完成。\n', modelName);
fprintf('子系统清单:\n');
fprintf('  1. T_amb_base          — 环境温度基准 (38°C)\n');
fprintf('  2. AntiCondensation_Controller — 分级防凝露控制器 (MATLAB Function)\n');
fprintf('  3. V_RateLimiter/Saturation — 母线电压执行器\n');
fprintf('  4. Fan_RateLimiter/Saturation — 风扇转速执行器\n');
fprintf('  5. P_loss 计算链        — P_loss ∝ V²\n');
fprintf('  6. R_th 计算链          — 热阻 ∝ 1/fan\n');
fprintf('  7. Integrator          — 热动态积分器\n');
fprintf('  8. Scope_Thermal       — 温度波形观测\n');
fprintf('  9. Scope_Control       — 控制量观测\n');
fprintf('\n运行: sim(''%s'')\n', modelName);

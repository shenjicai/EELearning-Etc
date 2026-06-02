%% 功率设备防凝露控制 — Simulink 模型构建脚本
%  基于专利 CN 121984334 A | 阳光电源股份有限公司
%  模型架构：环境温度 → 热动态 → 控制器 → (母线电压+, 风扇-) → 热动态(闭环)

% 安全清理（不清除调用方工作区变量）
bdclose('all');
modelName = 'anti_condensation_control';
fprintf('已清理所有模型。\n');

fprintf('===== 构建 Simulink 防凝露控制模型 =====\n');

%% ===== 仿真参数 (匹配 simulate_anti_condensation.m) =====
dt       = 0.5;
T_sim    = 5400;
t        = (0:dt:T_sim)';
N        = length(t);

C_th     = 1000;    % 热容 (J/K) — 匹配M文件
R_th0    = 0.48;    % 额定热阻 (K/W)
T_mod0   = 27;      % 模块初始温度 (°C)
P_loss0  = 12;      % 额定损耗 (W) — 匹配M文件（轻载工况）
V_nom    = 620;     % 额定母线电压 (V)
V_min    = 580;     % 母线电压下限 (V)
V_max    = 880;     % 母线电压上限 (V)
dT_th    = 5;       % 凝露温差阈值 (°C)
dT_first = 4;       % 第一温差（轻度风险，仅升压）
dT_2nd   = 7;       % 第二温差（高风险，大幅升压） — 匹配M文件
dV_step  = 40;      % 电压增量 (V) — 匹配M文件
fan_nom  = 100;     % 额定风扇转速 (%)
fan_min  = 50;      % 最低风扇转速 (%) — 匹配M文件

%% ===== 生成环境温度 T_amb (仅当base工作区没有时，匹配 simulate_anti_condensation.m) =====
if ~evalin('base', 'exist(''T_amb_sim'',''var'')')
    T_amb0 = 27;
    rng(42);
    env_rise = 18 * (1 - exp(-t/220));
    env_fall = exp(-t/10000);
    T_amb = T_amb0 + env_rise .* env_fall ...
          + 1.5 * sin(2*pi*t/600) + 0.5 * sin(2*pi*t/2000) ...
          + 0.2 * randn(N,1);
    T_amb_sim = [t, T_amb];  % N×2 矩阵: [时间列, 数据列]
    assignin('base', 'T_amb_sim', T_amb_sim);
    fprintf('环境温度数据已生成并存入工作区 (%.0f 采样点)。\n', N);
else
    fprintf('环境温度数据已存在于工作区，跳过生成。\n');
end

%% ===== 创建模型 =====
new_system(modelName);
open_system(modelName);

% 写入参数到模型工作区
mdlWS = get_param(modelName, 'ModelWorkspace');
assignin(mdlWS, 'C_th',     C_th);
assignin(mdlWS, 'R_th0',    R_th0);
assignin(mdlWS, 'T_mod0',   T_mod0);
assignin(mdlWS, 'P_loss0',  P_loss0);
assignin(mdlWS, 'V_nom',    V_nom);
assignin(mdlWS, 'V_min',    V_min);
assignin(mdlWS, 'V_max',    V_max);
assignin(mdlWS, 'dT_th',    dT_th);
assignin(mdlWS, 'dT_first', dT_first);
assignin(mdlWS, 'dT_2nd',   dT_2nd);
assignin(mdlWS, 'dV_step',  dV_step);
assignin(mdlWS, 'fan_nom',  fan_nom);
assignin(mdlWS, 'fan_min',  fan_min);
assignin(mdlWS, 'dt',       dt);

%% ===== 创建所有块 =====

% --- 环境温度 (From Workspace) ---
add_block('simulink/Sources/From Workspace', [modelName, '/T_amb']);
set_param([modelName, '/T_amb'], 'VariableName', 'T_amb_sim', ...
    'SampleTime', '0.5');

% --- MATLAB Function 控制器 (核心) ---
add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [modelName, '/AntiCondensation_Controller']);
set_param([modelName, '/AntiCondensation_Controller'], ...
    'Position', [350, 80, 650, 240]);

% 控制器代码 — 匹配 simulate_anti_condensation.m 逻辑（电压+风扇合并）
ctrlCode = sprintf([ ...
    'function [V_tgt, fan_cmd] = fcn(T_mod, T_amb, V_cur, fan_cur)\n' ...
    '%%#codegen\n' ...
    'persistent tick\n' ...
    'persistent V_target\n' ...
    'if isempty(tick)\n' ...
    '    tick = 0;\n' ...
    '    V_target = 620.0;\n' ...
    'end\n' ...
    'deltaT = T_amb - T_mod;\n' ...
    'fan_cmd = fan_cur;\n' ...
    'if tick == 0\n' ...
    '    if deltaT > 5.0\n' ...
    '        if deltaT > 7.0\n' ...
    '            V_target = min(V_cur + %.1f, %.0f);\n' ...
    '        elseif deltaT > 4.0\n' ...
    '            V_target = min(V_cur + %.0f, %.0f);\n' ...
    '        end\n' ...
    '    else\n' ...
    '        if V_cur > 625.0\n' ...
    '            V_target = max(620.0, V_cur - %.1f);\n' ...
    '        end\n' ...
    '        if V_cur > 700.0\n' ...
    '            fan_cmd = max(fan_cur - 6.0, %.0f);\n' ...
    '        end\n' ...
    '        if fan_cur < 97.0 && V_cur <= 630.0\n' ...
    '            fan_cmd = min(100.0, fan_cur + 5.0);\n' ...
    '        end\n' ...
    '    end\n' ...
    'end\n' ...
    'tick = tick + 1;\n' ...
    'if tick >= 40\n' ...
    '    tick = 0;\n' ...
    'end\n' ...
    'V_tgt = V_target;\n' ...
    'end\n'], ...
    dV_step*1.7, V_max, dV_step, V_max, dV_step*0.6, fan_min);

% 通过 Stateflow API 设置脚本
block_path = [modelName, '/AntiCondensation_Controller'];
rt = sfroot;
chart = rt.find('-isa', 'Stateflow.EMChart', 'Path', block_path);
if ~isempty(chart)
    chart.Script = ctrlCode;
    fprintf('MATLAB Function 控制器代码已设置 (匹配 simulate_anti_condensation.m)。\n');
else
    fprintf(2, '警告: 无法找到 Stateflow 图表对象。\n');
end

% --- 电压平滑 (一阶惯性环节, tau=8s) ---
% H(z) = alpha / (1 - (1-alpha)*z^-1), alpha = 1-exp(-dt/8) ≈ 0.0606
alpha = 1 - exp(-dt/8);
add_block('simulink/Discrete/Discrete Transfer Fcn', ...
    [modelName, '/V_Smoothing']);
set_param([modelName, '/V_Smoothing'], ...
    'Numerator', num2str(alpha), ...
    'Denominator', sprintf('[1, -%.6f]', 1-alpha), ...
    'SampleTime', '0.5', ...
    'InitialStates', '620');

% --- 电压执行器 (Saturation 限幅) ---
add_block('simulink/Discontinuities/Saturation', [modelName, '/V_Saturation']);
set_param([modelName, '/V_Saturation'], ...
    'UpperLimit', 'V_max', 'LowerLimit', 'V_min');

% --- 风扇执行器 (Saturation 限幅) ---
add_block('simulink/Discontinuities/Saturation', [modelName, '/Fan_Saturation']);
set_param([modelName, '/Fan_Saturation'], ...
    'UpperLimit', '100', 'LowerLimit', 'fan_min');

% --- 功率损耗计算 P_loss = P_loss0 * (V/V_nom)^1.8 ---
add_block('simulink/Math Operations/Divide', [modelName, '/V_ratio']);
add_block('simulink/Sources/Constant', [modelName, '/V_nom_src']);
set_param([modelName, '/V_nom_src'], 'Value', 'V_nom');
add_block('simulink/Math Operations/Math Function', [modelName, '/V_pow']);
set_param([modelName, '/V_pow'], 'Operator', 'pow');
add_block('simulink/Sources/Constant', [modelName, '/pow_const']);
set_param([modelName, '/pow_const'], 'Value', '1.8');
add_block('simulink/Math Operations/Product', [modelName, '/P_loss']);
set_param([modelName, '/P_loss'], 'Inputs', '**');
add_block('simulink/Sources/Constant', [modelName, '/P_loss0_src']);
set_param([modelName, '/P_loss0_src'], 'Value', 'P_loss0');

% --- 热阻计算 R_th = R_th0 * fan_nom / fan ---
add_block('simulink/Sources/Constant', [modelName, '/R_th0_src']);
set_param([modelName, '/R_th0_src'], 'Value', 'R_th0');
add_block('simulink/Sources/Constant', [modelName, '/fan_nom_src']);
set_param([modelName, '/fan_nom_src'], 'Value', 'fan_nom');
add_block('simulink/Math Operations/Product', [modelName, '/R_th_num']);
set_param([modelName, '/R_th_num'], 'Inputs', '**');
add_block('simulink/Math Operations/Divide', [modelName, '/R_th']);

% --- 热动态 dT/dt = (P_loss - (T_mod - T_amb)/R_th) / C_th ---
add_block('simulink/Math Operations/Subtract', [modelName, '/T_diff']);
add_block('simulink/Math Operations/Divide', [modelName, '/Q_cool']);
add_block('simulink/Math Operations/Subtract', [modelName, '/P_net']);
set_param([modelName, '/P_net'], 'Inputs', '+-');
add_block('simulink/Math Operations/Gain', [modelName, '/Gain_1C']);
set_param([modelName, '/Gain_1C'], 'Gain', '1/C_th');
add_block('simulink/Discrete/Discrete-Time Integrator', [modelName, '/Integrator']);
set_param([modelName, '/Integrator'], 'InitialCondition', 'T_mod0', ...
    'SampleTime', '0.5', 'IntegratorMethod', 'Integration: Forward Euler');

% --- 计算 Delta_T 用于监控 ---
add_block('simulink/Math Operations/Subtract', [modelName, '/Monitor_dT']);

% --- 单位延迟 (保存上一时刻的 V 和 fan 给控制器) ---
add_block('simulink/Discrete/Unit Delay', [modelName, '/V_Delay']);
set_param([modelName, '/V_Delay'], 'SampleTime', '0.5', ...
    'InitialCondition', '620');
add_block('simulink/Discrete/Unit Delay', [modelName, '/Fan_Delay']);
set_param([modelName, '/Fan_Delay'], 'SampleTime', '0.5', ...
    'InitialCondition', '100');

% ===== 无控制对照路径 (固定 V=620, fan=100) =====
% P_loss_nc = P_loss0 (固定)
add_block('simulink/Sources/Constant', [modelName, '/P_loss_nc']);
set_param([modelName, '/P_loss_nc'], 'Value', 'P_loss0');

% R_th_nc = R_th0 (固定)
add_block('simulink/Sources/Constant', [modelName, '/R_th_nc']);
set_param([modelName, '/R_th_nc'], 'Value', 'R_th0');

% 热动态 (无控制)
add_block('simulink/Math Operations/Subtract', [modelName, '/T_diff_nc']);
add_block('simulink/Math Operations/Divide', [modelName, '/Q_cool_nc']);
add_block('simulink/Math Operations/Subtract', [modelName, '/P_net_nc']);
set_param([modelName, '/P_net_nc'], 'Inputs', '+-');
add_block('simulink/Math Operations/Gain', [modelName, '/Gain_1C_nc']);
set_param([modelName, '/Gain_1C_nc'], 'Gain', '1/C_th');
add_block('simulink/Discrete/Discrete-Time Integrator', [modelName, '/Integrator_nc']);
set_param([modelName, '/Integrator_nc'], 'InitialCondition', 'T_mod0', ...
    'SampleTime', '0.5', 'IntegratorMethod', 'Integration: Forward Euler');

% Delta_T (无控制)
add_block('simulink/Math Operations/Subtract', [modelName, '/Monitor_dT_nc']);

% --- Scopes ---
add_block('simulink/Sinks/Scope', [modelName, '/Scope_Thermal']);
set_param([modelName, '/Scope_Thermal'], 'NumInputPorts', '4');
add_block('simulink/Sinks/Scope', [modelName, '/Scope_Control']);
set_param([modelName, '/Scope_Control'], 'NumInputPorts', '2');

% 有控制 To Workspace
add_block('simulink/Sinks/To Workspace', [modelName, '/ToWS_Tmod']);
set_param([modelName, '/ToWS_Tmod'], 'VariableName', 'T_mod_out');
add_block('simulink/Sinks/To Workspace', [modelName, '/ToWS_Vbus']);
set_param([modelName, '/ToWS_Vbus'], 'VariableName', 'V_bus_out');
add_block('simulink/Sinks/To Workspace', [modelName, '/ToWS_Fan']);
set_param([modelName, '/ToWS_Fan'], 'VariableName', 'fan_out');

% 无控制 To Workspace
add_block('simulink/Sinks/To Workspace', [modelName, '/ToWS_Tmod_nc']);
set_param([modelName, '/ToWS_Tmod_nc'], 'VariableName', 'T_mod_nc_out');
add_block('simulink/Sinks/To Workspace', [modelName, '/ToWS_dT_nc']);
set_param([modelName, '/ToWS_dT_nc'], 'VariableName', 'dT_nc_out');

% 环境温度 To Workspace (供 StopFcn 使用)
add_block('simulink/Sinks/To Workspace', [modelName, '/ToWS_Tamb']);
set_param([modelName, '/ToWS_Tamb'], 'VariableName', 'T_amb_out');

%% ===== Goto/From 信号标签块 (清理长距离连线) =====

% Goto 信号源
add_block('simulink/Signal Routing/Goto', [modelName, '/Goto_Tamb']);
set_param([modelName, '/Goto_Tamb'], 'GotoTag', 'Tamb', 'TagVisibility', 'local');

add_block('simulink/Signal Routing/Goto', [modelName, '/Goto_Tmod']);
set_param([modelName, '/Goto_Tmod'], 'GotoTag', 'Tmod', 'TagVisibility', 'local');

add_block('simulink/Signal Routing/Goto', [modelName, '/Goto_Vbus']);
set_param([modelName, '/Goto_Vbus'], 'GotoTag', 'Vbus', 'TagVisibility', 'local');

add_block('simulink/Signal Routing/Goto', [modelName, '/Goto_Fan']);
set_param([modelName, '/Goto_Fan'], 'GotoTag', 'Fan', 'TagVisibility', 'local');

% From 块 (读取 T_amb 信号到各使用点)
fromTambDests = {'Ctrl', 'Tdiff', 'MdT', 'TdiffNc', 'MdTNc'};
fromTambY     = [80, 275, 335, 455, 525];
for k = 1:length(fromTambDests)
    add_block('simulink/Signal Routing/From', [modelName, '/From_Tamb_' fromTambDests{k}]);
    set_param([modelName, '/From_Tamb_' fromTambDests{k}], 'GotoTag', 'Tamb');
end

% From 块 (读取 T_mod 信号)
add_block('simulink/Signal Routing/From', [modelName, '/From_Tmod_Ctrl']);
set_param([modelName, '/From_Tmod_Ctrl'], 'GotoTag', 'Tmod');
add_block('simulink/Signal Routing/From', [modelName, '/From_Tmod_Tdiff']);
set_param([modelName, '/From_Tmod_Tdiff'], 'GotoTag', 'Tmod');

% From 块 (读取 V_bus 信号)
add_block('simulink/Signal Routing/From', [modelName, '/From_Vbus_Pwr']);
set_param([modelName, '/From_Vbus_Pwr'], 'GotoTag', 'Vbus');

% From 块 (读取 Fan 信号)
add_block('simulink/Signal Routing/From', [modelName, '/From_Fan_Rth']);
set_param([modelName, '/From_Fan_Rth'], 'GotoTag', 'Fan');

%% ===== 模型配置 =====
set_param(modelName, 'Solver', 'FixedStepDiscrete');
set_param(modelName, 'FixedStep', '0.5');
set_param(modelName, 'StopTime', '5400');
set_param(modelName, 'UnderSpecifiedDimensionMsg', 'none');
set_param(modelName, 'ReturnWorkspaceOutputsName', 'simOut');  % StopFcn 回调中使用

%% ===== StopFcn 回调: 仿真结束后自动生成四格对比图 =====
% 从 base 工作区读取 To Workspace 块的输出数据
% 仅在变量存在时执行（Simulink UI 运行模式下变量会被写入 base 工作区）
stopLines = {
    'try'
    '    so = evalin(''base'', ''simOut'');'
    '    T_amb = so.get(''T_amb_out'').Data(:);'
    '    T_ctrl = so.get(''T_mod_out'').Data(:);'
    '    T_nc   = so.get(''T_mod_nc_out'').Data(:);'
    'catch'
    '    if ~exist(''T_amb_out'',''var'') || ~exist(''T_mod_out'',''var'') || ~exist(''T_mod_nc_out'',''var''); return; end'
    '    T_amb = T_amb_out(:); T_ctrl = T_mod_out(:); T_nc = T_mod_nc_out(:);'
    'end'
    'dt = 0.5; dT_threshold = 5; dT_second = 7;'
    'n = min([length(T_ctrl), length(T_nc), length(T_amb)]);'
    't_min = (0:dt:(n-1)*dt)'' / 60;'
    'T_ctrl = T_ctrl(1:n); T_nc = T_nc(1:n); T_amb = T_amb(1:n);'
    'dT_ctrl = T_amb - T_ctrl; dT_nc = T_amb - T_nc;'
    'in_risk_ctrl = dT_ctrl > dT_threshold; in_risk_nc = dT_nc > dT_threshold;'
    'risk_dur_ctrl = sum(in_risk_ctrl)*dt/60; risk_dur_nc = sum(in_risk_nc)*dt/60;'
    'peak_ctrl = max(dT_ctrl); peak_nc = max(dT_nc);'
    'exp_ctrl = sum(max(dT_ctrl-dT_threshold,0))*dt/60; exp_nc = sum(max(dT_nc-dT_threshold,0))*dt/60;'
    ''
    '% --- 绘图 ---'
    'set(groot,''DefaultFigureColor'',''w'',''DefaultAxesColor'',''w'');'
    'figure(''Name'',''防凝露控制效果对比 - Simulink仿真'',''Position'',[100 100 1400 500],''Color'',''w'');'
    ''
    'subplot(1,4,1); plot(t_min,T_ctrl,''b-'',''LineWidth'',1.8); hold on;'
    'plot(t_min,T_nc,''r--'',''LineWidth'',1.5); plot(t_min,T_amb,''k:'',''LineWidth'',1);'
    'xlabel(''时间 (min)''); ylabel(''温度 (°C)''); title(''模块温度对比'');'
    'legend(''有控制'',''无控制'',''环境'',''Location'',''best''); grid on;'
    ''
    'subplot(1,4,2); plot(t_min,dT_ctrl,''b-'',''LineWidth'',1.8); hold on;'
    'plot(t_min,dT_nc,''r--'',''LineWidth'',1.5); yline(dT_threshold,''k--'',''凝露阈值'',''LineWidth'',1.5);'
    'xlabel(''时间 (min)''); ylabel(''\Delta T = T_{amb} - T_{mod} (°C)''); title(''温差对比 (全程)'');'
    'legend(''有控制'',''无控制'',''Location'',''best''); grid on;'
    ''
    'subplot(1,4,3); idx20=find(t_min<=20);'
    'plot(t_min(idx20),dT_ctrl(idx20),''b-'',''LineWidth'',1.8); hold on;'
    'plot(t_min(idx20),dT_nc(idx20),''r--'',''LineWidth'',1.5);'
    'yline(dT_threshold,''k--'',''凝露阈值'',''LineWidth'',1.5);'
    'yline(dT_second,''m:'',''第二温差'',''LineWidth'',1);'
    'xlabel(''时间 (min)''); ylabel(''\Delta T (°C)''); title(''前20分钟瞬态放大'');'
    'legend(''有控制'',''无控制'',''Location'',''best''); grid on;'
    ''
    'subplot(1,4,4); bar_data=[risk_dur_ctrl risk_dur_nc; peak_ctrl peak_nc; exp_ctrl exp_nc]'';'
    'b=bar(bar_data); b(1).FaceColor=''b''; b(2).FaceColor=''r'';'
    'set(gca,''XTickLabel'',{''风险时长(min)'',''峰值温差(°C)'',''暴露量(°C·min)''});'
    'legend(''有控制'',''无控制'',''Location'',''northwest'');'
    'title(''防凝露效果对比''); ylabel(''指标值''); grid on;'
    ''
    'sgtitle(''防凝露控制效果对比 - Simulink 模型仿真 (专利 CN 121984334 A)'');'
    ''
    '% --- 控制台输出 ---'
    'disp('' '');'
    'disp(''===== 风险指标 (Simulink自动输出) ====='');'
    'disp([''风险时长: '' num2str(risk_dur_ctrl,''%.1f'') '' min (有控制) vs '' num2str(risk_dur_nc,''%.1f'') '' min (无控制) -> 缩短 '' num2str((risk_dur_nc-risk_dur_ctrl)/risk_dur_nc*100,''%.1f'') ''%'']);'
    'disp([''峰值温差: '' num2str(peak_ctrl,''%.2f'') '' degC (有控制) vs '' num2str(peak_nc,''%.2f'') '' degC (无控制) -> 降低 '' num2str((peak_nc-peak_ctrl)/peak_nc*100,''%.1f'') ''%'']);'
    'disp([''暴露量:   '' num2str(exp_ctrl,''%.1f'') '' degC*min (有控制) vs '' num2str(exp_nc,''%.1f'') '' degC*min (无控制) -> 减少 '' num2str((exp_nc-exp_ctrl)/exp_nc*100,''%.1f'') ''%'']);'
    ''
    '% --- 保存图片 ---'
    'scriptDir = fileparts(which(''anti_condensation_control''));'
    'if ~isempty(scriptDir)'
    '    saveas(gcf, fullfile(scriptDir, ''simulink_comparison.png''));'
    '    disp(''对比图已自动保存: simulink_comparison.png'');'
    'end'
};

% 合并为单个字符串，用换行符分隔
stopFcnCode = strjoin(stopLines, newline);
set_param(modelName, 'StopFcn', stopFcnCode);

%% ===== 连线 =====
fprintf('  连线中...\n');

% --- Goto 信号源 ---
add_line(modelName, 'T_amb/1', 'Goto_Tamb/1', 'autorouting', 'on');

% --- From 信号到各目的地 (T_amb) ---
add_line(modelName, 'From_Tamb_Ctrl/1',    'AntiCondensation_Controller/2', 'autorouting', 'on');
add_line(modelName, 'From_Tamb_Tdiff/1',   'T_diff/2', 'autorouting', 'on');
add_line(modelName, 'From_Tamb_MdT/1',     'Monitor_dT/2', 'autorouting', 'on');
add_line(modelName, 'From_Tamb_TdiffNc/1', 'T_diff_nc/2', 'autorouting', 'on');
add_line(modelName, 'From_Tamb_MdTNc/1',   'Monitor_dT_nc/2', 'autorouting', 'on');

% --- 有控制热动态 (本地信号) ---
add_line(modelName, 'Integrator/1', 'Goto_Tmod/1', 'autorouting', 'on');
add_line(modelName, 'From_Tmod_Ctrl/1',  'AntiCondensation_Controller/1', 'autorouting', 'on');
add_line(modelName, 'From_Tmod_Tdiff/1', 'T_diff/1', 'autorouting', 'on');
add_line(modelName, 'Integrator/1', 'Monitor_dT/1', 'autorouting', 'on');

% --- 反馈回路 ---
add_line(modelName, 'V_Delay/1', 'AntiCondensation_Controller/3', 'autorouting', 'on');
add_line(modelName, 'Fan_Delay/1', 'AntiCondensation_Controller/4', 'autorouting', 'on');

% --- 控制器 → 执行器 ---
add_line(modelName, 'AntiCondensation_Controller/1', 'V_Smoothing/1', 'autorouting', 'on');
add_line(modelName, 'V_Smoothing/1', 'V_Saturation/1', 'autorouting', 'on');
add_line(modelName, 'AntiCondensation_Controller/2', 'Fan_Saturation/1', 'autorouting', 'on');

% --- 延迟反馈 ---
add_line(modelName, 'V_Saturation/1', 'V_Delay/1', 'autorouting', 'on');
add_line(modelName, 'Fan_Saturation/1', 'Fan_Delay/1', 'autorouting', 'on');

% --- Goto 信号: V_bus, Fan ---
add_line(modelName, 'V_Saturation/1', 'Goto_Vbus/1', 'autorouting', 'on');
add_line(modelName, 'Fan_Saturation/1', 'Goto_Fan/1', 'autorouting', 'on');

% --- 功率计算链 ---
add_line(modelName, 'From_Vbus_Pwr/1', 'V_ratio/1', 'autorouting', 'on');
add_line(modelName, 'V_nom_src/1', 'V_ratio/2', 'autorouting', 'on');
add_line(modelName, 'V_ratio/1', 'V_pow/1', 'autorouting', 'on');
add_line(modelName, 'pow_const/1', 'V_pow/2', 'autorouting', 'on');
add_line(modelName, 'V_pow/1', 'P_loss/1', 'autorouting', 'on');
add_line(modelName, 'P_loss0_src/1', 'P_loss/2', 'autorouting', 'on');

% --- 热阻计算链 ---
add_line(modelName, 'R_th0_src/1', 'R_th_num/1', 'autorouting', 'on');
add_line(modelName, 'fan_nom_src/1', 'R_th_num/2', 'autorouting', 'on');
add_line(modelName, 'R_th_num/1', 'R_th/1', 'autorouting', 'on');
add_line(modelName, 'From_Fan_Rth/1', 'R_th/2', 'autorouting', 'on');

% --- 有控制热动态 ---
add_line(modelName, 'T_diff/1', 'Q_cool/1', 'autorouting', 'on');
add_line(modelName, 'R_th/1', 'Q_cool/2', 'autorouting', 'on');
add_line(modelName, 'P_loss/1', 'P_net/1', 'autorouting', 'on');
add_line(modelName, 'Q_cool/1', 'P_net/2', 'autorouting', 'on');
add_line(modelName, 'P_net/1', 'Gain_1C/1', 'autorouting', 'on');
add_line(modelName, 'Gain_1C/1', 'Integrator/1', 'autorouting', 'on');

% --- 无控制热动态 (本地信号) ---
add_line(modelName, 'Integrator_nc/1', 'T_diff_nc/1', 'autorouting', 'on');
add_line(modelName, 'Integrator_nc/1', 'Monitor_dT_nc/1', 'autorouting', 'on');
add_line(modelName, 'T_diff_nc/1', 'Q_cool_nc/1', 'autorouting', 'on');
add_line(modelName, 'R_th_nc/1', 'Q_cool_nc/2', 'autorouting', 'on');
add_line(modelName, 'P_loss_nc/1', 'P_net_nc/1', 'autorouting', 'on');
add_line(modelName, 'Q_cool_nc/1', 'P_net_nc/2', 'autorouting', 'on');
add_line(modelName, 'P_net_nc/1', 'Gain_1C_nc/1', 'autorouting', 'on');
add_line(modelName, 'Gain_1C_nc/1', 'Integrator_nc/1', 'autorouting', 'on');

% --- Scope 连接 ---
add_line(modelName, 'Integrator/1', 'Scope_Thermal/1', 'autorouting', 'on');
add_line(modelName, 'T_amb/1', 'Scope_Thermal/2', 'autorouting', 'on');
add_line(modelName, 'Monitor_dT/1', 'Scope_Thermal/3', 'autorouting', 'on');
add_line(modelName, 'Integrator_nc/1', 'Scope_Thermal/4', 'autorouting', 'on');
add_line(modelName, 'V_Saturation/1', 'Scope_Control/1', 'autorouting', 'on');
add_line(modelName, 'Fan_Saturation/1', 'Scope_Control/2', 'autorouting', 'on');

% --- To Workspace ---
add_line(modelName, 'Integrator/1', 'ToWS_Tmod/1', 'autorouting', 'on');
add_line(modelName, 'V_Saturation/1', 'ToWS_Vbus/1', 'autorouting', 'on');
add_line(modelName, 'Fan_Saturation/1', 'ToWS_Fan/1', 'autorouting', 'on');
add_line(modelName, 'Integrator_nc/1', 'ToWS_Tmod_nc/1', 'autorouting', 'on');
add_line(modelName, 'Monitor_dT_nc/1', 'ToWS_dT_nc/1', 'autorouting', 'on');
add_line(modelName, 'T_amb/1', 'ToWS_Tamb/1', 'autorouting', 'on');

%% ===== 模块布局 =====
% 信号流向: 源(左) → 控制器 → 执行器 → 功率/热计算 → 输出(右)
% Goto/From 标签替代长距离多分支连线

% 标准块尺寸: small=[x y x+40 y+30], mid=[x y x+60 y+30], wide=[x y x+80 y+30]
%             goto=[x y x+40 y+20], from=[x y x+40 y+20], scope=[x y x+65 y+65]
%             toWS=[x y x+100 y+30], tfcn=[x y x+80 y+30]

S  = @(x,y) [x, y, x+40, y+30];   % 小块: Constant, Gain, Product, Divide, Subtract
M  = @(x,y) [x, y, x+55, y+30];   % 中块: Integrator, UnitDelay, Saturation
MW = @(x,y) [x, y, x+80, y+30];   % 宽中块: TransferFcn, ToWorkspace
G  = @(x,y) [x, y, x+45, y+20];   % Goto
F  = @(x,y) [x, y, x+45, y+25];   % From
SC = @(x,y) [x, y, x+70, y+70];   % Scope

fprintf('  设置模块布局...\n');

% --- 第1列: 环境输入 (x=50) ---
set_param([modelName, '/T_amb'], 'Position', [50, 30, 170, 60]);

% --- Goto/From 信号 ---
set_param([modelName, '/Goto_Tamb'], 'Position', G(200, 27));
set_param([modelName, '/Goto_Tmod'], 'Position', G(975, 267));
set_param([modelName, '/Goto_Vbus'], 'Position', G(820, 50));
set_param([modelName, '/Goto_Fan'],  'Position', G(820, 115));

% From 块: T_amb 扇出
set_param([modelName, '/From_Tamb_Ctrl'],    'Position', F(220, 84));
set_param([modelName, '/From_Tamb_Tdiff'],   'Position', F(420, 262));
set_param([modelName, '/From_Tamb_MdT'],     'Position', F(420, 320));
set_param([modelName, '/From_Tamb_TdiffNc'], 'Position', F(420, 450));
set_param([modelName, '/From_Tamb_MdTNc'],   'Position', F(420, 515));

% From 块: T_mod 扇出
set_param([modelName, '/From_Tmod_Ctrl'],  'Position', F(220, 114));
set_param([modelName, '/From_Tmod_Tdiff'], 'Position', F(420, 234));

% From 块: V_bus, Fan
set_param([modelName, '/From_Vbus_Pwr'], 'Position', F(130, 187));
set_param([modelName, '/From_Fan_Rth'],  'Position', F(230, 310));

% --- 第2列: 控制器 (x=280) ---
set_param([modelName, '/AntiCondensation_Controller'], ...
    'Position', [280, 40, 530, 140]);

% --- 第3列: 执行器 V 路径 (x=600~800) ---
set_param([modelName, '/V_Smoothing'],  'Position', MW(610, 50));
set_param([modelName, '/V_Saturation'], 'Position', M(740, 50));

% --- 执行器 Fan 路径 ---
set_param([modelName, '/Fan_Saturation'], 'Position', M(740, 115));

% --- 第4列: 反馈延迟 (x=870) ---
set_param([modelName, '/V_Delay'],   'Position', M(890, 50));
set_param([modelName, '/Fan_Delay'], 'Position', M(890, 115));

% --- 常量 (y=215~375) ---
set_param([modelName, '/V_nom_src'],   'Position', S(60, 215));
set_param([modelName, '/pow_const'],   'Position', S(180, 215));
set_param([modelName, '/P_loss0_src'], 'Position', S(340, 215));
set_param([modelName, '/R_th0_src'],   'Position', S(60, 345));
set_param([modelName, '/fan_nom_src'], 'Position', S(140, 345));

% --- 功率计算链 (y=195) ---
set_param([modelName, '/V_ratio'], 'Position', S(180, 195));
set_param([modelName, '/V_pow'],   'Position', M(270, 195));
set_param([modelName, '/P_loss'],  'Position', S(370, 195));

% --- 热阻计算 (y=335) ---
set_param([modelName, '/R_th_num'], 'Position', S(240, 335));
set_param([modelName, '/R_th'],     'Position', S(330, 335));

% --- 有控制热动态 (x=500~940, y=260) ---
set_param([modelName, '/T_diff'],      'Position', S(500, 260));
set_param([modelName, '/Q_cool'],      'Position', S(590, 260));
set_param([modelName, '/P_net'],       'Position', S(680, 260));
set_param([modelName, '/Gain_1C'],     'Position', S(780, 260));
set_param([modelName, '/Integrator'],  'Position', M(890, 260));
set_param([modelName, '/Monitor_dT'],  'Position', S(680, 320));

% --- 无控制路径 (x=370~940, y=440~520) ---
set_param([modelName, '/P_loss_nc'],     'Position', S(370, 440));
set_param([modelName, '/R_th_nc'],       'Position', S(370, 500));
set_param([modelName, '/T_diff_nc'],     'Position', S(500, 455));
set_param([modelName, '/Q_cool_nc'],     'Position', S(590, 455));
set_param([modelName, '/P_net_nc'],      'Position', S(680, 455));
set_param([modelName, '/Gain_1C_nc'],    'Position', S(780, 455));
set_param([modelName, '/Integrator_nc'], 'Position', M(890, 455));
set_param([modelName, '/Monitor_dT_nc'], 'Position', S(680, 515));

% --- 输出块 (x=1020) ---
set_param([modelName, '/Scope_Thermal'], 'Position', SC(1020, 210));
set_param([modelName, '/Scope_Control'], 'Position', SC(1020, 305));
set_param([modelName, '/ToWS_Tmod'],     'Position', MW(1020, 400));
set_param([modelName, '/ToWS_Vbus'],     'Position', MW(1020, 445));
set_param([modelName, '/ToWS_Fan'],      'Position', MW(1020, 490));
set_param([modelName, '/ToWS_Tmod_nc'],  'Position', MW(1020, 540));
set_param([modelName, '/ToWS_dT_nc'],    'Position', MW(1020, 590));
set_param([modelName, '/ToWS_Tamb'],     'Position', MW(1020, 640));

%% ===== 功能模块注释: 彩色区域 + 文字说明 =====
%  Note 支持独立的 BackgroundColor(填充) 和 ForegroundColor(文字)
fprintf('  添加功能注释...\n');

% ----- ① 环境温度输入 (Green block) -----
h = add_block('built-in/Note', [modelName '/Grp_Env']);
set_param(h, 'Position', [38, 18, 253, 73], ...
    'BackgroundColor', '[0.83, 0.94, 0.84]', ...
    'ForegroundColor', '[0.15, 0.15, 0.15]', ...
    'Text', sprintf('① 环境温度 T_{amb}\nFrom Workspace 读取 | Goto 分发'), ...
    'FontSize', '10', 'FontWeight', 'bold', 'DropShadow', 'off');

% ----- ② 防凝露控制器 (Orange block) -----
h = add_block('built-in/Note', [modelName '/Grp_Ctrl']);
set_param(h, 'Position', [208, 32, 543, 150], ...
    'BackgroundColor', '[0.98, 0.84, 0.63]', ...
    'ForegroundColor', '[0.15, 0.15, 0.15]', ...
    'Text', sprintf(['② 防凝露控制器 (MATLAB Function)\n' ...
                     '  T_s = 20s, 分级温差判断:\n' ...
                     '  DeltaT > 7  => 大幅升压 (dV=68V)\n' ...
                     '  4 < DeltaT < 7 => 适度升压 (dV=40V)\n' ...
                     '  DeltaT < 5 => 降压恢复 + 风扇恢复']), ...
    'FontSize', '10', 'FontWeight', 'bold', 'DropShadow', 'off');

% ----- ③ 执行与反馈 (Light Blue block) -----
h = add_block('built-in/Note', [modelName '/Grp_Act']);
set_param(h, 'Position', [598, 40, 953, 152], ...
    'BackgroundColor', '[0.68, 0.84, 0.95]', ...
    'ForegroundColor', '[0.15, 0.15, 0.15]', ...
    'Text', sprintf(['③ 执行器 + 反馈延迟\n' ...
                     '  电压: 一阶惯性平滑 (tau=8s) -> Saturation 限幅 [580,880]V\n' ...
                     '  风扇: Saturation 限幅 [50,100]%%\n' ...
                     '  Unit Delay (dt=0.5s) 保存 V/Fan -> 反馈至控制器']), ...
    'FontSize', '10', 'FontWeight', 'bold', 'DropShadow', 'off');

% ----- ④ 功率与热阻计算 (Lavender block) -----
h = add_block('built-in/Note', [modelName '/Grp_PwrRth']);
set_param(h, 'Position', [40, 175, 422, 387], ...
    'BackgroundColor', '[0.84, 0.74, 0.89]', ...
    'ForegroundColor', '[0.15, 0.15, 0.15]', ...
    'Text', sprintf(['④ 功率损耗 & 热阻计算\n' ...
                     '  P_{loss} = P_{loss0} x (V/V_{nom})^{1.8}\n' ...
                     '  (开关损耗+导通损耗随电压升高)\n' ...
                     '  R_{th} = R_{th0} x fan_{nom} / fan\n' ...
                     '  (风扇转速越低, 等效热阻越大)']), ...
    'FontSize', '10', 'FontWeight', 'bold', 'DropShadow', 'off');

% ----- ⑤ 有控制热动态 (Cyan block) -----
h = add_block('built-in/Note', [modelName '/Grp_Thermal']);
set_param(h, 'Position', [408, 220, 1028, 360], ...
    'BackgroundColor', '[0.64, 0.89, 0.84]', ...
    'ForegroundColor', '[0.15, 0.15, 0.15]', ...
    'Text', sprintf(['⑤ 有控制热动态 (Forward Euler, dt=0.5s)\n' ...
                     '  dT/dt = [P_{loss} - (T_{mod}-T_{amb})/R_{th}] / C_{th}\n' ...
                     '  C_{th}=1000 J/K | 监控 dT = T_{amb} - T_{mod}\n' ...
                     '  输入: P_loss(受控) + R_th(受控) => T_mod(有控制)']), ...
    'FontSize', '10', 'FontWeight', 'bold', 'DropShadow', 'off');

% ----- ⑥ 无控制对照 (Pink block) -----
h = add_block('built-in/Note', [modelName '/Grp_NoCtrl']);
set_param(h, 'Position', [356, 426, 1028, 558], ...
    'BackgroundColor', '[0.96, 0.72, 0.69]', ...
    'ForegroundColor', '[0.15, 0.15, 0.15]', ...
    'Text', sprintf(['⑥ 无控制对照路径 (固定参数, 不调节)\n' ...
                     '  V=620V  fan=100%%  P_{loss}=P_{loss0}  R_{th}=R_{th0}\n' ...
                     '  相同 T_{amb} 激励, dT/dt = [P_{loss0}-(T_{mod}-T_{amb})/R_{th0}]/C_{th}\n' ...
                     '  与路径⑤对比, 量化防凝露控制的效果']), ...
    'FontSize', '10', 'FontWeight', 'bold', 'DropShadow', 'off');

% ----- ⑦ 输出与监控 (Yellow block) -----
h = add_block('built-in/Note', [modelName '/Grp_Out']);
set_param(h, 'Position', [1012, 198, 1107, 653], ...
    'BackgroundColor', '[0.98, 0.91, 0.62]', ...
    'ForegroundColor', '[0.15, 0.15, 0.15]', ...
    'Text', sprintf('⑦ 输出与监控\n\n\nScope\n\n\n\n\nTo\nWork\nspace\n(StopFcn\n绘图)'), ...
    'FontSize', '9', 'FontWeight', 'bold', 'DropShadow', 'off');

% ----- 信号流关键标签 (Light gray, small font) -----
addLabel = @(name, x, y, w, h, txt) ...
    set_param(add_block('built-in/Note', [modelName '/' name]), ...
        'Position', [x, y, x+w, y+h], 'Text', txt, ...
        'FontSize', '8', 'FontWeight', 'normal', ...
        'ForegroundColor', '[0.4, 0.4, 0.4]', ...
        'BackgroundColor', 'white', 'DropShadow', 'off');

addLabel('Lbl_TambSig',  180, 68, 80, 16, 'Goto: T_{amb}');
addLabel('Lbl_TmodSig',  950, 290, 80, 16, 'Goto: T_{mod}');
addLabel('Lbl_VbusSig',  790, 70, 45, 16, 'V_{bus}');
addLabel('Lbl_FanSig',   790, 130, 40, 16, 'Fan');
addLabel('Lbl_Feedback', 540, 80, 65, 28, '<-V_{bus}\n<-Fan');
addLabel('Lbl_CtrlOut',  510, 288, 110, 16, '-> 有控制 T_{mod}');
addLabel('Lbl_NoCtrlOut',510, 483, 110, 16, '-> 无控制 T_{mod}');

%% ===== 保存模型 =====
save_system(modelName);

fprintf('\n模型 %s.slx 构建完成。\n', modelName);
fprintf('=== 参数匹配 simulate_anti_condensation.m ===\n');
fprintf('  C_th=%.0f | P_loss0=%.0f | dT_2nd=%.0f | dV_step=%.0f | fan_min=%.0f\n', ...
    C_th, P_loss0, dT_2nd, dV_step, fan_min);
fprintf('  功率损耗: P_loss ∝ V^1.8 | 电压平滑: τ=8s 一阶惯性\n');
fprintf('  环境温度: 完整动态模型 | 无控制对照路径: 已添加\n');
fprintf('\n运行验证: validate_simulink_vs_script\n');

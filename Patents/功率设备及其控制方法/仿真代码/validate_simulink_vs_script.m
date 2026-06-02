%% Simulink 模型 vs MATLAB 脚本 交叉验证
%  运行 Simulink 模型并与 MATLAB 脚本仿真结果对比
%  确保两个独立实现使用相同参数和环境数据产生一致的物理行为

clear; clc;

% 统一白色背景
set(groot, 'DefaultFigureColor', 'w', 'DefaultAxesColor', 'w', ...
           'DefaultAxesXColor', 'k', 'DefaultAxesYColor', 'k', ...
           'DefaultTextColor', 'k', 'DefaultAxesFontSize', 12, ...
           'DefaultAxesFontName', 'Times New Roman', ...
           'DefaultTextFontName', 'Times New Roman', ...
           'DefaultLegendColor', 'w', 'DefaultLegendBox', 'off', ...
           'DefaultLegendTextColor', 'k');

fprintf('===== Simulink vs MATLAB 脚本 交叉验证 =====\n');

scriptDir = 'F:\Practices\Claude\功率设备及其控制方法\仿真代码';

%% ===== 共享参数和信号生成 =====
fprintf('\n[0/3] 生成共享环境温度数据...\n');

dt = 0.5;  T_sim = 5400;  t = (0:dt:T_sim)';  N = length(t);

C_th = 1000;  R_th0 = 0.48;  T_mod0 = 27;  P_loss0 = 12;
T_amb0 = 27;
V_nom = 620;  V_min = 580;  V_max = 880;
dT_th = 5;  dT_first = 4;  dT_2nd = 7;
dV_step = 40;  T_sample = 20;
fan_nom = 100;  fan_min = 50;

% 固定随机种子，确保MATLAB和Simulink使用完全相同的T_amb（匹配simulate_anti_condensation.m）
rng(42);
env_rise = 18 * (1 - exp(-t/220));
env_fall = exp(-t/10000);
T_amb = T_amb0 + env_rise .* env_fall ...
      + 1.5 * sin(2*pi*t/600) + 0.5 * sin(2*pi*t/2000) ...
      + 0.2 * randn(N,1);

% 存入基础工作区，供 Simulink From Workspace 块读取
T_amb_sim = [t, T_amb];  % N×2 矩阵: [时间列, 数据列]
assignin('base', 'T_amb_sim', T_amb_sim);
fprintf('环境温度已存入基础工作区 (rng=42, %.0f采样点)。\n', N);

%% ===== 1. 运行 MATLAB 脚本仿真（匹配 simulate_anti_condensation.m 逻辑）=====
fprintf('\n[1/3] 运行 MATLAB 脚本仿真...\n');

T_mod_ml = zeros(N,1);  T_mod_ml(1) = T_mod0;
V_ml = zeros(N,1);  V_ml(1) = V_nom;
fan_ml = zeros(N,1);  fan_ml(1) = fan_nom;
deltaT_ml = zeros(N,1);
ctrl_mode_ml = zeros(N,1);

last_sample_time = -T_sample;
V_target = V_nom;
V_bus = V_nom;
fan_speed = fan_nom;
cur_mode = 0;

for i = 1:N
    T_amb_i = T_amb(i);
    delta_T = T_amb_i - T_mod_ml(i);
    deltaT_ml(i) = delta_T;

    % 采样周期触发控制决策
    if t(i) - last_sample_time >= T_sample
        last_sample_time = t(i);
        if delta_T > dT_th
            if delta_T > dT_2nd
                V_target = min(V_bus + dV_step * 1.7, V_max);
                cur_mode = 2;
            elseif delta_T > dT_first
                V_target = min(V_bus + dV_step, V_max);
                cur_mode = 1;
            end
        else
            if V_bus > V_nom + 5
                V_target = max(V_nom, V_bus - dV_step * 0.6);
                if V_bus > V_nom + 80
                    fan_speed = max(fan_speed - 6, fan_min);
                end
            end
            if fan_speed < fan_nom - 3 && V_bus <= V_nom + 10
                fan_speed = min(fan_nom, fan_speed + 5);
            end
            cur_mode = 0;
        end
    end
    ctrl_mode_ml(i) = cur_mode;

    % 电压平滑（一阶惯性环节，tau=8s）— 每步执行
    tau_V = 8;
    V_bus = V_bus + (V_target - V_bus) * (1 - exp(-dt/tau_V));

    % 功率损耗和热动态
    P_loss = P_loss0 * (V_bus / V_nom)^1.8;
    R_th = R_th0 * (fan_nom / max(fan_speed, 1));
    dT_mod = (P_loss - (T_mod_ml(i) - T_amb_i) / R_th) / C_th;
    if i < N
        T_mod_ml(i+1) = T_mod_ml(i) + dT_mod * dt;
    end

    V_ml(i) = V_bus;
    fan_ml(i) = fan_speed;
end

fprintf('MATLAB脚本: T_mod终值=%.2f°C, V_bus峰值=%.0fV, fan最低=%.0f%%\n', ...
    T_mod_ml(end), max(V_ml), min(fan_ml));

%% ===== 2. 运行 Simulink 模型 =====
fprintf('\n[2/3] 运行 Simulink 模型仿真...\n');

% 重建模型（使用修正后的参数）
fprintf('  重建 Simulink 模型...\n');
run(fullfile(scriptDir, 'build_simulink_model.m'));

fprintf('  仿真中...\n');
simOut = sim('anti_condensation_control', 'StopTime', '5400');

T_mod_slx = simOut.get('T_mod_out').Data(:);
V_slx     = simOut.get('V_bus_out').Data(:);
fan_slx   = simOut.get('fan_out').Data(:);

fprintf('Simulink:  T_mod终值=%.2f°C, V_bus峰值=%.0fV, fan最低=%.0f%%\n', ...
    T_mod_slx(end), max(V_slx), min(fan_slx));

bdclose('all');

%% ===== 3. 交叉验证和可视化 =====
fprintf('\n[3/3] 交叉验证对比...\n');

% 对齐长度
n_slx = min(N, length(T_mod_slx));

% 计算差异
T_diff = T_mod_ml(1:n_slx) - T_mod_slx(1:n_slx);
V_diff = V_ml(1:n_slx) - V_slx(1:n_slx);
fan_diff = fan_ml(1:n_slx) - fan_slx(1:n_slx);

fprintf('T_mod 最大偏差: %.4f °C\n', max(abs(T_diff)));
fprintf('V_bus 最大偏差: %.1f V\n', max(abs(V_diff)));
fprintf('Fan   最大偏差: %.1f %%\n', max(abs(fan_diff)));

% 评估一致性
t_pass = max(abs(T_diff)) < 1.0;
v_pass = max(abs(V_diff)) < 50;
f_pass = max(abs(fan_diff)) < 10;

fprintf('\n偏差分析:\n');
fprintf('  - 温度偏差 %.2f°C → %s\n', max(abs(T_diff)), ...
    condstr(t_pass, 'PASS (< 1°C)', 'FAIL'));
fprintf('  - 电压偏差 %.0fV → %s\n', max(abs(V_diff)), ...
    condstr(v_pass, 'PASS (< 50V)', 'FAIL'));
fprintf('  - 风扇偏差 %.0f%% → %s\n', max(abs(fan_diff)), ...
    condstr(f_pass, 'PASS (< 10%%)', 'FAIL'));

if t_pass && v_pass && f_pass
    fprintf('\n*** 交叉验证 PASS: Simulink 与 MATLAB 脚本控制行为一致 ***\n');
else
    fprintf('\n*** 交叉验证 FAIL: 存在较大偏差，请检查参数 ***\n');
end

% 对比图
figure('Name', 'Simulink vs MATLAB 脚本 交叉验证（修正后）', ...
       'Position', [100, 100, 1400, 500], 'Color', 'w');

subplot(1,3,1);
plot(t(1:n_slx)/60, T_mod_ml(1:n_slx), 'b-', 'LineWidth', 2); hold on;
plot(t(1:n_slx)/60, T_mod_slx(1:n_slx), 'r--', 'LineWidth', 1.5);
plot(t(1:n_slx)/60, T_amb(1:n_slx), 'k:', 'LineWidth', 1);
xlabel('时间 (min)'); ylabel('温度 (°C)');
title('模块温度 T_{mod}', 'Color', 'k');
legend('MATLAB脚本', 'Simulink', '环境温度', 'Location', 'best');
grid on; box on;

subplot(1,3,2);
plot(t(1:n_slx)/60, V_ml(1:n_slx), 'b-', 'LineWidth', 2); hold on;
plot(t(1:n_slx)/60, V_slx(1:n_slx), 'r--', 'LineWidth', 1.5);
xlabel('时间 (min)'); ylabel('母线电压 (V)');
title('母线电压 V_{bus}', 'Color', 'k');
legend('MATLAB脚本', 'Simulink', 'Location', 'best');
grid on; box on;

subplot(1,3,3);
plot(t(1:n_slx)/60, fan_ml(1:n_slx), 'b-', 'LineWidth', 2); hold on;
plot(t(1:n_slx)/60, fan_slx(1:n_slx), 'r--', 'LineWidth', 1.5);
xlabel('时间 (min)'); ylabel('风扇转速 (%)');
title('风扇转速 Fan Speed', 'Color', 'k');
legend('MATLAB脚本', 'Simulink', 'Location', 'best');
ylim([0, 110]);
grid on; box on;

sgtitle('Simulink 模型 vs MATLAB 脚本 — 交叉验证（参数已修正）', ...
       'FontSize', 13, 'FontWeight', 'bold', 'Color', 'k');

saveas(gcf, fullfile(scriptDir, 'cross_validation.png'));
fprintf('\n对比图已保存: cross_validation.png\n');

fprintf('\n===== 交叉验证完成 =====\n');

function s = condstr(cond, t, f)
    if cond, s = t; else, s = f; end
end

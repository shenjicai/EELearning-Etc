%% Simulink 模型输出的防凝露控制效果对比图
%  与 simulate_anti_condensation.m 产生相同的四格对比图
%  自动运行 Simulink 模型（如果数据不存在）

% 安全清理
bdclose('all');

%% ===== 统一绘图风格 =====
set(groot, 'DefaultFigureColor', 'w', 'DefaultAxesColor', 'w', ...
           'DefaultAxesXColor', 'k', 'DefaultAxesYColor', 'k', ...
           'DefaultTextColor', 'k', 'DefaultAxesFontSize', 12, ...
           'DefaultAxesFontName', 'Times New Roman', ...
           'DefaultTextFontName', 'Times New Roman', ...
           'DefaultLegendColor', 'w', 'DefaultLegendBox', 'off', ...
           'DefaultLegendTextColor', 'k');

scriptDir = 'F:\Practices\Claude\功率设备及其控制方法\仿真代码';
cd(scriptDir);

fprintf('===== Simulink 防凝露控制效果对比 =====\n');

%% ===== 参数 =====
dt = 0.5;  T_sim = 5400;  t = (0:dt:T_sim)';  N = length(t);
dT_threshold = 5;  dT_second = 7;

%% ===== 步骤1: 生成 T_amb 数据并重建运行 Simulink 模型 =====
fprintf('[1/3] 生成环境温度数据...\n');
T_amb0 = 27;
rng(42);
env_rise = 18 * (1 - exp(-t/220));
env_fall = exp(-t/10000);
T_amb = T_amb0 + env_rise .* env_fall ...
      + 1.5 * sin(2*pi*t/600) + 0.5 * sin(2*pi*t/2000) ...
      + 0.2 * randn(N,1);
T_amb_sim = [t, T_amb];  % N×2 矩阵
assignin('base', 'T_amb_sim', T_amb_sim);

fprintf('[2/3] 重建并运行 Simulink 模型...\n');
run(fullfile(scriptDir, 'build_simulink_model.m'));
simOut = sim('anti_condensation_control', 'StopTime', '5400');
bdclose('all');

%% ===== 步骤2: 读取仿真结果 =====
fprintf('[3/3] 生成对比图...\n');

T_mod_ctrl = simOut.get('T_mod_out').Data(:);
T_mod_nc   = simOut.get('T_mod_nc_out').Data(:);
T_amb_vec  = T_amb;  % 使用本地生成的 T_amb

% 对齐长度
n = min(N, min(length(T_mod_ctrl), length(T_mod_nc)));
t_min = t(1:n) / 60;
T_amb_vec = T_amb_vec(1:n);
T_mod_ctrl = T_mod_ctrl(1:n);
T_mod_nc   = T_mod_nc(1:n);

% 计算温差
dT_ctrl = T_amb_vec - T_mod_ctrl;
dT_nc   = T_amb_vec - T_mod_nc;

%% ===== 步骤3: 计算风险指标 =====
in_risk_ctrl = dT_ctrl > dT_threshold;
in_risk_nc   = dT_nc > dT_threshold;

risk_duration_ctrl = sum(in_risk_ctrl) * dt / 60;
risk_duration_nc   = sum(in_risk_nc) * dt / 60;
peak_dT_ctrl = max(dT_ctrl);
peak_dT_nc   = max(dT_nc);
exposure_ctrl = sum(max(dT_ctrl - dT_threshold, 0)) * dt / 60;
exposure_nc   = sum(max(dT_nc - dT_threshold, 0)) * dt / 60;

fprintf('\n--- 风险指标 ---\n');
fprintf('风险时长: %.1f min (有控制) vs %.1f min (无控制) -> 缩短 %.1f%%\n', ...
    risk_duration_ctrl, risk_duration_nc, ...
    (risk_duration_nc - risk_duration_ctrl)/risk_duration_nc*100);
fprintf('峰值温差: %.2f °C (有控制) vs %.2f °C (无控制) -> 降低 %.1f%%\n', ...
    peak_dT_ctrl, peak_dT_nc, (peak_dT_nc - peak_dT_ctrl)/peak_dT_nc*100);
fprintf('暴露量:   %.1f °C*min (有控制) vs %.1f °C*min (无控制) -> 减少 %.1f%%\n', ...
    exposure_ctrl, exposure_nc, (exposure_nc - exposure_ctrl)/exposure_nc*100);

%% ===== 步骤4: 生成四格对比图 =====
figure('Name', '防凝露控制效果对比 — Simulink 仿真', ...
       'Position', [100, 100, 1400, 500], 'Color', 'w');

% 子图1: 模块温度对比
subplot(1,4,1);
plot(t_min, T_mod_ctrl, 'b-', 'LineWidth', 1.8); hold on;
plot(t_min, T_mod_nc, 'r--', 'LineWidth', 1.5);
plot(t_min, T_amb_vec, 'k:', 'LineWidth', 1);
xlabel('时间 (min)'); ylabel('温度 (°C)');
title('模块温度对比', 'Color', 'k', 'FontWeight', 'bold');
legend('有控制', '无控制', '环境', 'Location', 'best');
grid on; box on;

% 子图2: 温差对比（全程）
subplot(1,4,2);
plot(t_min, dT_ctrl, 'b-', 'LineWidth', 1.8); hold on;
plot(t_min, dT_nc, 'r--', 'LineWidth', 1.5);
yline(dT_threshold, 'k--', '凝露阈值', 'LineWidth', 1.5);
xlabel('时间 (min)'); ylabel('\Delta T = T_{amb} - T_{mod} (°C)');
title('温差对比 (全程)', 'Color', 'k', 'FontWeight', 'bold');
legend('有控制', '无控制', 'Location', 'best');
grid on; box on;

% 子图3: 前20分钟瞬态放大
subplot(1,4,3);
idx_20 = find(t_min <= 20);
plot(t_min(idx_20), dT_ctrl(idx_20), 'b-', 'LineWidth', 1.8); hold on;
plot(t_min(idx_20), dT_nc(idx_20), 'r--', 'LineWidth', 1.5);
yline(dT_threshold, 'k--', '凝露阈值', 'LineWidth', 1.5);
yline(dT_second, 'm:', '第二温差', 'LineWidth', 1);
xlabel('时间 (min)'); ylabel('\Delta T (°C)');
title('前20分钟瞬态放大', 'Color', 'k', 'FontWeight', 'bold');
legend('有控制', '无控制', 'Location', 'best');
grid on; box on;

% 子图4: 防凝露效果柱状图
subplot(1,4,4);
bar_data = [risk_duration_ctrl, risk_duration_nc;
            peak_dT_ctrl, peak_dT_nc;
            exposure_ctrl, exposure_nc]';
b = bar(bar_data);
b(1).FaceColor = 'b'; b(2).FaceColor = 'r';
set(gca, 'XTickLabel', {'风险时长(min)', '峰值温差(°C)', '暴露量(°C·min)'});
legend('有控制', '无控制', 'Location', 'northwest');
title('防凝露效果对比', 'Color', 'k', 'FontWeight', 'bold');
ylabel('指标值');
grid on; box on;

sgtitle('防凝露控制效果对比 — Simulink 模型仿真 (专利 CN 121984334 A)', ...
       'FontSize', 13, 'FontWeight', 'bold', 'Color', 'k');

saveas(gcf, fullfile(scriptDir, 'simulink_comparison.png'));
fprintf('\n对比图已保存: simulink_comparison.png\n');
fprintf('\n===== Simulink 对比图生成完成 =====\n');

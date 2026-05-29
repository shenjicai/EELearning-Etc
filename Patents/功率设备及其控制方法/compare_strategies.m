%% 防凝露控制策略对比仿真
%  三路对比：无控制 vs 仅升压 vs 升压+降风扇（本专利CN 121984334 A）
%  演示每种策略在相同热环境下的凝露防护效果

clear; clc;

% 统一白色背景
set(groot, 'DefaultFigureColor', 'w', 'DefaultAxesColor', 'w', ...
           'DefaultAxesXColor', 'k', 'DefaultAxesYColor', 'k', ...
           'DefaultTextColor', 'k', 'DefaultAxesFontSize', 12, ...
           'DefaultAxesFontName', 'Times New Roman', ...
           'DefaultTextFontName', 'Times New Roman', ...
           'DefaultLegendColor', 'w', 'DefaultLegendBox', 'off', ...
           'DefaultLegendTextColor', 'k');

%% ===================== 仿真参数 =====================
dt = 0.5;  T_sim = 3600;  t = (0:dt:T_sim)';  N = length(t);

% 热模型
C_th = 1000;  R_th0 = 0.48;  T_mod0 = 27;
T_amb_base = 27;
P_loss0 = 12;

% 电压范围
V_nom = 620;  V_min = 580;  V_max = 880;

% 控制参数
dT_threshold = 5;  dT_second = 7;
dV_step = 40;  T_sample = 20;
fan_nom = 100;  fan_min = 50;

% 环境温度：壳体快速升温后缓慢降温 + 周期性负载波动
env_rise = 18 * (1 - exp(-t/220));
env_fall = exp(-t/10000);
T_amb = T_amb_base + env_rise .* env_fall ...
      + 1.5 * sin(2*pi*t/600) + 0.5 * sin(2*pi*t/2000) ...
      + 0.2 * randn(N,1);

%% ===================== 策略1: 无控制（基准） =====================
fprintf('===== 策略对比仿真 =====\n\n');

T_mod_nc = zeros(N,1);  T_mod_nc(1) = T_mod0;
deltaT_nc = zeros(N,1);
V_nc = V_nom * ones(N,1);
fan_nc = fan_nom * ones(N,1);

for i = 1:N
    deltaT_nc(i) = T_amb(i) - T_mod_nc(i);
    P_loss = P_loss0;  % 固定损耗，无电压调节
    R_th = R_th0;
    dT = (P_loss - (T_mod_nc(i) - T_amb(i))/R_th) / C_th;
    if i < N, T_mod_nc(i+1) = T_mod_nc(i) + dT*dt; end
end

risk_dur_nc = sum(deltaT_nc > dT_threshold) * dt / 60;
peak_dT_nc = max(deltaT_nc);
exposure_nc = sum(max(deltaT_nc - dT_threshold, 0)) * dt / 60;
fprintf('策略1 (无控制): 风险时长=%.1fmin | 峰值ΔT=%.2f°C | 暴露量=%.1f °C·min | 最终ΔT=%.2f°C\n', ...
    risk_dur_nc, peak_dT_nc, exposure_nc, deltaT_nc(end));

%% ===================== 策略2: 仅升压（无风扇调节） =====================
T_mod_v = zeros(N,1);  T_mod_v(1) = T_mod0;
deltaT_v = zeros(N,1);
V_v = zeros(N,1);  V_v(1) = V_nom;  V_cur = V_nom;
fan_v = fan_nom * ones(N,1);
last_sample = -T_sample;

for i = 1:N
    deltaT_v(i) = T_amb(i) - T_mod_v(i);

    if t(i) - last_sample >= T_sample
        last_sample = t(i);
        if deltaT_v(i) > dT_threshold
            V_cur = min(V_cur + dV_step, V_max);
        else
            if V_cur > V_nom + 5, V_cur = max(V_nom, V_cur - dV_step*0.6); end
        end
    end
    V_v(i) = V_cur;
    P_loss = P_loss0 * (V_cur/V_nom)^1.8;
    R_th = R_th0;
    dT = (P_loss - (T_mod_v(i) - T_amb(i))/R_th) / C_th;
    if i < N, T_mod_v(i+1) = T_mod_v(i) + dT*dt; end
end

risk_dur_v = sum(deltaT_v > dT_threshold) * dt / 60;
peak_dT_v = max(deltaT_v);
exposure_v = sum(max(deltaT_v - dT_threshold, 0)) * dt / 60;
fprintf('策略2 (仅升压): 风险时长=%.1fmin | 峰值ΔT=%.2f°C | 暴露量=%.1f °C·min | 最终ΔT=%.2f°C | V_peak=%.0fV\n', ...
    risk_dur_v, peak_dT_v, exposure_v, deltaT_v(end), max(V_v));

%% ===================== 策略3: 升压+降风扇（本专利） =====================
T_mod_p = zeros(N,1);  T_mod_p(1) = T_mod0;
deltaT_p = zeros(N,1);
V_p = zeros(N,1);  V_p(1) = V_nom;  V_cur = V_nom;
fan_p = zeros(N,1);  fan_p(1) = fan_nom;  fan_cur = fan_nom;
last_sample = -T_sample;

for i = 1:N
    deltaT_p(i) = T_amb(i) - T_mod_p(i);

    if t(i) - last_sample >= T_sample
        last_sample = t(i);
        if deltaT_p(i) > dT_threshold
            if deltaT_p(i) > dT_second
                V_cur = min(V_cur + dV_step * 1.7, V_max);
            else
                V_cur = min(V_cur + dV_step, V_max);
            end
        else
            if V_cur > V_nom + 5
                V_cur = max(V_nom, V_cur - dV_step*0.6);
                if V_cur > V_nom + 80, fan_cur = max(fan_cur - 6, fan_min); end
            end
            if fan_cur < fan_nom - 3 && V_cur <= V_nom + 10
                fan_cur = min(fan_nom, fan_cur + 5);
            end
        end
    end
    V_p(i) = V_cur;  fan_p(i) = fan_cur;
    P_loss = P_loss0 * (V_cur/V_nom)^1.8;
    R_th = R_th0 * (fan_nom / max(fan_cur, 1));
    dT = (P_loss - (T_mod_p(i) - T_amb(i))/R_th) / C_th;
    if i < N, T_mod_p(i+1) = T_mod_p(i) + dT*dt; end
end

risk_dur_p = sum(deltaT_p > dT_threshold) * dt / 60;
peak_dT_p = max(deltaT_p);
exposure_p = sum(max(deltaT_p - dT_threshold, 0)) * dt / 60;
fprintf('策略3 (升压+降风扇): 风险时长=%.1fmin | 峰值ΔT=%.2f°C | 暴露量=%.1f °C·min | 最终ΔT=%.2f°C | V_peak=%.0fV | fan_min=%.0f%%\n', ...
    risk_dur_p, peak_dT_p, exposure_p, deltaT_p(end), max(V_p), min(fan_p));

%% ===================== 对比结果汇总 =====================
fprintf('\n===== 策略对比汇总 =====\n');
fprintf('指标                | 无控制   | 仅升压   | 升压+降风扇(专利)\n');
fprintf('--------------------|----------|----------|-----------------\n');
fprintf('风险持续时间 (min)   | %7.1f  | %7.1f  | %7.1f\n', risk_dur_nc, risk_dur_v, risk_dur_p);
fprintf('峰值温差 (°C)        | %7.2f  | %7.2f  | %7.2f\n', peak_dT_nc, peak_dT_v, peak_dT_p);
fprintf('凝露暴露量 (°C·min)  | %7.1f  | %7.1f  | %7.1f\n', exposure_nc, exposure_v, exposure_p);
fprintf('母线电压峰值 (V)     | %7.0f  | %7.0f  | %7.0f\n', V_nom, max(V_v), max(V_p));
fprintf('风扇转速最低 (%%)     | %7.0f  | %7.0f  | %7.0f\n', fan_nom, fan_nom, min(fan_p));
fprintf('最终模块温度 (°C)    | %7.2f  | %7.2f  | %7.2f\n', T_mod_nc(end), T_mod_v(end), T_mod_p(end));

%% ===================== 可视化 =====================
figure('Name', '防凝露策略三路对比 — 专利 CN 121984334 A', ...
       'Position', [50, 50, 1500, 900], 'Color', 'w');

% 子图1: 温差对比
subplot(2,3,1);
plot(t/60, deltaT_nc, 'k-', 'LineWidth', 1.2); hold on;
plot(t/60, deltaT_v,  'b-', 'LineWidth', 1.8);
plot(t/60, deltaT_p,  'r-', 'LineWidth', 1.8);
yline(dT_threshold, 'k--', '凝露阈值', 'LineWidth', 1.5);
yline(dT_second, 'm:', '第二温差', 'LineWidth', 1);
xlabel('时间 (min)'); ylabel('\Delta T = T_{amb} - T_{mod} (°C)');
title('温差对比 (三策略)', 'Color', 'k');
legend('无控制', '仅升压', '升压+降风扇(专利)', 'Location', 'best');
grid on; box on;

% 子图2: 前15分钟瞬态放大
subplot(2,3,2);
idx = find(t/60 <= 15);
plot(t(idx)/60, deltaT_nc(idx), 'k-', 'LineWidth', 1.2); hold on;
plot(t(idx)/60, deltaT_v(idx),  'b-', 'LineWidth', 1.8);
plot(t(idx)/60, deltaT_p(idx),  'r-', 'LineWidth', 1.8);
yline(dT_threshold, 'k--', '凝露阈值', 'LineWidth', 1.5);
xlabel('时间 (min)'); ylabel('\Delta T (°C)');
title('前15分钟瞬态放大', 'Color', 'k');
legend('无控制', '仅升压', '升压+降风扇(专利)', 'Location', 'best');
grid on; box on;

% 子图3: 模块温度对比
subplot(2,3,3);
plot(t/60, T_mod_nc, 'k-', 'LineWidth', 1.2); hold on;
plot(t/60, T_mod_v,  'b-', 'LineWidth', 1.8);
plot(t/60, T_mod_p,  'r-', 'LineWidth', 1.8);
plot(t/60, T_amb, 'g:', 'LineWidth', 1);
xlabel('时间 (min)'); ylabel('温度 (°C)');
title('模块温度对比', 'Color', 'k');
legend('无控制', '仅升压', '升压+降风扇(专利)', '环境温度', 'Location', 'best');
grid on; box on;

% 子图4: 母线电压对比
subplot(2,3,4);
plot(t/60, V_nc, 'k-', 'LineWidth', 1.2); hold on;
plot(t/60, V_v,  'b-', 'LineWidth', 1.8);
plot(t/60, V_p,  'r-', 'LineWidth', 1.8);
xlabel('时间 (min)'); ylabel('母线电压 (V)');
title('母线电压调节对比', 'Color', 'k');
legend('无控制 (固定)', '仅升压', '升压+降风扇(专利)', 'Location', 'best');
grid on; box on;

% 子图5: 风扇转速对比
subplot(2,3,5);
plot(t/60, fan_nc, 'k-', 'LineWidth', 1.2); hold on;
plot(t/60, fan_v,  'b-', 'LineWidth', 1.8);
plot(t/60, fan_p,  'r-', 'LineWidth', 1.8);
xlabel('时间 (min)'); ylabel('风扇转速 (%)');
title('风扇转速对比', 'Color', 'k');
legend('无控制', '仅升压', '升压+降风扇(专利)', 'Location', 'best');
ylim([0, 110]);
grid on; box on;

% 子图6: 关键指标柱状图
subplot(2,3,6);
bar_data = [risk_dur_nc, risk_dur_v, risk_dur_p;
            peak_dT_nc, peak_dT_v, peak_dT_p;
            exposure_nc, exposure_v, exposure_p]';
b = bar(bar_data, 0.5);
b(1).FaceColor = [0.3 0.3 0.3]; b(2).FaceColor = 'b'; b(3).FaceColor = 'r';
set(gca, 'XTickLabel', {'风险时长(min)', '峰值温差(°C)', '暴露量(°C·min)'});
legend('无控制', '仅升压', '升压+降风扇(专利)', 'Location', 'northwest');
ylabel('指标值');
title('防凝露效果三路对比 (越小越好)', 'Color', 'k');
grid on; box on;

sgtitle('防凝露控制策略三路对比 — 专利 CN 121984334 A', ...
       'FontSize', 14, 'FontWeight', 'bold', 'Color', 'k');

% 保存图像
scriptDir = 'F:\Practices\Claude\功率设备及其控制方法';
saveas(gcf, fullfile(scriptDir, 'strategy_comparison.png'));
fprintf('\n对比图已保存: strategy_comparison.png\n');
fprintf('仿真完成。\n');

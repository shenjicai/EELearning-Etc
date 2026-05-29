%% 功率设备防凝露控制方法仿真
%  基于专利 CN 121984334 A — 阳光电源股份有限公司
%  模拟母线电压调节 + 风扇转速控制的防凝露策略
%
%  核心原理：
%    当功率模块运行温度(T_mod)低于壳体内环境温度(T_amb)且差值超过阈值时，
%    说明模块温度接近露点温度，存在凝露风险。
%    通过提升母线电压 → 增加模块功率损耗 → 模块自发热 → 提升模块温度 → 消除凝露风险。
%    同时辅以降低风扇转速 → 减少散热 → 加速模块温升。

clear; clc; close all;

% 统一白色背景
set(groot, 'DefaultFigureColor', 'w', 'DefaultAxesColor', 'w', ...
           'DefaultAxesXColor', 'k', 'DefaultAxesYColor', 'k', ...
           'DefaultTextColor', 'k', 'DefaultAxesFontSize', 12, ...
           'DefaultAxesFontName', 'Times New Roman', ...
           'DefaultTextFontName', 'Times New Roman', ...
           'DefaultLegendColor', 'w', 'DefaultLegendBox', 'off', ...
           'DefaultLegendTextColor', 'k');

%% ===================== 仿真参数配置 =====================
% --- 时间参数 ---
dt       = 0.5;        % 仿真步长 (s)
T_sim    = 5400;       % 总仿真时间 (s) = 90分钟
t        = (0:dt:T_sim)';
N        = length(t);

% --- 热模型参数 ---
C_th     = 1000;       % 功率模块热容 (J/K)，决定温度变化惯性
R_th0    = 0.48;       % 额定热阻 (K/W)，风扇全速时散热能力最强
T_amb0   = 27;         % 壳体内环境温度基准 (°C) — 初始与模块同温
T_mod0   = 27;         % 模块初始温度 (°C) — DC/DC模块刚从待机/冷态激活
P_loss0  = 12;         % 模块在额定母线电压下的损耗 (W) — 轻载工况

% --- 电气参数 ---
V_bus_nom = 620;       % 额定母线电压 (V)
V_bus_min = 580;       % 母线电压下限 (V) — 并网门槛电压
V_bus_max = 880;       % 母线电压上限 (V) — 器件耐压限制
V_bus     = V_bus_nom; % 当前母线电压

% --- 控制器参数 ---
delta_T_threshold = 5;   % 凝露风险温差阈值 (°C)
delta_T_first     = 4;   % 第一温差：轻度风险，仅升压
delta_T_second    = 7;   % 第二温差：高度风险，大幅升压
dV_step           = 40;  % 每周期电压增量 (V)
T_sample          = 20;  % 温度采样周期 (s)

% --- 风扇参数 ---
fan_speed_nom = 100;     % 额定风扇转速 (%)
fan_speed     = 100;     % 当前风扇转速 (%)
fan_min       = 50;      % 最低风扇转速 (%)

%% ===================== 预分配存储数组 =====================
T_mod       = zeros(N, 1);  T_mod(1)       = T_mod0;
V_bus_hist  = zeros(N, 1);  V_bus_hist(1)  = V_bus;
fan_hist    = zeros(N, 1);  fan_hist(1)    = fan_speed;
delta_T_hist = zeros(N, 1);
P_loss_hist = zeros(N, 1);
ctrl_mode   = zeros(N, 1);  % 0=正常, 1=仅升压, 2=升压+降风扇

% --- 环境温度：壳体快速升温后缓慢降温 + 周期性负载波动 ---
% 模拟其余大功率模块突然满载运行→壳体快速升温→随后逐渐散热冷却
% T_amb 从27°C快速升至~45°C再缓慢回落至~28°C，叠加负载波动
env_rise = 18 * (1 - exp(-t/220));          % 快速升温：τ≈3.7min，升至27+18=45°C
env_fall = exp(-t/10000);                    % 缓慢冷却包络
T_amb_hist = T_amb0 + env_rise .* env_fall ...
           + 1.5 * sin(2*pi*t/600) + 0.5 * sin(2*pi*t/2000) ...
           + 0.2 * randn(N,1);

%% ===================== 主仿真循环 =====================
last_sample_time = -T_sample;
V_target = V_bus_nom;

fprintf('===== 功率设备防凝露控制仿真开始 =====\n');
fprintf('环境温度基准: %.0f°C | 模块初始温度: %.0f°C | 初始功率损耗: %.0f W\n', T_amb0, T_mod0, P_loss0);
fprintf('温差阈值: %.1f°C | 第一温差: %.1f°C | 第二温差: %.1f°C\n', ...
    delta_T_threshold, delta_T_first, delta_T_second);
fprintf('母线电压范围: [%.0f, %.0f] V\n\n', V_bus_min, V_bus_max);

cur_mode = 0;  % 当前控制模式状态（前向填充）

for i = 1:N
    T_amb_i = T_amb_hist(i);

    % ---- 计算温差 ----
    delta_T = T_amb_i - T_mod(i);
    delta_T_hist(i) = delta_T;

    % ---- 温度采样周期触发控制决策 ----
    if t(i) - last_sample_time >= T_sample
        last_sample_time = t(i);

        if delta_T > delta_T_threshold
            if delta_T > delta_T_second
                % 【高风险】大幅升压加速自发热
                V_target = min(V_bus + dV_step * 1.7, V_bus_max);
                cur_mode = 2;
            elseif delta_T > delta_T_first
                % 【轻度风险】适度升压
                V_target = min(V_bus + dV_step, V_bus_max);
                cur_mode = 1;
            end
        else
            % 【无风险】逐步恢复正常
            if V_bus > V_bus_nom + 5
                V_target = max(V_bus_nom, V_bus - dV_step * 0.6);
                % 升压回落期间适度降风扇保温，防止模块温度随环境冷却而回落
                if V_bus > V_bus_nom + 80
                    fan_speed = max(fan_speed - 6, fan_min);
                end
            end
            if fan_speed < fan_speed_nom - 3 && V_bus <= V_bus_nom + 10
                fan_speed = min(fan_speed_nom, fan_speed + 5);
            end
            cur_mode = 0;
        end
    end
    ctrl_mode(i) = cur_mode;  % 前向填充

    % ---- 母线电压平滑过渡（一阶惯性环节）----
    tau_V = 8;  % 电压调节时间常数
    V_bus = V_bus + (V_target - V_bus) * (1 - exp(-dt/tau_V));

    % ---- 功率损耗模型 ----
    % P_loss ∝ V_bus^1.8（开关损耗+导通损耗随电压升高而增加）
    P_loss = P_loss0 * (V_bus / V_bus_nom)^1.8;
    P_loss_hist(i) = P_loss;

    % ---- 热动态模型（一阶RC热网络）----
    % 风扇转速影响等效热阻：转速越低 → 热阻越大 → 散热越慢
    R_th = R_th0 * (fan_speed_nom / max(fan_speed, 1));
    % dT/dt = (P_loss - (T_mod - T_amb)/R_th) / C_th
    dT_mod = (P_loss - (T_mod(i) - T_amb_i) / R_th) / C_th;

    if i < N
        T_mod(i+1) = T_mod(i) + dT_mod * dt;
    end

    % ---- 存储历史 ----
    V_bus_hist(i) = V_bus;
    fan_hist(i)   = fan_speed;
end

%% ===================== 结果绘图 =====================
figure('Name', '功率设备防凝露控制仿真 — 专利 CN 121984334 A', ...
       'Position', [80, 80, 1400, 900], 'Color', 'w');

% ---- 子图1：温度曲线 ----
subplot(3,2,1);
plot(t/60, T_mod, 'b-', 'LineWidth', 1.8); hold on;
plot(t/60, T_amb_hist, 'r-', 'LineWidth', 1.2);
xlabel('时间 (min)', 'FontSize', 10);
ylabel('温度 (°C)', 'FontSize', 10);
title('功率模块温度 vs 环境温度', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'k');
legend('模块温度 T_{mod}', '环境温度 T_{amb}', 'Location', 'best');
grid on; box on;

% ---- 子图2：温差与阈值对比 ----
subplot(3,2,2);
plot(t/60, delta_T_hist, 'k-', 'LineWidth', 1.5); hold on;
yline(delta_T_threshold, 'r--', '凝露阈值', 'LineWidth', 1.5);
yline(delta_T_first,  'g--', '第一温差', 'LineWidth', 1.2);
yline(delta_T_second, 'm--', '第二温差', 'LineWidth', 1.2);
fill([t/60; flipud(t/60)], ...
     [zeros(N,1); flipud(max(delta_T_hist,0))], ...
     'b', 'FaceAlpha', 0.05, 'EdgeColor', 'none');
xlabel('时间 (min)', 'FontSize', 10);
ylabel('\Delta T = T_{amb} - T_{mod} (°C)', 'FontSize', 10);
title('温差监测与阈值判断', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'k');
legend('\Delta T', '凝露阈值', '第一温差', '第二温差', 'Location', 'best');
grid on; box on;

% ---- 子图3：母线电压 ----
subplot(3,2,3);
plot(t/60, V_bus_hist, 'b-', 'LineWidth', 1.8); hold on;
yline(V_bus_min, 'g--', 'V_{min} (并网门槛)', 'LineWidth', 1.2);
yline(V_bus_max, 'r--', 'V_{max} (耐压上限)', 'LineWidth', 1.2);
yline(V_bus_nom, 'k:', 'V_{nom}', 'LineWidth', 1);
xlabel('时间 (min)', 'FontSize', 10);
ylabel('母线电压 (V)', 'FontSize', 10);
title('直流母线电压调节', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'k');
ylim([V_bus_min-20, V_bus_max+20]);
grid on; box on;

% ---- 子图4：风扇转速 ----
subplot(3,2,4);
plot(t/60, fan_hist, 'b-', 'LineWidth', 1.8);
xlabel('时间 (min)', 'FontSize', 10);
ylabel('风扇转速 (%)', 'FontSize', 10);
title('散热风扇转速控制', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'k');
ylim([0, 110]);
grid on; box on;

% ---- 子图5：控制模式 ----
subplot(3,2,5);
stairs(t/60, ctrl_mode, 'b-', 'LineWidth', 1.8);
xlabel('时间 (min)', 'FontSize', 10);
ylabel('控制模式', 'FontSize', 10);
title('分级控制策略切换', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'k');
ylim([-0.3, 2.3]);
yticks([0, 1, 2]);
yticklabels({'0: 正常/恢复', '1: 适度升压', '2: 大幅升压'});
grid on; box on;

% ---- 子图6：功率损耗 ----
subplot(3,2,6);
plot(t/60, P_loss_hist, 'b-', 'LineWidth', 1.8);
xlabel('时间 (min)', 'FontSize', 10);
ylabel('模块损耗 (W)', 'FontSize', 10);
title('模块功率损耗 P_{loss} ∝ V_{bus}^{1.8}', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'k');
grid on; box on;

sgtitle('功率设备防凝露控制方法仿真 — 专利 CN 121984334 A', ...
       'FontSize', 14, 'FontWeight', 'bold', 'Color', 'k');

%% ===================== 关键指标输出 =====================
fprintf('\n===== 仿真结果统计 =====\n');
fprintf('初始温差: %.2f °C\n', delta_T_hist(1));
fprintf('最终温差: %.2f °C\n', delta_T_hist(end));
fprintf('最大温差: %.2f °C\n', max(delta_T_hist));
fprintf('最小温差: %.2f °C\n', min(delta_T_hist));
fprintf('最终母线电压: %.1f V\n', V_bus_hist(end));
fprintf('最终风扇转速: %.1f %%\n', fan_hist(end));
fprintf('最终模块温度: %.2f °C\n', T_mod(end));
fprintf('最终模块损耗: %.2f W\n', P_loss_hist(end));

% 控制模式统计
mode0_time = sum(ctrl_mode == 0) * dt / 60;
mode1_time = sum(ctrl_mode == 1) * dt / 60;
mode2_time = sum(ctrl_mode == 2) * dt / 60;
fprintf('\n--- 控制模式时间分布 ---\n');
fprintf('正常模式:   %.1f min (%.1f%%)\n', mode0_time, mode0_time/(T_sim/60)*100);
fprintf('仅升压模式: %.1f min (%.1f%%)\n', mode1_time, mode1_time/(T_sim/60)*100);
fprintf('升压+降风扇: %.1f min (%.1f%%)\n', mode2_time, mode2_time/(T_sim/60)*100);

% --- 控制动作时间线分析 ---
fprintf('\n--- 控制动作时间线 ---\n');
mode_changes = find(diff(ctrl_mode) ~= 0);
if isempty(mode_changes)
    fprintf('(无控制动作 — 全程处于安全区)\n');
else
    for k = 1:min(length(mode_changes), 15)
        idx = mode_changes(k);
        fprintf('t = %6.1f min: 模式 %d → %d | V_bus = %.0fV | fan = %.0f%% | ΔT = %.2f°C\n', ...
            t(idx)/60, ctrl_mode(idx), ctrl_mode(idx+1), ...
            V_bus_hist(idx), fan_hist(idx), delta_T_hist(idx));
    end
    if length(mode_changes) > 15
        fprintf('... (共 %d 次模式切换，仅显示前15次)\n', length(mode_changes));
    end
end

% --- 电压/风扇调节统计 ---
fprintf('\n--- 调节量统计 ---\n');
fprintf('母线电压峰值: %.0f V (增量 +%.0f V)\n', max(V_bus_hist), max(V_bus_hist) - V_bus_nom);
fprintf('风扇转速最低值: %.0f %%\n', min(fan_hist));
fprintf('模块损耗峰值: %.1f W (增量 +%.1f W)\n', max(P_loss_hist), max(P_loss_hist) - P_loss0);

%% ===================== 对比仿真：无控制策略 =====================
fprintf('\n\n===== 对比实验：无防凝露控制 =====\n');

V_bus_nc     = V_bus_nom * ones(N,1);
fan_nc       = fan_speed_nom * ones(N,1);
T_mod_nc     = zeros(N,1); T_mod_nc(1) = T_mod0;
delta_T_nc   = zeros(N,1);

for i = 1:N
    T_amb_i = T_amb_hist(i);
    delta_T_nc(i) = T_amb_i - T_mod_nc(i);
    P_loss_nc = P_loss0;  % 固定损耗
    R_th_nc   = R_th0;
    dT = (P_loss_nc - (T_mod_nc(i) - T_amb_i)/R_th_nc) / C_th;
    if i < N
        T_mod_nc(i+1) = T_mod_nc(i) + dT * dt;
    end
end

% 计算风险指标
in_risk_ctrl = delta_T_hist > delta_T_threshold;
in_risk_nc   = delta_T_nc   > delta_T_threshold;

risk_duration_ctrl = sum(in_risk_ctrl) * dt / 60;  % min
risk_duration_nc   = sum(in_risk_nc) * dt / 60;

peak_dT_ctrl = max(delta_T_hist);
peak_dT_nc   = max(delta_T_nc);

% 计算凝露暴露量（ΔT 在阈值以上的积分）
exposure_ctrl = sum(max(delta_T_hist - delta_T_threshold, 0)) * dt / 60;  % °C·min
exposure_nc   = sum(max(delta_T_nc   - delta_T_threshold, 0)) * dt / 60;

fprintf('风险持续时间: %.1f min (有控制) vs %.1f min (无控制) → 缩短 %.1f%%\n', ...
    risk_duration_ctrl, risk_duration_nc, ...
    (risk_duration_nc - risk_duration_ctrl)/risk_duration_nc*100);
fprintf('峰值温差: %.2f °C (有控制) vs %.2f °C (无控制) → 降低 %.1f%%\n', ...
    peak_dT_ctrl, peak_dT_nc, (peak_dT_nc - peak_dT_ctrl)/peak_dT_nc*100);
fprintf('凝露暴露量积分: %.1f °C·min (有控制) vs %.1f °C·min (无控制) → 减少 %.1f%%\n', ...
    exposure_ctrl, exposure_nc, (exposure_nc - exposure_ctrl)/exposure_nc*100);
fprintf('有控制无控制最终温差: %.2f °C vs %.2f °C\n', delta_T_hist(end), delta_T_nc(end));

% 对比图
figure('Name', '有无防凝露控制对比', 'Position', [100, 100, 1400, 500], 'Color', 'w');

subplot(1,4,1);
plot(t/60, T_mod, 'b-', 'LineWidth', 1.8); hold on;
plot(t/60, T_mod_nc, 'r--', 'LineWidth', 1.5);
plot(t/60, T_amb_hist, 'k:', 'LineWidth', 1);
xlabel('时间 (min)'); ylabel('温度 (°C)');
title('模块温度对比', 'Color', 'k');
legend('有控制', '无控制', '环境', 'Location', 'best');
grid on; box on;

subplot(1,4,2);
plot(t/60, delta_T_hist, 'b-', 'LineWidth', 1.8); hold on;
plot(t/60, delta_T_nc, 'r--', 'LineWidth', 1.5);
yline(delta_T_threshold, 'k--', '凝露阈值', 'LineWidth', 1.5);
xlabel('时间 (min)'); ylabel('\Delta T (°C)');
title('温差对比 (全程)', 'Color', 'k');
legend('有控制', '无控制', 'Location', 'best');
grid on; box on;

% 子图3：放大前20分钟瞬态（风险集中区）
subplot(1,4,3);
idx_20min = find(t/60 <= 20);
plot(t(idx_20min)/60, delta_T_hist(idx_20min), 'b-', 'LineWidth', 1.8); hold on;
plot(t(idx_20min)/60, delta_T_nc(idx_20min), 'r--', 'LineWidth', 1.5);
yline(delta_T_threshold, 'k--', '凝露阈值', 'LineWidth', 1.5);
yline(delta_T_second, 'm:', '第二温差', 'LineWidth', 1);
xlabel('时间 (min)'); ylabel('\Delta T (°C)');
title('前20分钟瞬态放大', 'Color', 'k');
legend('有控制', '无控制', 'Location', 'best');
grid on; box on;

% 子图4：关键指标柱状图
subplot(1,4,4);
bar_data = [risk_duration_ctrl, risk_duration_nc;
            peak_dT_ctrl, peak_dT_nc;
            exposure_ctrl, exposure_nc]';
b = bar(bar_data);
b(1).FaceColor = 'b'; b(2).FaceColor = 'r';
set(gca, 'XTickLabel', {'风险时长(min)', '峰值温差(°C)', '暴露量(°C·min)'});
legend('有控制', '无控制', 'Location', 'northwest');
title('防凝露效果对比', 'Color', 'k');
grid on; box on;

sgtitle('防凝露控制效果对比 — 专利方法 vs 无控制', ...
       'FontSize', 13, 'FontWeight', 'bold', 'Color', 'k');

fprintf('\n仿真完成。\n');

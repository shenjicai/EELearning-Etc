function [V_cmd, fan_cmd, mode] = anti_condensation_ctrl(T_mod, T_amb)
% 分级防凝露控制器 - 专利 CN 121984334 A
% 实现逐级增量调节 + 双机制协同控制
%
% 输入:  T_mod  — 功率模块温度 (°C)
%        T_amb  — 壳体内环境温度 (°C)
% 输出:  V_cmd  — 母线电压指令 (V)
%        fan_cmd — 风扇转速指令 (%)
%        mode   — 控制模式 (0=正常, 1=仅升压, 2=升压+降风扇)

persistent V_cur fan_cur tick
if isempty(V_cur)
    V_cur = 620;   % 额定母线电压
    fan_cur = 100; % 额定风扇转速
    tick = 0;      % 采样计数器
end

% 参数
dV_step = 15;      % 电压增量 (V)
V_min = 580;       % 电压下限
V_max = 880;       % 电压上限
dT_th  = 5;        % 凝露阈值 (°C)
dT_2nd = 8;        % 第二温差 (°C)
T_sample = 40;     % 采样周期 (仿真步数, dt=0.5s → 20s)

% 采样计数器
tick = tick + 1;
mode = 0;  % 默认正常模式

if tick >= T_sample
    tick = 0;
    deltaT = T_amb - T_mod;

    if deltaT > dT_th
        if deltaT > dT_2nd
            % 高风险：升压 + 降风扇
            V_cur = min(V_cur + dV_step, V_max);
            fan_cur = max(fan_cur - 12, 15);
            mode = 2;
        else
            % 轻度风险：仅升压
            V_cur = min(V_cur + dV_step, V_max);
            mode = 1;
        end
    else
        % 无风险：逐步恢复
        if V_cur > 625
            V_cur = max(620, V_cur - dV_step * 0.6);
        end
        if fan_cur < 97
            fan_cur = min(100, fan_cur + 6);
        end
    end
end

V_cmd = V_cur;
fan_cmd = fan_cur;
end

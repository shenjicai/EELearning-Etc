% Clark变换与Park变换 Simulink模型搭建脚本（From/Goto + 区域注释 + 对齐布局）
% 参考：三相坐标变换 (Clark变换与Park变换)

clear; clc;

%% 参数设置
f = 50;              % 基波频率 50Hz
A = 10;              % 电流幅值 10A
Ts = 1e-6;           % 仿真步长 1us
Tstop = 0.1;         % 仿真时间 0.1s

%% 创建新模型
modelName = 'Clark_Park_Transform';
if bdIsLoaded(modelName)
    close_system(modelName, 0);
end
new_system(modelName);
open_system(modelName);

%% 设置求解器
set_param(modelName, 'Solver', 'FixedStepDiscrete');
set_param(modelName, 'FixedStep', num2str(Ts));
set_param(modelName, 'StopTime', num2str(Tstop));

% ==================================================================
% 坐标网格常量（列 x, 行 y）—— 所有模块对齐到网格
% ==================================================================
% 列坐标（增宽间距，确保模块不重叠）
X0 = 30;   X1 = 180;  X2 = 340;  X3 = 600;  X4 = 860;
X5 = 1120; X6 = 1380; X7 = 1640; X8 = 1900;
% 行坐标
Y1 = 60;   Y2 = 150;  Y3 = 240;  Y4 = 340;
Y5 = 460;  Y6 = 580;  Y7 = 700;  Y8 = 800;
SUB_W = 120;  % 子系统统一宽度
SUB_H = 130;  % 子系统统一高度

%% ========== 1. 三相正弦电流源 + Goto标签 ==========
% 三相电流源 A相
add_block('simulink/Sources/Sine Wave', [modelName '/Ia'], ...
    'Amplitude', num2str(A), 'Frequency', num2str(2*pi*f), ...
    'Phase', num2str(pi/2), 'SampleTime', num2str(Ts), ...
    'Position', posGrid(X0, Y1, 60, 30));
add_block('simulink/Signal Routing/Goto', [modelName '/Goto_Ia'], ...
    'GotoTag', 'Ia_sig', 'TagVisibility', 'local', ...
    'Position', posGrid(X0+80, Y1, 40, 30));
add_line(modelName, 'Ia/1', 'Goto_Ia/1');

% 三相电流源 B相
add_block('simulink/Sources/Sine Wave', [modelName '/Ib'], ...
    'Amplitude', num2str(A), 'Frequency', num2str(2*pi*f), ...
    'Phase', num2str(pi/2 - 2*pi/3), 'SampleTime', num2str(Ts), ...
    'Position', posGrid(X0, Y2, 60, 30));
add_block('simulink/Signal Routing/Goto', [modelName '/Goto_Ib'], ...
    'GotoTag', 'Ib_sig', 'TagVisibility', 'local', ...
    'Position', posGrid(X0+80, Y2, 40, 30));
add_line(modelName, 'Ib/1', 'Goto_Ib/1');

% 三相电流源 C相
add_block('simulink/Sources/Sine Wave', [modelName '/Ic'], ...
    'Amplitude', num2str(A), 'Frequency', num2str(2*pi*f), ...
    'Phase', num2str(pi/2 + 2*pi/3), 'SampleTime', num2str(Ts), ...
    'Position', posGrid(X0, Y3, 60, 30));
add_block('simulink/Signal Routing/Goto', [modelName '/Goto_Ic'], ...
    'GotoTag', 'Ic_sig', 'TagVisibility', 'local', ...
    'Position', posGrid(X0+80, Y3, 40, 30));
add_line(modelName, 'Ic/1', 'Goto_Ic/1');

%% ========== 2. 角度信号 + Goto标签 ==========
add_block('simulink/Sources/Ramp', [modelName '/Theta_ramp'], ...
    'Slope', num2str(2*pi*f), ...
    'Position', posGrid(X0, Y4-40, 60, 30));
add_block('simulink/Sources/Constant', [modelName '/Const_2pi'], ...
    'Value', num2str(2*pi), ...
    'Position', posGrid(X0, Y4+20, 60, 30));
add_block('simulink/Math Operations/Math Function', [modelName '/mod2pi'], ...
    'Function', 'mod', ...
    'Position', posGrid(X0+100, Y4-10, 50, 40));
add_line(modelName, 'Const_2pi/1', 'mod2pi/2');
add_line(modelName, 'Theta_ramp/1', 'mod2pi/1');
add_block('simulink/Signal Routing/Goto', [modelName '/Goto_Theta'], ...
    'GotoTag', 'Theta_sig', 'TagVisibility', 'local', ...
    'Position', posGrid(X0+180, Y4-5, 40, 30));
add_line(modelName, 'mod2pi/1', 'Goto_Theta/1');

%% ========== 3. Clark变换子系统 + From接收 + Goto输出 ==========
clarkSub = [modelName '/Clark_Transform'];
add_block('simulink/Ports & Subsystems/Subsystem', clarkSub, ...
    'Position', posGrid(X2, Y1, SUB_W, SUB_H));
delete_line(clarkSub, 'In1/1', 'Out1/1');
% 在模块上显示核心公式
set_param(clarkSub, 'AttributesFormatString', ...
    'iα=ia  iβ=(ib-ic)/√3  i0=(ia+ib+ic)/3');
% 设置背景色区分功能区
set_param(clarkSub, 'BackgroundColor', '[0.7 1.0 0.7]');
delete_block([clarkSub '/In1']);
delete_block([clarkSub '/Out1']);

% 子系统内部输入端口
add_block('simulink/Ports & Subsystems/In1', [clarkSub '/Ia']);
add_block('simulink/Ports & Subsystems/In1', [clarkSub '/Ib']);
add_block('simulink/Ports & Subsystems/In1', [clarkSub '/Ic']);
set_param([clarkSub '/Ia'], 'Position', [60 50 90 70]);
set_param([clarkSub '/Ib'], 'Position', [60 120 90 140]);
set_param([clarkSub '/Ic'], 'Position', [60 190 90 210]);

% 子系统内部输出端口
add_block('simulink/Ports & Subsystems/Out1', [clarkSub '/Ialpha']);
add_block('simulink/Ports & Subsystems/Out1', [clarkSub '/Ibeta']);
add_block('simulink/Ports & Subsystems/Out1', [clarkSub '/I0']);
set_param([clarkSub '/Ialpha'], 'Position', [420 50 450 70]);
set_param([clarkSub '/Ibeta'], 'Position', [420 120 450 140]);
set_param([clarkSub '/I0'], 'Position', [420 190 450 210]);

% Clark内部：Ialpha = Ia
add_line(clarkSub, 'Ia/1', 'Ialpha/1');

% Clark内部：Ibeta = (Ib - Ic) / sqrt(3)
add_block('simulink/Math Operations/Subtract', [clarkSub '/Subtract_b_c'], ...
    'Position', [160 115 190 145]);
add_block('simulink/Math Operations/Gain', [clarkSub '/Gain_1_sqrt3'], ...
    'Gain', '1/sqrt(3)', 'Position', [260 118 300 142]);
add_line(clarkSub, 'Ib/1', 'Subtract_b_c/1');
add_line(clarkSub, 'Ic/1', 'Subtract_b_c/2');
add_line(clarkSub, 'Subtract_b_c/1', 'Gain_1_sqrt3/1');
add_line(clarkSub, 'Gain_1_sqrt3/1', 'Ibeta/1');

% Clark内部：I0 = (Ia + Ib + Ic) / 3
add_block('simulink/Math Operations/Add', [clarkSub '/Add_abc'], ...
    'Inputs', '+++', 'Position', [160 185 190 215]);
add_block('simulink/Math Operations/Gain', [clarkSub '/Gain_1_3'], ...
    'Gain', '1/3', 'Position', [260 188 300 212]);
add_line(clarkSub, 'Ia/1', 'Add_abc/1');
add_line(clarkSub, 'Ib/1', 'Add_abc/2');
add_line(clarkSub, 'Ic/1', 'Add_abc/3');
add_line(clarkSub, 'Add_abc/1', 'Gain_1_3/1');
add_line(clarkSub, 'Gain_1_3/1', 'I0/1');

% 外部 From/Goto
add_block('simulink/Signal Routing/From', [modelName '/From_Ia_Clark'], ...
    'GotoTag', 'Ia_sig', 'Position', posGrid(X2-60, Y1+10, 40, 30));
add_block('simulink/Signal Routing/From', [modelName '/From_Ib_Clark'], ...
    'GotoTag', 'Ib_sig', 'Position', posGrid(X2-60, Y1+50, 40, 30));
add_block('simulink/Signal Routing/From', [modelName '/From_Ic_Clark'], ...
    'GotoTag', 'Ic_sig', 'Position', posGrid(X2-60, Y1+90, 40, 30));
add_line(modelName, 'From_Ia_Clark/1', 'Clark_Transform/1');
add_line(modelName, 'From_Ib_Clark/1', 'Clark_Transform/2');
add_line(modelName, 'From_Ic_Clark/1', 'Clark_Transform/3');

% Clark输出 Goto
add_block('simulink/Signal Routing/Goto', [modelName '/Goto_Ialpha'], ...
    'GotoTag', 'Ialpha_sig', 'TagVisibility', 'local', ...
    'Position', posGrid(X2+160, Y1+10, 40, 30));
add_block('simulink/Signal Routing/Goto', [modelName '/Goto_Ibeta'], ...
    'GotoTag', 'Ibeta_sig', 'TagVisibility', 'local', ...
    'Position', posGrid(X2+160, Y1+50, 40, 30));
add_block('simulink/Signal Routing/Goto', [modelName '/Goto_I0'], ...
    'GotoTag', 'I0_sig', 'TagVisibility', 'local', ...
    'Position', posGrid(X2+160, Y1+90, 40, 30));
add_line(modelName, 'Clark_Transform/1', 'Goto_Ialpha/1');
add_line(modelName, 'Clark_Transform/2', 'Goto_Ibeta/1');
add_line(modelName, 'Clark_Transform/3', 'Goto_I0/1');

%% ========== 4. Park变换子系统 ==========
parkSub = [modelName '/Park_Transform'];
add_block('simulink/Ports & Subsystems/Subsystem', parkSub, ...
    'Position', posGrid(X3, Y1, SUB_W, SUB_H));
delete_line(parkSub, 'In1/1', 'Out1/1');
set_param(parkSub, 'AttributesFormatString', ...
    'id=iα·cosθ+iβ·sinθ  iq=-iα·sinθ+iβ·cosθ');
set_param(parkSub, 'BackgroundColor', '[1.0 0.8 0.5]');
delete_block([parkSub '/In1']);
delete_block([parkSub '/Out1']);

add_block('simulink/Ports & Subsystems/In1', [parkSub '/Ialpha']);
add_block('simulink/Ports & Subsystems/In1', [parkSub '/Ibeta']);
add_block('simulink/Ports & Subsystems/In1', [parkSub '/Theta']);
set_param([parkSub '/Ialpha'], 'Position', [60 50 90 70]);
set_param([parkSub '/Ibeta'], 'Position', [60 120 90 140]);
set_param([parkSub '/Theta'], 'Position', [60 220 90 240]);

add_block('simulink/Ports & Subsystems/Out1', [parkSub '/Id']);
add_block('simulink/Ports & Subsystems/Out1', [parkSub '/Iq']);
set_param([parkSub '/Id'], 'Position', [500 80 530 100]);
set_param([parkSub '/Iq'], 'Position', [500 180 530 200]);

% Park内部：sin/cos
add_block('simulink/Math Operations/Trigonometric Function', [parkSub '/cos_theta'], ...
    'Function', 'cos', 'Position', [160 210 200 250]);
add_block('simulink/Math Operations/Trigonometric Function', [parkSub '/sin_theta'], ...
    'Function', 'sin', 'Position', [160 280 200 320]);
add_line(parkSub, 'Theta/1', 'cos_theta/1');
add_line(parkSub, 'Theta/1', 'sin_theta/1');

% Park内部：Id = Ialpha*cos + Ibeta*sin
add_block('simulink/Math Operations/Product', [parkSub '/Prod_alpha_cos'], ...
    'Inputs', '**', 'Position', [280 60 310 100]);
add_block('simulink/Math Operations/Product', [parkSub '/Prod_beta_sin'], ...
    'Inputs', '**', 'Position', [280 120 310 160]);
add_block('simulink/Math Operations/Add', [parkSub '/Add_Id'], ...
    'Position', [380 80 410 110]);
add_line(parkSub, 'Ialpha/1', 'Prod_alpha_cos/1');
add_line(parkSub, 'cos_theta/1', 'Prod_alpha_cos/2');
add_line(parkSub, 'Ibeta/1', 'Prod_beta_sin/1');
add_line(parkSub, 'sin_theta/1', 'Prod_beta_sin/2');
add_line(parkSub, 'Prod_alpha_cos/1', 'Add_Id/1');
add_line(parkSub, 'Prod_beta_sin/1', 'Add_Id/2');
add_line(parkSub, 'Add_Id/1', 'Id/1');

% Park内部：Iq = -Ialpha*sin + Ibeta*cos
add_block('simulink/Math Operations/Product', [parkSub '/Prod_alpha_sin'], ...
    'Inputs', '**', 'Position', [280 200 310 240]);
add_block('simulink/Math Operations/Product', [parkSub '/Prod_beta_cos'], ...
    'Inputs', '**', 'Position', [280 260 310 300]);
add_block('simulink/Math Operations/Gain', [parkSub '/Gain_neg1'], ...
    'Gain', '-1', 'Position', [340 205 370 235]);
add_block('simulink/Math Operations/Add', [parkSub '/Add_Iq'], ...
    'Position', [420 220 450 250]);
add_line(parkSub, 'Ialpha/1', 'Prod_alpha_sin/1');
add_line(parkSub, 'sin_theta/1', 'Prod_alpha_sin/2');
add_line(parkSub, 'Prod_alpha_sin/1', 'Gain_neg1/1');
add_line(parkSub, 'Gain_neg1/1', 'Add_Iq/1');
add_line(parkSub, 'Ibeta/1', 'Prod_beta_cos/1');
add_line(parkSub, 'cos_theta/1', 'Prod_beta_cos/2');
add_line(parkSub, 'Prod_beta_cos/1', 'Add_Iq/2');
add_line(parkSub, 'Add_Iq/1', 'Iq/1');

% 外部 From/Goto
add_block('simulink/Signal Routing/From', [modelName '/From_Ialpha_Park'], ...
    'GotoTag', 'Ialpha_sig', 'Position', posGrid(X3-60, Y1+10, 40, 30));
add_block('simulink/Signal Routing/From', [modelName '/From_Ibeta_Park'], ...
    'GotoTag', 'Ibeta_sig', 'Position', posGrid(X3-60, Y1+50, 40, 30));
add_block('simulink/Signal Routing/From', [modelName '/From_Theta_Park'], ...
    'GotoTag', 'Theta_sig', 'Position', posGrid(X3-60, Y1+90, 40, 30));
add_line(modelName, 'From_Ialpha_Park/1', 'Park_Transform/1');
add_line(modelName, 'From_Ibeta_Park/1', 'Park_Transform/2');
add_line(modelName, 'From_Theta_Park/1', 'Park_Transform/3');

% Park输出 Goto
add_block('simulink/Signal Routing/Goto', [modelName '/Goto_Id'], ...
    'GotoTag', 'Id_sig', 'TagVisibility', 'local', ...
    'Position', posGrid(X3+160, Y1+10, 40, 30));
add_block('simulink/Signal Routing/Goto', [modelName '/Goto_Iq'], ...
    'GotoTag', 'Iq_sig', 'TagVisibility', 'local', ...
    'Position', posGrid(X3+160, Y1+60, 40, 30));
add_line(modelName, 'Park_Transform/1', 'Goto_Id/1');
add_line(modelName, 'Park_Transform/2', 'Goto_Iq/1');

%% ========== 5. 反Park变换子系统 ==========
iparkSub = [modelName '/InvPark_Transform'];
add_block('simulink/Ports & Subsystems/Subsystem', iparkSub, ...
    'Position', posGrid(X4, Y1, SUB_W, SUB_H));
delete_line(iparkSub, 'In1/1', 'Out1/1');
set_param(iparkSub, 'AttributesFormatString', ...
    'iα=id·cosθ-iq·sinθ  iβ=id·sinθ+iq·cosθ');
set_param(iparkSub, 'BackgroundColor', '[0.9 0.7 1.0]');
delete_block([iparkSub '/In1']);
delete_block([iparkSub '/Out1']);

add_block('simulink/Ports & Subsystems/In1', [iparkSub '/Id']);
add_block('simulink/Ports & Subsystems/In1', [iparkSub '/Iq']);
add_block('simulink/Ports & Subsystems/In1', [iparkSub '/Theta']);
set_param([iparkSub '/Id'], 'Position', [60 50 90 70]);
set_param([iparkSub '/Iq'], 'Position', [60 120 90 140]);
set_param([iparkSub '/Theta'], 'Position', [60 220 90 240]);

add_block('simulink/Ports & Subsystems/Out1', [iparkSub '/Ialpha_inv']);
add_block('simulink/Ports & Subsystems/Out1', [iparkSub '/Ibeta_inv']);
set_param([iparkSub '/Ialpha_inv'], 'Position', [500 80 530 100]);
set_param([iparkSub '/Ibeta_inv'], 'Position', [500 180 530 200]);

% 反Park内部：sin/cos
add_block('simulink/Math Operations/Trigonometric Function', [iparkSub '/cos_theta'], ...
    'Function', 'cos', 'Position', [160 210 200 250]);
add_block('simulink/Math Operations/Trigonometric Function', [iparkSub '/sin_theta'], ...
    'Function', 'sin', 'Position', [160 280 200 320]);
add_line(iparkSub, 'Theta/1', 'cos_theta/1');
add_line(iparkSub, 'Theta/1', 'sin_theta/1');

% 反Park内部：Ialpha_inv = Id*cos - Iq*sin
add_block('simulink/Math Operations/Product', [iparkSub '/Prod_d_cos'], ...
    'Inputs', '**', 'Position', [280 40 310 80]);
add_block('simulink/Math Operations/Product', [iparkSub '/Prod_q_sin'], ...
    'Inputs', '**', 'Position', [280 120 310 160]);
add_block('simulink/Math Operations/Gain', [iparkSub '/Gain_neg2'], ...
    'Gain', '-1', 'Position', [340 125 370 155]);
add_block('simulink/Math Operations/Add', [iparkSub '/Add_Ialpha_inv'], ...
    'Position', [420 60 450 90]);
add_line(iparkSub, 'Id/1', 'Prod_d_cos/1');
add_line(iparkSub, 'cos_theta/1', 'Prod_d_cos/2');
add_line(iparkSub, 'Iq/1', 'Prod_q_sin/1');
add_line(iparkSub, 'sin_theta/1', 'Prod_q_sin/2');
add_line(iparkSub, 'Prod_q_sin/1', 'Gain_neg2/1');
add_line(iparkSub, 'Gain_neg2/1', 'Add_Ialpha_inv/2');
add_line(iparkSub, 'Prod_d_cos/1', 'Add_Ialpha_inv/1');
add_line(iparkSub, 'Add_Ialpha_inv/1', 'Ialpha_inv/1');

% 反Park内部：Ibeta_inv = Id*sin + Iq*cos
add_block('simulink/Math Operations/Product', [iparkSub '/Prod_d_sin'], ...
    'Inputs', '**', 'Position', [280 200 310 240]);
add_block('simulink/Math Operations/Product', [iparkSub '/Prod_q_cos'], ...
    'Inputs', '**', 'Position', [280 260 310 300]);
add_block('simulink/Math Operations/Add', [iparkSub '/Add_Ibeta_inv'], ...
    'Position', [420 230 450 260]);
add_line(iparkSub, 'Id/1', 'Prod_d_sin/1');
add_line(iparkSub, 'sin_theta/1', 'Prod_d_sin/2');
add_line(iparkSub, 'Iq/1', 'Prod_q_cos/1');
add_line(iparkSub, 'cos_theta/1', 'Prod_q_cos/2');
add_line(iparkSub, 'Prod_d_sin/1', 'Add_Ibeta_inv/1');
add_line(iparkSub, 'Prod_q_cos/1', 'Add_Ibeta_inv/2');
add_line(iparkSub, 'Add_Ibeta_inv/1', 'Ibeta_inv/1');

% 外部 From/Goto
add_block('simulink/Signal Routing/From', [modelName '/From_Id_InvPark'], ...
    'GotoTag', 'Id_sig', 'Position', posGrid(X4-60, Y1+10, 40, 30));
add_block('simulink/Signal Routing/From', [modelName '/From_Iq_InvPark'], ...
    'GotoTag', 'Iq_sig', 'Position', posGrid(X4-60, Y1+50, 40, 30));
add_block('simulink/Signal Routing/From', [modelName '/From_Theta_InvPark'], ...
    'GotoTag', 'Theta_sig', 'Position', posGrid(X4-60, Y1+90, 40, 30));
add_line(modelName, 'From_Id_InvPark/1', 'InvPark_Transform/1');
add_line(modelName, 'From_Iq_InvPark/1', 'InvPark_Transform/2');
add_line(modelName, 'From_Theta_InvPark/1', 'InvPark_Transform/3');

% 反Park输出 Goto
add_block('simulink/Signal Routing/Goto', [modelName '/Goto_Ialpha_inv'], ...
    'GotoTag', 'Ialpha_inv_sig', 'TagVisibility', 'local', ...
    'Position', posGrid(X4+160, Y1+10, 40, 30));
add_block('simulink/Signal Routing/Goto', [modelName '/Goto_Ibeta_inv'], ...
    'GotoTag', 'Ibeta_inv_sig', 'TagVisibility', 'local', ...
    'Position', posGrid(X4+160, Y1+60, 40, 30));
add_line(modelName, 'InvPark_Transform/1', 'Goto_Ialpha_inv/1');
add_line(modelName, 'InvPark_Transform/2', 'Goto_Ibeta_inv/1');

%% ========== 6. 反Clark变换子系统 ==========
iclarkSub = [modelName '/InvClark_Transform'];
add_block('simulink/Ports & Subsystems/Subsystem', iclarkSub, ...
    'Position', posGrid(X5, Y1, SUB_W, SUB_H+20));
delete_line(iclarkSub, 'In1/1', 'Out1/1');
set_param(iclarkSub, 'AttributesFormatString', ...
    'ia=iα  ib=-0.5·iα+√3/2·iβ  ic=-0.5·iα-√3/2·iβ');
set_param(iclarkSub, 'BackgroundColor', '[1.0 0.7 0.8]');
delete_block([iclarkSub '/In1']);
delete_block([iclarkSub '/Out1']);

% 输入端口
add_block('simulink/Ports & Subsystems/In1', [iclarkSub '/Ialpha_inv']);
add_block('simulink/Ports & Subsystems/In1', [iclarkSub '/Ibeta_inv']);
set_param([iclarkSub '/Ialpha_inv'], 'Position', [60 50 90 70]);
set_param([iclarkSub '/Ibeta_inv'], 'Position', [60 150 90 170]);

% 输出端口
add_block('simulink/Ports & Subsystems/Out1', [iclarkSub '/Ia_inv']);
add_block('simulink/Ports & Subsystems/Out1', [iclarkSub '/Ib_inv']);
add_block('simulink/Ports & Subsystems/Out1', [iclarkSub '/Ic_inv']);
set_param([iclarkSub '/Ia_inv'], 'Position', [520 50 550 70]);
set_param([iclarkSub '/Ib_inv'], 'Position', [520 150 550 170]);
set_param([iclarkSub '/Ic_inv'], 'Position', [520 250 550 270]);

% 内部：Ia_inv = Ialpha_inv
add_line(iclarkSub, 'Ialpha_inv/1', 'Ia_inv/1');

% 内部：Ib_inv = -0.5*Ialpha_inv + sqrt(3)/2*Ibeta_inv
add_block('simulink/Math Operations/Gain', [iclarkSub '/Gain_neg_half'], ...
    'Gain', '-0.5', 'Position', [180 40 220 80]);
add_block('simulink/Math Operations/Gain', [iclarkSub '/Gain_sqrt3_2_b'], ...
    'Gain', 'sqrt(3)/2', 'Position', [180 140 220 180]);
add_block('simulink/Math Operations/Add', [iclarkSub '/Add_Ib_inv'], ...
    'Position', [350 70 380 100]);
add_line(iclarkSub, 'Ialpha_inv/1', 'Gain_neg_half/1');
add_line(iclarkSub, 'Ibeta_inv/1', 'Gain_sqrt3_2_b/1');
add_line(iclarkSub, 'Gain_neg_half/1', 'Add_Ib_inv/1');
add_line(iclarkSub, 'Gain_sqrt3_2_b/1', 'Add_Ib_inv/2');
add_line(iclarkSub, 'Add_Ib_inv/1', 'Ib_inv/1');

% 内部：Ic_inv = -0.5*Ialpha_inv - sqrt(3)/2*Ibeta_inv
add_block('simulink/Math Operations/Gain', [iclarkSub '/Gain_neg_half2'], ...
    'Gain', '-0.5', 'Position', [180 230 220 270]);
add_block('simulink/Math Operations/Gain', [iclarkSub '/Gain_sqrt3_2_c'], ...
    'Gain', 'sqrt(3)/2', 'Position', [180 300 220 340]);
add_block('simulink/Math Operations/Gain', [iclarkSub '/Gain_neg_sqrt3_2'], ...
    'Gain', '-1', 'Position', [270 305 300 335]);
add_block('simulink/Math Operations/Add', [iclarkSub '/Add_Ic_inv'], ...
    'Position', [350 250 380 280]);
add_line(iclarkSub, 'Ialpha_inv/1', 'Gain_neg_half2/1');
add_line(iclarkSub, 'Ibeta_inv/1', 'Gain_sqrt3_2_c/1');
add_line(iclarkSub, 'Gain_sqrt3_2_c/1', 'Gain_neg_sqrt3_2/1');
add_line(iclarkSub, 'Gain_neg_half2/1', 'Add_Ic_inv/1');
add_line(iclarkSub, 'Gain_neg_sqrt3_2/1', 'Add_Ic_inv/2');
add_line(iclarkSub, 'Add_Ic_inv/1', 'Ic_inv/1');

% 外部 From/Goto
add_block('simulink/Signal Routing/From', [modelName '/From_Ialpha_inv'], ...
    'GotoTag', 'Ialpha_inv_sig', 'Position', posGrid(X5-60, Y1+20, 40, 30));
add_block('simulink/Signal Routing/From', [modelName '/From_Ibeta_inv'], ...
    'GotoTag', 'Ibeta_inv_sig', 'Position', posGrid(X5-60, Y1+80, 40, 30));
add_line(modelName, 'From_Ialpha_inv/1', 'InvClark_Transform/1');
add_line(modelName, 'From_Ibeta_inv/1', 'InvClark_Transform/2');

% 反Clark输出 Goto
add_block('simulink/Signal Routing/Goto', [modelName '/Goto_Ia_inv'], ...
    'GotoTag', 'Ia_inv_sig', 'TagVisibility', 'local', ...
    'Position', posGrid(X5+160, Y1+10, 40, 30));
add_block('simulink/Signal Routing/Goto', [modelName '/Goto_Ib_inv'], ...
    'GotoTag', 'Ib_inv_sig', 'TagVisibility', 'local', ...
    'Position', posGrid(X5+160, Y1+60, 40, 30));
add_block('simulink/Signal Routing/Goto', [modelName '/Goto_Ic_inv'], ...
    'GotoTag', 'Ic_inv_sig', 'TagVisibility', 'local', ...
    'Position', posGrid(X5+160, Y1+110, 40, 30));
add_line(modelName, 'InvClark_Transform/1', 'Goto_Ia_inv/1');
add_line(modelName, 'InvClark_Transform/2', 'Goto_Ib_inv/1');
add_line(modelName, 'InvClark_Transform/3', 'Goto_Ic_inv/1');

%% ========== 7. 观测区（Scope + Display + 误差计算）==========
% 三相电流示波器
add_block('simulink/Sinks/Scope', [modelName '/Scope_3phase'], ...
    'NumInputPorts', '3', 'Position', posGrid(X6, Y1, 160, 120));
add_block('simulink/Signal Routing/From', [modelName '/From_Ia_Scope3'], ...
    'GotoTag', 'Ia_sig', 'Position', posGrid(X6-60, Y1+10, 40, 30));
add_block('simulink/Signal Routing/From', [modelName '/From_Ib_Scope3'], ...
    'GotoTag', 'Ib_sig', 'Position', posGrid(X6-60, Y1+45, 40, 30));
add_block('simulink/Signal Routing/From', [modelName '/From_Ic_Scope3'], ...
    'GotoTag', 'Ic_sig', 'Position', posGrid(X6-60, Y1+80, 40, 30));
add_line(modelName, 'From_Ia_Scope3/1', 'Scope_3phase/1');
add_line(modelName, 'From_Ib_Scope3/1', 'Scope_3phase/2');
add_line(modelName, 'From_Ic_Scope3/1', 'Scope_3phase/3');

% Clark输出示波器
add_block('simulink/Sinks/Scope', [modelName '/Scope_Clark'], ...
    'NumInputPorts', '3', 'Position', posGrid(X6+180, Y1, 160, 120));
add_block('simulink/Signal Routing/From', [modelName '/From_Ialpha_Scope'], ...
    'GotoTag', 'Ialpha_sig', 'Position', posGrid(X6+120, Y1+10, 40, 30));
add_block('simulink/Signal Routing/From', [modelName '/From_Ibeta_Scope'], ...
    'GotoTag', 'Ibeta_sig', 'Position', posGrid(X6+120, Y1+45, 40, 30));
add_block('simulink/Signal Routing/From', [modelName '/From_I0_Scope'], ...
    'GotoTag', 'I0_sig', 'Position', posGrid(X6+120, Y1+80, 40, 30));
add_line(modelName, 'From_Ialpha_Scope/1', 'Scope_Clark/1');
add_line(modelName, 'From_Ibeta_Scope/1', 'Scope_Clark/2');
add_line(modelName, 'From_I0_Scope/1', 'Scope_Clark/3');

% Park输出示波器
add_block('simulink/Sinks/Scope', [modelName '/Scope_Park'], ...
    'NumInputPorts', '2', 'Position', posGrid(X6, Y2, 160, 90));
add_block('simulink/Signal Routing/From', [modelName '/From_Id_Scope'], ...
    'GotoTag', 'Id_sig', 'Position', posGrid(X6-60, Y2+10, 40, 30));
add_block('simulink/Signal Routing/From', [modelName '/From_Iq_Scope'], ...
    'GotoTag', 'Iq_sig', 'Position', posGrid(X6-60, Y2+50, 40, 30));
add_line(modelName, 'From_Id_Scope/1', 'Scope_Park/1');
add_line(modelName, 'From_Iq_Scope/1', 'Scope_Park/2');

% 反Park输出示波器
add_block('simulink/Sinks/Scope', [modelName '/Scope_InvPark'], ...
    'NumInputPorts', '2', 'Position', posGrid(X6+180, Y2, 160, 90));
add_block('simulink/Signal Routing/From', [modelName '/From_Iainv_Scope'], ...
    'GotoTag', 'Ialpha_inv_sig', 'Position', posGrid(X6+120, Y2+10, 40, 30));
add_block('simulink/Signal Routing/From', [modelName '/From_Ibinv_Scope'], ...
    'GotoTag', 'Ibeta_inv_sig', 'Position', posGrid(X6+120, Y2+50, 40, 30));
add_line(modelName, 'From_Iainv_Scope/1', 'Scope_InvPark/1');
add_line(modelName, 'From_Ibinv_Scope/1', 'Scope_InvPark/2');

% 对比示波器（原始 vs 重建）
add_block('simulink/Sinks/Scope', [modelName '/Scope_Compare'], ...
    'NumInputPorts', '6', 'Position', posGrid(X6, Y3, 160, 140));
add_block('simulink/Signal Routing/From', [modelName '/From_Ia_comp'], ...
    'GotoTag', 'Ia_sig', 'Position', posGrid(X6-60, Y3+5, 40, 30));
add_block('simulink/Signal Routing/From', [modelName '/From_Ib_comp'], ...
    'GotoTag', 'Ib_sig', 'Position', posGrid(X6-60, Y3+30, 40, 30));
add_block('simulink/Signal Routing/From', [modelName '/From_Ic_comp'], ...
    'GotoTag', 'Ic_sig', 'Position', posGrid(X6-60, Y3+55, 40, 30));
add_block('simulink/Signal Routing/From', [modelName '/From_Iainv_comp'], ...
    'GotoTag', 'Ia_inv_sig', 'Position', posGrid(X6-60, Y3+80, 40, 30));
add_block('simulink/Signal Routing/From', [modelName '/From_Ibinv_comp'], ...
    'GotoTag', 'Ib_inv_sig', 'Position', posGrid(X6-60, Y3+105, 40, 30));
add_block('simulink/Signal Routing/From', [modelName '/From_Icinv_comp'], ...
    'GotoTag', 'Ic_inv_sig', 'Position', posGrid(X6-60, Y3+130, 40, 30));
add_line(modelName, 'From_Ia_comp/1', 'Scope_Compare/1');
add_line(modelName, 'From_Ib_comp/1', 'Scope_Compare/2');
add_line(modelName, 'From_Ic_comp/1', 'Scope_Compare/3');
add_line(modelName, 'From_Iainv_comp/1', 'Scope_Compare/4');
add_line(modelName, 'From_Ibinv_comp/1', 'Scope_Compare/5');
add_line(modelName, 'From_Icinv_comp/1', 'Scope_Compare/6');

% Display模块
add_block('simulink/Sinks/Display', [modelName '/Display_Id'], ...
    'Position', posGrid(X7, Y2, 80, 30));
add_block('simulink/Sinks/Display', [modelName '/Display_Iq'], ...
    'Position', posGrid(X7, Y2+40, 80, 30));
add_block('simulink/Signal Routing/From', [modelName '/From_Id_Disp'], ...
    'GotoTag', 'Id_sig', 'Position', posGrid(X7-60, Y2, 40, 30));
add_block('simulink/Signal Routing/From', [modelName '/From_Iq_Disp'], ...
    'GotoTag', 'Iq_sig', 'Position', posGrid(X7-60, Y2+40, 40, 30));
add_line(modelName, 'From_Id_Disp/1', 'Display_Id/1');
add_line(modelName, 'From_Iq_Disp/1', 'Display_Iq/1');

% 误差计算模块
add_block('simulink/Math Operations/Subtract', [modelName '/Err_Ia'], ...
    'Position', posGrid(X7, Y3-40, 60, 30));
add_block('simulink/Math Operations/Subtract', [modelName '/Err_Ib'], ...
    'Position', posGrid(X7, Y3, 60, 30));
add_block('simulink/Math Operations/Subtract', [modelName '/Err_Ic'], ...
    'Position', posGrid(X7, Y3+40, 60, 30));

add_block('simulink/Signal Routing/From', [modelName '/From_Ia_Err'], ...
    'GotoTag', 'Ia_sig', 'Position', posGrid(X7-60, Y3-40, 40, 30));
add_block('simulink/Signal Routing/From', [modelName '/From_Iainv_Err'], ...
    'GotoTag', 'Ia_inv_sig', 'Position', posGrid(X7-60, Y3-15, 40, 30));
add_block('simulink/Signal Routing/From', [modelName '/From_Ib_Err'], ...
    'GotoTag', 'Ib_sig', 'Position', posGrid(X7-60, Y3, 40, 30));
add_block('simulink/Signal Routing/From', [modelName '/From_Ibinv_Err'], ...
    'GotoTag', 'Ib_inv_sig', 'Position', posGrid(X7-60, Y3+25, 40, 30));
add_block('simulink/Signal Routing/From', [modelName '/From_Ic_Err'], ...
    'GotoTag', 'Ic_sig', 'Position', posGrid(X7-60, Y3+40, 40, 30));
add_block('simulink/Signal Routing/From', [modelName '/From_Icinv_Err'], ...
    'GotoTag', 'Ic_inv_sig', 'Position', posGrid(X7-60, Y3+65, 40, 30));

add_line(modelName, 'From_Ia_Err/1', 'Err_Ia/1');
add_line(modelName, 'From_Iainv_Err/1', 'Err_Ia/2');
add_line(modelName, 'From_Ib_Err/1', 'Err_Ib/1');
add_line(modelName, 'From_Ibinv_Err/1', 'Err_Ib/2');
add_line(modelName, 'From_Ic_Err/1', 'Err_Ic/1');
add_line(modelName, 'From_Icinv_Err/1', 'Err_Ic/2');

% 误差Display
add_block('simulink/Sinks/Display', [modelName '/Display_Err_A'], ...
    'Position', posGrid(X7+80, Y3-40, 80, 30));
add_block('simulink/Sinks/Display', [modelName '/Display_Err_B'], ...
    'Position', posGrid(X7+80, Y3, 80, 30));
add_block('simulink/Sinks/Display', [modelName '/Display_Err_C'], ...
    'Position', posGrid(X7+80, Y3+40, 80, 30));
add_line(modelName, 'Err_Ia/1', 'Display_Err_A/1');
add_line(modelName, 'Err_Ib/1', 'Display_Err_B/1');
add_line(modelName, 'Err_Ic/1', 'Display_Err_C/1');

%% ========== 8. 数据导出区（Mux + Outport）==========
% Mux1: [Ia, Ib, Ic, Ialpha, Ibeta, I0] — 6路
add_block('simulink/Signal Routing/Mux', [modelName '/Mux_Signals1'], ...
    'Inputs', '6', 'Position', posGrid(X8, Y1, 10, 140));

add_block('simulink/Signal Routing/From', [modelName '/From_Ia_Mux'], ...
    'GotoTag', 'Ia_sig', 'Position', posGrid(X8-60, Y1+5, 40, 30));
add_block('simulink/Signal Routing/From', [modelName '/From_Ib_Mux'], ...
    'GotoTag', 'Ib_sig', 'Position', posGrid(X8-60, Y1+25, 40, 30));
add_block('simulink/Signal Routing/From', [modelName '/From_Ic_Mux'], ...
    'GotoTag', 'Ic_sig', 'Position', posGrid(X8-60, Y1+45, 40, 30));
add_block('simulink/Signal Routing/From', [modelName '/From_Ialpha_Mux'], ...
    'GotoTag', 'Ialpha_sig', 'Position', posGrid(X8-60, Y1+65, 40, 30));
add_block('simulink/Signal Routing/From', [modelName '/From_Ibeta_Mux'], ...
    'GotoTag', 'Ibeta_sig', 'Position', posGrid(X8-60, Y1+85, 40, 30));
add_block('simulink/Signal Routing/From', [modelName '/From_I0_Mux'], ...
    'GotoTag', 'I0_sig', 'Position', posGrid(X8-60, Y1+105, 40, 30));

add_line(modelName, 'From_Ia_Mux/1', 'Mux_Signals1/1');
add_line(modelName, 'From_Ib_Mux/1', 'Mux_Signals1/2');
add_line(modelName, 'From_Ic_Mux/1', 'Mux_Signals1/3');
add_line(modelName, 'From_Ialpha_Mux/1', 'Mux_Signals1/4');
add_line(modelName, 'From_Ibeta_Mux/1', 'Mux_Signals1/5');
add_line(modelName, 'From_I0_Mux/1', 'Mux_Signals1/6');

add_block('simulink/Ports & Subsystems/Out1', [modelName '/Out_Signals1'], ...
    'Position', posGrid(X8+40, Y1+60, 30, 20));
add_line(modelName, 'Mux_Signals1/1', 'Out_Signals1/1');

% Mux2: [Id, Iq, Ia_inv, Ib_inv, Ic_inv] — 5路
add_block('simulink/Signal Routing/Mux', [modelName '/Mux_Signals2'], ...
    'Inputs', '5', 'Position', posGrid(X8, Y3, 10, 115));

add_block('simulink/Signal Routing/From', [modelName '/From_Id_Mux'], ...
    'GotoTag', 'Id_sig', 'Position', posGrid(X8-60, Y3+5, 40, 30));
add_block('simulink/Signal Routing/From', [modelName '/From_Iq_Mux'], ...
    'GotoTag', 'Iq_sig', 'Position', posGrid(X8-60, Y3+27, 40, 30));
add_block('simulink/Signal Routing/From', [modelName '/From_Iainv_Mux'], ...
    'GotoTag', 'Ia_inv_sig', 'Position', posGrid(X8-60, Y3+49, 40, 30));
add_block('simulink/Signal Routing/From', [modelName '/From_Ibinv_Mux'], ...
    'GotoTag', 'Ib_inv_sig', 'Position', posGrid(X8-60, Y3+71, 40, 30));
add_block('simulink/Signal Routing/From', [modelName '/From_Icinv_Mux'], ...
    'GotoTag', 'Ic_inv_sig', 'Position', posGrid(X8-60, Y3+93, 40, 30));

add_line(modelName, 'From_Id_Mux/1', 'Mux_Signals2/1');
add_line(modelName, 'From_Iq_Mux/1', 'Mux_Signals2/2');
add_line(modelName, 'From_Iainv_Mux/1', 'Mux_Signals2/3');
add_line(modelName, 'From_Ibinv_Mux/1', 'Mux_Signals2/4');
add_line(modelName, 'From_Icinv_Mux/1', 'Mux_Signals2/5');

add_block('simulink/Ports & Subsystems/Out1', [modelName '/Out_Signals2'], ...
    'Position', posGrid(X8+40, Y3+45, 30, 20));
add_line(modelName, 'Mux_Signals2/1', 'Out_Signals2/1');

%% ========== 9. 保存模型 ==========
save_system(modelName, [pwd filesep modelName '.slx']);
disp('Clark/Park变换 Simulink模型已创建完成（From/Goto版）！');
disp(['模型文件: ' pwd filesep modelName '.slx']);

%% ========== 10. 自动运行仿真并分析 ==========
disp(' ');
disp('正在自动运行仿真...');
out = sim(modelName, 'StopTime', num2str(Tstop));
disp('仿真完成！正在生成分析报告...');

% 从SimulationOutput对象提取数据
t_data = out.tout;
sig1 = out.yout.getElement(1).Values.Data;
sig2 = out.yout.getElement(2).Values.Data;

Ia_data = sig1(:,1);
Ib_data = sig1(:,2);
Ic_data = sig1(:,3);
Ialpha_data = sig1(:,4);
Ibeta_data = sig1(:,5);
I0_data = sig1(:,6);

Id_data = sig2(:,1);
Iq_data = sig2(:,2);
Ia_inv_data = sig2(:,3);
Ib_inv_data = sig2(:,4);
Ic_inv_data = sig2(:,5);

% 保存到base workspace
assignin('base', 't_data', t_data);
assignin('base', 'Ia_data', Ia_data);
assignin('base', 'Ib_data', Ib_data);
assignin('base', 'Ic_data', Ic_data);
assignin('base', 'Ialpha_data', Ialpha_data);
assignin('base', 'Ibeta_data', Ibeta_data);
assignin('base', 'I0_data', I0_data);
assignin('base', 'Id_data', Id_data);
assignin('base', 'Iq_data', Iq_data);
assignin('base', 'Ia_inv_data', Ia_inv_data);
assignin('base', 'Ib_inv_data', Ib_inv_data);
assignin('base', 'Ic_inv_data', Ic_inv_data);

% 计算重建误差
err_a = Ia_data - Ia_inv_data;
err_b = Ib_data - Ib_inv_data;
err_c = Ic_data - Ic_inv_data;

% 绘制对比分析图
figure('Name', 'Clark/Park变换仿真分析', 'Position', [100 100 1400 900]);

subplot(4, 2, 1);
plot(t_data, Ia_data, 'r', t_data, Ib_data, 'g', t_data, Ic_data, 'b');
title('原始三相电流'); legend('Ia', 'Ib', 'Ic');
xlabel('时间 (s)'); ylabel('电流 (A)'); grid on;

subplot(4, 2, 2);
plot(t_data, Ialpha_data, 'r', t_data, Ibeta_data, 'g', t_data, I0_data, 'b');
title('Clark变换输出 (\alpha, \beta, 0)');
legend('I\alpha', 'I\beta', 'I_0');
xlabel('时间 (s)'); ylabel('电流 (A)'); grid on;

subplot(4, 2, 3);
plot(t_data, Id_data, 'r', t_data, Iq_data, 'g');
title('Park变换输出 (d, q)'); legend('Id', 'Iq');
xlabel('时间 (s)'); ylabel('电流 (A)'); grid on;

subplot(4, 2, 4);
plot(t_data, Ia_inv_data, 'r', t_data, Ib_inv_data, 'g');
title('反Park重建 (\alpha, \beta)'); legend('I\alpha_{inv}', 'I\beta_{inv}');
xlabel('时间 (s)'); ylabel('电流 (A)'); grid on;

subplot(4, 2, 5);
plot(t_data, Ia_data, 'r-', 'LineWidth', 1.5); hold on;
plot(t_data, Ia_inv_data, 'b--', 'LineWidth', 1); hold off;
title('A相: 原始 vs 反Clark重建'); legend('原始 Ia', '重建 Ia_{inv}');
xlabel('时间 (s)'); ylabel('电流 (A)'); grid on;

subplot(4, 2, 6);
plot(t_data, Ib_data, 'g-', 'LineWidth', 1.5); hold on;
plot(t_data, Ib_inv_data, 'r--', 'LineWidth', 1); hold off;
title('B相: 原始 vs 反Clark重建'); legend('原始 Ib', '重建 Ib_{inv}');
xlabel('时间 (s)'); ylabel('电流 (A)'); grid on;

subplot(4, 2, 7);
plot(t_data, Ic_data, 'b-', 'LineWidth', 1.5); hold on;
plot(t_data, Ic_inv_data, 'g--', 'LineWidth', 1); hold off;
title('C相: 原始 vs 反Clark重建'); legend('原始 Ic', '重建 Ic_{inv}');
xlabel('时间 (s)'); ylabel('电流 (A)'); grid on;

subplot(4, 2, 8);
plot(t_data, err_a, 'r', t_data, err_b, 'g', t_data, err_c, 'b');
title('三相重建误差 (原始 - 重建)');
legend('Error Ia', 'Error Ib', 'Error Ic');
xlabel('时间 (s)'); ylabel('误差 (A)'); grid on;

fprintf('\n========== 反变换重建误差分析 ==========\n');
fprintf('A相误差 RMS: %.6e A\n', rms(err_a));
fprintf('B相误差 RMS: %.6e A\n', rms(err_b));
fprintf('C相误差 RMS: %.6e A\n', rms(err_c));
fprintf('A相误差最大值: %.6e A\n', max(abs(err_a)));
fprintf('B相误差最大值: %.6e A\n', max(abs(err_b)));
fprintf('C相误差最大值: %.6e A\n', max(abs(err_c)));
fprintf('========================================\n');

n = length(t_data);
steady_idx = floor(n/2):n;
Id_mean = mean(Id_data(steady_idx));
Iq_mean = mean(Iq_data(steady_idx));
Id_std = std(Id_data(steady_idx));
Iq_std = std(Iq_data(steady_idx));

fprintf('\n========== Park变换稳态输出分析 ==========\n');
fprintf('Id 平均值: %.6f A (理论值: %.6f A)\n', Id_mean, A);
fprintf('Iq 平均值: %.6f A (理论值: 0.000000 A)\n', Iq_mean);
fprintf('Id 标准差: %.6e A\n', Id_std);
fprintf('Iq 标准差: %.6e A\n', Iq_std);
fprintf('==========================================\n');

saveas(gcf, [pwd filesep 'Clark_Park_Analysis.png']);
disp(['分析结果图已保存: ' pwd filesep 'Clark_Park_Analysis.png']);
disp('所有任务已完成！');

% ==================================================================
% 辅助函数：将(x, y, w, h)转换为Simulink Position格式 [x y x+w y+h]
% ==================================================================
function pos = posGrid(x, y, w, h)
    pos = [x y x+w y+h];
end

modelName = 'PCS_3Phase_Grid';
if bdIsLoaded(modelName)
    close_system(modelName, 0);
end
if exist([modelName '.slx'], 'file')
    delete([modelName '.slx']);
end

new_system(modelName);
open_system(modelName);
set_param(modelName, 'Solver', 'ode4', 'FixedStep', '1e-5', 'StopTime', '0.5', ...
    'ReturnWorkspaceOutputs', 'on');

hModel = get_param(modelName, 'Handle');

% Parameters
Vrms = 220;               % Phase-to-phase RMS
f = 50;
L_val = 5e-3;
R_val = 0.1;

% =====================================================================
% POWER CIRCUIT (electrical domain)
%   Grid ── VI_Meas ── RLC ── PCS
% =====================================================================

add_block('powerlib/powergui', [modelName '/powergui'], ...
    'Position', [660 530 760 580]);

add_block('spsThreePhaseSourceLib/Three-Phase Source', [modelName '/Grid'], ...
    'Position', [60 150 140 320]);
set_param([modelName '/Grid'], 'InternalConnection', 'Yg', ...
    'Voltage', num2str(Vrms), 'PhaseAngle', '0', ...
    'Frequency', num2str(f), 'NonIdealSource', 'off');

add_block('powerlib/Measurements/Three-Phase V-I Measurement', [modelName '/VI_Meas'], ...
    'Position', [210 150 260 320]);

add_block('powerlib/Elements/Three-Phase Series RLC Branch', [modelName '/RLC'], ...
    'Position', [340 150 390 320]);
set_param([modelName '/RLC'], 'BranchType', 'RL', ...
    'Resistance', num2str(R_val), 'Inductance', num2str(L_val));

add_block('spsThreePhaseSourceLib/Three-Phase Source', [modelName '/PCS'], ...
    'Position', [470 150 550 320]);
set_param([modelName '/PCS'], 'InternalConnection', 'Yg', ...
    'Voltage', num2str(Vrms), 'PhaseAngle', '0', ...
    'Frequency', num2str(f), 'NonIdealSource', 'off');

% Wire power circuit
gR = get_param([modelName '/Grid'], 'PortHandles').RConn;
vL = get_param([modelName '/VI_Meas'], 'PortHandles').LConn;
vR = get_param([modelName '/VI_Meas'], 'PortHandles').RConn;
vOut = get_param([modelName '/VI_Meas'], 'PortHandles').Outport;
rL = get_param([modelName '/RLC'], 'PortHandles').LConn;
rR = get_param([modelName '/RLC'], 'PortHandles').RConn;
pR = get_param([modelName '/PCS'], 'PortHandles').RConn;

for i = 1:3
    add_line(modelName, gR(i), vL(i));
    add_line(modelName, vR(i), rL(i));
    add_line(modelName, rR(i), pR(i));
end

% =====================================================================
% SIGNAL PROCESSING — precise column grid
%
% Columns (x):  640  710  790  860  950 1030 1110 1170 1250 1330 1390 1480 1570 1630 1700 1770 1880 1930
%               Dem  Goto From Dly  S90  G90  FrPr Prod G_PQ FrSu Sum  Gsum FrOut TW   Gain Disp FrSc Scp
%
% Rows (y): V band = 90,140,190   I band = 270,320,370
%           gap between bands = 60px, phase spacing = 50px
% =====================================================================

vPhases  = {'Va','Vb','Vc'};
iPhases  = {'Ia','Ib','Ic'};
phLabels = {'a','b','c'};
yV = [90 140 190];
yI = [270 320 370];

% ---- C1 (x=640-670): Demux ----
add_block('simulink/Signal Routing/Demux', [modelName '/Demux_V'], ...
    'Position', [640 80 670 230], 'Outputs', '3');
add_block('simulink/Signal Routing/Demux', [modelName '/Demux_I'], ...
    'Position', [640 260 670 410], 'Outputs', '3');
add_line(modelName, vOut(1), get_param([modelName '/Demux_V'], 'PortHandles').Inport);
add_line(modelName, vOut(2), get_param([modelName '/Demux_I'], 'PortHandles').Inport);

% ---- C2 (x=710-750): Goto Vabc & Iabc ----
dVout = get_param([modelName '/Demux_V'], 'PortHandles').Outport;
dIout = get_param([modelName '/Demux_I'], 'PortHandles').Outport;
for i = 1:3
    add_block('simulink/Signal Routing/Goto', [modelName '/Goto_' vPhases{i}], ...
        'Position', [710 yV(i) 750 yV(i)+20], 'GotoTag', vPhases{i}, 'TagVisibility', 'local');
    add_line(modelName, dVout(i), get_param([modelName '/Goto_' vPhases{i}], 'PortHandles').Inport);

    add_block('simulink/Signal Routing/Goto', [modelName '/Goto_' iPhases{i}], ...
        'Position', [710 yI(i) 750 yI(i)+20], 'GotoTag', iPhases{i}, 'TagVisibility', 'local');
    add_line(modelName, dIout(i), get_param([modelName '/Goto_' iPhases{i}], 'PortHandles').Inport);
end

% ---- C3..C6 (x=790..1070): 90 degree shift chain (V band only) ----
%   From_V → Delay → Sign90 → Goto_V90
for i = 1:3
    add_block('simulink/Signal Routing/From', [modelName '/From_' vPhases{i}], ...
        'Position', [790 yV(i) 820 yV(i)+20], 'GotoTag', vPhases{i});
    add_block('simulink/Continuous/Transport Delay', [modelName '/Delay_' vPhases{i}], ...
        'Position', [860 yV(i) 910 yV(i)+20], 'DelayTime', '0.005');
    add_block('simulink/Math Operations/Gain', [modelName '/Sign90_' vPhases{i}], ...
        'Position', [950 yV(i) 990 yV(i)+20], 'Gain', '-1');
    add_block('simulink/Signal Routing/Goto', [modelName '/Goto_' vPhases{i} '90'], ...
        'Position', [1030 yV(i) 1070 yV(i)+20], 'GotoTag', [vPhases{i} '90'], 'TagVisibility', 'local');
end
for i = 1:3
    hFrom  = get_param([modelName '/From_' vPhases{i}], 'PortHandles');
    hDelay = get_param([modelName '/Delay_' vPhases{i}], 'PortHandles');
    hSign  = get_param([modelName '/Sign90_' vPhases{i}], 'PortHandles');
    hGoto  = get_param([modelName '/Goto_' vPhases{i} '90'], 'PortHandles');
    add_line(modelName, hFrom.Outport, hDelay.Inport);
    add_line(modelName, hDelay.Outport, hSign.Inport);
    add_line(modelName, hSign.Outport, hGoto.Inport);
end

% ---- C7..C9 (x=1110..1290): Products ----
%   P: From_V_P + From_I_P → Prod_P → Goto_P   (V band)
%   Q: From_V90  + From_I_Q → Prod_Q → Goto_Q   (I band)
for ph = 1:3
    nm = phLabels{ph};

    % --- P path (V band) ---
    add_block('simulink/Signal Routing/From', [modelName '/From_V' nm '_P'], ...
        'Position', [1110 yV(ph)     1140 yV(ph)+14], 'GotoTag', vPhases{ph});
    add_block('simulink/Signal Routing/From', [modelName '/From_I' nm '_P'], ...
        'Position', [1110 yV(ph)+16  1140 yV(ph)+30], 'GotoTag', iPhases{ph});
    add_block('simulink/Math Operations/Product', [modelName '/Prod_P' nm], ...
        'Position', [1170 yV(ph)+5   1210 yV(ph)+25], 'Inputs', '**');
    ppIn = get_param([modelName '/Prod_P' nm], 'PortHandles').Inport;
    add_line(modelName, get_param([modelName '/From_V' nm '_P'], 'PortHandles').Outport, ppIn(1));
    add_line(modelName, get_param([modelName '/From_I' nm '_P'], 'PortHandles').Outport, ppIn(2));

    add_block('simulink/Signal Routing/Goto', [modelName '/Goto_P' nm], ...
        'Position', [1250 yV(ph)     1290 yV(ph)+20], 'GotoTag', ['P' nm], 'TagVisibility', 'local');
    add_line(modelName, get_param([modelName '/Prod_P' nm], 'PortHandles').Outport, ...
        get_param([modelName '/Goto_P' nm], 'PortHandles').Inport);

    % --- Q path (I band) ---
    add_block('simulink/Signal Routing/From', [modelName '/From_V90_' nm], ...
        'Position', [1110 yI(ph)     1140 yI(ph)+14], 'GotoTag', [vPhases{ph} '90']);
    add_block('simulink/Signal Routing/From', [modelName '/From_I' nm '_Q'], ...
        'Position', [1110 yI(ph)+16  1140 yI(ph)+30], 'GotoTag', iPhases{ph});
    add_block('simulink/Math Operations/Product', [modelName '/Prod_Q' nm], ...
        'Position', [1170 yI(ph)+5   1210 yI(ph)+25], 'Inputs', '**');
    pqIn = get_param([modelName '/Prod_Q' nm], 'PortHandles').Inport;
    add_line(modelName, get_param([modelName '/From_V90_' nm], 'PortHandles').Outport, pqIn(1));
    add_line(modelName, get_param([modelName '/From_I' nm '_Q'], 'PortHandles').Outport, pqIn(2));

    add_block('simulink/Signal Routing/Goto', [modelName '/Goto_Q' nm], ...
        'Position', [1250 yI(ph)     1290 yI(ph)+20], 'GotoTag', ['Q' nm], 'TagVisibility', 'local');
    add_line(modelName, get_param([modelName '/Prod_Q' nm], 'PortHandles').Outport, ...
        get_param([modelName '/Goto_Q' nm], 'PortHandles').Inport);
end

% ---- C10..C11 (x=1330..1430): Sum blocks ----
%   From_P* → Sum_P → Goto_Psum   (V band)
%   From_Q* → Sum_Q → Goto_Qsum   (I band)
add_block('simulink/Math Operations/Sum', [modelName '/Sum_P'], ...
    'Position', [1390 80  1430 200], 'IconShape', 'rectangular', 'Inputs', '+++');
add_block('simulink/Math Operations/Sum', [modelName '/Sum_Q'], ...
    'Position', [1390 260 1430 380], 'IconShape', 'rectangular', 'Inputs', '+++');

sumP_in = get_param([modelName '/Sum_P'], 'PortHandles').Inport;
sumQ_in = get_param([modelName '/Sum_Q'], 'PortHandles').Inport;
for ph = 1:3
    nm = phLabels{ph};
    add_block('simulink/Signal Routing/From', [modelName '/From_P' nm], ...
        'Position', [1330 86+18*(ph-1)  1360 101+18*(ph-1)], 'GotoTag', ['P' nm]);
    add_line(modelName, get_param([modelName '/From_P' nm], 'PortHandles').Outport, sumP_in(ph));

    add_block('simulink/Signal Routing/From', [modelName '/From_Q' nm], ...
        'Position', [1330 266+18*(ph-1) 1360 281+18*(ph-1)], 'GotoTag', ['Q' nm]);
    add_line(modelName, get_param([modelName '/From_Q' nm], 'PortHandles').Outport, sumQ_in(ph));
end

sumP_out = get_param([modelName '/Sum_P'], 'PortHandles').Outport;
sumQ_out = get_param([modelName '/Sum_Q'], 'PortHandles').Outport;

% ---- C12 (x=1480-1520): Goto for Sum outputs ----
add_block('simulink/Signal Routing/Goto', [modelName '/Goto_Psum'], ...
    'Position', [1480 130 1520 150], 'GotoTag', 'Psum', 'TagVisibility', 'local');
add_block('simulink/Signal Routing/Goto', [modelName '/Goto_Qsum'], ...
    'Position', [1480 310 1520 330], 'GotoTag', 'Qsum', 'TagVisibility', 'local');
add_line(modelName, sumP_out, get_param([modelName '/Goto_Psum'], 'PortHandles').Inport);
add_line(modelName, sumQ_out, get_param([modelName '/Goto_Qsum'], 'PortHandles').Inport);

% Mux for I_abc (collect Ia,Ib,Ic → vector → To Workspace)
add_block('simulink/Signal Routing/Mux', [modelName '/Mux_I'], ...
    'Position', [1390 430 1410 490], 'Inputs', '3');
muxI_in = get_param([modelName '/Mux_I'], 'PortHandles').Inport;
for i = 1:3
    add_block('simulink/Signal Routing/From', [modelName '/From_I' phLabels{i} '_mux'], ...
        'Position', [1330 438+18*(i-1) 1360 453+18*(i-1)], 'GotoTag', iPhases{i});
    add_line(modelName, get_param([modelName '/From_I' phLabels{i} '_mux'], 'PortHandles').Outport, muxI_in(i));
end

% ---- C13..C15 (x=1570..1760): To Workspace & LPF ----
%   To Workspace (y=30,80,130)   LPF (y=320,400)   Gain (y=320)
add_block('simulink/Signal Routing/From', [modelName '/From_Psum_tw'], ...
    'Position', [1570 40  1600 60],  'GotoTag', 'Psum');
add_block('simulink/Signal Routing/From', [modelName '/From_Qsum_tw'], ...
    'Position', [1570 90  1600 110], 'GotoTag', 'Qsum');
add_block('simulink/Sinks/To Workspace', [modelName '/P_total'], ...
    'Position', [1630 30  1670 70],  'VariableName', 'P_total', 'SaveFormat', 'Array');
add_block('simulink/Sinks/To Workspace', [modelName '/Q_total'], ...
    'Position', [1630 80  1670 120], 'VariableName', 'Q_total', 'SaveFormat', 'Array');
add_block('simulink/Sinks/To Workspace', [modelName '/I_abc_out'], ...
    'Position', [1630 130 1670 170], 'VariableName', 'I_abc', 'SaveFormat', 'Array');
add_line(modelName, get_param([modelName '/From_Psum_tw'], 'PortHandles').Outport, ...
    get_param([modelName '/P_total'], 'PortHandles').Inport);
add_line(modelName, get_param([modelName '/From_Qsum_tw'], 'PortHandles').Outport, ...
    get_param([modelName '/Q_total'], 'PortHandles').Inport);
add_line(modelName, get_param([modelName '/Mux_I'], 'PortHandles').Outport, ...
    get_param([modelName '/I_abc_out'], 'PortHandles').Inport);

% LPF path
add_block('simulink/Signal Routing/From', [modelName '/From_Psum_lpf'], ...
    'Position', [1570 330 1600 350], 'GotoTag', 'Psum');
add_block('simulink/Signal Routing/From', [modelName '/From_Qsum_lpf'], ...
    'Position', [1570 410 1600 430], 'GotoTag', 'Qsum');
add_block('simulink/Continuous/Transfer Fcn', [modelName '/LPF_P'], ...
    'Position', [1630 320 1680 360], 'Numerator', '[1]', 'Denominator', '[0.05 1]');
add_block('simulink/Continuous/Transfer Fcn', [modelName '/LPF_Q'], ...
    'Position', [1630 400 1680 440], 'Numerator', '[1]', 'Denominator', '[0.05 1]');
add_block('simulink/Math Operations/Gain', [modelName '/Gain_Pinv'], ...
    'Position', [1700 320 1740 360], 'Gain', '-1');
add_line(modelName, get_param([modelName '/From_Psum_lpf'], 'PortHandles').Outport, ...
    get_param([modelName '/LPF_P'], 'PortHandles').Inport);
add_line(modelName, get_param([modelName '/From_Qsum_lpf'], 'PortHandles').Outport, ...
    get_param([modelName '/LPF_Q'], 'PortHandles').Inport);
lpfP_out = get_param([modelName '/LPF_P'], 'PortHandles').Outport;
lpfQ_out = get_param([modelName '/LPF_Q'], 'PortHandles').Outport;
add_line(modelName, lpfP_out, get_param([modelName '/Gain_Pinv'], 'PortHandles').Inport);

% ---- C16 (x=1770-1840): Display ----
add_block('simulink/Sinks/Display', [modelName '/Disp_P'], ...
    'Position', [1770 310 1840 370], 'Format', 'short', 'FontSize', '12');
add_block('simulink/Sinks/Display', [modelName '/Disp_Q'], ...
    'Position', [1770 400 1840 460], 'Format', 'short', 'FontSize', '12');
add_block('simulink/Sinks/Display', [modelName '/Disp_I'], ...
    'Position', [1770 490 1840 550], 'Format', 'short', 'FontSize', '12');

gainPinv_out = get_param([modelName '/Gain_Pinv'], 'PortHandles').Outport;
add_line(modelName, gainPinv_out, get_param([modelName '/Disp_P'], 'PortHandles').Inport);
add_line(modelName, lpfQ_out,      get_param([modelName '/Disp_Q'], 'PortHandles').Inport);

add_block('simulink/Signal Routing/From', [modelName '/From_Ia_disp'], ...
    'Position', [1730 510 1760 530], 'GotoTag', 'Ia');
add_line(modelName, get_param([modelName '/From_Ia_disp'], 'PortHandles').Outport, ...
    get_param([modelName '/Disp_I'], 'PortHandles').Inport);

% ---- C17..C18 (x=1880..1990): Scopes ----
add_block('simulink/Sinks/Scope', [modelName '/Scope_VI_a'], ...
    'Position', [1930 60  1990 140], 'NumInputPorts', '2');
add_block('simulink/Sinks/Scope', [modelName '/Scope_I3ph'], ...
    'Position', [1930 180 1990 260], 'NumInputPorts', '3');
add_block('simulink/Sinks/Scope', [modelName '/Scope_PQ'], ...
    'Position', [1930 310 1990 390], 'NumInputPorts', '2');

scopeVI_in = get_param([modelName '/Scope_VI_a'], 'PortHandles').Inport;
scopeI_in  = get_param([modelName '/Scope_I3ph'],  'PortHandles').Inport;
scopePQ_in = get_param([modelName '/Scope_PQ'],     'PortHandles').Inport;

add_block('simulink/Signal Routing/From', [modelName '/From_Va_scope'], ...
    'Position', [1880 70  1910 90],  'GotoTag', 'Va');
add_block('simulink/Signal Routing/From', [modelName '/From_Ia_scope'], ...
    'Position', [1880 105 1910 125], 'GotoTag', 'Ia');
add_line(modelName, get_param([modelName '/From_Va_scope'], 'PortHandles').Outport, scopeVI_in(1));
add_line(modelName, get_param([modelName '/From_Ia_scope'], 'PortHandles').Outport, scopeVI_in(2));

for i = 1:3
    add_block('simulink/Signal Routing/From', [modelName '/From_I' phLabels{i} '_scope3ph'], ...
        'Position', [1880 188+18*(i-1) 1910 203+18*(i-1)], 'GotoTag', iPhases{i});
    add_line(modelName, get_param([modelName '/From_I' phLabels{i} '_scope3ph'], 'PortHandles').Outport, scopeI_in(i));
end

add_block('simulink/Signal Routing/From', [modelName '/From_Psum_scope'], ...
    'Position', [1880 320 1910 340], 'GotoTag', 'Psum');
add_block('simulink/Signal Routing/From', [modelName '/From_Qsum_scope'], ...
    'Position', [1880 360 1910 380], 'GotoTag', 'Qsum');
add_line(modelName, get_param([modelName '/From_Psum_scope'], 'PortHandles').Outport, scopePQ_in(1));
add_line(modelName, get_param([modelName '/From_Qsum_scope'], 'PortHandles').Outport, scopePQ_in(2));

set_param([modelName '/Scope_VI_a'], 'Name', 'Scope_V_I_A相');
set_param([modelName '/Scope_I3ph'],  'Name', 'Scope_三相电流');
set_param([modelName '/Scope_PQ'],    'Name', 'Scope_总功率(常数)');

hDispP = get_param([modelName '/Disp_P'], 'Handle');
hDispQ = get_param([modelName '/Disp_Q'], 'Handle');
hDispI = get_param([modelName '/Disp_I'], 'Handle');

% =====================================================================
% ANNOTATIONS — area backgrounds + text notes
% Area positions precisely aligned to block groups
% Notes in two rows below blocks to avoid overlap
% =====================================================================

areaDefs = {
    % name         x    y    w    h   color
    {'Grid',       30, 115,  95, 240, '[0.82 0.90 1.00]'}
    {'VI',        185, 115,  75, 240, '[1.00 1.00 0.75]'}
    {'RLC',       320, 115,  70, 240, '[0.88 0.98 0.90]'}
    {'PCS',       450, 115,  95, 240, '[0.82 0.90 1.00]'}
    {'Demux',     620,  55,  55, 380, '[0.88 0.88 0.88]'}
    {'Shift90',   770,  55, 315, 175, '[0.90 0.85 1.00]'}
    {'Prod',     1090,  55, 215, 380, '[1.00 0.94 0.82]'}
    {'Sum',      1310,  55, 130, 460, '[1.00 0.88 0.72]'}
    {'Output',   1550,  20, 205, 440, '[0.85 0.92 0.88]'}
    {'Display',  1750, 275, 110, 300, '[1.00 0.88 0.92]'}
    {'Scope',    1860,  35, 145, 380, '[0.88 0.82 1.00]'}
};
for i = 1:length(areaDefs)
    d = areaDefs{i};
    add_block('built-in/Area', [modelName '/Area_' d{1}], ...
        'Position', [d{2} d{3} d{2}+d{4} d{3}+d{5}]);
    set_param([modelName '/Area_' d{1}], 'BackgroundColor', d{6}, 'DropShadow', 'off');
end

% Text notes in 2 rows (y=540 row A, y=640 row B) to avoid horizontal overlap
noteDefsA = {
    % text                                                                           x    y    BG
    {sprintf(['电网\n3-Phase Source(理想)\nYg,220V,50Hz,0°\nNonIdealSource=off']),    35, 540, '[0.75 0.85 1.00]'}
    {sprintf(['RL支路\n3-Phase Series RLC\nR=0.1Ω L=5mH\nBranchType=RL']),           310, 540, '[0.78 0.95 0.80]'}
    {sprintf(['信号分离\nDemux×2\nVabc→Va,Vb,Vc\nIabc→Ia,Ib,Ic']),                    605, 540, '[0.75 0.75 0.75]'}
    {sprintf(['功率乘积\nP=V×I  Q=V90×I\n每相独立计算\n输入:From  输出:Goto']),        1090, 540, '[1.00 0.88 0.72]'}
    {sprintf(['显示\nP/Q/I稳态值\n符号已统一\n动态更新名称']),                           1750, 540, '[1.00 0.85 0.90]'}
    {sprintf(['波形观测\nScope_V_I(A相)\nScope_三相电流\nScope_总功率(常数)']),          1860, 540, '[0.78 0.72 1.00]'}
};
noteDefsB = {
    {sprintf(['测量\n3-Phase V-I Meas\n提取Vabc/Iabc\n转为Simulink信号']),             180, 640, '[0.85 0.85 0.55]'}
    {sprintf(['PCS\n3-Phase Source(理想)\nYg,可控V和相位\nNonIdealSource=off']),       440, 640, '[0.75 0.85 1.00]'}
    {sprintf(['90°移相(超前)\nDelay 5ms+Gain(-1)\n=V(t+90°)用于Q计算\nQ=V90×I']),   775, 640, '[0.78 0.75 1.00]'}
    {sprintf(['三相求和\nPtotal=Pa+Pb+Pc\nQtotal=Qa+Qb+Qc\n→三相纹波互消!']),         1305, 640, '[1.00 0.80 0.60]'}
    {sprintf(['数据输出\nTo Workspace存P/Q/I\nLPF(0.05s)滤波\nGain(-1):P>0=PCS输出']), 1550, 640, '[0.75 0.78 0.75]'}
};

for i = 1:length(noteDefsA)
    d = noteDefsA{i};
    ann = Simulink.Annotation(hModel, d{1});
    ann.Position = [d{2} d{3} d{2}+140 d{3}+90];
    ann.BackgroundColor = d{4}; ann.ForegroundColor = '[0.05 0.05 0.05]';
    ann.FontSize = 9; ann.DropShadow = 'off'; ann.TeXMode = 'off';
end
for i = 1:length(noteDefsB)
    d = noteDefsB{i};
    ann = Simulink.Annotation(hModel, d{1});
    ann.Position = [d{2} d{3} d{2}+140 d{3}+90];
    ann.BackgroundColor = d{4}; ann.ForegroundColor = '[0.05 0.05 0.05]';
    ann.FontSize = 9; ann.DropShadow = 'off'; ann.TeXMode = 'off';
end

save_system(modelName);

% =====================================================================
% BATCH SIMULATION
% =====================================================================
cases = {
    {220, 220,   0, '0 deg (同幅)'}
    {220, 220,  10, '10 deg'}
    {220, 220,  45, '45 deg'}
    {220, 220,  90, '90 deg'}
    {220, 220, 135, '135 deg'}
    {220, 220, 180, '180 deg'}
    {220, 220, -45, '-45 deg'}
    {220, 220, -90, '-90 deg'}
    {220, 230,   0, 'Vp=230V,0°(感性示例)'}
};

fprintf('\n');
fprintf('================================================================================\n');
fprintf('  Three-Phase PCS Simulation (using native sps blocks)\n');
fprintf('================================================================================\n');
fprintf('  Grid: Three-Phase Source | RLC: Three-Phase Series RLC Branch\n');
fprintf('  PCS:  Three-Phase Source | Meas: Three-Phase V-I Measurement\n');
fprintf('  R=%.1fOhm, L=%.1fmH, f=50Hz\n', R_val, L_val*1e3);
fprintf('--------------------------------------------------------------------------------\n');
fprintf('  %-22s | %9s %9s | %10s %10s | %8s | %s\n', ...
    'Case', 'P_sim(W)', 'P_th(W)', 'Q_sim(var)', 'Q_th(var)', 'I_rms(A)', 'State');
fprintf('--------------------------------------------------------------------------------\n');

for k = 1:length(cases)
    c = cases{k};
    Vg_rms = c{1}; Vp_rms = c{2}; phi_deg = c{3}; label = c{4};

    set_param([modelName '/Grid'], 'Voltage', num2str(Vg_rms), 'PhaseAngle', '0');
    set_param([modelName '/PCS'],  'Voltage', num2str(Vp_rms), 'PhaseAngle', num2str(phi_deg));

    simOut = sim(modelName);

    t = simOut.tout;
    dt = 1e-5;
    n_steady = round(0.2 / dt);
    N = length(t);
    idx_start = max(1, N - n_steady + 1);

    P_total = simOut.get('P_total');
    Q_total = simOut.get('Q_total');
    I_abc   = simOut.get('I_abc');

    P_val = -mean(P_total(idx_start:N));
    Q_val =  mean(Q_total(idx_start:N));
    I_a_rms = sqrt(mean(I_abc(idx_start:N, 1).^2));
    I_b_rms = sqrt(mean(I_abc(idx_start:N, 2).^2));
    I_c_rms = sqrt(mean(I_abc(idx_start:N, 3).^2));
    I_avg = (I_a_rms + I_b_rms + I_c_rms) / 3;

    [P_th, Q_th, I_th] = calc_theory_3ph(phi_deg, Vg_rms/sqrt(3), Vp_rms/sqrt(3), R_val, L_val);

    if abs(P_val) < 150, pState = '-';
    elseif P_val > 0, pState = '输出有功';
    else pState = '吸收有功'; end

    if abs(Q_val) < 150, qState = '-';
    elseif Q_val > 0, qState = '输出无功(容性)';
    else qState = '吸收无功(感性)'; end

    fprintf('  %-22s | %9.1f %9.1f | %10.1f %10.1f | %8.2f | %s, %s\n', ...
        label, P_val, P_th, Q_val, Q_th, I_avg, pState, qState);

    set_param(hDispP, 'Name', sprintf('P_total\\n%.0f W', P_val));
    set_param(hDispQ, 'Name', sprintf('Q_total\\n%.0f var', Q_val));
    set_param(hDispI, 'Name', sprintf('I_a_rms\\n%.2f A', I_a_rms));
end

set_param(modelName, 'Description', ...
    sprintf('Three-Phase PCS Simulation (native sps blocks)\nR=%.1fOhm, L=%.1fmH\nP_total, Q_total = CONSTANT (no 100Hz ripple)', R_val, L_val*1e3));
save_system(modelName);

fprintf('================================================================================\n');
fprintf('Model: %s.slx\n', modelName);
fprintf('Uses native blocks: Three-Phase Source, Three-Phase Series RLC, V-I Measurement\n');
fprintf('P_total & Q_total are CONSTANT (3-phase ripple cancellation)\n');
fprintf('================================================================================\n');

% =====================================================================
% INPUT FIELDS (bottom of model)
% =====================================================================
add_block('simulink/Sources/Constant', [modelName '/Vpcs_set'], ...
    'Position', [60 760 160 800], 'Value', num2str(Vrms));
set_param([modelName '/Vpcs_set'], 'FontSize', '12');

add_block('simulink/Sources/Constant', [modelName '/Phase_set'], ...
    'Position', [200 760 300 800], 'Value', '0');
set_param([modelName '/Phase_set'], 'FontSize', '12');

initFcnStr = sprintf([ ...
    'v = str2double(get_param(''%s/Vpcs_set'', ''Value''));', ...
    'p = str2double(get_param(''%s/Phase_set'', ''Value''));', ...
    'set_param(''%s/PCS'', ''Voltage'', num2str(v));', ...
    'set_param(''%s/PCS'', ''PhaseAngle'', num2str(p));'], ...
    modelName, modelName, modelName, modelName);
set_param(modelName, 'InitFcn', initFcnStr);

add_block('built-in/Area', [modelName '/Area_Input'], ...
    'Position', [35 735 325 825]);
set_param([modelName '/Area_Input'], 'BackgroundColor', '[0.85 0.92 1.00]', 'DropShadow', 'off');

inputNote = Simulink.Annotation(hModel, sprintf(['参数输入\n' ...
    'Vpcs_set: PCS电压(V)\n' ...
    'Phase_set: 相位角(deg)\n' ...
    '双击输入数值→Run仿真']));
inputNote.Position = [50 835 50+180 835+80];
inputNote.BackgroundColor = '[0.75 0.85 1.00]';
inputNote.ForegroundColor = '[0.05 0.05 0.05]';
inputNote.FontSize = 9;
inputNote.DropShadow = 'off';
inputNote.TeXMode = 'off';

save_system(modelName);

% =====================================================================
function [P_th, Q_th, I_rms] = calc_theory_3ph(phi_deg, Vg_rms, Vp_rms, R, L)
    w = 2*pi*50;
    Z = R + 1j*w*L;
    Vg = Vg_rms;
    Vp = Vp_rms * exp(1j*deg2rad(phi_deg));
    I_ph = (Vp - Vg) / Z;
    I_rms = abs(I_ph);
    S_1ph = Vg * conj(I_ph);
    P_th = 3 * real(S_1ph);
    Q_th = 3 * (-imag(S_1ph));
end

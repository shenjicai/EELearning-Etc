modelName = 'PCS_Grid_Connection_v3';
% Clean up previous model
if bdIsLoaded(modelName)
    close_system(modelName, 0);
end
if exist([modelName '.slx'], 'file')
    delete([modelName '.slx']);
end

% Create model
new_system(modelName);
open_system(modelName);
set_param(modelName, 'Solver', 'ode4', 'FixedStep', '1e-5', 'StopTime', '0.5', ...
    'ReturnWorkspaceOutputs', 'on');

% Model handle (needed for annotation creation)
hModel = get_param(modelName, 'Handle');

% Parameters
V_peak = 220*sqrt(2);
w = 2*pi*50;            % rad/s (Sine Wave Frequency param is rad/s, NOT Hz)
L = 5e-3;
R = 0.1;

% =====================================================================
% LAYOUT — wide spacing for clean orthogonal auto-routing
% Columns: C1=60  C2=310  C3=510  C4=660  C5=810  C6=1000  C7=1140  C8=1290  C9=1500
% Rows:    R1(y~90) = P & display path    R2(y~260) = RL main chain
%          R3(y~440) = Q path    R4(y~375) = feedback Gain_R
% =====================================================================

% --- Col 1: Voltage sources ---
add_block('simulink/Sources/Sine Wave', [modelName '/Grid'], ...
    'Position', [60 60 140 100], ...
    'Amplitude', num2str(V_peak), 'Frequency', num2str(w), ...
    'Phase', '0', 'SampleTime', '0');
add_block('simulink/Sources/Sine Wave', [modelName '/PCS'], ...
    'Position', [60 240 140 280], ...
    'Amplitude', num2str(V_peak), 'Frequency', num2str(w), ...
    'Phase', '0', 'SampleTime', '0');
add_block('simulink/Sources/Sine Wave', [modelName '/Grid_90'], ...
    'Position', [60 420 140 460], ...
    'Amplitude', num2str(V_peak), 'Frequency', num2str(w), ...
    'Phase', num2str(pi/2), 'SampleTime', '0');

% --- Col 2: Voltage difference ---
add_block('simulink/Math Operations/Subtract', [modelName '/Vdiff'], ...
    'Position', [310 240 350 280], 'Inputs', '+-');

% --- Col 3-5: RL circuit core ---
add_block('simulink/Math Operations/Sum', [modelName '/Sum_RL'], ...
    'Position', [510 240 550 280], 'IconShape', 'rectangular', 'Inputs', '+-');
add_block('simulink/Math Operations/Gain', [modelName '/Gain_1overL'], ...
    'Position', [660 240 700 280], 'Gain', num2str(1/L));
add_block('simulink/Continuous/Integrator', [modelName '/Integrator_I'], ...
    'Position', [810 240 860 280], 'InitialCondition', '0');
add_block('simulink/Math Operations/Gain', [modelName '/Gain_R'], ...
    'Position', [660 355 700 395], 'Gain', num2str(R));

% --- Col 6: Power calculation + data output ---
add_block('simulink/Math Operations/Product', [modelName '/Prod_P'], ...
    'Position', [1000 70 1040 110]);
add_block('simulink/Sinks/To Workspace', [modelName '/P_inst'], ...
    'Position', [1000 130 1040 170], 'VariableName', 'P_inst', 'SaveFormat', 'Array');
add_block('simulink/Sinks/To Workspace', [modelName '/I_out'], ...
    'Position', [1000 240 1040 280], 'VariableName', 'I_sim', 'SaveFormat', 'Array');
add_block('simulink/Math Operations/Product', [modelName '/Prod_Q'], ...
    'Position', [1000 420 1040 460]);
add_block('simulink/Sinks/To Workspace', [modelName '/Q_inst'], ...
    'Position', [1000 480 1040 520], 'VariableName', 'Q_inst', 'SaveFormat', 'Array');

% --- Col 7: Low-pass filters ---
add_block('simulink/Continuous/Transfer Fcn', [modelName '/LPF_P'], ...
    'Position', [1140 70 1190 110], 'Numerator', '[1]', 'Denominator', '[0.05 1]');
add_block('simulink/Continuous/Transfer Fcn', [modelName '/LPF_Q'], ...
    'Position', [1140 420 1190 460], 'Numerator', '[1]', 'Denominator', '[0.05 1]');

% --- Col 8: Displays ---
add_block('simulink/Sinks/Display', [modelName '/Disp_P'], ...
    'Position', [1290 60 1360 120], 'Format', 'short', 'FontSize', '12');
add_block('simulink/Sinks/Display', [modelName '/Disp_I'], ...
    'Position', [1290 230 1360 290], 'Format', 'short', 'FontSize', '12');
add_block('simulink/Sinks/Display', [modelName '/Disp_Q'], ...
    'Position', [1290 410 1360 470], 'Format', 'short', 'FontSize', '12');

% --- Col 9: Scopes ---
add_block('simulink/Sinks/Scope', [modelName '/Scope_VI'], ...
    'Position', [1500 40 1540 200], 'NumInputPorts', '3');
add_block('simulink/Sinks/Scope', [modelName '/Scope_PQ'], ...
    'Position', [1500 280 1540 400], 'NumInputPorts', '2');

% Save handles
hDispP = get_param([modelName '/Disp_P'], 'Handle');
hDispQ = get_param([modelName '/Disp_Q'], 'Handle');
hDispI = get_param([modelName '/Disp_I'], 'Handle');
hScopeVI = get_param([modelName '/Scope_VI'], 'Handle');
hScopePQ = get_param([modelName '/Scope_PQ'], 'Handle');

% =====================================================================
% WIRING — blocks aligned by Y-center for clean orthogonal routing
% =====================================================================

% Voltage sources → Vdiff
add_line(modelName, 'PCS/1', 'Vdiff/1');    % Port 1 = + (upper)
add_line(modelName, 'Grid/1', 'Vdiff/2');   % Port 2 = − (lower)

% Main RL chain: Vdiff → Sum_RL → Gain_1overL → Integrator_I
add_line(modelName, 'Vdiff/1', 'Sum_RL/1');
add_line(modelName, 'Sum_RL/1', 'Gain_1overL/1');
add_line(modelName, 'Gain_1overL/1', 'Integrator_I/1');

% Feedback: Integrator_I → Gain_R → Sum_RL(−)
add_line(modelName, 'Integrator_I/1', 'Gain_R/1');
add_line(modelName, 'Gain_R/1', 'Sum_RL/2');

% P path: Grid×I → P_inst & LPF_P → Disp_P
add_line(modelName, 'Grid/1', 'Prod_P/1');
add_line(modelName, 'Integrator_I/1', 'Prod_P/2');
add_line(modelName, 'Prod_P/1', 'P_inst/1');
add_line(modelName, 'Prod_P/1', 'LPF_P/1');
add_line(modelName, 'LPF_P/1', 'Disp_P/1');

% Current branch
add_line(modelName, 'Integrator_I/1', 'I_out/1');
add_line(modelName, 'Integrator_I/1', 'Disp_I/1');

% Q path: Grid_90×I → Q_inst & LPF_Q → Disp_Q
add_line(modelName, 'Grid_90/1', 'Prod_Q/1');
add_line(modelName, 'Integrator_I/1', 'Prod_Q/2');
add_line(modelName, 'Prod_Q/1', 'Q_inst/1');
add_line(modelName, 'Prod_Q/1', 'LPF_Q/1');
add_line(modelName, 'LPF_Q/1', 'Disp_Q/1');

% Scopes
add_line(modelName, 'Grid/1', 'Scope_VI/1');
add_line(modelName, 'PCS/1', 'Scope_VI/2');
add_line(modelName, 'Integrator_I/1', 'Scope_VI/3');
add_line(modelName, 'Prod_P/1', 'Scope_PQ/1');
add_line(modelName, 'Prod_Q/1', 'Scope_PQ/2');

% Rename scopes to Chinese (AFTER all wiring)
set_param(hScopeVI, 'Name', 'Scope_电压电流');
set_param(hScopePQ, 'Name', 'Scope_功率');

% =====================================================================
% ANNOTATIONS — two-layer approach:
%   Layer 1: area_annotation (built-in/Area) for colored BACKGROUND regions
%            (note: area_annotation has linked BG/FG, so text is hidden;
%             these serve only as colored backdrop for visual grouping)
%   Layer 2: note_annotation (Simulink.Annotation) for visible TEXT labels
%            with proper BG/FG contrast, positioned at the top of each group
% =====================================================================

% --- Layer 1: Colored background areas ---
areaDefs = {
    % suffix   x    y    w    h    color (medium-light for visible backdrop)
    {'Src',     30,  30, 175, 480, '[0.88 0.94 1.00]'}
    {'RL',     285, 210, 600, 230, '[0.88 0.98 0.90]'}
    {'Pwr',    975,  40,  90, 510, '[1.00 0.94 0.82]'}
    {'Disp',  1115,  35, 270, 480, '[1.00 0.88 0.92]'}
    {'Scope', 1475,  15,  90, 415, '[0.90 0.85 1.00]'}
    {'Fb',     635, 340,  98,  80, '[0.95 0.97 0.80]'}
};

for i = 1:length(areaDefs)
    d = areaDefs{i};
    boxName = [modelName '/Area_' d{1}];
    x = d{2}; y = d{3}; w = d{4}; h = d{5};
    add_block('built-in/Area', boxName, 'Position', [x y x+w y+h]);
    set_param(boxName, 'BackgroundColor', d{6});
    set_param(boxName, 'DropShadow', 'off');
end

% --- Layer 2: Readable text annotations (multi-line for visible "boxes") ---
noteDefs = {
    % text (use \n for multi-line boxes),                                 x,   y,   BG color
    {sprintf(['【电压源】\n电网: 固定220Vrms/50Hz/0°\n' ...
              'PCS: 可控幅值与相位\n' ...
              'Grid_90: 超前90°,用于无功计算']),                          40, 495, '[0.75 0.85 1.00]'}
    {sprintf(['【RL电路核心】\nVdiff = U_pcs − U_grid\n' ...
              'Sum_RL = Vdiff − R·i\n' ...
              'Gain(1/L) → Integrator → i(t)\n' ...
              'L=5mH, R=0.1Ω, τ=L/R=50ms']),                           300, 445, '[0.78 0.95 0.80]'}
    {sprintf(['【功率计算】\nP = mean(Grid × I)\n' ...
              'Q = mean(Grid_90 × I)\n' ...
              '电流超前(容性)→Q>0输出无功\n' ...
              '电流滞后(感性)→Q<0吸收无功']),                            970, 555, '[1.00 0.88 0.72]'}
    {sprintf(['【滤波与显示】\nLPF = 1/(0.05s+1)\n' ...
              '滤除100Hz功率纹波\n' ...
              'Display实时显示稳态P/Q/I_rms\n' ...
              '脚本循环后动态更新Display名称']),                         1115, 520, '[1.00 0.82 0.88]'}
    {sprintf(['【波形观测】\nScope_电压电流:\n' ...
              '  Grid / PCS / I 三路波形\n' ...
              'Scope_功率:\n' ...
              '  瞬时P/Q波形(含100Hz纹波)']),                             1475, 435, '[0.88 0.82 1.00]'}
    {sprintf(['【电阻反馈】\ni(t) × R → 负反馈至Sum_RL\n' ...
              '形成闭环阻尼,消除纯积分器发散\n' ...
              '确保: L·di/dt + R·i = Vdiff']),                           635, 425, '[0.90 0.93 0.65]'}
};

for i = 1:length(noteDefs)
    d = noteDefs{i};
    ann = Simulink.Annotation(hModel, d{1});
    ann.Position = [d{2} d{3} d{2}+300 d{3}+120];
    ann.BackgroundColor = d{4};
    ann.ForegroundColor = '[0.05 0.05 0.05]';
    ann.FontSize = 10;
    ann.DropShadow = 'off';
    ann.TeXMode = 'off';
end

save_system(modelName);

% =====================================================================
% BATCH SIMULATION — 9 cases
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
    {220, 230,   0, 'Vp=230V,0 deg (感性示例)'}
};

fprintf('\n');
fprintf('================================================================================\n');
fprintf('  PCS Grid-Connected Simulation Results\n');
fprintf('================================================================================\n');
fprintf('  Base: Vgrid=220Vrms, L=%.1fmH, R=%.1f Ohm, f=50Hz\n', L*1e3, R);
fprintf('  Equation: L*di/dt = Vpcs - Vgrid - R*i\n');
fprintf('--------------------------------------------------------------------------------\n');
fprintf('  %-22s | %9s %9s | %10s %10s | %8s | %s\n', ...
    'Case', 'P_sim(W)', 'P_th(W)', 'Q_sim(var)', 'Q_th(var)', 'I_rms(A)', 'State');
fprintf('--------------------------------------------------------------------------------\n');

docSummary = sprintf('PCS Grid-Connected Simulation Results\n');
docSummary = [docSummary sprintf('Parameters: Vgrid=220Vrms, L=%.1fmH, R=%.1f Ohm, f=50Hz\n', L*1e3, R)];
docSummary = [docSummary sprintf('Equation: L*di/dt = Vpcs - Vgrid - R*i\n')];
docSummary = [docSummary sprintf('---------------------------------------------------------------\n')];
docSummary = [docSummary sprintf('%-22s %10s %12s %12s %s\n', 'Case', 'P(W)', 'Q(var)', 'I_rms(A)', 'State')];
docSummary = [docSummary sprintf('---------------------------------------------------------------\n')];

for k = 1:length(cases)
    c = cases{k};
    Vg_rms = c{1};
    Vp_rms = c{2};
    phi = c{3};
    label = c{4};

    set_param([modelName '/PCS'], 'Phase', num2str(deg2rad(phi)));
    set_param([modelName '/PCS'], 'Amplitude', num2str(Vp_rms*sqrt(2)));
    set_param([modelName '/Grid'], 'Amplitude', num2str(Vg_rms*sqrt(2)));
    set_param([modelName '/Grid_90'], 'Amplitude', num2str(Vg_rms*sqrt(2)));

    simOut = sim(modelName);

    t = simOut.tout;
    dt = 1e-5;
    n_steady = round(0.2 / dt);
    N = length(t);
    idx_start = max(1, N - n_steady + 1);

    P_inst = simOut.get('P_inst');
    Q_inst = simOut.get('Q_inst');
    I_sim = simOut.get('I_sim');
    P_val = mean(P_inst(idx_start:N));
    Q_val = mean(Q_inst(idx_start:N));
    I_val = sqrt(mean(I_sim(idx_start:N).^2));

    [P_th, Q_th, I_th, ~] = calc_theory(phi, Vg_rms, Vp_rms, R, L);

    if abs(P_val) < 50, pState = '-';
    elseif P_val > 0, pState = '输出有功';
    else pState = '吸收有功'; end

    if abs(Q_val) < 50, qState = '-';
    elseif Q_val > 0, qState = '输出无功(容性)';
    else qState = '吸收无功(感性)'; end

    fprintf('  %-22s | %9.1f %9.1f | %10.1f %10.1f | %8.2f | %s, %s\n', ...
        label, P_val, P_th, Q_val, Q_th, I_val, pState, qState);

    set_param(hDispP, 'Name', sprintf('P\\n%.0f W', P_val));
    set_param(hDispQ, 'Name', sprintf('Q\\n%.0f var', Q_val));
    set_param(hDispI, 'Name', sprintf('I_rms\\n%.2f A', I_val));

    stateStr = sprintf('%s, %s', pState, qState);
    docSummary = [docSummary sprintf('%-22s %10.1f %12.1f %12.2f %s\n', ...
        label, P_val, Q_val, I_val, stateStr)];
end

docSummary = [docSummary sprintf('---------------------------------------------------------------\n')];
docSummary = [docSummary sprintf('Key findings:\n')];
docSummary = [docSummary sprintf('1. Vpcs=Vgrid with phase sweep: current mostly capacitive (Q>0) due to\n')];
docSummary = [docSummary sprintf('   inductor impedance angle (86.4 deg) rotating the voltage difference vector.\n')];
docSummary = [docSummary sprintf('2. Inductive state (Q<0) requires Vpcs amplitude slightly above Vgrid (~230V)\n')];
docSummary = [docSummary sprintf('   with near-zero phase difference.\n')];
docSummary = [docSummary sprintf('3. P sign determined by PCS phase vs Grid. Q sign depends on both amplitude and phase.\n')];

set_param(modelName, 'Description', docSummary);

save_system(modelName);

fprintf('================================================================================\n');
fprintf('Model saved: %s.slx\n', modelName);
fprintf('Colored annotation boxes: 6 background areas + 6 text labels marking functional groups.\n');
fprintf('Full results table: File > Model Properties > Description\n');
fprintf('================================================================================\n');

% =====================================================================
% FUNCTIONS (must be at end of script, after all executable code)
% =====================================================================

function [P_th, Q_th, I_rms, theta_i_deg] = calc_theory(phi_deg, Vg_rms, Vp_rms, R, L)
    w = 2*pi*50;
    Z = R + 1j*w*L;
    Vg = Vg_rms;
    Vp = Vp_rms * exp(1j*deg2rad(phi_deg));
    I = (Vp - Vg) / Z;
    I_rms = abs(I);
    theta_i_deg = rad2deg(angle(I));
    S = Vg * conj(I);
    P_th = real(S);
    Q_th = -imag(S);
end

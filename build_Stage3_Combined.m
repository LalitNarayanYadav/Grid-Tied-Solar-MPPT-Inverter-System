%% build_Stage3_Combined.m
% ------------------------------------------------------------------------
% Stage 3: Fully coupled system -- PV array + MPPT boost converter feeding
% a real, dynamic DC bus into a grid-tied inverter with SOGI-PLL sync.
% This replaces Stage 2's stiff 400V assumption with Stage 1's actual
% boost converter output.
%
% DESIGN DECISION -- REALISTIC PANEL STRING SIZING:
% Stage 1's single panel only reaches ~60V, far short of the ~400V needed.
% Rather than forcing an unrealistic ~20x boost ratio, this models a
% 12-panel SERIES STRING (same panel as Stage 1, connected in series) --
% exactly how real solar installations reach several hundred volts DC.
% Series connection multiplies voltage while current stays the same:
%   Isc_string = 8.8A (unchanged), Voc_string = 37V*12 = 444V,
%   Vmp_string ~ 360V, Imp_string = 8.3A (unchanged)
% This needs only a modest, realistic ~1.1x boost ratio to reach 400V.
%
% NEW PHYSICS NOT PRESENT IN EITHER STAGE ALONE (read before testing):
% 1. DOUBLE-FREQUENCY DC BUS RIPPLE: a single-phase inverter delivers
%    power in pulses at 2x grid frequency (100Hz) even when working
%    perfectly -- this is real, unavoidable single-phase physics (three-
%    phase systems don't have this issue). Expect a real 100Hz ripple on
%    Scope_Vout -- that is CORRECT behavior, not a bug.
% 2. DYNAMIC SATURATION: Stage 2 clamped inverter output to a fixed
%    +-400V. Now that the DC bus is dynamic (from Stage 1), the clamp
%    must track it -- using a "Saturation Dynamic" block instead of a
%    fixed one.
% 3. DC-SIDE LOADING: the inverter now draws real current FROM the boost
%    converter's output (via power balance: i_dc_draw = v_inv*i_filter/
%    Vout, assuming lossless conversion), instead of Stage 1's fixed
%    resistive load.
%
% REUSED, ALREADY-VERIFIED CODE:
% The MPPT P&O algorithm (duty-direction-tracking) and the SOGI-PLL
% (clock-based adaptive timing, vd-driven PI) are carried over UNCHANGED
% from their last confirmed-working versions in Stage 1 and Stage 2.
%
% CLEANUP NOTE: an earlier version of this script accidentally contained
% two separate, duplicate attempts at the same low-pass filter on
% P_avail -- LPF_Pavail (a single Transfer Fcn block, correctly wired)
% AND a second Sum/Gain/Integrator-based version that was created but
% never wired to anything. That orphaned trio has been removed here;
% LPF_Pavail alone does the job.
% ------------------------------------------------------------------------

clear; clc;

modelName = 'Stage3_Combined';

if bdIsLoaded(modelName)
    close_system(modelName, 0);
end
if exist([modelName '.slx'], 'file')
    delete([modelName '.slx']);
end

new_system(modelName);
open_system(modelName);
set_param(modelName, 'Solver', 'ode45', 'MaxStep', '2e-5', 'StopTime', '3.0');

sys = modelName;

%% ---------------- Component values ----------------
Isc_string  = 8.8;
Voc_string  = 37 * 12;
n_diode     = 1.3;
Ns_string   = 60 * 12;
Vt          = 0.02585;

Cin   = 100e-6;
L     = 5e-3;
Cout  = 1000e-6;
Rload = 2000;

Vgrid_rms = 230;
Lf   = 5e-3;
Kp_i = 30;
Ki_i = 3000;
Kp_pll = 180;
Ki_pll = 3200;

% Scaled up from Stage 2's fixed 150W test case: this string can deliver
% ~3kW at MPP, so expected current is roughly (3000/230)*sqrt(2) ~= 18A
% peak, not the ~0.9A Stage 2 alone was scaled for.
scopeScale = 20;

%% ---------------- Block placement: PV + Boost + MPPT (Stage 1 side) ----------------

add_block('simulink/Sources/Constant', [sys '/Irradiance_G'], ...
    'Position', [40 40 90 70], 'Value', '1000');

add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [sys '/PV_Array'], 'Position', [140 30 260 90]);

add_block('simulink/Math Operations/Sum', [sys '/Sum_Vpv'], ...
    'Position', [340 30 370 70], 'Inputs', '+-');
add_block('simulink/Math Operations/Gain', [sys '/Gain_1_Cin'], ...
    'Position', [420 30 470 70], 'Gain', num2str(1/Cin));
add_block('simulink/Continuous/Integrator', [sys '/Integrator_Vpv'], ...
    'Position', [520 30 570 70], 'InitialCondition', '300');

add_block('simulink/Math Operations/Product', [sys '/Prod_PVPower'], ...
    'Position', [520 100 560 130]);

add_block('simulink/Math Operations/Sum', [sys '/Sum_iL'], ...
    'Position', [340 160 370 200], 'Inputs', '+-');
add_block('simulink/Math Operations/Gain', [sys '/Gain_1_L'], ...
    'Position', [420 160 470 200], 'Gain', num2str(1/L));
add_block('simulink/Continuous/Integrator', [sys '/Integrator_iL'], ...
    'Position', [520 160 570 200], 'InitialCondition', '0');

add_block('simulink/Math Operations/Product', [sys '/Prod_1mD_Vout'], ...
    'Position', [200 160 250 200]);
add_block('simulink/Math Operations/Product', [sys '/Prod_1mD_iL'], ...
    'Position', [200 290 250 330]);

add_block('simulink/Sources/Constant', [sys '/Const_One'], ...
    'Position', [40 240 70 270], 'Value', '1');
add_block('simulink/Math Operations/Sum', [sys '/Sum_OneMinusD'], ...
    'Position', [130 220 160 260], 'Inputs', '+-');

add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [sys '/MPPT_PO'], 'Position', [340 420 480 480]);

add_block('simulink/Math Operations/Sum', [sys '/Sum_Vout'], ...
    'Position', [340 290 370 330], 'Inputs', '+--');
add_block('simulink/Math Operations/Gain', [sys '/Gain_1_Cout'], ...
    'Position', [420 290 470 330], 'Gain', num2str(1/Cout));
add_block('simulink/Continuous/Integrator', [sys '/Integrator_Vout'], ...
    'Position', [520 290 570 330], 'InitialCondition', '350');

add_block('simulink/Math Operations/Gain', [sys '/Gain_1_Rload'], ...
    'Position', [340 380 390 410], 'Gain', num2str(1/Rload));

add_block('simulink/Math Operations/Product', [sys '/Prod_VinvIfilter'], ...
    'Position', [700 550 750 590]);
add_block('simulink/Discontinuities/Saturation', [sys '/Sat_VoutFloor'], ...
    'Position', [700 610 750 640], 'UpperLimit', '1000', 'LowerLimit', '50');
add_block('simulink/Math Operations/Product', [sys '/Prod_DCDraw'], ...
    'Position', [800 580 850 620], 'Inputs', '*/');

add_block('simulink/Math Operations/Product', [sys '/Prod_Pavail'], ...
    'Position', [700 480 750 520]);

add_block('simulink/Continuous/Transfer Fcn', [sys '/LPF_Pavail'], ...
    'Position', [780 480 830 520], 'Numerator', '[1]', 'Denominator', '[0.02 1]');

%% ---------------- Block placement: Grid + Inverter (Stage 2 side) ----------------

add_block('simulink/Sources/Sine Wave', [sys '/Grid_Voltage'], ...
    'Position', [40 700 90 730], ...
    'Amplitude', num2str(Vgrid_rms * sqrt(2)), ...
    'Frequency', num2str(2*pi*50), 'Phase', '0');

add_block('simulink/Sources/Constant', [sys '/Vgrid_rms_const'], ...
    'Position', [40 760 90 790], 'Value', num2str(Vgrid_rms));

add_block('simulink/Sources/Clock', [sys '/Sim_Clock'], ...
    'Position', [40 820 90 850]);

add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [sys '/Grid_Sync_PLL'], 'Position', [900 680 1060 820]);

add_block('simulink/Math Operations/Sum', [sys '/Sum_CurrentError'], ...
    'Position', [1140 600 1170 640], 'Inputs', '+-');
add_block('simulink/Math Operations/Gain', [sys '/Gain_Kp_i'], ...
    'Position', [1220 580 1270 610], 'Gain', num2str(Kp_i));
add_block('simulink/Math Operations/Gain', [sys '/Gain_Ki_i'], ...
    'Position', [1220 640 1270 670], 'Gain', num2str(Ki_i));
add_block('simulink/Continuous/Integrator', [sys '/Integrator_CurrentPI'], ...
    'Position', [1320 640 1360 670], 'InitialCondition', '0');
add_block('simulink/Math Operations/Sum', [sys '/Sum_PI_Output'], ...
    'Position', [1400 600 1430 640], 'Inputs', '++');

add_block('simulink/Math Operations/Sum', [sys '/Sum_FF_PI'], ...
    'Position', [1480 620 1510 660], 'Inputs', '++');

add_block('simulink/Math Operations/Gain', [sys '/Gain_NegVout'], ...
    'Position', [1480 700 1510 730], 'Gain', '-1');
add_block('simulink/Discontinuities/Saturation Dynamic', [sys '/Saturation_Vinv'], ...
    'Position', [1560 610 1610 650]);

add_block('simulink/Math Operations/Sum', [sys '/Sum_Lfilter'], ...
    'Position', [1660 600 1690 630], 'Inputs', '+-');
add_block('simulink/Math Operations/Gain', [sys '/Gain_1_Lf'], ...
    'Position', [1740 600 1780 630], 'Gain', num2str(1/Lf));
add_block('simulink/Continuous/Integrator', [sys '/Integrator_ifilter'], ...
    'Position', [1820 600 1860 630], 'InitialCondition', '0');

add_block('simulink/Math Operations/Gain', [sys '/Gain_Scope_Scale'], ...
    'Position', [1900 560 1930 590], 'Gain', num2str(scopeScale));

%% ---------------- Scopes ----------------
add_block('simulink/Sinks/Scope', [sys '/Scope_Sync'], ...
    'Position', [1980 560 2020 600]);
set_param([sys '/Scope_Sync'], 'NumInputPorts', '2');

add_block('simulink/Sinks/Scope', [sys '/Scope_Theta'], ...
    'Position', [1140 750 1180 790]);

add_block('simulink/Sinks/Scope', [sys '/Scope_Vinv'], ...
    'Position', [1660 700 1700 740]);

add_block('simulink/Sinks/Scope', [sys '/Scope_Vout'], ...
    'Position', [640 290 680 330]);

add_block('simulink/Sinks/Scope', [sys '/Scope_Power'], ...
    'Position', [640 30 680 70]);

%% ---------------- MATLAB Function code ----------------

pvArrayCode = [ ...
    'function I_array = PV_Array(V_pv, G)\n' ...
    '%% Simplified single-diode PV model for a 12-panel series STRING\n' ...
    ['Isc = ' num2str(Isc_string) '; Voc = ' num2str(Voc_string) ...
     '; n = ' num2str(n_diode) '; Ns = ' num2str(Ns_string) ...
     '; Vt = ' num2str(Vt) ';\n'] ...
    'Iph = Isc * (G/1000);\n' ...
    'Io  = Isc / (exp(Voc/(n*Vt*Ns)) - 1);\n' ...
    'I_array = Iph - Io*(exp(V_pv/(n*Vt*Ns)) - 1);\n' ...
    'I_array = max(I_array, 0);\n' ...
    'end\n'];

mpptCode = [ ...
    'function D = MPPT_PO(V_pv, I_pv)\n' ...
    '%% Perturb & Observe MPPT algorithm (duty-direction-tracking version)\n' ...
    '%% Step size reduced from 0.001 to 0.0002 (5x slower) to fix a real\n' ...
    '%% Stage 3 coupling bug: MPPT was reacting to instantaneous power\n' ...
    '%% measurements faster than the rest of the system (now behind a 20ms\n' ...
    '%% low-pass filter on the DC-AC coupling path) could actually settle,\n' ...
    '%% causing a sustained, undamped oscillation instead of convergence.\n' ...
    '%% Slowing MPPT gives the whole loop time to settle between\n' ...
    '%% perturbations, which is what the P&O algorithm assumes.\n' ...
    'persistent D_prev P_prev direction\n' ...
    'if isempty(D_prev)\n' ...
    '    D_prev = 0.5; P_prev = 0; direction = 1;\n' ...
    'end\n' ...
    'step = 0.0002;\n' ...
    'P = V_pv * I_pv;\n' ...
    'if P < P_prev\n' ...
    '    direction = -direction;\n' ...
    'end\n' ...
    'D = D_prev + direction * step;\n' ...
    'D = min(max(D, 0.05), 0.9);\n' ...
    'D_prev = D; P_prev = P;\n' ...
    'end\n'];

plCode = [ ...
    'function [i_ref, theta_out] = Grid_Sync_PLL(v_grid, P_avail, Vgrid_rms, t)\n' ...
    '%% SOGI-PLL (clock-based adaptive timing, vd-driven PI)\n' ...
    'persistent x1 x2 theta integral_term t_prev\n' ...
    'if isempty(x1)\n' ...
    '    x1 = 0; x2 = 0; theta = 0; integral_term = 0; t_prev = 0;\n' ...
    'end\n' ...
    'dt = t - t_prev;\n' ...
    't_prev = t;\n' ...
    'if dt <= 0\n' ...
    '    dt = 1e-6;\n' ...
    'end\n' ...
    'k = 1.4142;\n' ...
    'omega0 = 2*pi*50;\n' ...
    ['Kp_pll = ' num2str(Kp_pll) ';\n'] ...
    ['Ki_pll = ' num2str(Ki_pll) ';\n'] ...
    '\n' ...
    'x1_new = x1 + dt*omega0*(k*(v_grid - x1) - x2);\n' ...
    'x2_new = x2 + dt*omega0*x1;\n' ...
    'x1 = x1_new;\n' ...
    'x2 = x2_new;\n' ...
    '\n' ...
    'vd =  x1*cos(theta) + x2*sin(theta);\n' ...
    'vq = -x1*sin(theta) + x2*cos(theta);\n' ...
    '\n' ...
    'integral_term = integral_term + dt*Ki_pll*vd;\n' ...
    'domega = Kp_pll*vd + integral_term;\n' ...
    'omega = omega0 + domega;\n' ...
    '\n' ...
    'theta = theta + dt*omega;\n' ...
    'theta = mod(theta, 2*pi);\n' ...
    'theta_out = theta;\n' ...
    '\n' ...
    'I_ref_amplitude = (P_avail / Vgrid_rms) * sqrt(2);\n' ...
    'i_ref = I_ref_amplitude * sin(theta);\n' ...
    'end\n'];

autoFillOK = true;
try
    rt = sfroot;

    pvChart = rt.find('-isa', 'Stateflow.EMChart', 'Path', [modelName '/PV_Array']);
    pvChart.Script = sprintf(pvArrayCode);

    mpptChart = rt.find('-isa', 'Stateflow.EMChart', 'Path', [modelName '/MPPT_PO']);
    mpptChart.Script = sprintf(mpptCode);

    plChart = rt.find('-isa', 'Stateflow.EMChart', 'Path', [modelName '/Grid_Sync_PLL']);
    plChart.Script = sprintf(plCode);

    fprintf('[OK] All three MATLAB Function blocks were set automatically.\n');
catch ME
    autoFillOK = false;
    fprintf('\n[MANUAL STEP NEEDED] Paste each block''s code in manually:\n\n');
    fprintf('--- PV_Array ---\n%s\n', sprintf(pvArrayCode));
    fprintf('--- MPPT_PO ---\n%s\n', sprintf(mpptCode));
    fprintf('--- Grid_Sync_PLL ---\n%s\n', sprintf(plCode));
end

if ~autoFillOK
    error(['Stopping here so wiring does not run against incomplete blocks. ' ...
           'See the manual-step instructions printed above.']);
end

%% ---------------- Wiring: PV + Boost + MPPT ----------------

add_line(sys, 'Irradiance_G/1', 'PV_Array/2', 'autorouting', 'on');
add_line(sys, 'Integrator_Vpv/1', 'PV_Array/1', 'autorouting', 'on');

add_line(sys, 'PV_Array/1', 'Sum_Vpv/1', 'autorouting', 'on');
add_line(sys, 'PV_Array/1', 'MPPT_PO/2', 'autorouting', 'on');
add_line(sys, 'PV_Array/1', 'Prod_PVPower/2', 'autorouting', 'on');
add_line(sys, 'Integrator_Vpv/1', 'Prod_PVPower/1', 'autorouting', 'on');
add_line(sys, 'Prod_PVPower/1', 'Scope_Power/1', 'autorouting', 'on');

add_line(sys, 'Integrator_iL/1', 'Sum_Vpv/2', 'autorouting', 'on');
add_line(sys, 'Sum_Vpv/1', 'Gain_1_Cin/1', 'autorouting', 'on');
add_line(sys, 'Gain_1_Cin/1', 'Integrator_Vpv/1', 'autorouting', 'on');

add_line(sys, 'Integrator_Vpv/1', 'Sum_iL/1', 'autorouting', 'on');
add_line(sys, 'Prod_1mD_Vout/1', 'Sum_iL/2', 'autorouting', 'on');
add_line(sys, 'Sum_iL/1', 'Gain_1_L/1', 'autorouting', 'on');
add_line(sys, 'Gain_1_L/1', 'Integrator_iL/1', 'autorouting', 'on');

add_line(sys, 'Integrator_Vout/1', 'Prod_1mD_Vout/1', 'autorouting', 'on');
add_line(sys, 'Sum_OneMinusD/1', 'Prod_1mD_Vout/2', 'autorouting', 'on');
add_line(sys, 'Integrator_iL/1', 'Prod_1mD_iL/1', 'autorouting', 'on');
add_line(sys, 'Sum_OneMinusD/1', 'Prod_1mD_iL/2', 'autorouting', 'on');

add_line(sys, 'Const_One/1', 'Sum_OneMinusD/1', 'autorouting', 'on');
add_line(sys, 'MPPT_PO/1', 'Sum_OneMinusD/2', 'autorouting', 'on');
add_line(sys, 'Integrator_Vpv/1', 'MPPT_PO/1', 'autorouting', 'on');

add_line(sys, 'Prod_1mD_iL/1', 'Sum_Vout/1', 'autorouting', 'on');
add_line(sys, 'Integrator_Vout/1', 'Gain_1_Rload/1', 'autorouting', 'on');
add_line(sys, 'Gain_1_Rload/1', 'Sum_Vout/2', 'autorouting', 'on');
add_line(sys, 'Prod_DCDraw/1', 'Sum_Vout/3', 'autorouting', 'on');

add_line(sys, 'Sum_Vout/1', 'Gain_1_Cout/1', 'autorouting', 'on');
add_line(sys, 'Gain_1_Cout/1', 'Integrator_Vout/1', 'autorouting', 'on');
add_line(sys, 'Integrator_Vout/1', 'Scope_Vout/1', 'autorouting', 'on');

add_line(sys, 'Integrator_Vout/1', 'Prod_Pavail/1', 'autorouting', 'on');
add_line(sys, 'Prod_1mD_iL/1', 'Prod_Pavail/2', 'autorouting', 'on');

%% ---------------- Wiring: Grid + Inverter ----------------

add_line(sys, 'Grid_Voltage/1', 'Grid_Sync_PLL/1', 'autorouting', 'on');
add_line(sys, 'Prod_Pavail/1', 'LPF_Pavail/1', 'autorouting', 'on');
add_line(sys, 'LPF_Pavail/1', 'Grid_Sync_PLL/2', 'autorouting', 'on');
add_line(sys, 'Vgrid_rms_const/1', 'Grid_Sync_PLL/3', 'autorouting', 'on');
add_line(sys, 'Sim_Clock/1', 'Grid_Sync_PLL/4', 'autorouting', 'on');

add_line(sys, 'Grid_Sync_PLL/1', 'Sum_CurrentError/1', 'autorouting', 'on');
add_line(sys, 'Grid_Sync_PLL/2', 'Scope_Theta/1', 'autorouting', 'on');

add_line(sys, 'Integrator_ifilter/1', 'Sum_CurrentError/2', 'autorouting', 'on');
add_line(sys, 'Sum_CurrentError/1', 'Gain_Kp_i/1', 'autorouting', 'on');
add_line(sys, 'Sum_CurrentError/1', 'Gain_Ki_i/1', 'autorouting', 'on');
add_line(sys, 'Gain_Ki_i/1', 'Integrator_CurrentPI/1', 'autorouting', 'on');
add_line(sys, 'Gain_Kp_i/1', 'Sum_PI_Output/1', 'autorouting', 'on');
add_line(sys, 'Integrator_CurrentPI/1', 'Sum_PI_Output/2', 'autorouting', 'on');

add_line(sys, 'Grid_Voltage/1', 'Sum_FF_PI/1', 'autorouting', 'on');
add_line(sys, 'Sum_PI_Output/1', 'Sum_FF_PI/2', 'autorouting', 'on');

add_line(sys, 'Sum_FF_PI/1', 'Saturation_Vinv/1', 'autorouting', 'on');
add_line(sys, 'Integrator_Vout/1', 'Saturation_Vinv/2', 'autorouting', 'on');
add_line(sys, 'Integrator_Vout/1', 'Gain_NegVout/1', 'autorouting', 'on');
add_line(sys, 'Gain_NegVout/1', 'Saturation_Vinv/3', 'autorouting', 'on');

add_line(sys, 'Saturation_Vinv/1', 'Scope_Vinv/1', 'autorouting', 'on');
add_line(sys, 'Saturation_Vinv/1', 'Sum_Lfilter/1', 'autorouting', 'on');
add_line(sys, 'Grid_Voltage/1', 'Sum_Lfilter/2', 'autorouting', 'on');
add_line(sys, 'Sum_Lfilter/1', 'Gain_1_Lf/1', 'autorouting', 'on');
add_line(sys, 'Gain_1_Lf/1', 'Integrator_ifilter/1', 'autorouting', 'on');

add_line(sys, 'Integrator_ifilter/1', 'Gain_Scope_Scale/1', 'autorouting', 'on');
add_line(sys, 'Gain_Scope_Scale/1', 'Scope_Sync/1', 'autorouting', 'on');
add_line(sys, 'Grid_Voltage/1', 'Scope_Sync/2', 'autorouting', 'on');

add_line(sys, 'Saturation_Vinv/1', 'Prod_VinvIfilter/1', 'autorouting', 'on');
add_line(sys, 'Integrator_ifilter/1', 'Prod_VinvIfilter/2', 'autorouting', 'on');
add_line(sys, 'Integrator_Vout/1', 'Sat_VoutFloor/1', 'autorouting', 'on');
add_line(sys, 'Prod_VinvIfilter/1', 'Prod_DCDraw/1', 'autorouting', 'on');
add_line(sys, 'Sat_VoutFloor/1', 'Prod_DCDraw/2', 'autorouting', 'on');

%% ---------------- Save ----------------
save_system(sys);
fprintf('Model saved as %s.slx in your current MATLAB folder.\n', modelName);
fprintf('Press Ctrl+T or click Run to simulate.\n');
fprintf('\n');
fprintf('WHAT TO CHECK:\n');
fprintf('  Scope_Power  -> PV power should climb and roughly hold\n');
fprintf('  Scope_Vout   -> DC bus ~350-450V, with a visible 100Hz ripple\n');
fprintf('                  (correct single-phase physics, not a bug)\n');
fprintf('  Scope_Theta  -> clean 50Hz sawtooth\n');
fprintf('  Scope_Vinv   -> clean sine tracking the coupled DC bus\n');
fprintf('  Scope_Sync   -> current (now correctly scaled, ~18A peak\n');
fprintf('                  expected) in phase with grid voltage\n');

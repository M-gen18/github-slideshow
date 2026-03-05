% main_linmpc_vs_nmpc_casadi.m
% ------------------------------------------------------------
% Simple LMPC vs NMPC comparison (no baseline),
% with clear visual styles and prediction RMSE plots.
% ------------------------------------------------------------

clear; clc; close all;
addpath(genpath(pwd));

Ts = 1;
T_end_s = 2*3600;

[t, Pdem_W, meta] = load_uav_demand_profile(Ts, T_end_s, struct('use_drivetrain_efficiency_map_uav', true));
Pdem_W = Pdem_W(:);

par = batt_params_2rc_uav_lipo();
x0  = batt_init_state(par);
x0.SoC = 0.95;
x0.SoH = 1.00;
x0.T   = 25+273.15;
x0.V1  = 0;
x0.V2  = 0;
Tamb = 25+273.15;

cfg_pred = struct();
cfg_pred.enable_predlog = true;
cfg_pred.predlog_stride_s = 600;
cfg_pred.predlog_max = 8;

mpcL = mpc_settings_uav(); mpcL.Ts = Ts; mpcL.use_casadi = false;
[logL, predL] = simulate_uav_with_mpc(Pdem_W, t, Ts, par, x0, Tamb, mpcL, cfg_pred);

mpcN = mpc_settings_uav(); mpcN.Ts = Ts; mpcN.use_casadi = true;
[logN, predN] = simulate_uav_with_mpc(Pdem_W, t, Ts, par, x0, Tamb, mpcN, cfg_pred);

% Overlay only (no baseline)
figure('Name','Power: LMPC vs NMPC');
plot(t, Pdem_W, 'k-', 'LineWidth', 1.5); hold on;
plot(t, logL.Pdel, 'b--', 'LineWidth', 1.6);
plot(t, logN.Pdel, 'r-',  'LineWidth', 1.6);
grid on; xlabel('t [s]'); ylabel('Power [W]');
title('Power tracking: LMPC vs NMPC');
legend('Pdem','LMPC','NMPC','Location','Best');

% RMSE
plot_prediction_error_lmpc_nmpc(logL, predL, logN, predN, mpcL);

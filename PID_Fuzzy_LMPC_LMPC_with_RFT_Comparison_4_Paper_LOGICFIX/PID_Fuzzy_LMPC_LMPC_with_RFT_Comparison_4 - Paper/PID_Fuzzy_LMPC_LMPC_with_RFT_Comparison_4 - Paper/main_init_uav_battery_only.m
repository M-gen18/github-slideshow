clear; clc; close all;

% Self-contained UAV Battery-only baseline project
% Run this file. No external paths required.

addpath(genpath(pwd));

%% Mission settings
Ts = 1;                 % [s]
T_end_s = 2*3600;       % 2 hours

% Generate demand profile (shaft power) then convert to battery demand via drivetrain efficiency
[t, Pdem_W, meta] = load_uav_demand_profile(Ts, T_end_s, struct('use_drivetrain_map', true));

%% Battery pack parameters (paper-style: 3S 13000mAh LiPo), SoC reserve = 0.20
par = batt_params_2rc_uav_lipo();

%% Initial conditions
x0 = batt_init_state(par);
x0.SoC = 0.95;                 % start near full
x0.SoH = 1.00;
x0.T   = 25 + 273.15;
x0.V1  = 0;
x0.V2  = 0;

Tamb = 25 + 273.15;

%% Baseline controller config
cfg = struct();
cfg.enable_power_unmet = true;

%% Simulate
log = simulate_battery_only_uav(Pdem_W, t, Ts, par, x0, Tamb, cfg);

%% Plots
plot_uav_logs(log, par, meta);

%% Summary
E_pack_kWh = par.Vnom * par.Qnom_Ah / 1000;
usable_kWh = E_pack_kWh * (x0.SoC - par.SoC_min);
E_dem_kWh  = trapz(t, max(Pdem_W,0))/3600/1000;
E_del_kWh  = trapz(log.t, max(log.Pdel,0))/3600/1000;

fprintf('\n--- UAV Battery-Only Baseline Report ---\n');
fprintf('Mission duration: %.1f min\n', (t(end)-t(1))/60);
fprintf('Pack nominal energy: %.3f kWh\n', E_pack_kWh);
fprintf('Usable energy (SoC %.2f -> %.2f): %.3f kWh\n', x0.SoC, par.SoC_min, usable_kWh);
fprintf('Demanded discharge energy (approx): %.3f kWh\n', E_dem_kWh);
fprintf('Delivered discharge energy (approx): %.3f kWh\n', E_del_kWh);

fprintf('\nStop reason: %s\n', log.stop_reason);
fprintf('Endurance achieved: %.1f min (%.0f s)\n', log.endurance_min, log.endurance_s);
fprintf('End SoC: %.4f\n', log.SoC(end));
fprintf('End SoH: %.6f (loss=%.6f)\n', log.SoH(end), 1-log.SoH(end));
fprintf('Max T [C]: %.2f\n', max(log.T)-273.15);
fprintf('Peak |I| [A]: %.2f\n', max(abs(log.I)));
fprintf('RMSE(Pdel vs Pdem) [W]: %.2f\n', sqrt(mean((log.Pdel-log.Pdem).^2)));
fprintf('Unmet demand energy [kWh]: %.4f\n', log.unmet_kWh);

fprintf('\n--- Constraint NEAR-activation (%% time) ---\n');
fprintf('near_Imax:   %.2f%%\n', 100*mean(log.near_Imax));
fprintf('near_Vmin:   %.2f%%\n', 100*mean(log.near_Vmin));
fprintf('near_Tmax:   %.2f%%\n', 100*mean(log.near_Tmax));
fprintf('near_SoCmin: %.2f%%\n', 100*mean(log.near_SoCmin));
fprintf('near_any:    %.2f%%\n', 100*mean(log.near_any));

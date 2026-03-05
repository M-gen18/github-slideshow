clear; clc; close all;
addpath(genpath(pwd));

Ts = 1;
T_end_s = 2*3600;

[t, Pdem_W, meta] = load_uav_demand_profile(Ts, T_end_s, struct('use_drivetrain_efficiency_map_uav', true));
plot_uav_mission_profile(t, Pdem_W, meta);

par = batt_params_2rc_uav_lipo();
x0  = batt_init_state(par);
x0.SoC = 0.95;
x0.SoH = 1.00;
x0.T   = 25+273.15;
x0.V1  = 0;
x0.V2  = 0;

Tamb = 25+273.15;

cfg_base = struct('enable_power_unmet', true);
log_base = simulate_battery_only_uav(Pdem_W, t, Ts, par, x0, Tamb, cfg_base);

mpc = mpc_settings_uav();
mpc.Ts = Ts;

cfg_mpc = struct();
cfg_mpc.enable_predlog = true;
cfg_mpc.predlog_stride_s = 600; % every 10 minutes
cfg_mpc.predlog_max = 8;

[log_mpc, predlog] = simulate_uav_with_mpc(Pdem_W, t, Ts, par, x0, Tamb, mpc, cfg_mpc);

plot_compare_baseline_mpc(log_base, log_mpc, par, meta);
plot_mpc_prediction_debug(predlog, par);

Edem_Wh  = sum(max(Pdem_W,0))*Ts/3600;
Ebase_Wh = sum(max(log_base.Pdel,0))*Ts/3600;
Empc_Wh  = sum(max(log_mpc.Pdel,0))*Ts/3600;

Irms_base = sqrt(mean(log_base.I.^2));
Irms_mpc  = sqrt(mean(log_mpc.I.^2));

fprintf('\n--- Baseline vs Linear MPC Summary ---\n');
fprintf('Demanded mission energy: %.1f Wh\n', Edem_Wh);
fprintf('Baseline endurance: %.1f min, unmet kWh: %.4f\n', log_base.endurance_min, log_base.unmet_kWh);
fprintf('Linear MPC endurance: %.1f min, unmet kWh: %.4f\n', log_mpc.endurance_min, log_mpc.unmet_kWh);
fprintf('Baseline energy delivered: %.1f Wh, mean Pdel: %.2f W\n', Ebase_Wh, mean(log_base.Pdel));
fprintf('Linear MPC energy delivered: %.1f Wh, mean Pdel: %.2f W\n', Empc_Wh, mean(log_mpc.Pdel));
fprintf('Baseline peak |I|:  %.2f A, Irms: %.2f A, max T: %.2f C\n', max(abs(log_base.I)), Irms_base, max(log_base.T)-273.15);
fprintf('Linear MPC peak |I|: %.2f A, Irms: %.2f A, max T: %.2f C\n', max(abs(log_mpc.I)), Irms_mpc, max(log_mpc.T)-273.15);
fprintf('Mean battery demand (mission): %.2f W\n', mean(Pdem_W));
if isfield(meta,'Pshaft_W')
    fprintf('Mean shaft power: %.2f W, mean eta: %.3f\n', mean(meta.Pshaft_W), mean(meta.eta_drv));
if isfield(log_mpc,'qp_fail')
    fprintf('QP fail rate: %.2f %%\n', 100*mean(log_mpc.qp_fail));
end
end

fprintf('\nTUNING TIP (linear MPC):\n');
fprintf(' - If endurance is worse: decrease qP and/or increase qI, qdI, and qSoC_terminal.\n');
fprintf(' - Start tuning: qP in [0.2..1], qI in [0.05..0.2], qdI in [1..5], qSoC_terminal in [2..20].\n');
fprintf(' - Keep constraints hard; slack will come later for feasibility under transients.\n');


% QP failure diagnostics
if exist('plot_qp_fail_diagnostics','file')==2
    plot_qp_fail_diagnostics(log_mpc, par);
end

% main_mpc_robustness_comparison.m
% ------------------------------------------------------------
% Robustness comparison for ONE controller (set below: NMPC).
% Case A: Perfect future knowledge
% Case B: Prediction mismatch + disturbances
% Produces robustness plots + realized cost comparison.
% ------------------------------------------------------------

clear; clc; close all;
addpath(genpath(pwd));

Ts = 1;
T_end_s = 2*3600;

[t, Pdem_W, meta] = load_uav_demand_profile(Ts, T_end_s, struct('use_drivetrain_efficiency_map_uav', true));

Pdem_nom  = Pdem_W(:);
Pdem_real = Pdem_nom;

% Disturbances
rng(1);
idx_mid = (t >= 900) & (t <= 3300);
nmid = sum(idx_mid);
e = zeros(nmid,1);
for i=2:nmid
    e(i) = 0.97*e(i-1) + randn;
end
if std(e) > 0, e = e/std(e); end
noiseW = 6*e;

spike = zeros(nmid,1);
spk_idx = rand(nmid,1) < 0.003;
spike(spk_idx) = 20 + 15*rand(sum(spk_idx),1);

Pdem_real(idx_mid) = max(Pdem_real(idx_mid) + noiseW + spike, 0);

figure('Name','Demand profile: nominal vs disturbed');
plot(t, Pdem_nom, 'k-', 'LineWidth', 1.2); hold on;
plot(t, Pdem_real,'m--','LineWidth', 1.2);
grid on; xlabel('t [s]'); ylabel('P_{dem} [W]');
legend('Nominal (pred)','Disturbed (plant)','Location','Best');

plot_uav_mission_profile(t, Pdem_real, meta);

par = batt_params_2rc_uav_lipo();
x0  = batt_init_state(par);
x0.SoC = 0.95; x0.SoH = 1.00;
x0.T = 25+273.15; x0.V1=0; x0.V2=0;
Tamb = 25+273.15;

cfg_base = struct('enable_power_unmet', true);
log_base = simulate_battery_only_uav(Pdem_real, t, Ts, par, x0, Tamb, cfg_base);

mpc = mpc_settings_uav();
mpc.Ts = Ts;

% Choose controller here:
mpc.use_casadi = true;   % NMPC
% mpc.use_casadi = false; % LMPC

cfg_perf = struct();
cfg_perf.enable_predlog = true;
cfg_perf.predlog_stride_s = 600;
cfg_perf.predlog_max = 8;
cfg_perf.no_anticipation = false;

[log_perf, ~] = simulate_uav_with_mpc(Pdem_real, t, Ts, par, x0, Tamb, mpc, cfg_perf);

cfg_miss = cfg_perf;
cfg_miss.no_anticipation = true;
cfg_miss.Pdem_pred = Pdem_nom; % MPC predicts nominal (no noise)

[log_miss, ~] = simulate_uav_with_mpc(Pdem_real, t, Ts, par, x0, Tamb, mpc, cfg_miss);

plot_compare_baseline_lmpc_nmpc(log_base, log_perf, log_miss, par, meta); % reuse 5-plot style
plot_mpc_robustness_comparison(log_perf, log_miss, par, meta);

if exist('compute_mpc_realized_cost','file')==2
    J_perf = compute_mpc_realized_cost(log_perf, mpc);
    J_miss = compute_mpc_realized_cost(log_miss, mpc);

    fprintf('\n--- MPC Robustness: Realized Cost Comparison ---\n');
    fprintf('Perfect knowledge:   total=%.3e (track=%.3e, mag=%.3e, slew=%.3e, term=%.3e)\n', ...
        J_perf.total, J_perf.track, J_perf.mag, J_perf.slew, J_perf.term);
    fprintf('Prediction mismatch: total=%.3e (track=%.3e, mag=%.3e, slew=%.3e, term=%.3e)\n', ...
        J_miss.total, J_miss.track, J_miss.mag, J_miss.slew, J_miss.term);
    fprintf('Cost ratio (mismatch/perfect) = %.3f\n', J_miss.total / max(J_perf.total, eps));
end

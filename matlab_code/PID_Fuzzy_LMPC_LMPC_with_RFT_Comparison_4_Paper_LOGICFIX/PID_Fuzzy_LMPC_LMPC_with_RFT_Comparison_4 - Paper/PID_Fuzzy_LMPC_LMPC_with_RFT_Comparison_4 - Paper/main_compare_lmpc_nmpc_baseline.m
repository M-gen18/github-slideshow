clear; clc; close all;
addpath(genpath(pwd));

Ts = 1;
T_end_s = 2*3600;

[t, Pdem_W, meta] = load_uav_demand_profile(Ts, T_end_s, struct('use_drivetrain_map', true));
Pdem_W = Pdem_W(:);

% Disturbance/noise
use_noise = true;
Pdem_real = Pdem_W;
if use_noise
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
end

plot_uav_mission_profile(t, Pdem_real, meta);

par = batt_params_2rc_uav_lipo();
x0  = batt_init_state(par);
x0.SoC = 0.95;
x0.SoH = 1.00;
x0.T   = 25+273.15;
x0.V1  = 0;
x0.V2  = 0;
Tamb = 25+273.15;

% EMS settings (smooth derating near limits)
ems_cfg = struct();
ems_cfg.SoC_soft   = 0.30;
ems_cfg.SoC_hard   = 0.20;
ems_cfg.V_margin   = 0.20;
ems_cfg.eta_margin = 0.98;
ems_cfg.alpha      = 0.15;
ems_cfg.P_floor_W  = 30;     % minimum commanded power (W)
ems_cfg.f_min      = 0.25;   % minimum derating factor

[Pdem_cmd, ems_info] = limit_demand_uav(Pdem_real, t, Ts, par, x0, Tamb, ems_cfg);

figure('Name','EMS demand derating');
plot(t, Pdem_real, 'k-', 'LineWidth', 1.2); hold on;
plot(t, Pdem_cmd,  'k--', 'LineWidth', 2.0);
grid on; xlabel('t [s]'); ylabel('Power [W]');
title('Original mission demand vs EMS-limited demand');
legend('Original demand','Limited demand','Location','Best');

% Use original mission demand for the controller (EMS is advisory for plots/diagnostics)
Pdem_sim = Pdem_real;
meta.ems_cmd = Pdem_cmd;

cfg_base = struct('enable_power_unmet', true);
log_base = simulate_battery_only_uav(Pdem_sim, t, Ts, par, x0, Tamb, cfg_base);

cfg_pred = struct();
cfg_pred.enable_predlog = true;
cfg_pred.predlog_stride_s = 600;
cfg_pred.predlog_max = 8;
cfg_pred.no_anticipation = false;
cfg_pred.SoC_min_mpc_override = 0.01;  % allow deeper discharge; protect via soft penalties

mpcL = mpc_settings_uav();
mpcL.Ts = Ts;
mpcL.use_casadi = false;
% ---- Ensure MPC horizon field exists for QP builder ----
if ~isfield(mpcL,'Np') || isempty(mpcL.Np)
    if isfield(mpcL,'N') && ~isempty(mpcL.N)
        mpcL.Np = mpcL.N;
    elseif isfield(mpcL,'Hp') && ~isempty(mpcL.Hp)
        mpcL.Np = mpcL.Hp;
    elseif isfield(mpcL,'H') && ~isempty(mpcL.H) && isnumeric(mpcL.H)
        mpcL.Np = mpcL.H;
    else
        % last resort: set a reasonable default horizon
        mpcL.Np = 20;
    end
end

% ---- Ensure Ts exists too ----
if ~isfield(mpcL,'Ts') || isempty(mpcL.Ts)
    if exist('Ts','var') && ~isempty(Ts)
        mpcL.Ts = Ts;
    elseif isfield(mpcL,'dt') && ~isempty(mpcL.dt)
        mpcL.Ts = mpcL.dt;
    else
        mpcL.Ts = 1; % fallback
    end
end
if ~isfield(mpcL,'Np')
    if isfield(mpcL,'N')
        mpcL.Np = mpcL.N;
    else
        mpcL.Np = 20; % choose your horizon
    end
end
% ---- Professor RFT linear constraint (enabled) ----
mpcL.use_rft_constraint = true;
mpcL.RFT_spec_s = t(end);   % desired remaining flying time spec = remaining mission time
[log_lmpc, pred_lmpc] = simulate_uav_with_mpc(Pdem_sim, t, Ts, par, x0, Tamb, mpcL, cfg_pred);

mpcN = mpc_settings_uav();
mpcN.Ts = Ts;
mpcN.use_casadi = true;
mpcN.use_rft_constraint = true;
mpcN.RFT_spec_s = t(end);

[log_nmpc, pred_nmpc] = simulate_uav_with_mpc(Pdem_sim, t, Ts, par, x0, Tamb, mpcN, cfg_pred);

meta.t = t;
meta.Pdem_real = Pdem_real;
meta.Pdem_cmd  = Pdem_cmd;
plot_compare_baseline_lmpc_nmpc(log_base, log_lmpc, log_nmpc, par, meta);

% Extra: overlay predicted horizons on mission demand (power + current)
plot_power_current_predictions(t, Pdem_real, log_lmpc, pred_lmpc, log_nmpc, pred_nmpc, par);

plot_prediction_error_lmpc_nmpc(log_lmpc, pred_lmpc, log_nmpc, pred_nmpc, mpcL);
% -------- RFT Visualization --------
if isfield(log_lmpc,'RFT_active') && any(log_lmpc.RFT_active>0)
    figure('Name','RFT constraint evolution');
    grid on; hold on;

    plot(log_lmpc.t, log_lmpc.RFT_rem_s,'LineWidth',1.8);
    yyaxis right
    plot(log_lmpc.t, log_lmpc.RFT_SoC_RHS,'LineWidth',1.8);
    plot(log_lmpc.t, log_lmpc.RFT_SoC_term_pred,'--','LineWidth',1.8);

    xlabel('t [s]');
    yyaxis left; ylabel('Remaining Flying Time [s]');
    yyaxis right; ylabel('SoC [-]');
    title('Online RFT update and terminal SoC constraint');
    legend('RFT remaining','RFT SoC bound','Predicted SoC_{k+N}','Location','best');
end
% Summary output (energy and endurance)
Edem_real_Wh = sum(max(Pdem_real,0))*Ts/3600;
Edem_cmd_Wh  = sum(max(Pdem_cmd,0))*Ts/3600;
Ebase_Wh  = sum(max(log_base.Pdel,0))*Ts/3600;
Elmpc_Wh  = sum(max(log_lmpc.Pdel,0))*Ts/3600;
Enmpc_Wh  = sum(max(log_nmpc.Pdel,0))*Ts/3600;
Irms_base = sqrt(mean(log_base.I.^2));
Irms_lmpc = sqrt(mean(log_lmpc.I.^2));
Irms_nmpc = sqrt(mean(log_nmpc.I.^2));

fprintf('\n--- Baseline vs LMPC vs NMPC Summary (mission demand tracking; EMS advisory) ---\n');
fprintf('Original demanded mission energy: %.1f Wh\n', Edem_real_Wh);
fprintf('EMS-limited demanded energy:      %.1f Wh\n', Edem_cmd_Wh);
fprintf('Baseline endurance: %.1f min, unmet kWh: %.4f\n', log_base.endurance_min, log_base.unmet_kWh);
fprintf('LMPC endurance:     %.1f min, unmet kWh: %.4f\n', log_lmpc.endurance_min, log_lmpc.unmet_kWh);
fprintf('NMPC endurance:     %.1f min, unmet kWh: %.4f\n', log_nmpc.endurance_min, log_nmpc.unmet_kWh);
fprintf('Baseline energy delivered: %.1f Wh, mean Pdel: %.2f W\n', Ebase_Wh, mean(log_base.Pdel));
fprintf('LMPC energy delivered:     %.1f Wh, mean Pdel: %.2f W\n', Elmpc_Wh, mean(log_lmpc.Pdel));
fprintf('NMPC energy delivered:     %.1f Wh, mean Pdel: %.2f W\n', Enmpc_Wh, mean(log_nmpc.Pdel));
fprintf('Baseline peak |I|: %.2f A, Irms: %.2f A, max T: %.2f C\n', max(abs(log_base.I)), Irms_base, max(log_base.T)-273.15);
fprintf('LMPC peak |I|:     %.2f A, Irms: %.2f A, max T: %.2f C\n', max(abs(log_lmpc.I)), Irms_lmpc, max(log_lmpc.T)-273.15);
fprintf('NMPC peak |I|:     %.2f A, Irms: %.2f A, max T: %.2f C\n', max(abs(log_nmpc.I)), Irms_nmpc, max(log_nmpc.T)-273.15);
fprintf('Mean battery demand (original): %.2f W\n', mean(Pdem_real));
fprintf('Mean battery demand (limited):  %.2f W\n', mean(Pdem_cmd));
if isfield(meta,'Pshaft_W')
    fprintf('Mean shaft power: %.2f W, mean eta: %.3f\n', mean(meta.Pshaft_W), mean(meta.eta_drv));
end
if isfield(log_lmpc,'qp_fail')
    fprintf('LMPC QP fail rate:  %.2f %%\n', 100*mean(log_lmpc.qp_fail));
end
if isfield(log_nmpc,'nlp_fail')
    fprintf('NMPC NLP fail rate: %.2f %%\n', 100*mean(log_nmpc.nlp_fail));
elseif isfield(log_nmpc,'qp_fail')
    fprintf('NMPC fail rate:     %.2f %%\n', 100*mean(log_nmpc.qp_fail));
end
cfgPub = struct('export_dir', fullfile(pwd,'fig_pub'), ...
                'export', true, ...
                'show_pred', true, ...
                'n_pred_snap', 8, ...
                'zoom_window_s', 220);

plot_uav_publication_figures(log_base, log_lmpc, log_nmpc, par, meta, cfgPub);
clear functions; rehash; type(which('mpc_build_qp_uav'))
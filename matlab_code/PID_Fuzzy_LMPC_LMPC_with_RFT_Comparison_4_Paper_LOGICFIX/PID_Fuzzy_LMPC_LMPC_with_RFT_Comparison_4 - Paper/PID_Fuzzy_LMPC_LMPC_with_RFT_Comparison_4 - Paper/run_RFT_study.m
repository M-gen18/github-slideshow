clear; clc; close all;
addpath(genpath(pwd));
ieee_plot_defaults();
ST = ctrl_styles();   

PLOT_INCLUDE_FUZZY = false;   % set false to drop fuzzy from ALL plots/tables
PLOT_ADD_ZOOM      = true;   % auto inset zoom where curves differ most
%% Settings
Ts = 1;
T_end_s = 50*60; % 50 minutes flight
RFT_hours_list = [0.85 1.0 1.5 2.0];   % stringency sweep
MC_runs = 10;                               % Monte Carlo                               % initial test of Monte Carlo

% Disturbances
wind_sigma_ms = 1.8;       % m/s correlated wind (headwind positive)
wind_tau_s    = 60;        % correlation time
Tamb_mean_C   = 25;
Tamb_amp_C    = 4;         % swing
Tamb_rw_C     = 0.25;       % random walk strength

% extra “late mission” stress
late_gust_start_s = 2700;  % last 25 min
late_gust_sigma_W = 6;
late_spike_prob   = 0.002;
late_spike_W      = [20 40];

% Phase-adaptive window
t_critical_start = 1500; % start reserve tightening earlier
t_critical_end   = 3000; % end at mission end
RFT_hi_hr = 3.0; % more conservative in critical window
RFT_lo_hr = 2.0; % more conservative nominal reserve

% RFT activity detection threshold
RFT_ACTIVE_TOL = 1e-3; %1e-3
RFT_VIOL_TOL   = -0.008; %(0.8% dip is the "real" violation)

%% Load demand
[t, Pdem_nom_W, meta] = load_uav_demand_profile(Ts, T_end_s, struct('use_drivetrain_map', true));
t = t(:); Pdem_nom_W = Pdem_nom_W(:);

figure('Name','Nominal demand');
plot(t,Pdem_nom_W,'k','LineWidth',1.2); grid on;
xlabel('$t~[\mathrm{s}]$'); ylabel('$P_{\mathrm{dem}}~[\mathrm{W}]$');
title('Nominal mission demand');

%% Battery
par = batt_params_2rc_uav_lipo();
x0  = batt_init_state(par);

%% Disturbances: wind + temperature (stochastic environment)
rng(1);
N = numel(t);

% Wind model (AR(1))
a_w = exp(-Ts/wind_tau_s);
w = zeros(N,1);
for k=2:N
    w(k) = a_w*w(k-1) + sqrt(1-a_w^2)*wind_sigma_ms*randn;
end
wind_head_ms = w;

% Vehicle speed profile
if isfield(meta,'V_ms') && numel(meta.V_ms)==N
    V = max(meta.V_ms(:), 8);
else
    V = 20*ones(N,1);
end

% Penalize headwind only
Veff = max(6, V + max(wind_head_ms,0));
pow_mult = (Veff./V).^3;
pow_mult = min(pow_mult, 1.20);

% Ambient temperature: sinusoid + random walk
Tamb_C = Tamb_mean_C + Tamb_amp_C*sin(2*pi*t/(2*3600));
rw = zeros(N,1);
for k=2:N
    rw(k) = rw(k-1) + Tamb_rw_C*0.02*randn;
end
Tamb_C = Tamb_C + rw;
Tamb_K = Tamb_C + 273.15;

Pdem_true_W = max(Pdem_nom_W .* pow_mult, 0);

figure('Name','Disturbances used in simulation');
yyaxis left;  plot(t, wind_head_ms,'LineWidth',1.2); ylabel('Headwind [m/s]'); grid on;
yyaxis right; plot(t, Tamb_C,'LineWidth',1.2); ylabel('$T_{\mathrm{amb}}~[^{\circ}\mathrm{C}]$');
xlabel('$t~[\mathrm{s}]$'); title('Wind + Temperature disturbances');

figure('Name','Nominal vs Wind-disturbed demand'); hold on; grid on;
plot(t, Pdem_nom_W,'k','LineWidth',1.2);
plot(t, Pdem_true_W,'k--','LineWidth',1.6);
xlabel('$t~[\mathrm{s}]$'); ylabel('$P_{\mathrm{dem}}~[\mathrm{W}]$');
title('Nominal demand vs Wind-disturbed demand');
legend('Nominal','With wind','Location','best');

%% ====== MPC config ======
cfg_pred = struct();
cfg_pred.enable_predlog = true;
cfg_pred.predlog_stride_s = 100;
cfg_pred.predlog_max = 8;
cfg_pred.no_anticipation = false;
cfg_pred.SoC_min_mpc_override = par.SoC_min;
cfg_pred.quiet = false;

mpc0 = mpc_settings_uav();
runMPC = @(Pdem_in, mpc_in, Tamb_in) simulate_uav_with_mpc(Pdem_in, t, Ts, par, x0, Tamb_in, mpc_in, cfg_pred);

%% ====== PID + Fuzzy config ======
pid = pid_settings_uav();
fz  = fuzzy_settings_uav();
runPID   = @(Pdem_in, Tamb_in) simulate_uav_with_pid(Pdem_in, t, Ts, par, x0, Tamb_in, pid);
runFuzzy = @(Pdem_in, Tamb_in) simulate_uav_with_fuzzy(Pdem_in, t, Ts, par, x0, Tamb_in, fz);

%% ==========================================================
% 1) Single disturbed mission 
%% ==========================================================
log_pid   = runPID(Pdem_true_W, Tamb_K);

if PLOT_INCLUDE_FUZZY
    log_fuzzy = runFuzzy(Pdem_true_W, Tamb_K);
else
    log_fuzzy = [];
end

mpc_no = mpc0; mpc_no.use_rft_constraint = false;
[log_lmpc, ~] = runMPC(Pdem_true_W, mpc_no, Tamb_K);

mpc_r = mpc0; mpc_r.use_rft_constraint = true;
mpc_r.RFT_spec_s = 2.5*3600;
[log_rft, ~] = runMPC(Pdem_true_W, mpc_r, Tamb_K);

% ---- Plots: robust to unequal lengths (controllers stop early) ----
figure('Name','SoC comparison'); clf;
plotNctrl({log_pid, log_fuzzy, log_lmpc, log_rft}, {'PID','Fuzzy','LMPC','LMPC+RFT'}, ...
    'SoC', '$\mathrm{SoC}$', 'SoC comparison (same disturbed mission)', ST, PLOT_INCLUDE_FUZZY);
if PLOT_ADD_ZOOM, add_zoom_insetN({log_pid, log_fuzzy, log_lmpc, log_rft}, 'SoC', PLOT_INCLUDE_FUZZY); end

figure('Name','Terminal voltage comparison'); clf;
plotNctrl({log_pid, log_fuzzy, log_lmpc, log_rft}, {'PID','Fuzzy','LMPC','LMPC+RFT'}, ...
    'Vt', '$V_t~[\mathrm{V}]$', 'Terminal voltage (same disturbed mission)', ST, PLOT_INCLUDE_FUZZY);
if PLOT_ADD_ZOOM, add_zoom_insetN({log_pid, log_fuzzy, log_lmpc, log_rft}, 'Vt', PLOT_INCLUDE_FUZZY); end

figure('Name','Tracking trade-off'); clf;
plotNctrl({log_pid, log_fuzzy, log_lmpc, log_rft}, {'PID','Fuzzy','LMPC','LMPC+RFT'}, ...
    'Punmet', '$P_{\mathrm{unmet}}~[\mathrm{W}]$', 'Tracking trade-off (same disturbed mission)', ST, PLOT_INCLUDE_FUZZY);
set(gca,'YLim',[0, max(1, 1.05*max(get_unmet_series(log_rft)))]);
if PLOT_ADD_ZOOM, add_zoom_insetN({log_pid, log_fuzzy, log_lmpc, log_rft}, 'Punmet', PLOT_INCLUDE_FUZZY); end

if isstruct(log_rft) && isfield(log_rft,'RFT_margin') && ~isempty(log_rft.RFT_margin)
    figure('Name','RFT margin (active near 0)'); grid on; hold on;
    plot(log_rft.t, log_rft.RFT_margin, 'k','LineWidth',1.6);
    yline(0,'k--','LineWidth',1.2);
    xlabel('$t~[\mathrm{s}]$');
    ylabel('$m=\mathrm{SoC}_{\mathrm{term,pred}}-\mathrm{RHS}$');
    title('RFT margin (binding near 0)');
end

%% Single-mission summary table
S_pid   = summarize_log_uav(log_pid,   par, 'RFT_ACTIVE_TOL',RFT_ACTIVE_TOL, 't_critical_start',t_critical_start, 't_critical_end',t_critical_end);
S_lmpc  = summarize_log_uav(log_lmpc,  par, 'RFT_ACTIVE_TOL',RFT_ACTIVE_TOL, 't_critical_start',t_critical_start, 't_critical_end',t_critical_end);
S_rft   = summarize_log_uav(log_rft,   par, 'RFT_ACTIVE_TOL',RFT_ACTIVE_TOL, 't_critical_start',t_critical_start, 't_critical_end',t_critical_end);

if PLOT_INCLUDE_FUZZY
    S_fuzzy = summarize_log_uav(log_fuzzy, par, 'RFT_ACTIVE_TOL',RFT_ACTIVE_TOL, 't_critical_start',t_critical_start, 't_critical_end',t_critical_end);
    Tsingle = struct2table([S_pid; S_fuzzy; S_lmpc; S_rft], 'AsArray', true);
    Tsingle.Controller = {'PID';'Fuzzy';'LMPC';'LMPC+RFT'};
else
    Tsingle = struct2table([S_pid; S_lmpc; S_rft], 'AsArray', true);
    Tsingle.Controller = {'PID';'LMPC';'LMPC+RFT'};
end
Tsingle = movevars(Tsingle,'Controller','Before',1);
disp('Single mission summary');
disp(Tsingle);

%% ==========================================================
% 2) Stringency sweep 
%% ==========================================================
sweep_end  = nan(numel(RFT_hours_list),1);
sweep_unm  = nan(numel(RFT_hours_list),1);
sweep_bind = nan(numel(RFT_hours_list),1);
Tracking_RMSE_Sweep = nan(numel(RFT_hours_list),1);

for i=1:numel(RFT_hours_list)
    rft_hr = RFT_hours_list(i);
    mpc_s = mpc0;
    mpc_s.use_rft_constraint = true;
    mpc_s.RFT_spec_s = rft_hr*3600;

    [log_s, ~] = runMPC(Pdem_true_W, mpc_s, Tamb_K);

    sweep_end(i) = log_s.endurance_min;
    sweep_unm(i) = log_s.unmet_kWh;

    punmet_s = get_unmet_series(log_s);
    Tracking_RMSE_Sweep(i) = sqrt(mean(punmet_s.^2));

    if isfield(log_s,'RFT_margin') && ~isempty(log_s.RFT_margin)
        sweep_bind(i) = 100*mean(abs(log_s.RFT_margin(:)) <= RFT_ACTIVE_TOL);
    else
        sweep_bind(i) = NaN;
    end
end

figure('Name','Endurance vs RFT stringency'); grid on;
plot(RFT_hours_list, sweep_end, '-o', 'LineWidth',1.6);
xlabel('RFT requirement [h]'); ylabel('Endurance [min]');
title('Endurance vs RFT stringency (disturbed mission)');

figure('Name','Unmet energy vs RFT stringency'); grid on;
plot(RFT_hours_list, sweep_unm, '-o', 'LineWidth',1.6);
xlabel('RFT requirement [h]'); ylabel('Unmet energy [kWh]');
title('Unmet energy vs RFT stringency (disturbed mission)');

figure('Name','When does RFT become active?'); grid on;
plot(RFT_hours_list, sweep_bind, '-o', 'LineWidth',1.6);
xlabel('RFT requirement [h]');
ylabel('RFT active time [%]','Interpreter','none'); % avoid latex warning
title('When does RFT become active?');

%% ==========================================================
% 3) Phase-adaptive RFT
%% ==========================================================
mpc_pa = mpc0;
mpc_pa.use_rft_constraint = true;
mpc_pa.RFT_schedule = struct('t1',t_critical_start,'t2',t_critical_end,...
                             'RFT_hi_s',RFT_hi_hr*3600,'RFT_lo_s',RFT_lo_hr*3600);

[log_phase, ~] = runMPC(Pdem_true_W, mpc_pa, Tamb_K);

figure('Name','Constant RFT vs Phase-adaptive RFT'); hold on; grid on;
plot(log_rft.t,   log_rft.SoC,   'k','LineWidth',1.6);
plot(log_phase.t, log_phase.SoC, '--','LineWidth',1.6);
xline(t_critical_start,'k--'); xline(t_critical_end,'k--');
xlabel('$t~[\mathrm{s}]$'); ylabel('$\mathrm{SoC}$');
title('Constant RFT vs Phase-adaptive RFT (disturbed mission)');
legend('Constant RFT','Phase-adaptive','critical window','Location','best');

%% ==========================================================
% 4) Monte Carlo + unified safety metric
%% ==========================================================
rng(2);

mc_RMSE_pid   = nan(MC_runs,1);
mc_RMSE_fuzzy = nan(MC_runs,1);
mc_RMSE_no    = nan(MC_runs,1);
mc_RMSE_rft   = nan(MC_runs,1);

mc_viol_pid   = nan(MC_runs,1);
mc_viol_fuzzy = nan(MC_runs,1);
mc_viol_no    = nan(MC_runs,1);
mc_viol_rft   = nan(MC_runs,1);

mc_end_no  = nan(MC_runs,1);  mc_unm_no  = nan(MC_runs,1);
mc_end_rft = nan(MC_runs,1);  mc_unm_rft = nan(MC_runs,1);
mc_end_pid = nan(MC_runs,1);  mc_unm_pid = nan(MC_runs,1);
mc_end_fuzzy = nan(MC_runs,1); mc_unm_fuzzy = nan(MC_runs,1);

mc_margin_rft = cell(MC_runs,1);
mc_t_rft      = cell(MC_runs,1);

Treq_s = 2*3600; % reserve requirement used for ALL controllers (2 hours)

for r=1:MC_runs
    fprintf('Starting Monte Carlo Run %d of %d... (Time: %s)\n', r, MC_runs, datestr(now,'HH:MM:SS'));

    % wind
    w = zeros(N,1);
    for k=2:N
        w(k) = a_w*w(k-1) + sqrt(1-a_w^2)*wind_sigma_ms*randn;
    end
    Veff = max(6, V + max(w,0));
    pow_mult_r = min((Veff./V).^3, 1.35);

    % temp
    Tamb_C_r = Tamb_mean_C + Tamb_amp_C*sin(2*pi*t/(2*3600));
    rw = zeros(N,1);
    for k=2:N
        rw(k) = rw(k-1) + Tamb_rw_C*0.02*randn;
    end
    Tamb_C_r = Tamb_C_r + rw;
    Tamb_K_r = Tamb_C_r + 273.15;

    % demand
    Preal_W = max(Pdem_nom_W .* pow_mult_r, 0);

    % late stress
    idxL = (t >= late_gust_start_s);
    e = randn(sum(idxL),1);
    if std(e)>0, e = e/std(e); end
    Preal_W(idxL) = max(Preal_W(idxL) + late_gust_sigma_W*e, 0);

    spk = (rand(sum(idxL),1) < late_spike_prob);
    spk_add = (late_spike_W(1) + (late_spike_W(2)-late_spike_W(1))*rand(sum(spk),1));
    idx_first = find(idxL,1,'first');
    Preal_W(idx_first-1 + find(spk)) = Preal_W(idx_first-1 + find(spk)) + spk_add;

    % LMPC
    mpc_no = mpc0; mpc_no.use_rft_constraint = false;
    [log_no, ~] = simulate_uav_with_mpc(Preal_W, t, Ts, par, x0, Tamb_K_r, mpc_no, cfg_pred);
    mc_RMSE_no(r) = sqrt(mean(get_unmet_series(log_no).^2));
    mc_end_no(r)  = log_no.endurance_min;
    mc_unm_no(r)  = log_no.unmet_kWh;

    % PID
    log_pid_r = simulate_uav_with_pid(Preal_W, t, Ts, par, x0, Tamb_K_r, pid);
    mc_RMSE_pid(r) = sqrt(mean(get_unmet_series(log_pid_r).^2));
    mc_end_pid(r)  = log_pid_r.endurance_min;
    mc_unm_pid(r)  = log_pid_r.unmet_kWh;

    % Fuzzy (optional)
    if PLOT_INCLUDE_FUZZY
        log_fz_r = simulate_uav_with_fuzzy(Preal_W, t, Ts, par, x0, Tamb_K_r, fz);
        mc_RMSE_fuzzy(r) = sqrt(mean(get_unmet_series(log_fz_r).^2));
        mc_end_fuzzy(r)  = log_fz_r.endurance_min;
        mc_unm_fuzzy(r)  = log_fz_r.unmet_kWh;
    else
        log_fz_r = [];
        mc_RMSE_fuzzy(r) = NaN;
        mc_end_fuzzy(r)  = NaN;
        mc_unm_fuzzy(r)  = NaN;
    end

    % LMPC + RFT
    mpc_r = mpc0; mpc_r.use_rft_constraint = true; mpc_r.RFT_spec_s = Treq_s;
    [log_r, ~] = simulate_uav_with_mpc(Preal_W, t, Ts, par, x0, Tamb_K_r, mpc_r, cfg_pred);
    mc_RMSE_rft(r) = sqrt(mean(get_unmet_series(log_r).^2));
    mc_end_rft(r)  = log_r.endurance_min;
    mc_unm_rft(r)  = log_r.unmet_kWh;
    % ==========================================================
    m_pid   = virtual_rft_margin_horizon(log_pid_r, par, Treq_s, mpc0.N, Ts);
    m_no    = virtual_rft_margin_horizon(log_no,    par, Treq_s, mpc0.N, Ts);
    if PLOT_INCLUDE_FUZZY
        m_fuzzy = virtual_rft_margin_horizon(log_fz_r, par, Treq_s, mpc0.N, Ts);
    else
        m_fuzzy = [];
    end

    mc_viol_pid(r) = any(m_pid < RFT_VIOL_TOL, 'all');
    mc_viol_no(r)  = any(m_no  < RFT_VIOL_TOL, 'all');
    if PLOT_INCLUDE_FUZZY
        mc_viol_fuzzy(r) = any(m_fuzzy < RFT_VIOL_TOL, 'all');
    else
        mc_viol_fuzzy(r) = NaN;
    end

    % For LMPC+RFT: use ACTUAL MPC margin if available, else virtual
    if isfield(log_r,'RFT_margin') && ~isempty(log_r.RFT_margin)
        mc_viol_rft(r) = any(log_r.RFT_margin(:) < RFT_VIOL_TOL);
        mc_margin_rft{r} = log_r.RFT_margin(:);
        mc_t_rft{r}      = log_r.t(:);
    else
        m_rft = virtual_rft_margin_horizon(log_r, par, Treq_s, mpc0.N, Ts);
        mc_viol_rft(r) = any(m_rft < RFT_VIOL_TOL, 'all');
        mc_margin_rft{r} = [];
        mc_t_rft{r}      = [];
    end
end

% Statistical report (LMPC+RFT RMSE)
mean_RMSE = mean(mc_RMSE_rft,'omitnan');
std_RMSE  = std(mc_RMSE_rft,'omitnan');

fprintf('\n==================================================\n');
fprintf('STATISTICAL QUALITY REPORT (LMPC+RFT)\n');
fprintf('==================================================\n');
fprintf('Tracking RMSE (Mean): %.4f W\n', mean_RMSE);
fprintf('Tracking RMSE (Std):  %.4f W\n', std_RMSE);
fprintf('--------------------------------------------------\n');

RMSE_Results = table(RFT_hours_list(:), Tracking_RMSE_Sweep(:), ...
    'VariableNames', {'RFT_Requirement_Hours', 'RMSE_Watts'});
disp('--- RMSE across different Safety Levels ---');
disp(RMSE_Results);

%% ===== Monte Carlo plots =====
figure('Name','Monte Carlo endurance'); hold on; grid on;
if PLOT_INCLUDE_FUZZY
    allEnd = [mc_end_pid; mc_end_fuzzy; mc_end_no; mc_end_rft];
else
    allEnd = [mc_end_pid; mc_end_no; mc_end_rft];
end
edgesE = linspace(floor(min(allEnd)), ceil(max(allEnd)), 10);
histogram(mc_end_pid,   edgesE, 'DisplayStyle','bar', 'FaceAlpha',0.35);
if PLOT_INCLUDE_FUZZY, histogram(mc_end_fuzzy, edgesE, 'DisplayStyle','bar', 'FaceAlpha',0.35); end
histogram(mc_end_no,    edgesE, 'DisplayStyle','bar', 'FaceAlpha',0.35);
histogram(mc_end_rft,   edgesE, 'DisplayStyle','bar', 'FaceAlpha',0.35);
xlabel('Endurance [min]'); ylabel('Count');
title(sprintf('Monte Carlo endurance (N=%d) - wind+temp+late stress',MC_runs));
if PLOT_INCLUDE_FUZZY
    legend('PID','Fuzzy','LMPC','LMPC+RFT(3h)','Location','best');
else
    legend('PID','LMPC','LMPC+RFT(3h)','Location','best');
end

figure('Name','Monte Carlo unmet energy'); hold on; grid on;
if PLOT_INCLUDE_FUZZY
    allU = [mc_unm_pid; mc_unm_fuzzy; mc_unm_no; mc_unm_rft];
else
    allU = [mc_unm_pid; mc_unm_no; mc_unm_rft];
end
edgesU = linspace(min(allU), max(allU), 12);
histogram(mc_unm_pid,   edgesU, 'FaceAlpha',0.35);
if PLOT_INCLUDE_FUZZY, histogram(mc_unm_fuzzy, edgesU, 'FaceAlpha',0.35); end
histogram(mc_unm_no,    edgesU, 'FaceAlpha',0.35);
histogram(mc_unm_rft,   edgesU, 'FaceAlpha',0.35);
set(gca,'XScale','log');
xlabel('Unmet energy [kWh] (log scale)'); ylabel('Count');
title(sprintf('Monte Carlo unmet energy (N=%d) - wind+temp+late stress',MC_runs));
if PLOT_INCLUDE_FUZZY
    legend('PID','Fuzzy','LMPC','LMPC+RFT(3h)','Location','best');
else
    legend('PID','LMPC','LMPC+RFT(3h)','Location','best');
end

%% ===== Monte Carlo summary table =====
MC = table();
if PLOT_INCLUDE_FUZZY
    MC.Controller = {'PID';'Fuzzy';'LMPC';'LMPC+RFT'};
    MC.Endurance_mean = [mean(mc_end_pid,'omitnan'); mean(mc_end_fuzzy,'omitnan'); mean(mc_end_no,'omitnan'); mean(mc_end_rft,'omitnan')];
    MC.Endurance_std  = [std(mc_end_pid,'omitnan');  std(mc_end_fuzzy,'omitnan');  std(mc_end_no,'omitnan');  std(mc_end_rft,'omitnan')];
    MC.RMSE_mean      = [mean(mc_RMSE_pid,'omitnan'); mean(mc_RMSE_fuzzy,'omitnan'); mean(mc_RMSE_no,'omitnan'); mean(mc_RMSE_rft,'omitnan')];
    MC.RMSE_std       = [std(mc_RMSE_pid,'omitnan');  std(mc_RMSE_fuzzy,'omitnan');  std(mc_RMSE_no,'omitnan');  std(mc_RMSE_rft,'omitnan')];
    MC.Safety_Violation_Pct = 100 * [mean(mc_viol_pid,'omitnan'); mean(mc_viol_fuzzy,'omitnan'); mean(mc_viol_no,'omitnan'); mean(mc_viol_rft,'omitnan')];
else
    MC.Controller = {'PID';'LMPC';'LMPC+RFT'};
    MC.Endurance_mean = [mean(mc_end_pid,'omitnan'); mean(mc_end_no,'omitnan'); mean(mc_end_rft,'omitnan')];
    MC.Endurance_std  = [std(mc_end_pid,'omitnan');  std(mc_end_no,'omitnan');  std(mc_end_rft,'omitnan')];
    MC.RMSE_mean      = [mean(mc_RMSE_pid,'omitnan'); mean(mc_RMSE_no,'omitnan'); mean(mc_RMSE_rft,'omitnan')];
    MC.RMSE_std       = [std(mc_RMSE_pid,'omitnan');  std(mc_RMSE_no,'omitnan');  std(mc_RMSE_rft,'omitnan')];
    MC.Safety_Violation_Pct = 100 * [mean(mc_viol_pid,'omitnan'); mean(mc_viol_no,'omitnan'); mean(mc_viol_rft,'omitnan')];
end

disp('=== Final Monte Carlo Performance & Safety Summary ===');
disp(MC);

%% ===== RFT margin distribution (Monte Carlo) =====
allMargin = [];
for r=1:MC_runs
    if ~isempty(mc_margin_rft{r})
        allMargin = [allMargin; mc_margin_rft{r}]; %#ok<AGROW>
    end
end
if ~isempty(allMargin)
    figure('Name','RFT margin distribution across Monte Carlo'); grid on; hold on;
    histogram(allMargin, 40);
    xline(0,'k--','LineWidth',1.2);
    xlabel('$m=\mathrm{SoC}_{\mathrm{term,pred}}-\mathrm{RHS}$','Interpreter','latex');
    ylabel('Count');
    title(sprintf('RFT margin distribution across Monte Carlo (N=%d)',MC_runs));
end

disp('DONE: All scenarios executed.');

%% ===== Survival Results =====
fprintf('\n========================================\n');
fprintf('     MISSION SURVIVAL ANALYSIS\n');
fprintf('========================================\n');
fprintf('Planned Mission Duration:  %.2f min\n', t(end)/60);
fprintf('----------------------------------------\n');
fprintf('PID Survival:              %.2f min\n', (numel(log_pid.Vt)*Ts)/60);
if PLOT_INCLUDE_FUZZY
    fprintf('Fuzzy Survival:            %.2f min\n', (numel(log_fuzzy.Vt)*Ts)/60);
end
fprintf('Standard LMPC Survival:    %.2f min\n', (numel(log_lmpc.Vt)*Ts)/60);
fprintf('LMPC + RFT Survival:  %.2f min\n', (numel(log_rft.Vt)*Ts)/60);
fprintf('Phase-Adaptive Survival:   %.2f min\n', (numel(log_phase.Vt)*Ts)/60);
fprintf('========================================\n');

%% ===== helper: robust unmet series =====
function Pun = get_unmet_series(log)
if isempty(log), Pun = []; return; end
if isfield(log,'Punmet') && ~isempty(log.Punmet)
    Pun = log.Punmet(:);
    return;
end
if isfield(log,'Pdem') && isfield(log,'Pdel')
    Pun = max(log.Pdem(:) - log.Pdel(:), 0);
else
    Pun = zeros(numel(log.t),1);
end
end

function m = virtual_rft_margin_horizon(log, par, Treq_s, N, Ts)

if isempty(log) || ~isfield(log,'SoC') || ~isfield(log,'t')
    m = NaN; return;
end

SoC = log.SoC(:);
tt  = log.t(:);
K   = numel(tt);

m = NaN(K,1);
if K < N+1, return; end

for k=1:(K-N)
    RFT_rem = max(Treq_s - tt(k), Ts);
    alpha = (N*Ts) / RFT_rem;
    alpha = min(max(alpha, 0), 1); % safety
    RHS = SoC(k) + (par.SoC_min - SoC(k))*alpha;

    SoC_term = SoC(k+N);
    m(k) = SoC_term - RHS;
end
end

function ieee_plot_defaults()
set(groot,'defaultFigureColor','w');
set(groot,'defaultAxesFontName','Times New Roman');
set(groot,'defaultTextFontName','Times New Roman');
set(groot,'defaultAxesFontSize',10);
set(groot,'defaultTextFontSize',10);
set(groot,'defaultLegendInterpreter','latex');
set(groot,'defaultTextInterpreter','latex');
set(groot,'defaultAxesTickLabelInterpreter','latex');
set(groot,'defaultAxesXGrid','on');
set(groot,'defaultAxesYGrid','on');
set(groot,'defaultAxesGridAlpha',0.20);
end

function ST = ctrl_styles()
    % PID: Solid Blue
    ST.PID.Color   = [0 0 1]; 
    ST.PID.LS      = '-'; 
    ST.PID.LW      = 1.5; 
    ST.PID.M       = 'none';

    % Standard LMPC: Dashed Black
    ST.LMPC.Color  = [0 0 0]; 
    ST.LMPC.LS     = '--'; 
    ST.LMPC.LW     = 1.5; 
    ST.LMPC.M      = 'none';

    % LMPC + RFT: Solid Red
    ST.RFT.Color   = [1 0 0]; 
    ST.RFT.LS      = '-'; 
    ST.RFT.LW      = 2.0; 
    ST.RFT.M       = 'none';

    % Fuzzy (if needed)
    ST.Fuzzy.Color = [0.5 0.5 0.5]; ST.Fuzzy.LS = ':'; ST.Fuzzy.LW = 1.2; ST.Fuzzy.M = 'none';
end

function plotNctrl(logs, names, field, yLab, ttl, ST, includeFuzzy)
    hold on; box on; grid on;
    idx = [1 3 4]; 
    if includeFuzzy, idx = [1 2 3 4]; end
    
    h_list = []; % Store handles for legend
    n_list = {}; % Store names for legend

    for ii = idx
        lg = logs{ii};
        if isempty(lg) || ~isfield(lg,field), continue; end
        
        tt = lg.t(:);
        yy = lg.(field)(:);
        
        % Assign style based on controller name
        switch names{ii}
            case 'PID'
                h = plot(tt, yy, 'Color', ST.PID.Color, 'LineStyle', ST.PID.LS, 'LineWidth', ST.PID.LW);
            case 'LMPC'
                h = plot(tt, yy, 'Color', ST.LMPC.Color, 'LineStyle', ST.LMPC.LS, 'LineWidth', ST.LMPC.LW);
            case 'LMPC+RFT'
                h = plot(tt, yy, 'Color', ST.RFT.Color, 'LineStyle', ST.RFT.LS, 'LineWidth', ST.RFT.LW);
                uistack(h, 'top');
            case 'Fuzzy'
                h = plot(tt, yy, 'Color', ST.Fuzzy.Color, 'LineStyle', ST.Fuzzy.LS, 'LineWidth', ST.Fuzzy.LW);
        end
        h_list = [h_list, h];
        n_list{end+1} = names{ii};
    end
    
    xlabel('$t~[\mathrm{s}]$'); ylabel(yLab); title(ttl);
    if ~isempty(h_list)
        legend(h_list, n_list, 'Location', 'best');
    end
end

function add_zoom_insetN(logs, field, includeFuzzy)

idx = [1 3 4];
if includeFuzzy, idx = [1 2 3 4]; end

tmax = 0;
for ii=idx
    lg = logs{ii};
    if ~isempty(lg) && isfield(lg,'t') && ~isempty(lg.t)
        tmax = max(tmax, lg.t(end));
    end
end
if tmax <= 0, return; end

tgrid = (0:1:ceil(tmax))';
Y = nan(numel(tgrid), numel(idx));
for j=1:numel(idx)
    lg = logs{idx(j)};
    if isempty(lg) || ~isfield(lg,field), continue; end
    Y(:,j) = interp1(lg.t(:), lg.(field)(:), tgrid, 'linear', NaN);
end

spread = max(Y,[],2,'omitnan') - min(Y,[],2,'omitnan');
[~,k] = max(spread);
if ~isfinite(k), return; end

k1 = max(1, k-round(numel(tgrid)*0.04));
k2 = min(numel(tgrid), k+round(numel(tgrid)*0.04));

figure('Name',['Zoomed Inset: ', field]); 
hold on; box on; grid on;

for j=1:numel(idx)
    lg = logs{idx(j)};
    if isempty(lg) || ~isfield(lg,field), continue; end
    tt = lg.t(:); yy = lg.(field)(:);
    plot(tt, yy, 'LineWidth',1.5); 
end

xlim([tgrid(k1) tgrid(k2)]);
xlabel('$t~[\mathrm{s}]$');
ylabel(field, 'Interpreter','none');
title(['Zoomed Details for ', field]);
end
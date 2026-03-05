clear; clc; close all;
addpath(genpath(pwd));

plant = uav_plant_lib();
mpclib = uav_mpc_lib();

%% ====== Settings ======
Ts = 1; T_end_s = 2*3600;
RFT_hours_list = [2.0 2.5 3.0 3.5 4.0];
MC_runs = 200;

wind_sigma_ms = 3.0; wind_tau_s = 60;
Tamb_mean_C = 25; Tamb_amp_C = 6; Tamb_rw_C = 0.6;

late_gust_start_s = 2700;
late_gust_sigma_W = 14;
late_spike_prob   = 0.006;
late_spike_W      = [35 70];

t_critical_start = 1200; t_critical_end = 1800;
RFT_hi_hr = 3.0; RFT_lo_hr = 1.5;
RFT_ACTIVE_TOL = 5e-3;

%% ====== Load demand ======
[t, Pdem_nom_W, meta] = plant.load_demand(Ts, T_end_s, struct('use_drivetrain_map', true));

t = t(:); Pdem_nom_W = Pdem_nom_W(:);
figure('Name','Nominal demand'); plot(t,Pdem_nom_W,'k','LineWidth',1.2); grid on;
xlabel('t [s]'); ylabel('P_{dem} [W]'); title('Nominal mission demand');

%% ====== Battery ======
par = plant.batt_params();
x0  = plant.batt_init(par);
x0.SoC = 0.88; x0.SoH = 1.00; x0.T = (Tamb_mean_C+273.15);

%% ====== Disturbances ======
rng(1);
N = numel(t);
a_w = exp(-Ts/wind_tau_s);
w = zeros(N,1);
for k=2:N
    w(k) = a_w*w(k-1) + sqrt(1-a_w^2)*wind_sigma_ms*randn;
end
wind_head_ms = w;

if isfield(meta,'V_ms') && numel(meta.V_ms)==N
    V = max(meta.V_ms(:), 8);
else
    V = 20*ones(N,1);
end

Veff = max(6, V + max(wind_head_ms,0));
pow_mult = min((Veff./V).^3, 1.35);

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
yyaxis right; plot(t, Tamb_C,'LineWidth',1.2); ylabel('T_{amb} [C]');
xlabel('t [s]'); title('Wind + Temperature disturbances');

figure('Name','Nominal vs Wind-disturbed demand'); hold on; grid on;
plot(t, Pdem_nom_W,'k','LineWidth',1.2);
plot(t, Pdem_true_W,'k--','LineWidth',1.6);
xlabel('t [s]'); ylabel('P_{dem} [W]');
title('Nominal demand vs Wind-disturbed demand');
legend('Nominal','With wind','Location','best');

%% ====== Controller configs ======
mpc0 = mpclib.mpc_settings();
mpc0.Ts = Ts;
mpc0.qP  = 2500; mpc0.qdI = 3;

cfg_pred = struct('enable_predlog',true,'predlog_stride_s',600,'predlog_max',8, ...
    'no_anticipation',false,'SoC_min_mpc_override',par.SoC_min,'quiet',false);

pid = plant.pid_settings();
fz  = plant.fuzzy_settings();

runPID   = @(Pdem,Tamb) plant.sim_pid(Pdem, t, Ts, par, x0, Tamb, pid);
runFuzzy = @(Pdem,Tamb) plant.sim_fuzzy(Pdem, t, Ts, par, x0, Tamb, fz);
runMPC   = @(Pdem,mpc,Tamb) mpclib.sim_mpc(Pdem, t, Ts, par, x0, Tamb, mpc, cfg_pred);

%% ==========================================================
% 1) Same disturbed mission (4 controllers)
%% ==========================================================
log_pid   = runPID(Pdem_true_W, Tamb_K);
log_fuzzy = runFuzzy(Pdem_true_W, Tamb_K);

mpc_no = mpc0; mpc_no.use_rft_constraint = false;
[log_lmpc, ~] = runMPC(Pdem_true_W, mpc_no, Tamb_K);

mpc_r = mpc0; mpc_r.use_rft_constraint = true; mpc_r.RFT_spec_s = 3.0*3600;
[log_rft, ~] = runMPC(Pdem_true_W, mpc_r, Tamb_K);

figure('Name','SoC comparison'); hold on; grid on;
plot(log_pid.t,   log_pid.SoC,   'LineWidth',1.6);
plot(log_fuzzy.t, log_fuzzy.SoC, 'LineWidth',1.6);
plot(log_lmpc.t,  log_lmpc.SoC,  'LineWidth',1.6);
plot(log_rft.t,   log_rft.SoC,   'LineWidth',1.6);
xlabel('t [s]'); ylabel('SoC');
title('SoC comparison (same disturbed mission)');
legend('PID','Fuzzy','LMPC','LMPC+RFT','Location','best');

figure('Name','Terminal voltage comparison'); hold on; grid on;
plot(log_pid.t,   log_pid.Vt,   'LineWidth',1.4);
plot(log_fuzzy.t, log_fuzzy.Vt, 'LineWidth',1.4);
plot(log_lmpc.t,  log_lmpc.Vt,  'LineWidth',1.4);
plot(log_rft.t,   log_rft.Vt,   'LineWidth',1.4);
xlabel('t [s]'); ylabel('V_t [V]');
title('Terminal voltage (same disturbed mission)');
legend('PID','Fuzzy','LMPC','LMPC+RFT','Location','best');

figure('Name','Tracking trade-off'); hold on; grid on;
plot(log_pid.t,   plant.get_unmet(log_pid),   'LineWidth',1.2);
plot(log_fuzzy.t, plant.get_unmet(log_fuzzy), 'LineWidth',1.2);
plot(log_lmpc.t,  plant.get_unmet(log_lmpc),  'LineWidth',1.2);
plot(log_rft.t,   plant.get_unmet(log_rft),   'LineWidth',1.2);
xlabel('t [s]'); ylabel('P_{unmet} [W]');
title('Tracking trade-off (same disturbed mission)');
legend('PID','Fuzzy','LMPC','LMPC+RFT','Location','best');

if isfield(log_rft,'RFT_slack') && ~isempty(log_rft.RFT_slack)
    figure('Name','RFT constraint slack (\approx 0 active)'); grid on;
    plot(log_rft.t, log_rft.RFT_slack, 'LineWidth',1.5);
    xlabel('t [s]'); ylabel('RHS - SoC_{term,pred}');
    title('RFT constraint slack (\approx 0 active)');
end

S_pid   = plant.summarize(log_pid,   par, RFT_ACTIVE_TOL, t_critical_start, t_critical_end);
S_fuzzy = plant.summarize(log_fuzzy, par, RFT_ACTIVE_TOL, t_critical_start, t_critical_end);
S_lmpc  = plant.summarize(log_lmpc,  par, RFT_ACTIVE_TOL, t_critical_start, t_critical_end);
S_rft   = plant.summarize(log_rft,   par, RFT_ACTIVE_TOL, t_critical_start, t_critical_end);

Tsingle = struct2table([S_pid; S_fuzzy; S_lmpc; S_rft], 'AsArray', true);
Tsingle.Controller = {'PID';'Fuzzy';'LMPC';'LMPC+RFT'};
Tsingle = movevars(Tsingle,'Controller','Before',1);

disp('=== Single mission summary (same disturbed mission) ===');
disp(Tsingle);

%% ==========================================================
% 2) Stringency sweep (2h->4h)
%% ==========================================================
sweep_end  = nan(numel(RFT_hours_list),1);
sweep_unm  = nan(numel(RFT_hours_list),1);
sweep_bind = nan(numel(RFT_hours_list),1);

for i=1:numel(RFT_hours_list)
    rft_hr = RFT_hours_list(i);
    mpc_s = mpc0; mpc_s.use_rft_constraint = true; mpc_s.RFT_spec_s = rft_hr*3600;
    [log_s, ~] = runMPC(Pdem_true_W, mpc_s, Tamb_K);
    sweep_end(i) = log_s.endurance_min;
    sweep_unm(i) = log_s.unmet_kWh;
    if isfield(log_s,'RFT_slack') && ~isempty(log_s.RFT_slack)
RFT_ACTIVE_TOL = 1e-4;
sweep_bind(i) = 100*mean(abs(log_s.RFT_slack(:)) <= RFT_ACTIVE_TOL);    else
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
xlabel('RFT requirement [h]'); ylabel('RFT active time [%]');
title('When does RFT become active?');

%% ==========================================================
% 3) Hand-chosen phase-adaptive RFT
%% ==========================================================
mpc_pa = mpc0; mpc_pa.use_rft_constraint = true;
mpc_pa.RFT_schedule = struct('t1',t_critical_start,'t2',t_critical_end,...
    'RFT_hi_s',RFT_hi_hr*3600,'RFT_lo_s',RFT_lo_hr*3600);

[log_phase, ~] = runMPC(Pdem_true_W, mpc_pa, Tamb_K);

figure('Name','Constant RFT vs Phase-adaptive RFT'); hold on; grid on;
plot(log_rft.t,   log_rft.SoC,   'LineWidth',1.5);
plot(log_phase.t, log_phase.SoC, 'LineWidth',1.5);
xline(t_critical_start,'--'); xline(t_critical_end,'--');
xlabel('t [s]'); ylabel('SoC');
title('Constant RFT vs Phase-adaptive RFT (disturbed mission)');
legend('Constant RFT','Phase-adaptive','Location','best');

%% ==========================================================
% 4) Monte Carlo + slack analysis
%% ==========================================================
rng(2);
mc_end_no  = nan(MC_runs,1); mc_unm_no  = nan(MC_runs,1);
mc_end_rft = nan(MC_runs,1); mc_unm_rft = nan(MC_runs,1);
mc_end_pid = nan(MC_runs,1); mc_unm_pid = nan(MC_runs,1);
mc_end_fz  = nan(MC_runs,1); mc_unm_fz  = nan(MC_runs,1);

mc_slack_rft = cell(MC_runs,1);
mc_t_rft     = cell(MC_runs,1);

for r=1:MC_runs
    % correlated wind
    w = zeros(N,1);
    for k=2:N
        w(k) = a_w*w(k-1) + sqrt(1-a_w^2)*wind_sigma_ms*randn;
    end
    Veff = max(6, V + max(w,0));
    pow_mult_r = min((Veff./V).^3, 1.35);

    % temperature
    Tamb_C_r = Tamb_mean_C + Tamb_amp_C*sin(2*pi*t/(2*3600));
    rw = zeros(N,1);
    for k=2:N
        rw(k) = rw(k-1) + Tamb_rw_C*0.02*randn;
    end
    Tamb_C_r = Tamb_C_r + rw;
    Tamb_K_r = Tamb_C_r + 273.15;

    % demand
    Preal_W = max(Pdem_nom_W .* pow_mult_r, 0);

    % late mission stress
    idxL = (t >= late_gust_start_s);
    e = randn(sum(idxL),1); if std(e)>0, e = e/std(e); end
    Preal_W(idxL) = max(Preal_W(idxL) + late_gust_sigma_W*e, 0);

    spk = (rand(sum(idxL),1) < late_spike_prob);
    spk_add = (late_spike_W(1) + (late_spike_W(2)-late_spike_W(1))*rand(sum(spk),1));
    idx_first = find(idxL,1,'first');
    Preal_W(idx_first-1 + find(spk)) = Preal_W(idx_first-1 + find(spk)) + spk_add;

    % LMPC
    mpc_no = mpc0; mpc_no.use_rft_constraint = false;
    [log_no, ~] = mpclib.sim_mpc(Preal_W, t, Ts, par, x0, Tamb_K_r, mpc_no, cfg_pred);
    mc_end_no(r) = log_no.endurance_min; mc_unm_no(r) = log_no.unmet_kWh;

    % PID
    logp = plant.sim_pid(Preal_W, t, Ts, par, x0, Tamb_K_r, pid);
    mc_end_pid(r) = logp.endurance_min; mc_unm_pid(r) = logp.unmet_kWh;

    % Fuzzy
    logf = plant.sim_fuzzy(Preal_W, t, Ts, par, x0, Tamb_K_r, fz);
    mc_end_fz(r) = logf.endurance_min; mc_unm_fz(r) = logf.unmet_kWh;

    % LMPC+RFT
    mpc_r = mpc0; mpc_r.use_rft_constraint = true; mpc_r.RFT_spec_s = 3.0*3600;
    [logr, ~] = mpclib.sim_mpc(Preal_W, t, Ts, par, x0, Tamb_K_r, mpc_r, cfg_pred);
    mc_end_rft(r) = logr.endurance_min; mc_unm_rft(r) = logr.unmet_kWh;

    if isfield(logr,'RFT_slack') && ~isempty(logr.RFT_slack)
        mc_slack_rft{r} = logr.RFT_slack(:);
        mc_t_rft{r} = logr.t(:);
    else
        mc_slack_rft{r} = []; mc_t_rft{r} = [];
    end
end

figure('Name','Monte Carlo endurance'); hold on; grid on;
allEnd = [mc_end_pid; mc_end_fz; mc_end_no; mc_end_rft];
edgesE = linspace(floor(min(allEnd)), ceil(max(allEnd)), 10);
histogram(mc_end_pid, edgesE, 'FaceAlpha',0.35);
histogram(mc_end_fz,  edgesE, 'FaceAlpha',0.35);
histogram(mc_end_no,  edgesE, 'FaceAlpha',0.35);
histogram(mc_end_rft, edgesE, 'FaceAlpha',0.35);
xlabel('Endurance [min]'); ylabel('Count');
title(sprintf('Monte Carlo endurance (N=%d) - wind+temp+late stress',MC_runs));
legend('PID','Fuzzy','LMPC','LMPC+RFT(3h)','Location','best');

figure('Name','Monte Carlo unmet energy'); hold on; grid on;
allU = [mc_unm_pid; mc_unm_fz; mc_unm_no; mc_unm_rft];
edgesU = linspace(min(allU), max(allU), 12);
histogram(mc_unm_pid, edgesU, 'FaceAlpha',0.35);
histogram(mc_unm_fz,  edgesU, 'FaceAlpha',0.35);
histogram(mc_unm_no,  edgesU, 'FaceAlpha',0.35);
histogram(mc_unm_rft, edgesU, 'FaceAlpha',0.35);
set(gca,'XScale','log');
xlabel('Unmet energy [kWh] (log scale)'); ylabel('Count');
title(sprintf('Monte Carlo unmet energy (N=%d) - wind+temp+late stress',MC_runs));
legend('PID','Fuzzy','LMPC','LMPC+RFT(3h)','Location','best');

MC = table();
MC.Controller = {'PID';'Fuzzy';'LMPC';'LMPC+RFT'};
MC.Endurance_mean = [mean(mc_end_pid); mean(mc_end_fz); mean(mc_end_no); mean(mc_end_rft)];
MC.Endurance_std  = [std(mc_end_pid);  std(mc_end_fz);  std(mc_end_no);  std(mc_end_rft)];
MC.Unmet_mean_kWh = [mean(mc_unm_pid); mean(mc_unm_fz); mean(mc_unm_no); mean(mc_unm_rft)];
MC.Unmet_std_kWh  = [std(mc_unm_pid);  std(mc_unm_fz);  std(mc_unm_no);  std(mc_unm_rft)];

wU = 1.0; wE = 0.10;
MC.TradeoffScore = wU*MC.Unmet_mean_kWh + wE*(max(MC.Endurance_mean)-MC.Endurance_mean);

disp('=== Monte Carlo summary (mean/std) ===');
disp(MC);

% Slack distribution and envelope
allSlack = []; 
for r=1:MC_runs
    if ~isempty(mc_slack_rft{r}), allSlack = [allSlack; mc_slack_rft{r}]; end
end

if ~isempty(allSlack)
    figure('Name','RFT slack distribution (Monte Carlo)'); grid on; hold on;
    histogram(allSlack, 40);
    xline(0,'k--','LineWidth',1.2);
    xlabel('RFT slack (RHS - SoC_{term,pred})'); ylabel('Count');
    title(sprintf('RFT slack distribution across Monte Carlo (N=%d)',MC_runs));

    tgrid = t(:);
    S = nan(numel(tgrid), MC_runs);
    for r=1:MC_runs
        tr = mc_t_rft{r}; sr = mc_slack_rft{r};
        if isempty(tr), continue; end
        S(:,r) = interp1(tr, sr, tgrid, 'linear', NaN);
    end

    medS = nanmedian(S,2);
    p10  = prctile(S',10)';
    p90  = prctile(S',90)';

    figure('Name','RFT slack envelope (Monte Carlo)'); hold on; grid on;
    plot(tgrid, medS, 'LineWidth',1.8);
    plot(tgrid, p10, '--', 'LineWidth',1.2);
    plot(tgrid, p90, '--', 'LineWidth',1.2);
    xline(t_critical_start,'k--'); xline(t_critical_end,'k--');
    xlabel('t [s]'); ylabel('RFT slack');
    title('RFT slack vs time (median, 10-90% band)');
    legend('median','p10','p90','critical window','Location','best');

    % Auto-tune schedule (research-grade)
    optsTune = struct('active_tol',RFT_ACTIVE_TOL,'target_crit_active_pct',60,'max_outer_active_pct',40, ...
        'base_hi_hr',3.0,'base_lo_hr',1.5,'step_hr',0.25,'min_lo_hr',1.0,'max_hi_hr',4.0);
    sched_auto = mpclib.auto_tune(tgrid, S, t_critical_start, t_critical_end, optsTune);
    disp('=== Auto-tuned schedule from Monte Carlo slack ===');
    disp(sched_auto); disp(sched_auto.stats);

    mpc_auto = mpc0; mpc_auto.use_rft_constraint = true; mpc_auto.RFT_schedule = sched_auto;
    [log_auto, ~] = runMPC(Pdem_true_W, mpc_auto, Tamb_K);

    figure('Name','Constant vs Auto-tuned Phase-adaptive RFT'); hold on; grid on;
    plot(log_rft.t,  log_rft.SoC,  'LineWidth',1.6);
    plot(log_auto.t, log_auto.SoC, 'LineWidth',1.6);
    xline(t_critical_start,'--'); xline(t_critical_end,'--');
    xlabel('t [s]'); ylabel('SoC');
    title('Constant RFT vs Auto-tuned Phase-adaptive RFT');
    legend('Constant RFT (3h)','Auto-tuned phase-adaptive','Location','best');

    % Sweep (paper figure)
    hi_list_hr = 2.5:0.25:4.0;
    lo_list_hr = 1.0:0.25:2.5;
    simfun = @(mpc_in) mpclib.sim_mpc(Pdem_true_W, t, Ts, par, x0, Tamb_K, mpc_in, cfg_pred);
    OUT = mpclib.sweep(simfun, par, t_critical_start, t_critical_end, hi_list_hr, lo_list_hr);
    disp('=== Sweep results table (phase-adaptive schedules) ===');
    disp(OUT);
end

disp('DONE: All scenarios executed.');

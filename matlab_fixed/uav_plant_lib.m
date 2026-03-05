function plant = uav_plant_lib()
% UAV Plant/Controllers library (Option A, clean and robust)
% Exposes: load_demand, batt_params, batt_init, pid_settings, fuzzy_settings,
%          sim_pid, sim_fuzzy, summarize

plant.load_demand   = @load_demand;
plant.batt_params   = @batt_params;
plant.batt_init     = @batt_init;

plant.pid_settings  = @pid_settings;
plant.fuzzy_settings= @fuzzy_settings;

plant.sim_pid       = @simulate_uav_with_pid;
plant.sim_fuzzy     = @simulate_uav_with_fuzzy;

plant.summarize     = @summarize_log_uav;
end

%% ============================================================
% SETTINGS (PID / FUZZY)
%% ============================================================
function pid = pid_settings()
% Prefer original if available (keeps identical behavior)
if exist('pid_settings_uav','file') == 2
    pid = pid_settings_uav();
    return;
end
% Fallback PID (reasonable baseline)
pid = struct();
pid.Kp = 0.0045;
pid.Ki = 0.00025;
pid.Kd = 0.0;
end

function fz = fuzzy_settings()
% Prefer original if available (keeps identical behavior)
if exist('fuzzy_settings_uav','file') == 2
    fz = fuzzy_settings_uav();
    return;
end
% Fallback fuzzy settings (only used if your original missing)
fz = struct();
fz.e_scale_W = 90;
fz.dI_max_A = 70;
fz.du_max_A_per_s = 45;
fz.Ki = 0.012;
fz.int_max = 0.30;
fz.soc_low  = 0.14;
fz.soc_high = 0.32;
fz.low_soc_limit_scale = 0.80;
fz.Vm_low_V  = 0.15;
fz.Vm_mid_V  = 0.35;
fz.Vm_high_V = 0.70;
fz.I_vsoft_factor = 0.95;
fz.enable_iff_smoothing = true;
fz.iff_tau_s = 3.0;
end

%% ============================================================
% DEMAND LOADER
%% ============================================================
function [t, Pdem_nom_W, meta] = load_demand(Ts, T_end_s, opts)
if nargin < 3, opts = struct(); end

% Use your original mission if present
if exist('load_uav_demand_profile','file') == 2
    [t, Pdem_nom_W, meta] = load_uav_demand_profile(Ts, T_end_s, opts);
    t = t(:); Pdem_nom_W = Pdem_nom_W(:);
    return;
end

% Fallback mission (only if original missing)
N = floor(T_end_s/Ts)+1;
t = (0:Ts:T_end_s).';
meta = struct();
meta.V_ms = 20*ones(N,1);

Pdem_nom_W = 180*ones(N,1);
Pdem_nom_W(t>=200 & t<800)   = 240;
Pdem_nom_W(t>=800 & t<1200)  = 200;
Pdem_nom_W(t>=1200 & t<1800) = 280;
Pdem_nom_W(t>=1800 & t<2600) = 220;
Pdem_nom_W(t>=2600)          = 260;
Pdem_nom_W = Pdem_nom_W(:);
end

%% ============================================================
% BATTERY PARAMS + INIT
%% ============================================================
function par = batt_params()
% Prefer your original LiPo model
if exist('batt_params_2rc_uav_lipo','file') == 2
    par = batt_params_2rc_uav_lipo();
else
    % fallback minimal params
    par = struct();
    par.I_max = 120;
    par.Q_Ah = 10;
    par.SoC_min = 0.05;
    par.Vmin_limit = 9.6;
    par.R0 = 0.015; par.R1 = 0.010; par.C1 = 2000;
    par.R2 = 0.020; par.C2 = 3000;
    par.T0 = 298.15;
end

% Normalize common names
if ~isfield(par,'Vmin_limit')
    if isfield(par,'Vmin'), par.Vmin_limit = par.Vmin;
    elseif isfield(par,'V_min'), par.Vmin_limit = par.V_min;
    else, par.Vmin_limit = 9.6;
    end
end
if ~isfield(par,'SoC_min'), par.SoC_min = 0.05; end
if ~isfield(par,'I_max'), par.I_max = 120; end
end

function x0 = batt_init(par)
if exist('batt_init_state','file') == 2
    x0 = batt_init_state(par);
    return;
end
x0 = struct();
x0.SoC = 0.9;
x0.SoH = 1.0;
x0.V1  = 0;
x0.V2  = 0;
x0.T   = getfield_def(par,'T0',298.15);
end

%% ============================================================
% PID SIMULATION
%% ============================================================
function log = simulate_uav_with_pid(Pdem_W, t, Ts, par, x0, Tamb, pid)

Nsim = numel(t);
Pdem_W = Pdem_W(:); t = t(:);

% Robust Tamb
if isscalar(Tamb)
    Tamb_vec = Tamb*ones(Nsim,1);
else
    Tamb_vec = Tamb(:);
    if numel(Tamb_vec) ~= Nsim
        error('simulate_uav_with_pid: Tamb must be scalar or same length as t.');
    end
end

log = init_log(t, Pdem_W);

x = x0;
e_int = 0;
e_prev = 0;

for k = 1:Nsim
    Pdel_prev = 0;
    if k>1, Pdel_prev = log.Pdel(k-1); end

    e = Pdem_W(k) - Pdel_prev;
    e_int = e_int + e*Ts;
    de = (e - e_prev)/Ts;
    e_prev = e;

    Icmd = pid.Kp*e + pid.Ki*e_int + pid.Kd*de;
    Icmd = max(0, min(Icmd, par.I_max));

    Tamb_k = Tamb_vec(k);

    [xnext,y] = batt_step_dispatch(x, Icmd, Ts, par, Tamb_k);

    log.I(k)=Icmd; log.Vt(k)=y.Vt; log.Pdel(k)=y.Pdel;
    log.SoC(k)=xnext.SoC; log.T(k)=xnext.T;
    log.Punmet(k)=max(Pdem_W(k)-y.Pdel,0);

    if (y.Vt <= par.Vmin_limit) || (xnext.SoC <= par.SoC_min)
        log = trim_log_to_k(log,k);
        break;
    end
    x = xnext;
end

log = finalize_log(log);
end

%% ============================================================
% FUZZY SIMULATION
%% ============================================================
function log = simulate_uav_with_fuzzy(Pdem_W, t, Ts, par, x0, Tamb, fz)

Nsim = numel(t);
Pdem_W = Pdem_W(:); t = t(:);

% Robust Tamb
if isscalar(Tamb)
    Tamb_vec = Tamb*ones(Nsim,1);
else
    Tamb_vec = Tamb(:);
    if numel(Tamb_vec) ~= Nsim
        error('simulate_uav_with_fuzzy: Tamb must be scalar or same length as t.');
    end
end

log = init_log(t, Pdem_W);

x = x0;
st = [];

for k = 1:Nsim
    Tamb_k = Tamb_vec(k);

    Pdel_prev = 0;
    if k>1, Pdel_prev = log.Pdel(k-1); end

    if exist('uav_fuzzy_power_controller','file') == 2
        [Icmd, st] = uav_fuzzy_power_controller(Pdem_W(k), Pdel_prev, x, par, Ts, fz, st);
    else
        Voc = batt_ocv_dispatch(x.SoC, par);
        Icmd = Pdem_W(k)/max(Voc,1);
    end

    Icmd = max(0, min(Icmd, par.I_max));

    [xnext,y] = batt_step_dispatch(x, Icmd, Ts, par, Tamb_k);

    log.I(k)=Icmd; log.Vt(k)=y.Vt; log.Pdel(k)=y.Pdel;
    log.SoC(k)=xnext.SoC; log.T(k)=xnext.T;
    log.Punmet(k)=max(Pdem_W(k)-y.Pdel,0);

    if (y.Vt <= par.Vmin_limit) || (xnext.SoC <= par.SoC_min)
        log = trim_log_to_k(log,k);
        break;
    end
    x = xnext;
end

log = finalize_log(log);
end

%% ============================================================
% SUMMARY METRICS
%% ============================================================
function S = summarize_log_uav(log, par, tol, tcrit1, tcrit2)
if nargin < 3 || isempty(tol), tol = 5e-3; end

dt = median(diff(log.t));
Pun = log.Punmet(:);

S = struct();
S.Endurance_min = log.t(end)/60;
S.Unmet_kWh = sum(Pun)*dt/3600/1000;
S.Mean_Punmet_W = mean(Pun);
S.Max_Punmet_W  = max(Pun);

S.SoC_end = log.SoC(end);
S.SoC_min = min(log.SoC);
S.Vmin_V = min(log.Vt);
S.Tmax = max(log.T);

S.ReserveMargin_end = NaN;
if isfield(par,'SoC_min')
    S.ReserveMargin_end = log.SoC(end) - par.SoC_min;
end

% RFT fields (only if present in log)
if isfield(log,'RFT_slack') && ~isempty(log.RFT_slack)
    s = log.RFT_slack(:);
    S.RFTslack_min  = min(s);
    S.RFTslack_mean = mean(s);
    S.RFT_active_pct = 100*mean(abs(s)<tol | s<tol);

    if nargin>=5 && ~isempty(tcrit1) && ~isempty(tcrit2)
        tg = log.t(:);
        idxE = tg < tcrit1;
        idxC = tg >= tcrit1 & tg <= tcrit2;
        idxL = tg > tcrit2;
        S.RFT_active_early_pct = 100*mean(abs(s(idxE))<tol | s(idxE)<tol);
        S.RFT_active_crit_pct  = 100*mean(abs(s(idxC))<tol | s(idxC)<tol);
        S.RFT_active_late_pct  = 100*mean(abs(s(idxL))<tol | s(idxL)<tol);
    else
        S.RFT_active_early_pct = NaN;
        S.RFT_active_crit_pct  = NaN;
        S.RFT_active_late_pct  = NaN;
    end
else
    S.RFTslack_min = NaN; S.RFTslack_mean = NaN; S.RFT_active_pct = NaN;
    S.RFT_active_early_pct = NaN; S.RFT_active_crit_pct = NaN; S.RFT_active_late_pct = NaN;
end
end

%% ============================================================
% INTERNAL HELPERS
%% ============================================================
function log = init_log(t, Pdem)
Nsim = numel(t);
log = struct();
log.t = t;
log.Pdem = Pdem;
log.I = zeros(Nsim,1);
log.Pdel = zeros(Nsim,1);
log.Vt = zeros(Nsim,1);
log.SoC = zeros(Nsim,1);
log.T = zeros(Nsim,1);
log.Punmet = zeros(Nsim,1);
end

function log = finalize_log(log)
log.endurance_min = log.t(end)/60;
dt = median(diff(log.t));
log.unmet_kWh = sum(log.Punmet)*dt/3600/1000;
end

function log = trim_log_to_k(log,k)
fn = fieldnames(log);
for i=1:numel(fn)
    v = log.(fn{i});
    if (isnumeric(v) || islogical(v)) && isvector(v) && numel(v)>=k
        log.(fn{i}) = v(1:k);
    end
end
log.t = log.t(1:k);
log.Pdem = log.Pdem(1:k);
end

function [xnext,y] = batt_step_dispatch(x, I, Ts, par, Tamb)
if exist('batt_step_2rc_thermal_soh_uav','file') == 2
    [xnext,y] = batt_step_2rc_thermal_soh_uav(x, I, Ts, par, Tamb);
else
    [xnext,y] = batt_step_fallback(x, I, Ts, par, Tamb);
end
end

function Voc = batt_ocv_dispatch(SoC, par)
if exist('batt_ocv_uav','file') == 2
    Voc = batt_ocv_uav(SoC, par);
else
    Voc = 12.0 + 1.2*(SoC-0.5);
end
end

function [xnext, y] = batt_step_fallback(x, I, Ts, par, Tamb)
Voc = batt_ocv_dispatch(x.SoC, par);
R0  = getfield_def(par,'R0',0.015);
Vt  = Voc - I*R0;
Pdel= max(Vt*I,0);

Q_Ah = getfield_def(par,'Q_Ah',10);
SoC  = x.SoC - (I*Ts)/(Q_Ah*3600);
SoC  = min(max(SoC,0),1);

xnext = x;
xnext.SoC = SoC;
xnext.T = 0.98*x.T + 0.02*Tamb;

y = struct('Vt',Vt,'Pdel',Pdel);
end

function v = getfield_def(S, name, def)
if isstruct(S) && isfield(S,name) && ~isempty(S.(name))
    v = S.(name);
else
    v = def;
end
end
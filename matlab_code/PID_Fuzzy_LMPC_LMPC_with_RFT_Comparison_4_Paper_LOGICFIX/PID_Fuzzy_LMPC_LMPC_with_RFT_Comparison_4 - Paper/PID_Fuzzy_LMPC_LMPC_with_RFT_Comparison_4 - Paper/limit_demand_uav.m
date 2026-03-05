function [Pdem_limited, info] = limit_demand_uav(Pdem_in, t, Ts, par, x0, Tamb, cfg)
% EMS demand derating layer for UAV battery (NO slack MPC)
% Goal: keep demand feasible and prevent commanded demand from collapsing to ~0
% by adding:
%   - P_floor_W : minimum commanded power (if feasible)
%   - f_min     : minimum derating factor

if nargin < 7 || isempty(cfg), cfg = struct(); end

Pdem_in = Pdem_in(:);
t = t(:);
N = numel(t);

Imax = par.I_max;

if isfield(par,'Vmin_limit')
    Vmin = par.Vmin_limit;
elseif isfield(par,'V_min')
    Vmin = par.V_min;
elseif isfield(par,'Vmin')
    Vmin = par.Vmin;
else
    Vmin = 9.6;
end

SoC_soft   = getfield_def(cfg,'SoC_soft', 0.30);
SoC_hard   = getfield_def(cfg,'SoC_hard', 0.20);
V_margin   = getfield_def(cfg,'V_margin', 0.20);
eta_margin = getfield_def(cfg,'eta_margin', 0.98);
alpha      = getfield_def(cfg,'alpha', 0.15);

% NEW knobs
P_floor = getfield_def(cfg,'P_floor_W', 0);  % e.g., 30 W
f_min   = getfield_def(cfg,'f_min', 0);      % e.g., 0.25

x = x0;
Pdem_limited = zeros(N,1);
Pprev = Pdem_in(1);

info.SoC = zeros(N,1);
info.Vt  = zeros(N,1);
info.Pmax_feasible = zeros(N,1);
info.derate_factor = zeros(N,1);

for k=1:N
    info.SoC(k) = x.SoC;

    % Conservative 1-step voltage estimate at Imax
    [~, y_test] = batt_step_2rc_thermal_soh_uav(x, Imax, Ts, par, Tamb);
    Vt_test = y_test.Vt;
    info.Vt(k) = Vt_test;

    % Feasible cap
    Pmax = max(0, Vt_test * Imax * eta_margin);
    info.Pmax_feasible(k) = Pmax;

    % SoC derating factor
    f_soc = 1.0;
    if x.SoC <= SoC_soft
        f_soc = (x.SoC - SoC_hard) / max(1e-6, (SoC_soft - SoC_hard));
        f_soc = min(max(f_soc, 0), 1);
    end

    % Voltage derating factor
    f_v = 1.0;
    if Vt_test <= (Vmin + V_margin)
        f_v = (Vt_test - Vmin) / max(1e-6, V_margin);
        f_v = min(max(f_v, 0), 1);
    end

    % Combine + clamp to avoid f -> 0
    f = min(f_soc, f_v);
    f = max(f, f_min);
    info.derate_factor(k) = f;

    % Raw command: cap by feasible power and apply derating
    Praw = min(Pdem_in(k), Pmax);
    Praw = f * Praw;

    % Smooth command
    Pcmd = (1-alpha)*Pprev + alpha*Praw;

    % NEW: keep minimum flight power, but never exceed feasible cap
    if P_floor > 0
        Pcmd = max(Pcmd, min(P_floor, Pmax));
    end

    Pdem_limited(k) = max(Pcmd, 0);
    Pprev = Pdem_limited(k);

    % Observer propagation: approximate tracking current
    Vuse = max(Vt_test, Vmin);
    Itrack = min(max(Pdem_limited(k) / Vuse, 0), Imax);

    [xnext, ~] = batt_step_2rc_thermal_soh_uav(x, Itrack, Ts, par, Tamb);
    xnext.SoC = min(max(xnext.SoC,0),1);
    x = xnext;
end
end

function v = getfield_def(s, f, def)
if isstruct(s) && isfield(s,f) && ~isempty(s.(f))
    v = s.(f);
else
    v = def;
end
end

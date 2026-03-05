function log = simulate_uav_with_fuzzy(Pdem_W, t, Ts, par, x0, Tamb, fz)
%SIMULATE_UAV_WITH_FUZZY Plant simulation with a lightweight fuzzy controller.
%   Fair comparison policy:
%   - NO hidden SoC guards
%   - ONLY physical current bounds [I_min, I_max]
%   - Same stop criteria as MPC (Vmin / SoC_min)
%   - Optional rate limit (physical actuator/slew)

if nargin < 7 || isempty(fz)
    fz = fuzzy_settings_uav();
end

Nsim = numel(t);
Pdem_W = Pdem_W(:);
t = t(:);

% Tamb can be scalar OR profile
if isscalar(Tamb)
    Tamb_vec = Tamb*ones(Nsim,1);
else
    Tamb_vec = Tamb(:);
    if numel(Tamb_vec) ~= Nsim
        error('simulate_uav_with_fuzzy: Tamb profile must be scalar or same length as t.');
    end
end

% Voltage stop limit (mirror MPC)
if isfield(par,'Vmin_limit')
    Vmin_stop = par.Vmin_limit;
elseif isfield(par,'V_min')
    Vmin_stop = par.V_min;
elseif isfield(par,'Vmin')
    Vmin_stop = par.Vmin;
else
    Vmin_stop = 9.6;
end

% Logging
log = struct();
log.t      = t;
log.Pdem   = Pdem_W;
log.I      = zeros(Nsim,1);
log.Pdel   = zeros(Nsim,1);
log.Vt     = zeros(Nsim,1);
log.SoC    = zeros(Nsim,1);
log.T      = zeros(Nsim,1);
log.SoH    = zeros(Nsim,1);
log.Punmet = zeros(Nsim,1);

% Controller memory
st = struct();
x = x0;
Pdel_prev = 0;

% Physical bounds
Imin = par.I_min;
Imax = par.I_max;

% Rate limit (A/s) – physical / actuator realism (not a "guard")
du_max = fz.du_max_A_per_s;

for k=1:Nsim
    Tamb_k = Tamb_vec(k);

    % Controller proposes current based on tracking error
    [Icmd, st] = uav_fuzzy_power_controller(Pdem_W(k), Pdel_prev, x, par, Ts, fz, st);

    % Fair physical clamp only
    Icmd = min(max(Icmd, Imin), Imax);

    % Optional rate limiting (physical)
    if ~isfield(st,'Iprev'), st.Iprev = 0; end
    Icmd = min(max(Icmd, st.Iprev - du_max*Ts), st.Iprev + du_max*Ts);
    st.Iprev = Icmd;

    % Plant step
    [xnext, y] = batt_step_2rc_thermal_soh_uav(x, Icmd, Ts, par, Tamb_k);

    % Log
    log.I(k)      = Icmd;
    log.Vt(k)     = y.Vt;
    log.Pdel(k)   = y.Pdel;
    log.SoC(k)    = xnext.SoC;
    log.T(k)      = xnext.T;
    log.SoH(k)    = xnext.SoH;
    log.Punmet(k) = max(Pdem_W(k) - y.Pdel, 0);

    Pdel_prev = y.Pdel;
    x = xnext;

    % Stop conditions (same as MPC)
    if (y.Vt <= Vmin_stop) || (xnext.SoC <= par.SoC_min)
        log = trim_log_to_k(log,k);
        break;
    end
end

% Summary
log.endurance_min = log.t(end)/60;
dt = median(diff(log.t));
unmet_Wh = sum(max(log.Punmet,0))*dt/3600;
log.unmet_kWh = unmet_Wh/1000;
end

function log = trim_log_to_k(log, k)
fn = fieldnames(log);
for i = 1:numel(fn)
    f = fn{i};
    v = log.(f);
    if isnumeric(v) || islogical(v)
        if isvector(v) && numel(v) >= k
            log.(f) = v(1:k);
        end
    end
end
if isfield(log,'t') && numel(log.t) >= k
    log.t = log.t(1:k);
end
end
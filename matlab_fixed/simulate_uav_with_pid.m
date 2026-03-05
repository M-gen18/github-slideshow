function log = simulate_uav_with_pid(Pdem_W, t, Ts, par, x0, Tamb, pid)

if nargin < 7 || isempty(pid)
    pid = pid_settings_uav();
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
        error('simulate_uav_with_pid: Tamb profile must be scalar or same length as t.');
    end
end

% Voltage stop limit
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
log.t = t;
log.Pdem = Pdem_W;
log.I = zeros(Nsim,1);
log.Pdel = zeros(Nsim,1);
log.Vt = zeros(Nsim,1);
log.SoC = zeros(Nsim,1);
log.T = zeros(Nsim,1);
log.SoH = zeros(Nsim,1);
log.Punmet = zeros(Nsim,1);

% PID state
e_prev = 0;
Iint_A = 0;

x = x0;
Pdel_prev = 0;

for k=1:Nsim

    Tamb_k = Tamb_vec(k);

    % Power tracking error (use previous delivered power)
    e = Pdem_W(k) - Pdel_prev;

    % Base current reference (feed-forward)
    Voc = batt_ocv_uav(x.SoC, par);
    Iff = Pdem_W(k) / max(Voc,1);

    % PID terms (convert to delta-current through Voc)
    d = (e - e_prev) / max(Ts,1e-6);
    Iint_A = Iint_A + (pid.Ki_W * e) * Ts / max(Voc,1);
    Iint_A = min(max(Iint_A, -pid.Iint_max_A), pid.Iint_max_A);
    dI = (pid.Kp_W * e) / max(Voc,1) + Iint_A + (pid.Kd_W * d) / max(Voc,1);

    Icmd = Iff + dI;

    % SoC guard (optional): taper current near minimum SoC
    if isfield(pid,'enable_soc_guard') && pid.enable_soc_guard
        s1 = pid.soc_guard_start;
        s0 = pid.soc_guard_min;
        if x.SoC <= s1
            if x.SoC <= s0
                g = pid.soc_guard_scale;
            else
                g = pid.soc_guard_scale + (1-pid.soc_guard_scale)*(x.SoC - s0)/(s1 - s0);
            end
            Icmd = g*Icmd;
        end
    end

    % Apply limits consistent with baseline protections
    Icmd = uav_apply_current_limits(Icmd, x, par);

    % Plant step
    [xnext, y] = batt_step_2rc_thermal_soh_uav(x, Icmd, Ts, par, Tamb_k);

    log.I(k) = Icmd;
    log.Vt(k) = y.Vt;
    log.Pdel(k) = y.Pdel;
    log.SoC(k) = xnext.SoC;
    log.T(k) = xnext.T;
    log.SoH(k) = xnext.SoH;
    log.Punmet(k) = max(Pdem_W(k) - y.Pdel, 0);

    Pdel_prev = y.Pdel;
    e_prev = e;
    x = xnext;

    % Stop conditions
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
function log = simulate_battery_only_uav(Pdem_W, t, Ts, par, x0, Tamb, cfg)
if nargin<7, cfg=struct(); end
if ~isfield(cfg,'enable_power_unmet'), cfg.enable_power_unmet=true; end

N = numel(t);
log.t=t(:); log.Pdem=Pdem_W(:);
log.I=zeros(N,1); log.Pdel=zeros(N,1); log.Vt=zeros(N,1);
log.SoC=zeros(N,1); log.T=zeros(N,1); log.SoH=zeros(N,1);
log.Punmet=zeros(N,1);

log.near_Imax=false(N,1); log.near_Imin=false(N,1);
log.near_Vmin=false(N,1); log.near_Vmax=false(N,1);
log.near_Tmax=false(N,1);
log.near_SoCmin=false(N,1); log.near_SoCmax=false(N,1);
log.near_any=false(N,1);

log.stop_reason="completed_full_horizon";
log.stop_time_s=t(end);

I_margin=2; V_margin=0.10; T_margin=2; SoC_margin=0.01;

% -------- STOP CONDITION SETUP: Voltage instead of SoC --------
if isfield(par,'Vmin_limit')
    Vmin_stop = par.Vmin_limit;
elseif isfield(par,'V_min')
    Vmin_stop = par.V_min;
elseif isfield(par,'Vmin')
    Vmin_stop = par.Vmin;
else
    Vmin_stop = 9.6;
end
% -------------------------------------------------------------

x=x0;
for k=1:N
    [Icmd, ~] = bms_baseline_controller_uav(Pdem_W(k), x, par, cfg);
    [xnext, y] = batt_step_2rc_thermal_soh_uav(x, Icmd, Ts, par, Tamb);
    xnext.SoC = min(max(xnext.SoC,0),1);

    log.I(k)=Icmd; log.Pdel(k)=y.Pdel; log.Vt(k)=y.Vt;
    log.SoC(k)=x.SoC; log.T(k)=x.T; log.SoH(k)=x.SoH;

    if cfg.enable_power_unmet
        log.Punmet(k)=max(0, log.Pdem(k)-log.Pdel(k));
    end

    log.near_Imax(k) = (Icmd >= par.I_max - I_margin);
    log.near_Imin(k) = (Icmd <= par.I_min + I_margin);

    % Use existing limits if they exist (for near flags)
    if isfield(par,'Vmin_limit'), Vmin_flag = par.Vmin_limit; else, Vmin_flag = Vmin_stop; end
    if isfield(par,'Vmax_limit'), Vmax_flag = par.Vmax_limit;
    elseif isfield(par,'V_max'),  Vmax_flag = par.V_max;
    elseif isfield(par,'Vmax'),   Vmax_flag = par.Vmax;
    else, Vmax_flag = 12.6;
    end

    log.near_Vmin(k) = (y.Vt <= Vmin_flag + V_margin);
    log.near_Vmax(k) = (y.Vt >= Vmax_flag - V_margin);

    if isfield(par,'Tmax'), Tmax_flag = par.Tmax;
    elseif isfield(par,'T_max'), Tmax_flag = par.T_max;
    else, Tmax_flag = 60+273.15;
    end
    log.near_Tmax(k) = (xnext.T >= Tmax_flag - T_margin);

    log.near_SoCmin(k) = isfield(par,'SoC_min') && (xnext.SoC <= par.SoC_min + SoC_margin);
    log.near_SoCmax(k) = isfield(par,'SoC_max') && (xnext.SoC >= par.SoC_max - SoC_margin);

    log.near_any(k) = log.near_Imax(k)||log.near_Imin(k)||log.near_Vmin(k)||log.near_Vmax(k)|| ...
                      log.near_Tmax(k)||log.near_SoCmin(k)||log.near_SoCmax(k);

    % -------- Voltage-based stop (instead of SoC stop) --------
    if y.Vt <= Vmin_stop
        log.stop_reason="Vmin_limit_reached";
        log.stop_time_s=t(k);
        fn=fieldnames(log);
        for i=1:numel(fn)
            v=log.(fn{i});
            if (isnumeric(v)||islogical(v)) && numel(v)==N
                log.(fn{i})=v(1:k);
            end
        end
        break;
    end
    % ---------------------------------------------------------

    x=xnext;
end

log.unmet_kWh = sum(log.Punmet)*Ts/3600/1000;
log.endurance_s = log.stop_time_s - t(1);
log.endurance_min = log.endurance_s/60;
end

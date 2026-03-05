function Icmd = uav_apply_current_limits(Icmd, x, par)
%UAV_APPLY_CURRENT_LIMITS Apply basic actuator + safety limits to current.
%   Lightweight helper used by non-MPC controllers (PID/Fuzzy) to keep
%   plant protections consistent with the baseline controller.

% Hard bounds
Icmd = min(max(Icmd, par.I_min), par.I_max);

% Thermal derating (same logic as bms_baseline_controller_uav)
T_derate = par.Tmax - 10;
if x.T >= T_derate
    if x.T >= par.Tmax
        Icmd = min(Icmd, 0);
    else
        alpha = (par.Tmax - x.T) / (par.Tmax - T_derate);
        Icmd = min(Icmd, alpha*par.I_max);
    end
end

% Voltage protection (simple forward check)
Voc = batt_ocv_uav(x.SoC, par);
for it=1:12
    Vt_pred = Voc - Icmd*par.R0 - x.V1 - x.V2;
    if Vt_pred < par.Vmin_limit && Icmd > 0
        Icmd = 0.85*Icmd;
    elseif Vt_pred > par.Vmax_limit && Icmd < 0
        Icmd = 0.85*Icmd;
    else
        break;
    end
end

% Final clamp (guard numerical)
Icmd = min(max(Icmd, par.I_min), par.I_max);
end
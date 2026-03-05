function [Icmd, info] = bms_baseline_controller_uav(Pdem_W, x, par, cfg)
if nargin<4, cfg=struct(); end
Voc = batt_ocv_uav(x.SoC, par);

Iref = Pdem_W / max(Voc, 1);
Icmd = min(max(Iref, par.I_min), par.I_max);

% Thermal derating
T_derate = par.Tmax - 10;
if x.T >= T_derate
    if x.T >= par.Tmax
        Icmd = min(Icmd, 0);
    else
        alpha = (par.Tmax - x.T) / (par.Tmax - T_derate);
        Icmd = min(Icmd, alpha*par.I_max);
    end
end

% Voltage protection (simple)
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

info=struct('Iref',Iref,'Voc',Voc);
end

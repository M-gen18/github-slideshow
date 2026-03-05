function [xnext, y] = batt_step_2rc_thermal_soh_uav(x, I, Ts, par, Tamb)
%BATT_STEP_2RC_THERMAL_SOH_UAV One step of 2RC + thermal + SoH.

Qeff_C = max(1e-6, par.Qnom_C * x.SoH);
Voc = batt_ocv_uav(x.SoC, par);

dV1 = -(x.V1/(par.R1*par.C1)) + I/par.C1;
dV2 = -(x.V2/(par.R2*par.C2)) + I/par.C2;
V1n = x.V1 + Ts*dV1;
V2n = x.V2 + Ts*dV2;

R0eff = par.R0 * (1 + par.k_Rgrow*(1 - x.SoH));
Vt = Voc - I*R0eff - x.V1 - x.V2;
Pdel = Vt * I;

SoCn = x.SoC - Ts*I/Qeff_C;

Rloss = R0eff + par.R1 + par.R2;
Qdot = (I^2)*Rloss;
Tn = x.T + Ts*( Qdot - par.hA*(x.T - Tamb) )/par.Cth;

dAh = abs(I)*Ts/3600;
temp_factor = exp(par.k_T*((x.T-(25+273.15))/10));
SoHn = x.SoH - par.k_cap_Ah * dAh * temp_factor;
SoHn = min(max(SoHn,0),1);

xnext = x;
xnext.SoC=SoCn; xnext.V1=V1n; xnext.V2=V2n; xnext.T=Tn; xnext.SoH=SoHn;

y = struct('Voc',Voc,'Vt',Vt,'Pdel',Pdel);
end

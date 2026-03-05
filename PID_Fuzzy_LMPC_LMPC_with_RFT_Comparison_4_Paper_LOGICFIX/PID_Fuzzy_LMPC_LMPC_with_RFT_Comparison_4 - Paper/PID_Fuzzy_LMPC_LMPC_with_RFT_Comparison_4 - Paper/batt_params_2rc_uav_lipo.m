function par = batt_params_2rc_uav_lipo()
%BATT_PARAMS_2RC_UAV_LIPO 3S 13Ah LiPo baseline, SoC_min=0.20.

par.Ns = 3; par.Np = 1;

par.Qnom_Ah = 13.0;
par.Qnom_C  = par.Qnom_Ah * 3600;

par.Vmax = 4.2 * par.Ns;      % 12.6
par.Vnom = 3.7 * par.Ns;      % 11.1
par.Vmin = 3.2 * par.Ns;      % 9.6 (conservative)

par.Vmin_limit = par.Vmin;
par.Vmax_limit = par.Vmax;

par.SoC_min = 0.2;
par.SoC_max = 0.98;

% Initial resistance guesses (identify later)
par.R0 = 0.020;
tau1 = 8; tau2 = 80;
par.R1 = 0.30*par.R0;
par.R2 = 0.50*par.R0;
par.C1 = tau1/par.R1;
par.C2 = tau2/par.R2;

par.I_max = 60;
par.I_min = 0;           % [A] discharge-only (no regen)
par.Tmax =  60 + 273.15;
par.Cth  = 1200;
par.hA   = 6;

% OCV curve parameters (pack-level)
par.ocv_cell_a = 3.40;
par.ocv_cell_b = 0.75;
par.ocv_cell_c = 0.10;
par.ocv_cell_d = 8;
par.ocv_cell_e = 0.5;

par.ocv_a = par.Ns * par.ocv_cell_a;
par.ocv_b = par.Ns * par.ocv_cell_b;
par.ocv_c = par.Ns * par.ocv_cell_c;
par.ocv_d = par.ocv_cell_d;
par.ocv_e = par.ocv_cell_e;

par.k_cap_Ah = 8e-7;
par.k_T      = 0.06;
par.k_Rgrow  = 0.30;
end

clear; clc; close all;
addpath(genpath(fileparts(pwd)));

data_csv = fullfile(pwd,'pulse_test_data.csv');
Ts_override = [];
SoC0 = 0.95;
Tamb_default = 25 + 273.15;

par0 = batt_params_2rc_uav_lipo();

D = readtable(data_csv);
req = {'t_s','I_A','V_V'};
for i=1:numel(req)
    if ~ismember(req{i}, D.Properties.VariableNames)
        error('pulse_test_data.csv must contain column "%s"', req{i});
    end
end

t = D.t_s(:);
I = D.I_A(:);
V = D.V_V(:);

if isempty(Ts_override)
    Ts = median(diff(t));
else
    Ts = Ts_override;
end

if ismember('T_K', D.Properties.VariableNames), T = D.T_K(:); else, T = []; end
if ismember('Tamb_K', D.Properties.VariableNames), Tamb = D.Tamb_K(:); else, Tamb = Tamb_default*ones(size(t)); end

SoC = SoC0 - cumsum(I)*Ts/par0.Qnom_C;
SoC = min(max(SoC,0),1);
Voc = arrayfun(@(s)batt_ocv_uav(s, par0), SoC);

opt = struct('verbose', true, 'maxIter', 4000);
id_elec = fit_2rc_least_squares(t, I, V, Voc, Ts, opt);

id_th = [];
if ~isempty(T)
    id_th = fit_thermal_least_squares(t, I, T, Tamb, id_elec, Ts);
end

save('id_results.mat','id_elec','id_th','Ts','t','I','V','Voc','SoC');

fprintf('\n=== Electrical identification ===\n'); disp(id_elec);
if ~isempty(id_th), fprintf('\n=== Thermal identification ===\n'); disp(id_th);
else, fprintf('\nThermal identification skipped (no T_K).\n'); end

function results = tune_mpc_weights_uav()
clear; clc;
addpath(genpath(pwd));

Ts = 1;
T_end_s = 2*3600;
[t, Pdem_W, meta] = load_uav_demand_profile(Ts, T_end_s, struct('use_drivetrain_map', true));

par = batt_params_2rc_uav_lipo();
x0  = batt_init_state(par);
x0.SoC = 0.95;
x0.SoH = 1.00;
x0.T   = 25+273.15;
x0.V1  = 0;
x0.V2  = 0;
Tamb = 25+273.15;

% Baseline once
cfg_base = struct('enable_power_unmet', true);
log_base = simulate_battery_only_uav(Pdem_W, t, Ts, par, x0, Tamb, cfg_base);

% Sweep ranges (small and practical)
qP_list  = [0.4 0.6 0.8 1.2];
qI_list  = [0.10 0.18 0.30];
qdI_list = [1.0 2.0 3.0 5.0];

rows = [];
k=0;

for qP = qP_list
  for qI = qI_list
    for qdI = qdI_list
        k = k + 1;
        mpc = mpc_settings_uav();
        mpc.Ts = Ts;
        mpc.qP = qP;
        mpc.qI = qI;
        mpc.qdI = qdI;

        cfg_mpc = struct('enable_predlog', false);
        log_mpc = simulate_uav_with_mpc(Pdem_W, t, Ts, par, x0, Tamb, mpc, cfg_mpc);

        % Metrics
        Edem_Wh = sum(max(Pdem_W,0))*Ts/3600;
        Ebase_Wh= sum(max(log_base.Pdel,0))*Ts/3600;
        Empc_Wh = sum(max(log_mpc.Pdel,0))*Ts/3600;

        Irms_base = sqrt(mean(log_base.I.^2));
        Irms_mpc  = sqrt(mean(log_mpc.I.^2));

        unmet_Wh = log_mpc.unmet_kWh*1000;

        rows = [rows; {qP,qI,qdI, log_mpc.endurance_min, unmet_Wh, Empc_Wh, Irms_mpc, max(abs(log_mpc.I)), max(log_mpc.T)-273.15}];
        fprintf('(%2d) qP=%.2f qI=%.2f qdI=%.1f -> end=%.1f min, unmet=%.1f Wh, Irms=%.2f A\n', ...
            k,qP,qI,qdI,log_mpc.endurance_min,unmet_Wh,Irms_mpc);
    end
  end
end

results = cell2table(rows, 'VariableNames', ...
    {'qP','qI','qdI','endurance_min','unmet_Wh','Edel_Wh','Irms_A','Ipk_A','Tmax_C'});

% Baseline summary for reference
fprintf('\nBASELINE: end=%.1f min, unmet=%.2f Wh, Irms=%.2f A, Ipk=%.2f A\n', ...
    log_base.endurance_min, log_base.unmet_kWh*1000, Irms_base, max(abs(log_base.I)));

% Select "good" candidates:
%   - unmet energy very small (<= 2 Wh) => mission-following
%   - minimize Irms, then maximize endurance
good = results(results.unmet_Wh <= 2.0, :);
if ~isempty(good)
    good = sortrows(good, {'Irms_A','endurance_min'}, {'ascend','descend'});
    disp('--- Top mission-following candidates (unmet <= 2 Wh) ---');
    disp(good(1:min(10,height(good)),:));
else
    disp('No candidates met unmet<=2 Wh; consider increasing qP or adding slack later.');
end

% Also show a Pareto-style view (unmet vs Irms)
figure('Name','MPC tuning: unmet vs Irms');
scatter(results.unmet_Wh, results.Irms_A); grid on;
xlabel('Unmet energy [Wh]'); ylabel('I_{rms} [A]');
title('Trade-off: tracking vs stress');

end

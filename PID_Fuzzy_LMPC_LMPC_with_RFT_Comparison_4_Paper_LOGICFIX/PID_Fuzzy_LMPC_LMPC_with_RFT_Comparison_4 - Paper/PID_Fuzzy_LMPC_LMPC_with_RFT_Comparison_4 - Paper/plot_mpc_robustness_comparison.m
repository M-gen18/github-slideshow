function plot_mpc_robustness_comparison(log_perf, log_miss, par, meta)
% plot_mpc_robustness_comparison
% Robustness plots: demand vs prediction vs delivered, zooms, tracking error,
% unmet power, and constraint responses.

t = log_perf.t(:);

%% Demand vs Delivered (full mission)
figure('Name','Robustness: Demand vs Prediction vs Delivered');
plot(t, log_perf.Pdem, 'k-', 'LineWidth', 1.4); hold on;
plot(t, log_miss.Ppred0, 'm:', 'LineWidth', 1.8);
plot(t, log_miss.Pdel, 'b--', 'LineWidth', 1.6);
plot(t, log_perf.Pdel, 'r-',  'LineWidth', 1.6);
grid on; xlabel('t [s]'); ylabel('Power [W]');
title('Robustness test: MPC prediction mismatch vs perfect knowledge');
legend('Actual Demand (real)','MPC Internal Prediction (mismatch)','Delivered Power (mismatch)','Delivered Power (perfect)','Location','Best');

%% Zoom views: mid + end
figure('Name','Robustness: Zoom views (mid + end)');

subplot(2,1,1);
idx_mid = (t>=1000) & (t<=2500);
plot(t(idx_mid), log_perf.Pdem(idx_mid), 'k-', 'LineWidth', 1.2); hold on;
plot(t(idx_mid), log_miss.Pdel(idx_mid), 'b--', 'LineWidth', 1.4);
plot(t(idx_mid), log_perf.Pdel(idx_mid), 'r-',  'LineWidth', 1.4);
grid on; ylabel('Power [W]'); title('Mid-mission (disturbance/noise region)');
legend('Pdem','Mismatch delivered','Perfect delivered','Location','Best');

subplot(2,1,2);
idx_end = (t>=0) & (t<=t(end));
plot(t(idx_end), log_perf.Pdem(idx_end), 'k-', 'LineWidth', 1.2); hold on;
plot(t(idx_end), log_miss.Pdel(idx_end), 'b--', 'LineWidth', 1.4);
plot(t(idx_end), log_perf.Pdel(idx_end), 'r-',  'LineWidth', 1.4);
grid on; xlabel('t [s]'); ylabel('Power [W]'); title('End-of-mission (shutdown/constraints)');

%% Tracking error + unmet
figure('Name','Robustness: Tracking error / Unmet power');

subplot(2,1,1);
err_perf = log_perf.Pdem - log_perf.Pdel;
err_miss = log_miss.Pdem - log_miss.Pdel;
plot(t, err_perf, 'r-',  'LineWidth', 1.4); hold on;
plot(t, err_miss, 'b--', 'LineWidth', 1.4);
grid on; ylabel('P_{dem}-P_{del} [W]');
title('Tracking error (shows lag during unexpected spikes)');
legend('Perfect','Mismatch','Location','Best');

subplot(2,1,2);
plot(t, log_perf.Punmet, 'r-',  'LineWidth', 1.4); hold on;
plot(t, log_miss.Punmet, 'b--', 'LineWidth', 1.4);
grid on; xlabel('t [s]'); ylabel('Unmet power [W]');
title('Unmet power');
legend('Perfect','Mismatch','Location','Best');

%% Constraints: V and T
Vmin = 9.6; Vmax = 12.6;
if isfield(par,'V_min'), Vmin = par.V_min; end
if isfield(par,'V_max'), Vmax = par.V_max; end
TmaxC = 60;
if isfield(par,'T_max'), TmaxC = par.T_max - 273.15; end

figure('Name','Robustness: Constraints');

subplot(2,1,1);
plot(t, log_perf.Vt, 'r-',  'LineWidth', 1.4); hold on;
plot(t, log_miss.Vt, 'b--', 'LineWidth', 1.4);
yline(Vmin,'k--','LineWidth',1.0);
yline(Vmax,'k--','LineWidth',1.0);
grid on; ylabel('Vt [V]'); title('Terminal voltage response');
legend('Perfect MPC','Mismatch MPC','Vmin','Vmax','Location','Best');

subplot(2,1,2);
plot(t, log_perf.T-273.15, 'r-',  'LineWidth', 1.4); hold on;
plot(t, log_miss.T-273.15, 'b--', 'LineWidth', 1.4);
yline(TmaxC,'k--','LineWidth',1.0);
grid on; xlabel('t [s]'); ylabel('T [C]'); title('Temperature response');
legend('Perfect MPC','Mismatch MPC','Tmax','Location','Best');
end

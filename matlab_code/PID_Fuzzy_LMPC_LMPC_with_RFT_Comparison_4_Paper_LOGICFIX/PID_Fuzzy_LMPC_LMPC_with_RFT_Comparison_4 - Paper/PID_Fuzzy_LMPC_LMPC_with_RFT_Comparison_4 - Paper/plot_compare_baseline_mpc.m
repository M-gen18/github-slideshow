function plot_compare_baseline_mpc(log_base, log_mpc, par, meta)

% Power

% ---- Align lengths (simulations may stop early at SoC_min) ----
minLen = min([numel(log_base.t), numel(log_mpc.t)]);
if nargin >= 4 && isstruct(meta)
    if isfield(meta,'Pshaft_W'), minLen = min(minLen, numel(meta.Pshaft_W)); end
    if isfield(meta,'eta_drv'),  minLen = min(minLen, numel(meta.eta_drv));  end
end
tb = log_base.t(1:minLen);
figure('Name','Power: Baseline vs MPC');
plot(tb, log_base.Pdem(1:minLen), 'k', 'LineWidth',1.0); hold on;
plot(tb, log_base.Pdel(1:minLen), 'r', 'LineWidth',1.0);
plot(tb, log_mpc.Pdel(1:minLen),  'b', 'LineWidth',1.0);
grid on; xlabel('t [s]'); ylabel('Power [W]');
legend('Pdem','Pdel baseline','Pdel MPC');
title('Power tracking comparison');

% SoC
figure('Name','SoC: Baseline vs MPC');
plot(tb, log_base.SoC(1:minLen), 'r', 'LineWidth',1.0); hold on;
plot(tb, log_mpc.SoC(1:minLen),  'b', 'LineWidth',1.0);
yline(par.SoC_min,'--');
grid on; xlabel('t [s]'); ylabel('SoC [-]');
legend('Baseline','MPC','SoCmin');
title('SoC comparison');

% Current
figure('Name','Current: Baseline vs MPC');
plot(tb, log_base.I(1:minLen), 'r', 'LineWidth',1.0); hold on;
plot(tb, log_mpc.I(1:minLen),  'b', 'LineWidth',1.0);
yline(par.I_max,'--'); yline(par.I_min,'--');
grid on; xlabel('t [s]'); ylabel('I [A]');
legend('Baseline','MPC','Imax','Imin');

% Voltage
figure('Name','Voltage: Baseline vs MPC');
plot(tb, log_base.Vt(1:minLen), 'r', 'LineWidth',1.0); hold on;
plot(tb, log_mpc.Vt(1:minLen),  'b', 'LineWidth',1.0);
yline(par.Vmin_limit,'--'); yline(par.Vmax_limit,'--');
grid on; xlabel('t [s]'); ylabel('Vt [V]');
legend('Baseline','MPC','Vmin','Vmax');

% Temperature
figure('Name','Temperature: Baseline vs MPC');
plot(log_base.t, log_base.T-273.15, 'r', 'LineWidth',1.0); hold on;
plot(log_mpc.t,  log_mpc.T-273.15,  'b', 'LineWidth',1.0);
yline(par.Tmax-273.15,'--');
grid on; xlabel('t [s]'); ylabel('T [C]');
legend('Baseline','MPC','Tmax');

% Mission
if nargin>3 && isfield(meta,'Pshaft_W')
    figure('Name','Mission: Shaft power & eta');
    yyaxis left; plot(tb, meta.Pshaft_W(1:minLen),'LineWidth',1.0); ylabel('Pshaft [W]');
    yyaxis right; plot(tb, meta.eta_drv(1:minLen),'LineWidth',1.0); ylabel('eta');
    grid on; xlabel('t [s]');
end
end

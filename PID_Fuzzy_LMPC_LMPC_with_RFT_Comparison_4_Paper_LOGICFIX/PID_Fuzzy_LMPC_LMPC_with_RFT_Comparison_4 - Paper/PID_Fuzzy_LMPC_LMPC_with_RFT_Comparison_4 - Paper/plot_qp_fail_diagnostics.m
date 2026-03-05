function plot_qp_fail_diagnostics(log_mpc, par)
%PLOT_QP_FAIL_DIAGNOSTICS Visualize where QPs failed and constraint margins.
if ~isfield(log_mpc,'qp_fail')
    disp('No qp_fail log found.');
    return;
end
idx = find(log_mpc.qp_fail);
if isempty(idx)
    disp('QP fail rate is 0%.');
    return;
end

t = log_mpc.t(idx);

figure('Name','QP failure diagnostics');
tiledlayout(4,1,'Padding','compact','TileSpacing','compact');

nexttile;
stem(t, log_mpc.qp_exitflag(idx),'filled'); grid on;
ylabel('exitflag'); title(sprintf('QP fails: %d (%.2f%%)', numel(idx), 100*mean(log_mpc.qp_fail)));

nexttile;
plot(log_mpc.t, log_mpc.margin_soc,'LineWidth',1.0); grid on;
yline(0,'--'); ylabel('SoC-SoC_{min}');
title('SoC margin');

nexttile;
plot(log_mpc.t, log_mpc.margin_v,'LineWidth',1.0); grid on;
yline(0,'--'); ylabel('V - V_{min} [V]');
title('Voltage margin');

nexttile;
plot(log_mpc.t, (log_mpc.margin_t),'LineWidth',1.0); grid on;
yline(0,'--'); ylabel('T_{max}-T [K]');
xlabel('t [s]'); title('Thermal margin');
end

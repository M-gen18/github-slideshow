function plot_mpc_prediction_debug(predlog, par)
%PLOT_MPC_PREDICTION_DEBUG Plot MPC predicted trajectories vs realized at selected times.
%
% predlog is produced by simulate_uav_with_mpc when cfg.enable_predlog=true.

if isempty(predlog) || ~isfield(predlog,'k')
    warning('No prediction log to plot.');
    return;
end

for idx = 1:numel(predlog.k)
    k = predlog.k(idx);
    t0 = predlog.t0(idx);
    tau = predlog.tau{idx};

    figure('Name',sprintf('MPC prediction @ k=%d (t=%.1fs)',k,t0));
    tiledlayout(2,2,'Padding','compact','TileSpacing','compact');

    nexttile;
    plot(tau, predlog.Pdem{idx}, 'k', 'LineWidth',1.0); hold on;
    plot(tau, predlog.Pdel_pred{idx}, 'b', 'LineWidth',1.2);
    yline(predlog.Pdel_real(idx),'r--','LineWidth',1.0);
    grid on; xlabel('t [s]'); ylabel('Power [W]');
    legend('Pdem forecast','Pdel predicted','Pdel realized (k)','Location','best');
    title('Power prediction');

    nexttile;
    plot(tau, predlog.I_pred{idx}, 'b', 'LineWidth',1.2); hold on;
    yline(predlog.I_real(idx),'r--','LineWidth',1.0);
    yline(par.I_max,'k--'); yline(par.I_min,'k--');
    grid on; xlabel('t [s]'); ylabel('I [A]');
    legend('I predicted','I applied','Imax/Imin','Location','best');
    title('Current plan');

    nexttile;
    plot(tau, predlog.SoC_pred{idx}, 'b', 'LineWidth',1.2); hold on;
    yline(predlog.SoC_real(idx),'r--','LineWidth',1.0);
    yline(par.SoC_min,'k--');
    grid on; xlabel('t [s]'); ylabel('SoC [-]');
    legend('SoC predicted','SoC at k','SoCmin','Location','best');
    title('SoC prediction');

    nexttile;
    plot(tau, predlog.Vt_pred{idx}, 'b', 'LineWidth',1.2); hold on;
    yline(predlog.Vt_real(idx),'r--','LineWidth',1.0);
    yline(par.Vmin_limit,'k--'); yline(par.Vmax_limit,'k--');
    grid on; xlabel('t [s]'); ylabel('Vt [V]');
    legend('Vt predicted','Vt at k','Vmin/Vmax','Location','best');
    title('Voltage prediction');

end
end

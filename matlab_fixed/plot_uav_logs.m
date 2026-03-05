function plot_uav_logs(log, par, meta)
t=log.t;

figure('Name','Power');
plot(t,log.Pdem,'LineWidth',1.0); hold on;
plot(t,log.Pdel,'LineWidth',1.0);
grid on; xlabel('t [s]'); ylabel('Power [W]');
legend('Pdem','Pdel');

figure('Name','SoC');
plot(t,log.SoC,'LineWidth',1.0); hold on;
yline(par.SoC_min,'--'); yline(par.SoC_max,'--');
grid on; xlabel('t [s]'); ylabel('SoC');

figure('Name','Voltage');
plot(t,log.Vt,'LineWidth',1.0); hold on;
yline(par.Vmin_limit,'--'); yline(par.Vmax_limit,'--');
grid on; xlabel('t [s]'); ylabel('Vt [V]');

figure('Name','Current');
plot(t,log.I,'LineWidth',1.0); hold on;
yline(par.I_max,'--'); yline(par.I_min,'--');
grid on; xlabel('t [s]'); ylabel('I [A]');

figure('Name','Temperature');
plot(t,log.T-273.15,'LineWidth',1.0); hold on;
yline(par.Tmax-273.15,'--');
grid on; xlabel('t [s]'); ylabel('T [C]');

figure('Name','Near-active constraints');
stairs(t,double(log.near_Imax),'LineWidth',1.0); hold on;
stairs(t,double(log.near_Vmin),'LineWidth',1.0);
stairs(t,double(log.near_Tmax),'LineWidth',1.0);
stairs(t,double(log.near_SoCmin),'LineWidth',1.0);
stairs(t,double(log.near_any),'k','LineWidth',1.5);
grid on; xlabel('t [s]'); ylabel('Near active (0/1)');
legend('Imax','Vmin','Tmax','SoCmin','Any');

if nargin>2 && isfield(meta,'Pshaft_W')
    figure('Name','Shaft power and efficiency');
    yyaxis left; plot(meta.t, meta.Pshaft_W,'LineWidth',1.0); ylabel('Pshaft [W]');
    yyaxis right; plot(meta.t, meta.eta_drv,'LineWidth',1.0); ylabel('eta');
    grid on; xlabel('t [s]');
end
end

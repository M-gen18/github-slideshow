function plot_uav_mission_profile(t, Pdem_W, meta)
%PLOT_UAV_MISSION_PROFILE Plot shaft power, battery demand power, and efficiency.

figure('Name','UAV mission profile');
tiledlayout(3,1,'Padding','compact','TileSpacing','compact');

nexttile;
plot(t, Pdem_W,'k','LineWidth',1.2); grid on;
ylabel('P_{dem} [W]');
title('Battery-side demand power');

nexttile;
plot(t, meta.Pshaft_W,'b','LineWidth',1.2); grid on;
ylabel('P_{shaft} [W]');
title('Shaft power (propulsive)');

nexttile;
plot(t, meta.eta_drv,'r','LineWidth',1.2); grid on;
ylabel('\eta'); xlabel('t [s]');
title('Drivetrain efficiency');
end

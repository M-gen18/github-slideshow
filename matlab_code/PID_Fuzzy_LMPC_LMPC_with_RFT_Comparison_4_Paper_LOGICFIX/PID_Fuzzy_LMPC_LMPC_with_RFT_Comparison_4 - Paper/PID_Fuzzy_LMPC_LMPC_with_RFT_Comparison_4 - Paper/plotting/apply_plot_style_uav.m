function apply_plot_style_uav()
%APPLY_PLOT_STYLE_UAV Set consistent defaults for UAV EMS/MPC plots.
    set(groot,'defaultLineLineWidth',1.4);
    set(groot,'defaultAxesFontSize',11);
    set(groot,'defaultAxesGridAlpha',0.15);
    set(groot,'defaultAxesXGrid','on');
    set(groot,'defaultAxesYGrid','on');
    set(groot,'defaultLegendBox','on');
    set(groot,'defaultLegendInterpreter','tex');
end

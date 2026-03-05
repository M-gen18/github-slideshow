
function uav_style(fig)
if nargin<1 || isempty(fig), fig=gcf; end
set(fig,'Color','w','MenuBar','none','ToolBar','none');
set(0,'DefaultLineLineWidth',1.8);
set(0,'DefaultAxesFontSize',11);
set(0,'DefaultAxesLineWidth',1.0);
set(0,'DefaultAxesBox','on');
set(0,'DefaultTextInterpreter','latex');
set(0,'DefaultAxesTickLabelInterpreter','latex');
set(0,'DefaultLegendInterpreter','latex');
set(0,'DefaultLineLineWidth',1.6);
set(0,'DefaultAxesFontSize',9);
end

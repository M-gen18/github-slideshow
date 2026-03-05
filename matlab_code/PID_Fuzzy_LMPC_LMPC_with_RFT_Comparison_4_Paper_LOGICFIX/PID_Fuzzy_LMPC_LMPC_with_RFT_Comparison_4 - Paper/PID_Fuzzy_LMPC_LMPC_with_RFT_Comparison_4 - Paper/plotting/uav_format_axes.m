
function uav_format_axes(ax, small)
if nargin<1 || isempty(ax), ax = gca; end
if nargin<2, small = false; end
set(ax, 'FontName','Times New Roman');
set(ax, 'LineWidth',1.0);
if small, set(ax,'FontSize',9); else, set(ax,'FontSize',12); end
grid(ax,'on'); box(ax,'on');
end

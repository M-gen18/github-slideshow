
function uav_export_fig(fig, outpath_noext)
if nargin<1 || isempty(fig), fig=gcf; end
if nargin<2 || isempty(outpath_noext), outpath_noext = fullfile(pwd,'figure'); end
set(fig,'Color','w','MenuBar','none','ToolBar','none');
drawnow;
try, print(fig, [outpath_noext '.pdf'], '-dpdf', '-painters'); catch, end
try, print(fig, [outpath_noext '.png'], '-dpng', '-r300'); catch, end
end

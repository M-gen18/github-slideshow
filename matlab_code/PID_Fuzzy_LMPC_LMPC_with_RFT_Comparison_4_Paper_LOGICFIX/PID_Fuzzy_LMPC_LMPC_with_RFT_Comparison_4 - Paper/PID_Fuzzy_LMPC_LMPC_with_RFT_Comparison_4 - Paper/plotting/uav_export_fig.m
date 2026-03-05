function uav_export_fig(fig, outBase)
% Export to PDF (vector) and PNG (300 dpi) robustly without toolbar artifacts.
if nargin < 1 || isempty(fig), fig = gcf; end
if nargin < 2 || isempty(outBase), outBase = fullfile(pwd,'figure'); end
[folder,~,~] = fileparts(outBase);
if ~isempty(folder) && ~exist(folder,'dir'), mkdir(folder); end

plotting.uav_plot_style(fig); % ensure style

set(fig, 'PaperPositionMode','auto');
print(fig, [outBase '.pdf'], '-dpdf', '-painters');
print(fig, [outBase '.png'], '-dpng', '-r300');
end

function uav_plot_style(fig)
% Publication-friendly defaults (removes toolbars from exports)
if nargin < 1 || isempty(fig), fig = gcf; end
set(fig, 'Color','w');
try, set(fig,'Toolbar','none'); end %#ok<TRYNC>
try, set(fig,'MenuBar','none'); end %#ok<TRYNC>
try
    ax = findall(fig,'Type','axes');
    for k = 1:numel(ax)
        set(ax(k), 'FontName','Times New Roman', 'FontSize',11, ...
            'LineWidth',1.0, 'Box','on');
        grid(ax(k),'on'); ax(k).GridAlpha = 0.15;
        % Disable axes toolbar if present (R2018b+)
        if isprop(ax(k),'Toolbar') && ~isempty(ax(k).Toolbar)
            ax(k).Toolbar.Visible = 'off';
        end
    end
end
end

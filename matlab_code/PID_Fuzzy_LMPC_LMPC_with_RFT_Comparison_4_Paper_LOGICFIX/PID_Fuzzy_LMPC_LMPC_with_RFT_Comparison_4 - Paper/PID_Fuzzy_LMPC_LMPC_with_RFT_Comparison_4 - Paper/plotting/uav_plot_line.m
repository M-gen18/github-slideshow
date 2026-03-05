
function h = uav_plot_line(t, y, st, name)
h = [];
if isempty(t) || isempty(y), return; end
t = t(:); y = y(:);
n = min(numel(t), numel(y));
t = t(1:n); y = y(1:n);

% main line
h = plot(t, y, 'LineStyle', st.LineStyle, 'LineWidth', st.LineWidth, ...
    'Color', st.Color, 'DisplayName', name);

% overlay sparse markers (simple & robust)
if isfield(st,'Marker') && ~strcmpi(st.Marker,'none')
    step = max(1, floor(n/25));
    idx = unique([1:step:n, n]);
    plot(t(idx), y(idx), 'LineStyle','none', 'Marker', st.Marker, ...
        'MarkerSize', st.MarkerSize, 'MarkerEdgeColor', st.Color, ...
        'MarkerFaceColor', st.MarkerFaceColor, 'HandleVisibility','off');
end
end

function h = plot_with_markers_sparse(x, y, lineStyle, markerStyle, nMarkers, lineWidth, displayName, varargin)
%PLOT_WITH_MARKERS_SPARSE Plot a line plus sparse markers for readability.
%   h = plot_with_markers_sparse(x,y,'-','o',120,1.5,'name')
%
% This avoids MarkerIndices (for older MATLAB compatibility) by overlaying a
% marker-only plot at sparse indices.
%
% Inputs:
%   x,y           vectors (numeric)
%   lineStyle     e.g. '-', '--'
%   markerStyle   e.g. 'o','s','^'
%   nMarkers      approximate number of markers to draw across the curve
%   lineWidth     line width
%   displayName   legend name for the LINE (markers are hidden from legend)
%   varargin      passed to line plot (e.g. 'Color',...)
%
% Returns:
%   h             line handle

    if nargin < 7
        error('plot_with_markers_sparse:NotEnoughInputs','Not enough inputs.');
    end
    if isempty(x) || isempty(y)
        h = gobjects(1);
        return;
    end

    x = x(:); y = y(:);
    n = min(numel(x), numel(y));
    x = x(1:n); y = y(1:n);

    % Line
    h = plot(x, y, lineStyle, 'LineWidth', lineWidth, 'DisplayName', displayName, varargin{:});
    hold on;

    % Sparse markers overlay (hidden from legend)
    nMarkers = max(3, round(nMarkers));
    step = max(1, floor(n / nMarkers));
    idx = 1:step:n;
    hm = plot(x(idx), y(idx), markerStyle, 'LineStyle', 'none', ...
              'HandleVisibility','off', 'MarkerSize', 5, varargin{:});
    %#ok<NASGU>
end

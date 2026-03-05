function varargout = uav_align_by_minlen(varargin)
% Align vectors to the minimum common length (ignore empties).
lens = [];
for i = 1:nargin
    v = varargin{i};
    if isnumeric(v) && ~isempty(v)
        lens(end+1) = numel(v); %#ok<AGROW>
    end
end
if isempty(lens)
    varargout = varargin;
    return;
end
n = min(lens);
varargout = cell(1,nargin);
for i = 1:nargin
    v = varargin{i};
    if isnumeric(v) && ~isempty(v) && numel(v) >= n
        varargout{i} = v(1:n);
    else
        varargout{i} = v;
    end
end
end

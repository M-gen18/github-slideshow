
function varargout = uav_align_minlen(varargin)
lens = [];
for i=1:nargin
    v = varargin{i};
    if ~isempty(v) && isnumeric(v)
        lens(end+1) = numel(v); %#ok<AGROW>
    end
end
if isempty(lens)
    varargout = varargin; return;
end
L = min(lens);
varargout = varargin;
for i=1:nargin
    v = varargin{i};
    if ~isempty(v) && isnumeric(v)
        varargout{i} = v(1:L);
    end
end
end

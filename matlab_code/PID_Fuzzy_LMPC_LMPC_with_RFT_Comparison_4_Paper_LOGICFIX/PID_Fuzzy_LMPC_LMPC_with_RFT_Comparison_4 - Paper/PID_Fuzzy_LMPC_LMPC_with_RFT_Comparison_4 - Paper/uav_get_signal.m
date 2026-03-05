
function y = uav_get_signal(s, names)
y = [];
if nargin < 2 || isempty(names) || ~isstruct(s), return; end
for i = 1:numel(names)
    nm = names{i};
    if isfield(s, nm)
        v = s.(nm);
        if isnumeric(v)
            y = v(:); return;
        elseif isstruct(v) && isfield(v,'data') && isnumeric(v.data)
            y = v.data(:); return;
        end
    end
end
end

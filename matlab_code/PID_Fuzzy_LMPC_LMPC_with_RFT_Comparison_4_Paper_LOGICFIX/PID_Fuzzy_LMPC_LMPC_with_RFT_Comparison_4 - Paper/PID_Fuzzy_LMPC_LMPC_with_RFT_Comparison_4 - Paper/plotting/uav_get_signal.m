function y = uav_get_signal(S, fieldList)
% Return first available numeric vector among candidate fields.
y = [];
if nargin < 2, fieldList = {}; end
if isempty(S), return; end
for i = 1:numel(fieldList)
    f = fieldList{i};
    if isfield(S, f)
        v = S.(f);
        if isnumeric(v) && ~isempty(v)
            y = v(:);
            return;
        end
    end
end
end


function v = uav_get_scalar(S, names, def)
v = def;
if isempty(S) || ~isstruct(S), return; end
for i=1:numel(names)
    f = names{i};
    if isfield(S,f) && ~isempty(S.(f))
        vv = S.(f);
        if isnumeric(vv) && isscalar(vv)
            v = double(vv);
            return;
        end
    end
end
end

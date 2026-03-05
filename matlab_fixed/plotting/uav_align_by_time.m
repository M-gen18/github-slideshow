
function [t_out, a, b, c, d] = uav_align_by_time(t_ref, a, b, c, d, t_b, t_c)
t_out = t_ref(:);
if isempty(t_out)
    if nargin>=6 && ~isempty(t_b), t_out = t_b(:);
    elseif nargin>=7 && ~isempty(t_c), t_out = t_c(:);
    else, t_out = []; end
end

vecs = {t_out,a,b,c,d};
lens = [];
for i=1:numel(vecs)
    if ~isempty(vecs{i}) && isnumeric(vecs{i})
        lens(end+1) = numel(vecs{i}); %#ok<AGROW>
    end
end
if isempty(lens), return; end
n = min(lens);

t_out = trunc(t_out,n);
a = trunc(a,n);
b = trunc(b,n);
c = trunc(c,n);
d = trunc(d,n);
end

function x = trunc(x,n)
if isempty(x) || ~isnumeric(x), x = []; return; end
x = x(:);
x = x(1:min(n,numel(x)));
end

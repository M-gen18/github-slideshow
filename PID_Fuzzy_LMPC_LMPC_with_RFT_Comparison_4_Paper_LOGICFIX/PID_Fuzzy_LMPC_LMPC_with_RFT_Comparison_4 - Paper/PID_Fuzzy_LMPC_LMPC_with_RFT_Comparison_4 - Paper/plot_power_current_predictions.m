function plot_power_current_predictions(t, Pdem_mission, log_lmpc, log_nmpc, pred_lmpc, pred_nmpc, par)

if nargin < 7, par = struct(); end
if ~isnumeric(t) || numel(t)==0, return; end
t = t(:);

Pdel_l = getfield_def(log_lmpc,'Pdel',[]);
Pdel_n = getfield_def(log_nmpc,'Pdel',[]);
I_l    = getfield_def(log_lmpc,'I',getfield_def(log_lmpc,'Ibat',[]));
I_n    = getfield_def(log_nmpc,'I',getfield_def(log_nmpc,'Ibat',[]));

[t, Pdem_mission, Pdel_l, Pdel_n] = align_by_minlen(t, Pdem_mission, Pdel_l, Pdel_n);
[~, I_l, I_n] = align_by_minlen(t, I_l, I_n);

apply_plot_style_uav_if_available();

%% -------- POWER --------
figure('Name','Mission demand vs delivered power');
hold on; grid on;

if numel(Pdem_mission)>0
    plot(t, Pdem_mission,'k-','LineWidth',2.0,'DisplayName','P_{dem} mission');
end
if numel(Pdel_l)>0
    plot(t, Pdel_l,'--','LineWidth',1.8,'DisplayName','Pdel LMPC');
end
if numel(Pdel_n)>0
    plot(t, Pdel_n,'-','LineWidth',1.8,'DisplayName','Pdel NMPC');
end

plot_pred_horizons(pred_lmpc,'--');
plot_pred_horizons(pred_nmpc,'-');

xlabel('t [s]');
ylabel('Power [W]');
title('Delivered power with predicted horizons');
legend('Location','best');

%% -------- CURRENT --------
figure('Name','Delivered current');
hold on; grid on;

if isfield(par,'I_max')
    yline(par.I_max,'k--','HandleVisibility','off');
end
if isfield(par,'I_min')
    yline(par.I_min,'k-.','HandleVisibility','off');
end

if numel(I_l)>0
    plot(t, I_l,'--','LineWidth',1.8,'DisplayName','I LMPC');
end
if numel(I_n)>0
    plot(t, I_n,'-','LineWidth',1.8,'DisplayName','I NMPC');
end

plot_pred_horizons_current(pred_lmpc,'--');
plot_pred_horizons_current(pred_nmpc,'-');

xlabel('t [s]');
ylabel('|I| [A]');
title('Delivered current with predicted horizons');
legend('Location','best');

end

% =========================================================
% ==================== HELPERS =============================
% =========================================================

function v = getfield_def(s, f, def)
v = def;
if ~isstruct(s), return; end
if ~isfield(s,f), return; end
tmp = s.(f);
if numel(tmp)==0, return; end
v = tmp;
end

function varargout = align_by_minlen(varargin)

L = inf;
for i=1:nargin
    a = varargin{i};
    if isnumeric(a) && numel(a)>0
        L = min(L, numel(a));
    end
end

if isinf(L), L = 0; end

varargout = cell(1,nargin);

for i=1:nargin
    a = varargin{i};
    if isnumeric(a) && numel(a)>0
        a = a(:);
        varargout{i} = a(1:L);
    else
        varargout{i} = [];
    end
end

end

function apply_plot_style_uav_if_available()
if exist('apply_plot_style_uav','file')==2
    try
        apply_plot_style_uav();
    catch
    end
end
end

function plot_pred_horizons(pred, ls)

if ~isstruct(pred), return; end
if ~isfield(pred,'snapshots'), return; end

snaps = pred.snapshots;
if numel(snaps)==0, return; end

M = numel(snaps);
idx = unique(round(linspace(1,M,min(6,M))));

for ii = idx
    s = snaps(ii);
    if isfield(s,'t_pred') && isfield(s,'Pdel_pred')
        tp = s.t_pred(:);
        pp = s.Pdel_pred(:);
        n = min(numel(tp), numel(pp));
        if n>1
            plot(tp(1:n), pp(1:n), ls, 'LineWidth', 0.8,'HandleVisibility','off');
        end
    end
end

end

function plot_pred_horizons_current(pred, ls)

if ~isstruct(pred), return; end
if ~isfield(pred,'snapshots'), return; end

snaps = pred.snapshots;
if numel(snaps)==0, return; end

M = numel(snaps);
idx = unique(round(linspace(1,M,min(6,M))));

for ii = idx
    s = snaps(ii);
    if isfield(s,'t_pred') && isfield(s,'I_pred')
        tp = s.t_pred(:);
        ip = s.I_pred(:);
        n = min(numel(tp), numel(ip));
        if n>1
            plot(tp(1:n), ip(1:n), ls, 'LineWidth', 0.8,'HandleVisibility','off');
        end
    end
end

end
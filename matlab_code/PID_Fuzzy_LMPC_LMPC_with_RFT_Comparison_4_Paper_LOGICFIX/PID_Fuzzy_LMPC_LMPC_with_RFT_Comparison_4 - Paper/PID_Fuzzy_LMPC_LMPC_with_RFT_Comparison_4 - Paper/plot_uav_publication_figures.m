
function metrics = plot_uav_publication_figures(log_base, log_lmpc, log_nmpc, par, meta, cfg)
if nargin<6, cfg=struct(); end
export_dir = get_def(cfg,'export_dir', fullfile(pwd,'fig_pub'));
do_export  = get_def(cfg,'export', true);
show_pred  = get_def(cfg,'show_pred', true);
zoom_window_s = get_def(cfg,'zoom_window_s', 220);
n_pred_snap = get_def(cfg,'n_pred_snap', 8);
if ~exist(export_dir,'dir'), mkdir(export_dir); end

tB = uav_get_signal(log_base, {'t','time'});
tL = uav_get_signal(log_lmpc, {'t','time'});
tN = uav_get_signal(log_nmpc, {'t','time'});
t = first_nonempty(tB,tL,tN);

Pdem = [];
if nargin>=5 && isstruct(meta)
    Pdem = uav_get_signal(meta, {'Pdem_real','Pdem_mission','Pdem','Pdem_W'});
end
if isempty(Pdem)
    Pdem = uav_get_signal(log_base, {'Pdem','Pdem_W','P_request'});
end

PdelB = uav_get_signal(log_base, {'Pdel','P_del','Pdel_W'});
PdelL = uav_get_signal(log_lmpc, {'Pdel','P_del','Pdel_W'});
PdelN = uav_get_signal(log_nmpc, {'Pdel','P_del','Pdel_W'});

IB = uav_get_signal(log_base, {'I','Ibatt','Ibat'});
IL = uav_get_signal(log_lmpc, {'I','Ibatt','Ibat'});
IN = uav_get_signal(log_nmpc, {'I','Ibatt','Ibat'});

VB = uav_get_signal(log_base, {'Vt','V','Vterm'});
VL = uav_get_signal(log_lmpc, {'Vt','V','Vterm'});
VN = uav_get_signal(log_nmpc, {'Vt','V','Vterm'});

SoCB = uav_get_signal(log_base, {'SoC','soc'});
SoCL = uav_get_signal(log_lmpc, {'SoC','soc'});
SoCN = uav_get_signal(log_nmpc, {'SoC','soc'});

TB = uav_get_signal(log_base, {'T','Temp','Tcell'});
TL = uav_get_signal(log_lmpc, {'T','Temp','Tcell'});
TN = uav_get_signal(log_nmpc, {'T','Temp','Tcell'});

SoHB = uav_get_signal(log_base, {'SoH','soh'});
SoHL = uav_get_signal(log_lmpc, {'SoH','soh'});
SoHN = uav_get_signal(log_nmpc, {'SoH','soh'});

[t, Pdem, PdelB, PdelL, PdelN] = uav_align_minlen(t, Pdem, PdelB, PdelL, PdelN);
[~, IB, IL, IN] = uav_align_minlen(t, IB, IL, IN);
[~, VB, VL, VN] = uav_align_minlen(t, VB, VL, VN);
[~, SoCB, SoCL, SoCN] = uav_align_minlen(t, SoCB, SoCL, SoCN);
[~, TB, TL, TN] = uav_align_minlen(t, TB, TL, TN);
[~, SoHB, SoHL, SoHN] = uav_align_minlen(t, SoHB, SoHL, SoHN);

%% Power tracking (clean)
fig1 = figure('Name','Power tracking (paper)');
uav_style(fig1); hold on; grid on;
plot(t, Pdem, 'k-', 'DisplayName','P_{dem} (mission)');
plot(t, PdelB,'-','DisplayName','P_{del} baseline');
plot(t, PdelL,'--','DisplayName','P_{del} LMPC');
plot(t, PdelN,'-','DisplayName','P_{del} NMPC');
xlabel('t [s]'); ylabel('Power [W]');
title('Power tracking comparison');
legend('Location','southoutside','Orientation','horizontal');
if do_export, uav_export_fig(fig1, fullfile(export_dir,'fig_power_tracking')); end

% Zoom figure separate
dB = Pdem - PdelB; dL = Pdem - PdelL; dN = Pdem - PdelN;
[~, k0] = max(max([abs(dB), abs(dL), abs(dN)],[],2));
t0 = t(k0);
mask = (t >= t0-zoom_window_s/2) & (t <= t0+zoom_window_s/2);

fig1z = figure('Name','Power tracking (zoom)');
uav_style(fig1z); hold on; grid on;
plot(t(mask), Pdem(mask), 'k-', 'DisplayName','P_{dem} (mission)');
plot(t(mask), PdelB(mask),'-','DisplayName','baseline');
plot(t(mask), PdelL(mask),'--','DisplayName','LMPC');
plot(t(mask), PdelN(mask),'-','DisplayName','NMPC');
xlabel('t [s]'); ylabel('Power [W]');
title(sprintf('Zoom around t=%.0f s', t0));
legend('Location','best');
if do_export, uav_export_fig(fig1z, fullfile(export_dir,'fig_power_tracking_zoom')); end

%% Error + unmet
fig2 = figure('Name','Tracking error and unmet energy');
uav_style(fig2);
tiledlayout(2,1,'TileSpacing','compact','Padding','compact');
nexttile; hold on; grid on;
plot(t, dB, '-', 'DisplayName','\DeltaP baseline');
plot(t, dL, '--','DisplayName','\DeltaP LMPC');
plot(t, dN, '-', 'DisplayName','\DeltaP NMPC');
yline(0,'k:','HandleVisibility','off');
ylabel('P_{dem}-P_{del} [W]');
title('Tracking error (positive = unmet power)');
legend('Location','southoutside','Orientation','horizontal');

nexttile; hold on; grid on;
unmetB = cumtrapz(t, max(dB,0))/3600;
unmetL = cumtrapz(t, max(dL,0))/3600;
unmetN = cumtrapz(t, max(dN,0))/3600;
plot(t, unmetB, '-', 'DisplayName','Unmet baseline');
plot(t, unmetL, '--','DisplayName','Unmet LMPC');
plot(t, unmetN, '-', 'DisplayName','Unmet NMPC');
xlabel('t [s]'); ylabel('Unmet energy [Wh]');
title('Cumulative unmet energy');
legend('Location','southoutside','Orientation','horizontal');
if do_export, uav_export_fig(fig2, fullfile(export_dir,'fig_error_unmet')); end

%% Current
fig3 = figure('Name','Current comparison');
uav_style(fig3); hold on; grid on;
plot(t, IB, '-', 'DisplayName','I baseline');
plot(t, IL, '--','DisplayName','I LMPC');
plot(t, IN, '-', 'DisplayName','I NMPC');
xlabel('t [s]'); ylabel('|I| [A]');
title('Delivered current comparison');
Imax = get_par(par, {'I_max','Imax'}, []);
Imin = get_par(par, {'I_min','Imin'}, []);
if ~isempty(Imax), yline(Imax,'k--','HandleVisibility','off'); end
if ~isempty(Imin), yline(Imin,'k:','HandleVisibility','off'); end
legend('Location','southoutside','Orientation','horizontal');
if do_export, uav_export_fig(fig3, fullfile(export_dir,'fig_current')); end

%% Voltage
fig4 = figure('Name','Voltage comparison');
uav_style(fig4); hold on; grid on;
plot(t, VB, '-', 'DisplayName','V_t baseline');
plot(t, VL, '--','DisplayName','V_t LMPC');
plot(t, VN, '-', 'DisplayName','V_t NMPC');
xlabel('t [s]'); ylabel('V_t [V]');
title('Terminal voltage comparison');
Vmin = get_par(par, {'Vmin_limit','Vmin','V_min'}, []);
Vmax = get_par(par, {'Vmax_limit','Vmax','V_max'}, []);
if ~isempty(Vmin), yline(Vmin,'k--','HandleVisibility','off'); end
if ~isempty(Vmax), yline(Vmax,'k--','HandleVisibility','off'); end
legend('Location','southoutside','Orientation','horizontal');
if do_export, uav_export_fig(fig4, fullfile(export_dir,'fig_voltage')); end

%% SoC
fig5 = figure('Name','SoC comparison');
uav_style(fig5); hold on; grid on;
plot(t, SoCB, '-', 'DisplayName','SoC baseline');
plot(t, SoCL, '--','DisplayName','SoC LMPC');
plot(t, SoCN, '-', 'DisplayName','SoC NMPC');
xlabel('t [s]'); ylabel('SoC [-]');
title('State of charge comparison');
SoCmin = get_par(par, {'SoC_min','SoCmin'}, []);
if ~isempty(SoCmin), yline(SoCmin,'k--','HandleVisibility','off'); end
legend('Location','southoutside','Orientation','horizontal');
if do_export, uav_export_fig(fig5, fullfile(export_dir,'fig_soc')); end

%% Temp optional
%% Temp optional (Kelvin internal -> Celsius for plotting)
if ~isempty(TB) || ~isempty(TL) || ~isempty(TN)
    fig6 = figure('Name','Temperature comparison');
    uav_style(fig6); hold on; grid on;

    % Detect if temperature looks like Kelvin (e.g., around 298..330)
    % If it already looks like Celsius (e.g., 20..80), don't convert.
    Tk_all = [TB(:); TL(:); TN(:)];
    Tk_all = Tk_all(~isnan(Tk_all) & isfinite(Tk_all));
    assume_kelvin = ~isempty(Tk_all) && median(Tk_all) > 150;

    if assume_kelvin
        if ~isempty(TB), TBp = TB - 273.15; else, TBp = []; end
        if ~isempty(TL), TLp = TL - 273.15; else, TLp = []; end
        if ~isempty(TN), TNp = TN - 273.15; else, TNp = []; end
        Tmax = get_par(par, {'T_max','Tmax'}, []);
        if ~isempty(Tmax), Tmaxp = Tmax - 273.15; else, Tmaxp = []; end
    else
        TBp = TB; TLp = TL; TNp = TN;
        Tmaxp = get_par(par, {'T_max','Tmax'}, []);
    end

    if ~isempty(TBp), plot(t, TBp,'-','DisplayName','T baseline'); end
    if ~isempty(TLp), plot(t, TLp,'--','DisplayName','T LMPC'); end
    if ~isempty(TNp), plot(t, TNp,'-','DisplayName','T NMPC'); end

    xlabel('t [s]');
    ylabel('T [^{\circ}C]');
    title('Temperature comparison');

    if ~isempty(Tmaxp)
        yline(Tmaxp,'k--','HandleVisibility','off');
    end

    legend('Location','southoutside','Orientation','horizontal');
    if do_export, uav_export_fig(fig6, fullfile(export_dir,'fig_temp')); end
end


%% SoH optional
if ~isempty(SoHB) || ~isempty(SoHL) || ~isempty(SoHN)
    fig7 = figure('Name','SoH comparison');
    uav_style(fig7); hold on; grid on;
    if ~isempty(SoHB), plot(t, SoHB,'-','DisplayName','SoH baseline'); end
    if ~isempty(SoHL), plot(t, SoHL,'--','DisplayName','SoH LMPC'); end
    if ~isempty(SoHN), plot(t, SoHN,'-','DisplayName','SoH NMPC'); end
    xlabel('t [s]'); ylabel('SoH [-]');
    title('State of health comparison');
    legend('Location','southoutside','Orientation','horizontal');
    if do_export, uav_export_fig(fig7, fullfile(export_dir,'fig_soh')); end
end

metrics = struct();
metrics.unmet_Wh_baseline = unmetB(end);
metrics.unmet_Wh_LMPC = unmetL(end);
metrics.unmet_Wh_NMPC = unmetN(end);

if show_pred
    try
        plot_pred_clean(log_lmpc, log_nmpc, t, Pdem, export_dir, do_export, n_pred_snap);
    catch
    end
end
end

function plot_pred_clean(log_l, log_n, t, Pdem, export_dir, do_export, n_snap)
predL = []; predN = [];
if isfield(log_l,'pred'), predL = log_l.pred; end
if isfield(log_n,'pred'), predN = log_n.pred; end
if isempty(predL) && isfield(log_l,'predlog'), predL = log_l.predlog; end
if isempty(predN) && isfield(log_n,'predlog'), predN = log_n.predlog; end

PdelL = uav_get_signal(log_l, {'Pdel','P_del','Pdel_W'});
PdelN = uav_get_signal(log_n, {'Pdel','P_del','Pdel_W'});
[t2, PdelL, PdelN] = uav_align_minlen(t, PdelL, PdelN);

figP = figure('Name','Predicted horizons: power');
uav_style(figP); hold on; grid on;
plot(t, Pdem, 'k-', 'DisplayName','P_{dem} (mission)');
plot(t2, PdelL,'--','DisplayName','P_{del} LMPC');
plot(t2, PdelN,'-','DisplayName','P_{del} NMPC');
plot_pred_bundle(predL, n_snap, '--', 'LMPC predicted horizon');
plot_pred_bundle(predN, n_snap, ':',  'NMPC predicted horizon');
xlabel('t [s]'); ylabel('Power [W]');
title('Delivered power with predicted horizons');
legend('Location','southoutside','Orientation','horizontal');
if do_export, uav_export_fig(figP, fullfile(export_dir,'fig_pred_power')); end

IL = uav_get_signal(log_l, {'I','Ibatt','Ibat'});
IN = uav_get_signal(log_n, {'I','Ibatt','Ibat'});
[t3, IL, IN] = uav_align_minlen(t, IL, IN);

figI = figure('Name','Predicted horizons: current');
uav_style(figI); hold on; grid on;
plot(t3, IL,'--','DisplayName','I LMPC');
plot(t3, IN,'-','DisplayName','I NMPC');
plot_pred_bundle_current(predL, n_snap, '--', 'LMPC predicted horizon');
plot_pred_bundle_current(predN, n_snap, ':',  'NMPC predicted horizon');
xlabel('t [s]'); ylabel('|I| [A]');
title('Delivered current with predicted horizons');
legend('Location','southoutside','Orientation','horizontal');
if do_export, uav_export_fig(figI, fullfile(export_dir,'fig_pred_current')); end
end

function plot_pred_bundle(pred, n_snap, ls, name)
if isempty(pred) || ~isstruct(pred), return; end
idx = round(linspace(1,numel(pred), min(n_snap,numel(pred))));
first=true;
for k = idx
    pk = pred(k);
    if isfield(pk,'t') && isfield(pk,'Pdel')
        tt = pk.t(:); yy = pk.Pdel(:);
        if first
            plot(tt, yy, ls, 'DisplayName', name);
            first=false;
        else
            plot(tt, yy, ls, 'HandleVisibility','off');
        end
    end
end
end

function plot_pred_bundle_current(pred, n_snap, ls, name)
if isempty(pred) || ~isstruct(pred), return; end
idx = round(linspace(1,numel(pred), min(n_snap,numel(pred))));
first=true;
for k = idx
    pk = pred(k);
    if isfield(pk,'t') && isfield(pk,'I')
        tt = pk.t(:); yy = pk.I(:);
        if first
            plot(tt, yy, ls, 'DisplayName', name);
            first=false;
        else
            plot(tt, yy, ls, 'HandleVisibility','off');
        end
    end
end
end

function v = get_def(s, f, def)
if isstruct(s) && isfield(s,f) && ~isempty(s.(f)), v = s.(f); else, v = def; end
end

function y = first_nonempty(varargin)
y = [];
for i=1:nargin
    if ~isempty(varargin{i}), y = varargin{i}(:); return; end
end
end

function v = get_par(par, names, def)
v = def;
if ~isstruct(par), return; end
for i=1:numel(names)
    nm = names{i};
    if isfield(par,nm) && ~isempty(par.(nm)), v = par.(nm); return; end
end
end

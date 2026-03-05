function plot_flight_error_proxy(t, Pdem_mission, Pdel_b, Pdel_l, Pdel_n, Ts)
%PLOT_FLIGHT_ERROR_PROXY Simple flight-error proxies from power deficit.
%   - relative deficit (%)
%   - cumulative unmet energy (Wh)
%   - Flight Deviation Index (FDI): low-pass filtered deficit integral

    if nargin < 6 || isempty(Ts), Ts = median(diff(t)); end
    t = t(:);

    [t, Pdem_mission, Pdel_b, Pdel_l, Pdel_n] = align_by_minlen(t, Pdem_mission, Pdel_b, Pdel_l, Pdel_n);

    if isempty(Pdem_mission)
        warning('No mission demand provided; cannot compute proxies.');
        return;
    end

    epsP = 1e-6;
    frac = @(Pdel) max(0, (Pdem_mission - Pdel) ./ max(Pdem_mission, epsP)); % unmet fraction

    f_b = []; f_l = []; f_n = [];
    if ~isempty(Pdel_b), f_b = frac(Pdel_b); end
    if ~isempty(Pdel_l), f_l = frac(Pdel_l); end
    if ~isempty(Pdel_n), f_n = frac(Pdel_n); end

    apply_plot_style_uav_if_available();

    % Relative deficit
    figure('Name','Flight error proxy: relative power deficit');
    hold on; grid on;
    if ~isempty(f_b), plot_with_markers_sparse(t, 100*f_b, '-',  'o', 80, 1.4, 'baseline'); end
    if ~isempty(f_l), plot_with_markers_sparse(t, 100*f_l, '--', 's', 80, 1.6, 'LMPC'); end
    if ~isempty(f_n), plot_with_markers_sparse(t, 100*f_n, '-',  '^', 80, 1.6, 'NMPC'); end
    xlabel('t [s]'); ylabel('Deficit [% of P_{dem}]');
    title('Relative power deficit (proxy for flight error)');
    legend('Location','best');

    % Cumulative unmet energy (Wh)
    Wh = @(f) cumsum(f .* Pdem_mission) * Ts / 3600;
    figure('Name','Flight error proxy: cumulative unmet energy');
    hold on; grid on;
    if ~isempty(f_b), plot_with_markers_sparse(t, Wh(f_b), '-',  'o', 80, 1.4, 'baseline'); end
    if ~isempty(f_l), plot_with_markers_sparse(t, Wh(f_l), '--', 's', 80, 1.6, 'LMPC'); end
    if ~isempty(f_n), plot_with_markers_sparse(t, Wh(f_n), '-',  '^', 80, 1.6, 'NMPC'); end
    xlabel('t [s]'); ylabel('Unmet energy [Wh]');
    title('Cumulative unmet energy');
    legend('Location','best');

    % FDI
    tau = 10; % seconds, low-pass
    a = Ts/(tau+Ts);
    lp = @(f) filter(a, [1 -(1-a)], f); % simple IIR
    FDI = @(f) cumsum(lp(f)) * Ts;

    figure('Name','Flight error proxy: FDI');
    hold on; grid on;
    if ~isempty(f_b), plot_with_markers_sparse(t, FDI(f_b), '-',  'o', 80, 1.4, 'baseline'); end
    if ~isempty(f_l), plot_with_markers_sparse(t, FDI(f_l), '--', 's', 80, 1.6, 'LMPC'); end
    if ~isempty(f_n), plot_with_markers_sparse(t, FDI(f_n), '-',  '^', 80, 1.6, 'NMPC'); end
    xlabel('t [s]'); ylabel('FDI [s]');
    title('Flight Deviation Index (filtered deficit integral)');
    legend('Location','best');
end

function varargout = align_by_minlen(varargin)
    L = inf;
    for i=1:nargin
        a = varargin{i};
        if isnumeric(a) && ~isempty(a), L = min(L, numel(a)); end
    end
    if isinf(L), L = 0; end
    varargout = cell(1,nargin);
    for i=1:nargin
        a = varargin{i};
        if isnumeric(a) && ~isempty(a)
            a = a(:); varargout{i} = a(1:L);
        else
            varargout{i} = [];
        end
    end
end

function apply_plot_style_uav_if_available()
    if exist('apply_plot_style_uav','file')==2
        try, apply_plot_style_uav(); catch, end
    end
end

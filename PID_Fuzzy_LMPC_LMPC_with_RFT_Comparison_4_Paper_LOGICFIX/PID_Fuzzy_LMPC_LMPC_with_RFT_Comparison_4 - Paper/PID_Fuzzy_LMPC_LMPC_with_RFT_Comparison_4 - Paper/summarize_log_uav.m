function S = summarize_log_uav(log, par, varargin)
p = inputParser;
p.addParameter('RFT_ACTIVE_TOL', 5e-3);
p.addParameter('RFT_VIOL_TOL', -1e-5);
p.addParameter('t_critical_start', []);
p.addParameter('t_critical_end', []);
p.parse(varargin{:});
tol = p.Results.RFT_ACTIVE_TOL;
viol_tol = p.Results.RFT_VIOL_TOL;
t1  = p.Results.t_critical_start;
t2  = p.Results.t_critical_end;

% robust dt
if numel(log.t) >= 2
    dt = median(diff(log.t));
else
    dt = NaN;
end

% unmet power series
Pun = zeros(numel(log.t),1);
if isfield(log,'Punmet') && ~isempty(log.Punmet)
    Pun = log.Punmet(:);
elseif isfield(log,'Pdem') && isfield(log,'Pdel')
    Pun = max(log.Pdem(:) - log.Pdel(:), 0);
end

S = struct();

% Basic mission metrics
S.Endurance_min = log.t(end)/60;

S.Unmet_kWh = sum(max(Pun,0))*dt/3600/1000;      % W*s -> Wh -> kWh
S.Mean_Punmet_W = mean(Pun);
S.Max_Punmet_W  = max(Pun);

% Optional tracking RMSE (W)
S.Tracking_RMSE_W = sqrt(mean((Pun).^2));

% Energy delivered (kWh) if available
if isfield(log,'Pdel') && ~isempty(log.Pdel)
    S.Delivered_kWh = sum(max(log.Pdel(:),0))*dt/3600/1000;
else
    S.Delivered_kWh = NaN;
end

% Battery state metrics
S.SoC_end = log.SoC(end);
S.SoC_min = min(log.SoC);

if isfield(log,'Vt') && ~isempty(log.Vt)
    S.Vmin_V = min(log.Vt);
else
    S.Vmin_V = NaN;
end

if isfield(log,'T') && ~isempty(log.T)
    S.Tmax = max(log.T);
else
    S.Tmax = NaN;
end

if isfield(par,'SoC_min')
    S.ReserveMargin_end = log.SoC(end) - par.SoC_min;
else
    S.ReserveMargin_end = NaN;
end

% =========================
% RFT metrics (sign-consistent)
% =========================
% Use passed parameters instead of hardcoded values
% tol = RFT_ACTIVE_TOL (binding/active detection threshold)
% viol_tol = RFT_VIOL_TOL (TRUE violation threshold)

% Prefer canonical field if exists
if isfield(log,'RFT_margin') && ~isempty(log.RFT_margin)
    m = log.RFT_margin(:);
elseif isfield(log,'RFT_slack') && ~isempty(log.RFT_slack)
    m = log.RFT_slack(:); % treated as margin for backward compatibility
else
    m = [];
end

if ~isempty(m)
    S.RFTmargin_min   = min(m);
    S.RFTmargin_mean  = mean(m);
    S.RFT_active_pct  = 100*mean(abs(m) <= tol);
    S.RFT_violation_pct = 100*mean(m < viol_tol);
else
    S.RFTmargin_min   = NaN;
    S.RFTmargin_mean  = NaN;
    S.RFT_active_pct  = NaN;
    S.RFT_violation_pct = NaN;
end
end

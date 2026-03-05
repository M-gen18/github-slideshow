function sched = auto_tune_rft_schedule_from_mc(tgrid, S_slack, t1, t2, opts)
%AUTO_TUNE_RFT_SCHEDULE_FROM_MC
% Learns RFT_hi / RFT_lo from Monte Carlo slack statistics.
% Idea:
%   - If constraint is rarely active in critical window -> tighten (increase RFT_hi)
%   - If constraint too active outside critical -> relax (decrease RFT_lo)

if nargin < 5 || isempty(opts), opts = struct(); end

tol              = getOpt(opts,'active_tol',5e-3);
targetCritPct    = getOpt(opts,'target_crit_active_pct',60);
maxOuterPct       = getOpt(opts,'max_outer_active_pct',40);
base_hi_hr        = getOpt(opts,'base_hi_hr',3.0);
base_lo_hr        = getOpt(opts,'base_lo_hr',1.5);
step_hr           = getOpt(opts,'step_hr',0.25);
min_lo_hr         = getOpt(opts,'min_lo_hr',1.0);
max_hi_hr         = getOpt(opts,'max_hi_hr',4.0);

idxEarly = (tgrid < t1);
idxCrit  = (tgrid >= t1) & (tgrid <= t2);
idxLate  = (tgrid > t2);

actEarly = activePct(S_slack(idxEarly,:), tol);
actCrit  = activePct(S_slack(idxCrit,:),  tol);
actLate  = activePct(S_slack(idxLate,:),  tol);

hi_hr = base_hi_hr;
lo_hr = base_lo_hr;

% ---- tune HI using critical activity
if ~isnan(actCrit)
    if actCrit < targetCritPct
        n = ceil((targetCritPct - actCrit)/10);
        hi_hr = hi_hr + n*step_hr;
    elseif actCrit > targetCritPct + 15
        n = ceil((actCrit - (targetCritPct+15))/10);
        hi_hr = hi_hr - n*step_hr;
    end
end

% clamp hi
hi_hr = min(max(hi_hr, lo_hr + step_hr), max_hi_hr);

% ---- tune LO using outer activity
outer = max([actEarly actLate]);
if ~isnan(outer) && outer > maxOuterPct
    n = ceil((outer - maxOuterPct)/15);
    lo_hr = lo_hr - n*step_hr;   % relax outside
end

lo_hr = max(lo_hr, min_lo_hr);
lo_hr = min(lo_hr, hi_hr - step_hr);

sched = struct();
sched.t1 = t1;
sched.t2 = t2;

% IMPORTANT: hi is strict, lo is relaxed
sched.RFT_hi_s = hi_hr*3600;
sched.RFT_lo_s = lo_hr*3600;

sched.stats = struct();
sched.stats.activeEarly_pct = actEarly;
sched.stats.activeCrit_pct  = actCrit;
sched.stats.activeLate_pct  = actLate;
sched.stats.tol             = tol;
sched.stats.targetCritPct   = targetCritPct;
sched.stats.maxOuterPct     = maxOuterPct;
sched.stats.hi_hr           = hi_hr;
sched.stats.lo_hr           = lo_hr;
end

%% helpers
function v = getOpt(s, name, def)
if isfield(s,name) && ~isempty(s.(name)), v = s.(name); else, v = def; end
end

function pct = activePct(Sphase, tol)
x = Sphase(:);
x = x(~isnan(x));
if isempty(x), pct = NaN; return; end
pct = 100*mean(abs(x) < tol | x < tol);
end
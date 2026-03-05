function [log, predlog] = simulate_uav_with_mpc(Pdem_W, t, Ts, par, x0, Tamb, mpc, cfg)

% 1) Defaults
if nargin < 8 || isempty(cfg), cfg = struct(); end
if ~isfield(cfg,'enable_predlog'),      cfg.enable_predlog = true;  end
if ~isfield(cfg,'predlog_stride_s'),    cfg.predlog_stride_s = 100; end
if ~isfield(cfg,'predlog_max'),         cfg.predlog_max = 12;       end
if ~isfield(cfg,'no_anticipation'),     cfg.no_anticipation = false;end
if ~isfield(cfg,'SoC_min_mpc_override'),cfg.SoC_min_mpc_override = []; end
if ~isfield(cfg,'quiet'),               cfg.quiet = false;          end

% 2) Inputs
N = mpc.N;
Nsim = numel(t);
Pdem_W = Pdem_W(:);
t = t(:);

% Tamb must be vectorized
if isscalar(Tamb)
    Tamb_vec = Tamb*ones(Nsim,1);
else
    Tamb_vec = Tamb(:);
end

if ~isfield(cfg,'Pdem_pred') || isempty(cfg.Pdem_pred)
    cfg.Pdem_pred = Pdem_W;
end
Pdem_pred = cfg.Pdem_pred(:);

Vmin_stop = par.Vmin;

% Apply SoC override only inside MPC optimization
par_mpc = par;
if ~isempty(cfg.SoC_min_mpc_override)
    par_mpc.SoC_min = cfg.SoC_min_mpc_override;
end

% 3) Log prealloc
log = struct();
log.t = t;
log.Pdem = Pdem_W;
log.Ppred0 = zeros(Nsim,1);
log.I   = zeros(Nsim,1);
log.Pdel= zeros(Nsim,1);
log.Vt  = zeros(Nsim,1);
log.SoC = zeros(Nsim,1);
log.T   = zeros(Nsim,1);
log.SoH = zeros(Nsim,1);
log.Punmet = zeros(Nsim,1);

log.qp_exitflag = zeros(Nsim,1);
log.qp_fail     = zeros(Nsim,1);
log.nlp_fail    = zeros(Nsim,1);

% RFT logs (always exist)
log.RFT_active       = zeros(Nsim,1);
log.RFT_rem_s        = nan(Nsim,1);
log.RFT_alpha        = nan(Nsim,1);
log.RFT_SoC_RHS      = nan(Nsim,1);
log.RFT_SoC_term_pred= nan(Nsim,1);
log.RFT_RHS          = nan(Nsim,1);
log.RFT_margin       = nan(Nsim,1);
log.RFT_slack        = nan(Nsim,1); % backward compat: equals margin
log.RFT_bind         = zeros(Nsim,1);
log.RFT_violate      = zeros(Nsim,1);

predlog = [];
pred_count = 0;
next_pred_t = t(1);

x = x0;
u_prev = 0;

if ~cfg.quiet
    fprintf('Starting MPC Simulation (No Scaling, High Accuracy)...\n');
end

% Tolerances (reviewer-grade)
RFT_ACTIVE_TOL = 1e-4;  % "near binding"
RFT_VIOL_TOL   = -1e-5; % true violation

for k = 1:Nsim
    Tamb_k = Tamb_vec(k);

    % Demand forecast
    idx_end = min(Nsim, k+N-1);
    Pfc = Pdem_pred(k:idx_end);
    if numel(Pfc) < N, Pfc(end+1:N) = Pfc(end); end
    log.Ppred0(k) = Pfc(1);

    % RFT scheduling / remaining requirement
    mpc.t_now = t(k);
    if isfield(mpc,'use_rft_constraint') && mpc.use_rft_constraint
        if ~isfield(mpc,'RFT_spec_s') || isempty(mpc.RFT_spec_s) || mpc.RFT_spec_s <= 0
            mpc.RFT_spec_s = t(end);
        end
        if isfield(mpc,'RFT_schedule') && ~isempty(mpc.RFT_schedule)
            sch = mpc.RFT_schedule;
            if t(k) >= sch.t1 && t(k) <= sch.t2
                mpc.RFT_spec_s = sch.RFT_hi_s;
            else
                mpc.RFT_spec_s = sch.RFT_lo_s;
            end
        end
        mpc.RFT_rem_s = max(mpc.RFT_spec_s - mpc.t_now, Ts);
        log.RFT_active(k) = 1;
        log.RFT_rem_s(k)  = mpc.RFT_rem_s;
    end

    % Solve MPC
    x_mpc = [x.SoC; x.V1; x.V2; x.T];
    sol = struct();
    [u0, sol] = mpc_solve_uav_qp(x_mpc, u_prev, Pfc, par_mpc, Tamb_k, mpc);

    % Solver status
    if isfield(sol,'exitflag')
        log.qp_exitflag(k) = sol.exitflag;
        log.qp_fail(k)     = (sol.exitflag <= 0);
        log.nlp_fail(k)    = log.qp_fail(k);
    else
        log.qp_exitflag(k) = 1;
        log.qp_fail(k)     = 0;
        log.nlp_fail(k)    = 0;
    end

    % Apply physical bounds
    Icmd = min(max(u0, 0), par.I_max);

    % --- RFT margin logging (unified) ---
    if isfield(mpc,'use_rft_constraint') && mpc.use_rft_constraint
        has_rft = isfield(sol,'qp') && isfield(sol.qp,'rft') && ...
                  isfield(sol,'U')  && ~isempty(sol.U) && ...
                  isfield(sol.qp.rft,'SoC_term_row') && ...
                  isfield(sol.qp.rft,'SoC_term_aff') && ...
                  isfield(sol.qp.rft,'RHS') && isfield(sol.qp.rft,'alpha');

        if has_rft
            log.RFT_alpha(k)   = sol.qp.rft.alpha;
            log.RFT_SoC_RHS(k) = sol.qp.rft.RHS;

            Uopt = sol.U(:);
            if numel(Uopt) < N
                Uopt(end+1:N) = Uopt(end);
            else
                Uopt = Uopt(1:N);
            end

            RHS_req = sol.qp.rft.RHS;
            SoC_term_pred = sol.qp.rft.SoC_term_aff + sol.qp.rft.SoC_term_row * Uopt;
            m = SoC_term_pred - RHS_req;

            log.RFT_RHS(k)           = RHS_req;
            log.RFT_SoC_term_pred(k) = SoC_term_pred;
            log.RFT_margin(k)        = m;
            log.RFT_slack(k)         = m; % compat

            log.RFT_bind(k)    = (abs(m) <= RFT_ACTIVE_TOL);
            log.RFT_violate(k) = (m < RFT_VIOL_TOL);

            if ~cfg.quiet && mod(round(t(k)),300)==0
                fprintf('[RFT] t=%.0fs | RFT_rem=%.0fs | alpha=%.4f | SoC=%.3f | RHS=%.3f | m=%.4g\n', ...
                    t(k), mpc.RFT_rem_s, log.RFT_alpha(k), x.SoC, RHS_req, m);
            end
        else
            if ~cfg.quiet && mod(round(t(k)),300)==0
                fprintf('[RFT] t=%.0fs | RFT_rem=%.0fs | alpha=n/a | SoC=%.3f | RHS=n/a\n', ...
                    t(k), mpc.RFT_rem_s, x.SoC);
            end
        end
    end

    % Plant step
    [xnext, y] = batt_step_2rc_thermal_soh_uav(x, Icmd, Ts, par, Tamb_k);

    log.I(k)   = Icmd;
    log.Vt(k)  = y.Vt;
    log.Pdel(k)= y.Pdel;
    log.SoC(k) = xnext.SoC;
    log.T(k)   = xnext.T;
    log.SoH(k) = xnext.SoH;

    log.Punmet(k) = max(Pdem_W(k) - y.Pdel, 0);

    if (y.Vt <= Vmin_stop) || (xnext.SoC <= par.SoC_min)
        log = trim_log_to_k(log, k);
        break;
    end

    % Pred snapshots
    if cfg.enable_predlog && (t(k) >= next_pred_t) && pred_count < cfg.predlog_max
        pred_count  = pred_count + 1;
        next_pred_t = t(k) + cfg.predlog_stride_s;
        pk = struct();
        pk.t0 = t(k);
        pk.t  = t(k) + (0:N-1)'*Ts;
        if isfield(sol,'U') && ~isempty(sol.U)
            pk.I = sol.U(1:N);
        end
        if isempty(predlog), predlog = pk; else, predlog(end+1) = pk; end %#ok<AGROW>
    end

    x = xnext;
    u_prev = Icmd;
end

% Summary
log.endurance_min = log.t(end)/60;
dt = median(diff(log.t));
unmet_Wh = sum(max(log.Punmet,0))*dt/3600;
log.unmet_kWh = unmet_Wh/1000;
end

function log = trim_log_to_k(log, k)
fn = fieldnames(log);
for i = 1:numel(fn)
    f = fn{i};
    v = log.(f);
    if isnumeric(v) || islogical(v)
        if isvector(v) && numel(v) >= k
            log.(f) = v(1:k);
        elseif ismatrix(v) && size(v,1) == k
            log.(f) = v(1:k,:);
        elseif ismatrix(v) && size(v,2) == k
            log.(f) = v(:,1:k);
        end
    end
end
if isfield(log,'t') && numel(log.t) >= k
    log.t = log.t(1:k);
end
end
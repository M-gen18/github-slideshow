function mpclib = uav_mpc_lib()
% UAV linear MPC + RFT constraint library.
mpclib.mpc_settings = @mpc_settings_uav;
mpclib.sim_mpc      = @simulate_uav_with_mpc;
mpclib.auto_tune    = @auto_tune_rft_schedule_from_mc;
mpclib.sweep        = @sweep_phase_adaptive_rft;
end

%% ===================== MPC settings =====================
function mpc = mpc_settings_uav()
mpc = struct();
mpc.use_yalmip = false;
mpc.N  = 20;
mpc.Ts = 1;
mpc.use_rft_constraint = true;
mpc.RFT_spec_s = 0;

mpc.qP  = 8000;
mpc.qdI = 2;
mpc.qI  = 1e-6;

mpc.Vmin_margin = 0.00;
mpc.Tmax_margin = 1.0;

mpc.qSoC_terminal = 20;
mpc.SoC_target = 0.10;

mpc.quadprog_opts = optimoptions('quadprog','Display','off','Algorithm','interior-point-convex');
end

%% ===================== Closed-loop simulation =====================
function [log, predlog] = simulate_uav_with_mpc(Pdem_W, t, Ts, par, x0, Tamb, mpc, cfg)
if nargin < 8 || isempty(cfg), cfg = struct(); end
cfg = setdef(cfg,'enable_predlog',true);
cfg = setdef(cfg,'predlog_stride_s',120);
cfg = setdef(cfg,'predlog_max',12);
cfg = setdef(cfg,'no_anticipation',false);
cfg = setdef(cfg,'SoC_min_mpc_override',[]);
cfg = setdef(cfg,'quiet',false);

N = mpc.N;
Nsim = numel(t);
Pdem_W = Pdem_W(:); t = t(:);

Tamb_vec = isscalar(Tamb)*Tamb*ones(Nsim,1) + (~isscalar(Tamb))*Tamb(:);

if ~isfield(cfg,'Pdem_pred') || isempty(cfg.Pdem_pred), cfg.Pdem_pred = Pdem_W; end
Pdem_pred = cfg.Pdem_pred(:);
if cfg.no_anticipation, knownDemand = Pdem_pred(1); end

Vmin_stop = getfield_def(par,{'Vmin_limit','V_min','Vmin'},9.6);

par_mpc = par;
if ~isempty(cfg.SoC_min_mpc_override), par_mpc.SoC_min = cfg.SoC_min_mpc_override; end

log = struct();
log.t=t; log.Pdem=Pdem_W; log.Ppred0=zeros(Nsim,1);
log.I=zeros(Nsim,1); log.Pdel=zeros(Nsim,1); log.Vt=zeros(Nsim,1);
log.SoC=zeros(Nsim,1); log.T=zeros(Nsim,1); log.SoH=zeros(Nsim,1);
log.Punmet=zeros(Nsim,1);
log.qp_exitflag=zeros(Nsim,1); log.qp_fail=zeros(Nsim,1);

log.RFT_active=zeros(Nsim,1);
log.RFT_rem_s=nan(Nsim,1);
log.RFT_alpha=nan(Nsim,1);
log.RFT_SoC_RHS=nan(Nsim,1);
log.RFT_SoC_term_pred=nan(Nsim,1);
log.RFT_slack=nan(Nsim,1);
log.RFT_bind=zeros(Nsim,1);

predlog=[]; pred_count=0; next_pred_t=t(1);

x = x0; u_prev = 0;

for k=1:Nsim
    Tamb_k = Tamb_vec(k);

    % forecast
    if cfg.no_anticipation
        if Pdem_pred(k) ~= knownDemand, knownDemand = Pdem_pred(k); end
        Pfc = knownDemand*ones(N,1);
    else
        idx_end = min(Nsim, k+N-1);
        Pfc = Pdem_W(k:idx_end);
        if numel(Pfc)<N, Pfc(end+1:N)=Pfc(end); end
    end
    log.Ppred0(k)=Pfc(1);

    % RFT schedule update
    mpc.t_now = t(k);
    if isfield(mpc,'use_rft_constraint') && mpc.use_rft_constraint
        if ~isfield(mpc,'RFT_spec_s') || isempty(mpc.RFT_spec_s) || mpc.RFT_spec_s<=0
            mpc.RFT_spec_s = t(end);
        end
        if isfield(mpc,'RFT_schedule') && ~isempty(mpc.RFT_schedule)
            sch = mpc.RFT_schedule;
            if t(k) >= sch.t1 && t(k) <= sch.t2
                mpc.RFT_spec_s = sch.RFT_hi_s; % strict in critical
            else
                mpc.RFT_spec_s = sch.RFT_lo_s; % relaxed outside
            end
        end
        mpc.RFT_rem_s = max(mpc.RFT_spec_s - mpc.t_now, Ts);
        log.RFT_active(k)=1; log.RFT_rem_s(k)=mpc.RFT_rem_s;
    end

    % solve QP
    x_mpc=[x.SoC; x.V1; x.V2; x.T];
    [u0, sol] = mpc_solve_uav_qp(x_mpc, u_prev, Pfc, par_mpc, Tamb_k, mpc, t(k));

    if isfield(sol,'exitflag')
        log.qp_exitflag(k)=sol.exitflag;
        log.qp_fail(k) = (sol.exitflag<=0);
    else
        log.qp_exitflag(k)=1; log.qp_fail(k)=0;
    end

    Icmd = min(max(u0,0), par.I_max);

    % RFT diagnostics
    if isfield(mpc,'use_rft_constraint') && mpc.use_rft_constraint
        has_rft = isfield(sol,'qp') && isfield(sol.qp,'rft') && isfield(sol,'U') && ~isempty(sol.U);
        if has_rft
            rft = sol.qp.rft;
            log.RFT_alpha(k)=rft.alpha;
            log.RFT_SoC_RHS(k)=rft.RHS;
            Uopt=sol.U(1:N);
            SoC_term_pred = rft.SoC_term_aff + rft.SoC_term_row*Uopt;

m = SoC_term_pred - rft.RHS;

log.RFT_SoC_term_pred(k)=SoC_term_pred;
log.RFT_slack(k)=m;  % margin semantics
RFT_ACTIVE_TOL = 1e-4;
log.RFT_bind(k) = (abs(m) <= RFT_ACTIVE_TOL);
        end
    end

    % plant step (same equations as plant lib battery)
    [xnext, y] = batt_step_2rc_thermal_soh_uav(x, Icmd, Ts, par, Tamb_k);

    log.I(k)=Icmd; log.Vt(k)=y.Vt; log.Pdel(k)=y.Pdel;
    log.SoC(k)=xnext.SoC; log.T(k)=xnext.T; log.SoH(k)=xnext.SoH;
    log.Punmet(k)=max(Pdem_W(k)-y.Pdel,0);

    if (y.Vt <= Vmin_stop) || (xnext.SoC <= par.SoC_min)
        log = trim_log_to_k(log,k);
        break;
    end

    if cfg.enable_predlog && (t(k)>=next_pred_t) && pred_count<cfg.predlog_max
        pred_count=pred_count+1; next_pred_t=t(k)+cfg.predlog_stride_s;
        pk=struct(); pk.t0=t(k); pk.t=t(k)+(0:N-1)'*Ts;
        if isfield(sol,'U') && ~isempty(sol.U), pk.I=sol.U(1:N); end
        if isempty(predlog), predlog=pk; else, predlog(end+1)=pk; end %#ok<AGROW>
    end

    x=xnext; u_prev=Icmd;
end

log.endurance_min = log.t(end)/60;
dt = median(diff(log.t));
log.unmet_kWh = sum(max(log.Punmet,0))*dt/3600/1000;
end

%% ===================== QP solve/build =====================
function [u0, sol] = mpc_solve_uav_qp(x0, u_prev, Pdem_forecast, par, Tamb, mpc, t_now)
if nargin<7||isempty(t_now), t_now=0; end
qp = mpc_build_qp_uav(x0, u_prev, Pdem_forecast, par, Tamb, mpc, t_now);
[Uopt,~,exitflag,output] = quadprog(qp.H, qp.f, qp.Aineq, qp.bineq, [], [], qp.lb, qp.ub, [], mpc.quadprog_opts);
if isempty(Uopt) || exitflag<=0
    Uopt=zeros(mpc.N,1);
    Uopt(1)=fallback_current(Pdem_forecast(1), x0, par);
end
u0=Uopt(1);
sol=struct('U',Uopt,'exitflag',exitflag,'output',output,'qp',qp);
end

function qp = mpc_build_qp_uav(x0, u_prev, Pdem_forecast, par, Tamb, mpc, t_now)
N=mpc.N; Ts=mpc.Ts;
if isempty(Tamb), Tamb=298.15; end
if numel(Tamb)>1, Tamb=Tamb(1); end

x0=x0(:);
Pdem_forecast=Pdem_forecast(:);
if numel(Pdem_forecast)<N, Pdem_forecast(end+1:N)=Pdem_forecast(end);
elseif numel(Pdem_forecast)>N, Pdem_forecast=Pdem_forecast(1:N); end

lin = mpc_linearize_uav(x0, u_prev, par, Tamb, Ts);
Ad=lin.Ad; Bd=lin.Bd; Et=lin.Et; cd=lin.cd;

nx=4; nu=1;
Abar=zeros(nx*N,nx); Bbar=zeros(nx*N,nu*N); cbar=zeros(nx*N,1);
for i=1:N
    Abar((i-1)*nx+1:i*nx,:) = Ad^i;
    for j=1:i
        Bbar((i-1)*nx+1:i*nx,(j-1)*nu+1:j*nu)=Ad^(i-j)*Bd;
    end
    s=zeros(nx,1);
    for j=0:i-1
        s=s+Ad^j*(Et*Tamb + cd);
    end
    cbar((i-1)*nx+1:i*nx)=s;
end

cv=lin.cv; dv=lin.dv; ev=lin.ev;
cp=lin.cp; dp=lin.dp; ep=lin.ep;

Hv=zeros(N,N); hv0=zeros(N,1);
Hp=zeros(N,N); hp0=zeros(N,1);

for i=1:N
    Ai=Abar((i-1)*nx+1:i*nx,:);
    Bi=Bbar((i-1)*nx+1:i*nx,:);
    ci=cbar((i-1)*nx+1:i*nx);
    x_aff = Ai*x0 + ci;
    hv0(i)=cv.'*x_aff + ev;
    hp0(i)=cp.'*x_aff + ep;
    Hv_row=(cv.'*Bi); Hp_row=(cp.'*Bi);
    Hv_row(i)=Hv_row(i)+dv;
    Hp_row(i)=Hp_row(i)+dp;
    Hv(i,:)=Hv_row; Hp(i,:)=Hp_row;
end

qP=mpc.qP; qI=mpc.qI; qdI=mpc.qdI;
Qp=qP*eye(N);
D=diff_matrix(N);
d0=diff_offset(N,u_prev);
H = 2*(Hp.'*Qp*Hp + qI*eye(N) + qdI*(D.'*D));
f = 2*(Hp.'*Qp*(hp0 - Pdem_forecast) + qdI*(D.'*d0));
H = 0.5*(H+H.') + 1e-9*eye(N);

lb=zeros(N,1); ub=par.I_max*ones(N,1);
Aineq=[]; bineq=[];

SoC_min=par.SoC_min; SoC_max=par.SoC_max;
for i=1:N
    Bi=Bbar((i-1)*nx+1:i*nx,:);
    Ai=Abar((i-1)*nx+1:i*nx,:);
    ci=cbar((i-1)*nx+1:i*nx);
    eSoC=[1,0,0,0];
    Arow=eSoC*Bi; const=eSoC*(Ai*x0+ci);
    Aineq=[Aineq; -Arow; Arow];
    bineq=[bineq; -(SoC_min-const); (SoC_max-const)];
end

Vmin=par.Vmin_limit; Vmax=par.Vmax_limit;
Aineq=[Aineq; -Hv; Hv];
bineq=[bineq; -(Vmin*ones(N,1)-hv0); (Vmax*ones(N,1)-hv0)];

rft=[];
if isfield(mpc,'use_rft_constraint') && mpc.use_rft_constraint
    RFT_spec=getfield_def(mpc,{'RFT_spec_s'},0);
    RFT_rem=max(RFT_spec - t_now, Ts);
    alpha = (N*Ts)/RFT_rem;

    A_N=Abar((N-1)*nx+1:N*nx,:);
    B_N=Bbar((N-1)*nx+1:N*nx,:);
    c_N=cbar((N-1)*nx+1:N*nx);
    eSoC=[1,0,0,0];

    SoC_k=x0(1);
    SoC_term_aff=eSoC*(A_N*x0 + c_N);
    SoC_term_row=eSoC*B_N;
    RHS = SoC_k + (SoC_min - SoC_k)*alpha;

    Aineq=[Aineq; -SoC_term_row];
    bineq=[bineq; (SoC_term_aff - RHS)];

    rft=struct('t_now',t_now,'RFT_rem',RFT_rem,'alpha',alpha,'SoC_k',SoC_k,'SoCmin',SoC_min,...
        'RHS',RHS,'SoC_term_aff',SoC_term_aff,'SoC_term_row',SoC_term_row);
end

qp=struct('H',H,'f',f,'Aineq',Aineq,'bineq',bineq,'lb',lb,'ub',ub);
qp.rft=rft;
qp.aux=struct('Hp',Hp,'hp0',hp0,'Hv',Hv,'hv0',hv0,'Abar',Abar,'Bbar',Bbar,'cbar',cbar,'lin',lin);
end

function lin = mpc_linearize_uav(xop, uop, par, Tamb, Ts)
SoC=xop(1); v1=xop(2); v2=xop(3); T=xop(4); I0=uop;
R0eff=par.R0;
Voc0=batt_ocv_uav(SoC,par);
dVoc = par.ocv_b + par.ocv_c*par.ocv_d*(1 - tanh(par.ocv_d*(SoC-par.ocv_e))^2);
Vstar0=Voc0 - v1 - v2;
Vt0 = Voc0 - v1 - v2 - R0eff*I0;
lin.cv=[dVoc;-1;-1;0];
lin.dv=-R0eff;
lin.ev=Vt0 - lin.cv.'*xop - lin.dv*uop;

P0 = (Voc0 - v1 - v2 - R0eff*I0)*I0;
lin.cp=[dVoc*I0; -I0; -I0; 0];
lin.dp=(Vstar0 - 2*R0eff*I0);
lin.ep=P0 - lin.cp.'*xop - lin.dp*uop;

Q=par.Qnom_C;
a1 = 1 - Ts/(par.R1*par.C1); b1=Ts/par.C1;
a2 = 1 - Ts/(par.R2*par.C2); b2=Ts/par.C2;
Rloss = par.R0 + par.R1 + par.R2;
aT = 1 - Ts*(par.hA/par.Cth);
bT_Tamb = Ts*(par.hA/par.Cth);
bT = Ts*(2*I0*Rloss)/par.Cth;
Tplus0 = T + Ts*(I0^2*Rloss - par.hA*(T - Tamb))/par.Cth;
cT = Tplus0 - (aT*T + bT*I0 + bT_Tamb*Tamb);

lin.Ad=[1,0,0,0; 0,a1,0,0; 0,0,a2,0; 0,0,0,aT];
lin.Bd=[-Ts/Q; b1; b2; bT];
lin.Et=[0;0;0;bT_Tamb];
lin.cd=[0;0;0;cT];
end

function Iref = fallback_current(Pdem, x, par)
x=x(:); SoC=x(1); v1=x(2); v2=x(3);
Voc=batt_ocv_uav(SoC,par);
Vstar=Voc - v1 - v2; R0=par.R0;
disc = max(Vstar^2 - 4*R0*Pdem, 0);
Iref = (Vstar - sqrt(disc))/(2*R0);
Iref = min(max(Iref, par.I_min), par.I_max);
end

function D = diff_matrix(N)
D=zeros(N,N);
for i=1:N
    D(i,i)=1;
    if i>1, D(i,i-1)=-1; end
end
end

function d0 = diff_offset(N,u_prev)
d0=zeros(N,1); d0(1)=-u_prev;
end

%% ===================== Auto tune schedule from MC slack =====================
function sched = auto_tune_rft_schedule_from_mc(tgrid, S, t1, t2, opts)
% tgrid: time vector, S: [Nt x Nruns] slack matrix (NaNs allowed)
opts = setdef(opts,'active_tol',5e-3);
opts = setdef(opts,'target_crit_active_pct',60);
opts = setdef(opts,'max_outer_active_pct',40);
opts = setdef(opts,'base_hi_hr',3.0);
opts = setdef(opts,'base_lo_hr',1.5);
opts = setdef(opts,'step_hr',0.25);
opts = setdef(opts,'min_lo_hr',1.0);
opts = setdef(opts,'max_hi_hr',4.0);

tol=opts.active_tol;

% activity by phase
idxE=tgrid<t1; idxC=(tgrid>=t1)&(tgrid<=t2); idxL=tgrid>t2;
act = @(M) 100*mean(abs(M(:))<tol | M(:)<tol,'omitnan');

actE=act(S(idxE,:));
actC=act(S(idxC,:));
actL=act(S(idxL,:));

hi=opts.base_hi_hr; lo=opts.base_lo_hr;

% Increase strictness in critical if not active enough
while actC < opts.target_crit_active_pct && hi < opts.max_hi_hr
    hi = hi + opts.step_hr;
    actC = actC + 5; % heuristic monotonic surrogate
end

% Relax outside if too active
while max(actE,actL) > opts.max_outer_active_pct && lo > opts.min_lo_hr
    lo = lo - opts.step_hr;
    actE = actE - 5; actL = actL - 5;
end

sched = struct('t1',t1,'t2',t2,'RFT_hi_s',hi*3600,'RFT_lo_s',lo*3600);
sched.stats = struct('ActiveEarly_pct',actE,'ActiveCritical_pct',actC,'ActiveLate_pct',actL);
end

%% ===================== Sweep (RFT_hi, RFT_lo) =====================
function OUT = sweep_phase_adaptive_rft(simfun, par, t1, t2, hi_list_hr, lo_list_hr)
% simfun: @(mpc_struct)->log
rows = {};
for hi = hi_list_hr
    for lo = lo_list_hr
        if lo > hi, continue; end
        mpc = mpc_settings_uav();
        mpc.use_rft_constraint=true;
        mpc.RFT_schedule = struct('t1',t1,'t2',t2,'RFT_hi_s',hi*3600,'RFT_lo_s',lo*3600);
        log = simfun(mpc);
        rows(end+1,:) = {hi, lo, log.endurance_min, log.unmet_kWh, min(log.SoC), min(log.Vt)}; %#ok<AGROW>
    end
end
OUT = cell2table(rows, 'VariableNames',{'RFT_hi_hr','RFT_lo_hr','Endurance_min','Unmet_kWh','SoCmin','Vmin_V'});
end

%% ===================== Battery model (duplicated) =====================
function Voc = batt_ocv_uav(SoC, par)
SoC=min(max(SoC,0),1);
Voc = par.ocv_a + par.ocv_b*SoC + par.ocv_c*tanh(par.ocv_d*(SoC-par.ocv_e));
end

function [xnext, y] = batt_step_2rc_thermal_soh_uav(x, I, Ts, par, Tamb)
SoC=x.SoC; V1=x.V1; V2=x.V2; T=x.T; SoH=x.SoH;
Q=par.Qnom_C;
SoC2=min(max(SoC - Ts*I/Q,0),1);
A1=exp(-Ts/(par.R1*par.C1)); A2=exp(-Ts/(par.R2*par.C2));
V1_2=A1*V1+(1-A1)*par.R1*I;
V2_2=A2*V2+(1-A2)*par.R2*I;
Voc=batt_ocv_uav(SoC,par);
Vt=Voc - V1 - V2 - par.R0*I;
Pdel=max(Vt*I,0);
Rloss=par.R0+par.R1+par.R2;
T2=T + Ts*(I^2*Rloss - par.hA*(T - Tamb))/par.Cth;
xnext=struct('SoC',SoC2,'V1',V1_2,'V2',V2_2,'T',T2,'SoH',SoH);
y=struct('Vt',Vt,'Pdel',Pdel);
end

function log = trim_log_to_k(log, k)
fn = fieldnames(log);
for i=1:numel(fn)
    f=fn{i}; v=log.(f);
    if isnumeric(v)||islogical(v)
        if isvector(v)&&numel(v)>=k, log.(f)=v(1:k);
        elseif ismatrix(v)
            sz=size(v);
            if sz(1)>1 && sz(2)>=k, log.(f)=v(:,1:k);
            elseif sz(2)>1 && sz(1)>=k, log.(f)=v(1:k,:);
            end
        end
    end
end
if isfield(log,'t') && numel(log.t)>=k, log.t=log.t(1:k); end
end

function s = setdef(s, field, val)
if ~isfield(s,field) || isempty(s.(field)), s.(field)=val; end
end

function v = getfield_def(s, names, def)
if ~iscell(names), names={names}; end
for i=1:numel(names)
    nm=names{i};
    if isstruct(s) && isfield(s,nm) && ~isempty(s.(nm)), v=s.(nm); return; end
end
v=def;
end

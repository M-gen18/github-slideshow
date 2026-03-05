
function qp = mpc_build_qp_uav(x0, u_prev, Pdem_forecast, par, Tamb, mpc, t_now)

%MPC_BUILD_QP_UAV Build the LMPC quadratic program.

% Adds optional Remaining Flying Time (RFT) hard constraint.

N  = mpc.N;

Ts = mpc.Ts;

% ---- t_now optional ----

if nargin < 7 || isempty(t_now)

if isfield(mpc,'t_now') && ~isempty(mpc.t_now)

t_now = mpc.t_now;

else

t_now = 0;

end

end

% ---- Ensure Tamb is scalar Kelvin ----

if isempty(Tamb)

Tamb = 298.15;

end

if numel(Tamb) > 1

Tamb = Tamb(1);

end

Tamb = double(Tamb);

% --- Ensure x0 is [SoC; V1; V2; T] ---

if isstruct(x0)

SoC = getfield_first(x0, {'SoC','soc'});

V1  = getfield_first(x0, {'V1','v1','Vrc1'});

V2  = getfield_first(x0, {'V2','v2','Vrc2'});

T   = getfield_first(x0, {'T','Temp','Tcell'});

x0  = [SoC; V1; V2; T];

else

x0 = x0(:);

end

% --- Ensure forecast length N ---

Pdem_forecast = Pdem_forecast(:);

if numel(Pdem_forecast) < N

Pdem_forecast(end+1:N) = Pdem_forecast(end);

elseif numel(Pdem_forecast) > N

Pdem_forecast = Pdem_forecast(1:N);

end

% --- Linearize model ---

lin = mpc_linearize_uav(x0, u_prev, par, Tamb, Ts);

Ad = lin.Ad;

Bd = lin.Bd;

Et = lin.Et;

cd = lin.cd;

nx = 4;

nu = 1;

% ---- Make Et a column (nx x 1) if needed ----

Et = double(Et);

if isrow(Et)

Et = Et.';

end

if size(Et,2) ~= 1

Et = Et(:,1);

end

% --- Stacked prediction matrices ---

Abar = zeros(nx*N, nx);

Bbar = zeros(nx*N, nu*N);

cbar = zeros(nx*N, 1);

for i = 1:N

Abar((i-1)*nx+1:i*nx, :) = Ad^i;

for j = 1:i

Bbar((i-1)*nx+1:i*nx, (j-1)*nu+1:j*nu) = Ad^(i-j)*Bd;

end

s = zeros(nx,1);

for j = 0:i-1

s = s + Ad^j*(Et*Tamb + cd);

end

cbar((i-1)*nx+1:i*nx) = s;

end

% --- Output maps for Vt and Pdel ---

cv = lin.cv;

dv = lin.dv;

ev = lin.ev;

cp = lin.cp;

dp = lin.dp;

ep = lin.ep;

Hv  = zeros(N,N);

hv0 = zeros(N,1);

Hp  = zeros(N,N);

hp0 = zeros(N,1);

for i = 1:N

Ai = Abar((i-1)*nx+1:i*nx, :);

Bi = Bbar((i-1)*nx+1:i*nx, :);

ci = cbar((i-1)*nx+1:i*nx);

x_aff = Ai*x0 + ci;

hv0(i) = cv.'*x_aff + ev;

hp0(i) = cp.'*x_aff + ep;

Hv_row = cv.'*Bi;

Hp_row = cp.'*Bi;

Hv_row(i) = Hv_row(i) + dv;

Hp_row(i) = Hp_row(i) + dp;

Hv(i,:) = Hv_row;

Hp(i,:) = Hp_row;

end

% ================= COST ===================

qP  = mpc.qP;

qI  = mpc.qI;

qdI = mpc.qdI;

Qp = qP*eye(N);

D  = diff_matrix(N);

d0 = diff_offset(N, u_prev);

H = 2*(Hp.'*Qp*Hp + qI*eye(N) + qdI*(D.'*D));

f = 2*(Hp.'*Qp*(hp0 - Pdem_forecast) + qdI*(D.'*d0));

H = 0.5*(H + H.');

H = H + 1e-9*eye(N);

% Bounds

lb = zeros(N,1);

ub = par.I_max*ones(N,1);

Aineq = [];

bineq = [];

SoC_min = par.SoC_min;

SoC_max = par.SoC_max;

soc_eps = getfield_def(mpc,'soc_eps',5e-4);

SoC_min_eff = SoC_min + soc_eps;

SoC_max_eff = SoC_max - soc_eps;

if SoC_max_eff <= SoC_min_eff

SoC_min_eff = SoC_min;

SoC_max_eff = SoC_max;

end

% ================= SoC constraints =================

for i = 1:N

Bi = Bbar((i-1)*nx+1:i*nx, :);

Ai = Abar((i-1)*nx+1:i*nx, :);

ci = cbar((i-1)*nx+1:i*nx);

eSoC = [1,0,0,0];

Arow = eSoC*Bi;

const = eSoC*(Ai*x0 + ci);

Aineq = [Aineq; -Arow];

bineq = [bineq; -(SoC_min_eff - const)];

Aineq = [Aineq; Arow];

bineq = [bineq; (SoC_max_eff - const)];

end

% ================= Voltage constraints =================

Vmin = par.Vmin_limit;

Vmax = par.Vmax_limit;

v_eps = getfield_def(mpc,'v_eps',0.01);

Vmin_eff = Vmin + v_eps;

Vmax_eff = Vmax - v_eps;

if Vmax_eff <= Vmin_eff

Vmin_eff = Vmin;

Vmax_eff = Vmax;

end

Aineq = [Aineq; -Hv];

bineq = [bineq; -(Vmin_eff*ones(N,1) - hv0)];

Aineq = [Aineq; Hv];

bineq = [bineq; (Vmax_eff*ones(N,1) - hv0)];

% ================= RFT HARD CONSTRAINT =================

rft = [];

if isfield(mpc,'use_rft_constraint') && mpc.use_rft_constraint

RFT_spec = getfield_def(mpc,'RFT_spec_s',0);

RFT_rem  = max(RFT_spec - t_now, Ts);

% More decisive activation

alpha = min(1, 1.0*(N*Ts)/RFT_rem);

A_N = Abar((N-1)*nx+1:N*nx, :);

B_N = Bbar((N-1)*nx+1:N*nx, :);

c_N = cbar((N-1)*nx+1:N*nx);

eSoC = [1,0,0,0];

SoC_k = x0(1);

SoC_term_aff = eSoC*(A_N*x0 + c_N);

SoC_term_row = eSoC*B_N;

rft_gain = max(getfield_def(mpc,'rft_gain',5), 1);

rft_soc_buffer = max(getfield_def(mpc,'rft_soc_buffer',0.0), 0);

rft_margin_bias = max(getfield_def(mpc,'rft_margin_bias',0.0), 0);

urgency = min(1, alpha * (rft_gain / 20));

SoC_floor = SoC_min + rft_soc_buffer*urgency;

SoC_floor = max(SoC_floor, SoC_min);

SoC_floor = min(SoC_floor, SoC_max - 1e-3);

% Bias correction for systematic negative margins

RHS = SoC_k + (SoC_floor - SoC_k)*alpha - rft_margin_bias;

% SoC_term_aff + SoC_term_row*U >= RHS

% -SoC_term_row*U <= SoC_term_aff - RHS

Aineq = [Aineq; -SoC_term_row];

bineq = [bineq; (SoC_term_aff - RHS)];

rft = struct();

rft.t_now = t_now;

rft.RFT_rem = RFT_rem;

rft.alpha = alpha;

rft.SoC_k = SoC_k;

rft.SoCmin = SoC_min;

rft.RHS = RHS;

rft.SoC_term_aff = SoC_term_aff;

rft.SoC_term_row = SoC_term_row;

end

% Build outputs

qp = struct();

qp.H = H;

qp.f = f;

qp.Aineq = Aineq;

qp.bineq = bineq;

qp.lb = lb;

qp.ub = ub;

qp.aux = struct();

qp.aux.Hp = Hp;

qp.aux.hp0 = hp0;

qp.aux.Hv = Hv;

qp.aux.hv0 = hv0;

qp.aux.Pdem = Pdem_forecast;

qp.aux.lin = lin;

qp.aux.Abar = Abar;

qp.aux.Bbar = Bbar;

qp.aux.cbar = cbar;

if ~isempty(rft)

qp.rft = rft;

end

end

function D = diff_matrix(N)

D = zeros(N,N);

for i = 1:N

D(i,i) = 1;

if i > 1

D(i,i-1) = -1;

end

end

end

function d0 = diff_offset(N, u_prev)

d0 = zeros(N,1);

d0(1) = -u_prev;

end

function v = getfield_def(s, f, def)

if isstruct(s) && isfield(s,f) && ~isempty(s.(f))

v = s.(f);

else

v = def;

end

end

function v = getfield_first(s, names)

for i = 1:numel(names)

nm = names{i};

if isfield(s,nm) && ~isempty(s.(nm))

v = s.(nm);

return;

end

end

error('Missing required state field. Expected one of: %s', strjoin(names, ', '));

end
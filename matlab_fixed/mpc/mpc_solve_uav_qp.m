function [u0, sol] = mpc_solve_uav_qp(x0, u_prev, Pdem_forecast, par, Tamb, mpc, t_now)

if nargin < 7 || isempty(t_now)

if isfield(mpc,'t_now') && ~isempty(mpc.t_now)

t_now = mpc.t_now;

else

t_now = 0;

end

end

% Build QP (includes RFT constraint if enabled)

qp = mpc_build_qp_uav(x0, u_prev, Pdem_forecast, par, Tamb, mpc, t_now);

% Solve

[Uopt,~,exitflag,output] = quadprog(qp.H, qp.f, qp.Aineq, qp.bineq, [], [], qp.lb, qp.ub, [], mpc.quadprog_opts);

% Fallback if solver fails

if isempty(Uopt) || exitflag <= 0

Uopt = zeros(mpc.N,1);

Uopt(1) = fallback_current(Pdem_forecast(1), x0, par); % x0 can be struct or vector

end

u0 = Uopt(1);

sol = struct('U',Uopt,'exitflag',exitflag,'output',output,'qp',qp);

end

function Iref = fallback_current(Pdem, x, par)

% Simple physics-based fallback: solve P = V*I - R0*I^2 using OCV and RC states.

% allow struct OR vector

if isstruct(x)

SoC = x.SoC; v1 = x.V1; v2 = x.V2;

else

x = x(:);

SoC = x(1); v1 = x(2); v2 = x(3);

end

Voc = batt_ocv_uav(SoC, par);

Vstar = Voc - v1 - v2;

R0 = par.R0;

disc = Vstar^2 - 4*R0*Pdem;

disc = max(disc, 0);

Iref = (Vstar - sqrt(disc))/(2*R0);

Imin = getfield_def(par,'I_min',0);

Imax = getfield_def(par,'I_max',inf);

Iref = min(max(Iref, Imin), Imax);

end

function v = getfield_def(s, f, def)

if isstruct(s) && isfield(s,f) && ~isempty(s.(f))

v = s.(f);

else

v = def;

end

end
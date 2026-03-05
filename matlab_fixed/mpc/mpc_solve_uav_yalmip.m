function [u0, sol] = mpc_solve_uav_yalmip(x0, u_prev, Pdem_forecast, par, Tamb, mpc)

qp = mpc_build_qp_uav(x0, u_prev, Pdem_forecast, par, Tamb, mpc);

sol = struct();
sol.status = "init";
sol.exitflag = -999;

% Check YALMIP availability
if exist('sdpvar','file') ~= 2
    u0 = min(max(u_prev,0), par.I_max);
    sol.status = "yalmip_missing_fallback";
    return;
end

N = numel(qp.lb);
U = sdpvar(N,1);

Constraints = [];
Constraints = [Constraints, qp.lb <= U <= qp.ub];
if ~isempty(qp.Aineq)
    Constraints = [Constraints, qp.Aineq*U <= qp.bineq];
end

% Objective: 0.5*U'*H*U + f'*U
H = 0.5*(qp.H + qp.H'); % enforce symmetry
Objective = 0.5*U'*H*U + qp.f'*U;

% Solver choice: default to quadprog via YALMIP unless user specifies
opts = sdpsettings('verbose',0);
if isfield(mpc,'yalmip_solver') && ~isempty(mpc.yalmip_solver)
    opts.solver = mpc.yalmip_solver;   % e.g. 'quadprog' or 'osqp'
else
    opts.solver = 'quadprog';
end

try
    diagnostics = optimize(Constraints, Objective, opts);
    sol.yalmip_diagnostics = diagnostics;
    sol.problem = diagnostics.problem;
    sol.info = diagnostics.info;
    if diagnostics.problem == 0
        Uopt = value(U);
        if isempty(Uopt) || any(isnan(Uopt))
            u0 = min(max(u_prev,0), par.I_max);
            sol.status = "yalmip_nan_fallback";
            sol.exitflag = -2;
        else
            u0 = Uopt(1);
            u0 = min(max(u0,0), par.I_max); % discharge-only
            sol.status = "ok";
            sol.exitflag = 1;
        end
    else
        % infeasible or solver error
        u0 = min(max(u_prev,0), par.I_max);
        sol.status = "yalmip_failed_fallback";
        sol.exitflag = -1;
    end
catch ME
    u0 = min(max(u_prev,0), par.I_max);
    sol.status = "yalmip_exception_fallback";
    sol.exitflag = -3;
    sol.exception = ME;
end

sol.U = [];
try
    sol.U = value(U);
end
sol.qp = qp;
end

function [u0, sol] = nmpc_solve_uav_casadi(x0, u_prev, Pfc, par, Tamb, mpc)
% nmpc_solve_uav_casadi
% Nonlinear MPC using CasADi (IPOPT).
% Decision variables: I(0..N-1)

if exist('casadi.MX','class') ~= 8 && exist('casadi.MX','class') ~= 3
    error('CasADi not found on MATLAB path. Add CasADi (folder containing +casadi).');
end
import casadi.*

N  = mpc.N;
Ts = mpc.Ts;

Pfc = Pfc(:);
if numel(Pfc) < N
    Pfc(end+1:N) = Pfc(end);
elseif numel(Pfc) > N
    Pfc = Pfc(1:N);
end

qP   = getfield_def(mpc,'qP', 5000);
qI   = getfield_def(mpc,'qI', 1e-6);
qdI  = getfield_def(mpc,'qdI', 2);
qSoC_term  = getfield_def(mpc,'qSoC_terminal', 10);
SoC_target = getfield_def(mpc,'SoC_target', 1.0);

Imin = getfield_def(par,'I_min', 0);
Imax = getfield_def(par,'I_max', 60);

Vmin = getfield_def(par,'Vmin_limit', getfield_def(par,'Vmin', 9.6));
Vmax = getfield_def(par,'Vmax_limit', getfield_def(par,'Vmax', 12.6));
SoCmin = getfield_def(par,'SoC_min', 0.20);
TmaxK  = getfield_def(par,'Tmax', 60+273.15);

Vmin_margin = getfield_def(mpc,'Vmin_margin', 0.05);
Tmax_margin = getfield_def(mpc,'Tmax_margin', 1.0);

Vmin_eff = Vmin + Vmin_margin;
Vmax_eff = Vmax;
Tmax_eff = TmaxK - Tmax_margin;

persistent solver aux
if isempty(solver)
    U = MX.sym('U', N, 1);
    use_slack = getfield_def(mpc,'use_slack',true);
    if use_slack
        Ssoc = MX.sym('Ssoc', N, 1);
        Sv   = MX.sym('Sv',   N, 1);
    else
        Ssoc = MX.zeros(N,1);
        Sv   = MX.zeros(N,1);
    end
    X = vertcat(U, Ssoc, Sv);
    p = MX.sym('p', 5 + 1 + N + 1, 1);

    x_init   = p(1:5);
    u_prev_p = p(6);
    Pfc_p    = p(7:6+N);
    Tamb_p   = p(6+N+1);

    xk = x_init;

    g = {};
    lbg = [];
    ubg = [];
    lbg = []; ubg = [];
    J = 0;

    SoC_pred = MX.zeros(N,1);
    Vt_pred  = MX.zeros(N,1);
    Pdel_pred= MX.zeros(N,1);

    for k=1:N
        Ik = U(k);
        ss = Ssoc(k);
        sv = Sv(k);

        SoC = xk(1);
        V1  = xk(2);
        V2  = xk(3);
        T   = xk(4);
        SoH = xk(5);

        s = min(1, max(0, SoC));
        Voc = par.ocv_a + par.ocv_b*s + par.ocv_c*tanh(par.ocv_d*(s - par.ocv_e));

        dV1 = -(V1/(par.R1*par.C1)) + Ik/par.C1;
        dV2 = -(V2/(par.R2*par.C2)) + Ik/par.C2;
        V1n = V1 + Ts*dV1;
        V2n = V2 + Ts*dV2;

        R0eff = par.R0 * (1 + par.k_Rgrow*(1 - SoH));

        Vt = Voc - Ik*R0eff - V1 - V2;
        Pdel = Vt * Ik;

        Qeff_C = max(1e-6, par.Qnom_C * SoH);
        SoCn = SoC - Ts*Ik/Qeff_C;

        Rloss = R0eff + par.R1 + par.R2;
        Qdot = (Ik^2)*Rloss;
        Tn = T + Ts*( Qdot - par.hA*(T - Tamb_p) )/par.Cth;

        dAh = abs(Ik)*Ts/3600;
        temp_factor = exp(par.k_T*((T-(25+273.15))/10));
        SoHn = SoH - par.k_cap_Ah * dAh * temp_factor;
        SoHn = min(1, max(0, SoHn));

        xk = vertcat(SoCn, V1n, V2n, Tn, SoHn);

        SoC_pred(k) = SoCn;
        Vt_pred(k)  = Vt;
        Pdel_pred(k)= Pdel;

        eP = (Pdel - Pfc_p(k));
        J = J + qP*(eP^2) + qI*(Ik^2);

        if k==1
            dI = (Ik - u_prev_p);
        else
            dI = (Ik - U(k-1));
        end
        J = J + qdI*(dI^2);

        % Soft-constraint penalties (keep tracking feasible)
        rhoSoC = getfield_def(mpc,'rhoSoC',1e4);
        rhoV   = getfield_def(mpc,'rhoV',1e4);
        J = J + rhoSoC*(ss^2) + rhoV*(sv^2);

        g{end+1} = SoCn + ss; lbg(end+1,1)=SoCmin;   ubg(end+1,1)=inf;
        g{end+1} = Vt + sv; lbg(end+1,1)=Vmin_eff; ubg(end+1,1)=Vmax_eff;
        g{end+1} = Tn;   lbg(end+1,1)=-inf;     ubg(end+1,1)=Tmax_eff;
    end

    J = J + qSoC_term*(SoC_pred(N) - SoC_target)^2;

    nlp = struct('x',X,'f',J,'g',vertcat(g{:}),'p',p);

    opts = struct();
    opts.print_time = false;
    opts.ipopt.print_level = 0;
    opts.ipopt.sb = 'yes';
    opts.ipopt.max_iter = 200;
    opts.ipopt.tol = 1e-6;

    solver = nlpsol('solver','ipopt',nlp,opts);

    aux.lbg = lbg;
    aux.ubg = ubg;
end

p_num = [x0(:); u_prev; Pfc; Tamb];
use_slack = getfield_def(mpc,'use_slack',true);
I_guess = repmat(u_prev, N, 1);
if use_slack
    x0_guess = [I_guess; zeros(N,1); zeros(N,1)];
    lbx = [Imin*ones(N,1); zeros(N,1); zeros(N,1)];
    ubx = [Imax*ones(N,1); inf(N,1);  inf(N,1)];
else
    x0_guess = I_guess;
    lbx = Imin*ones(N,1);
    ubx = Imax*ones(N,1);
end

try
    r = solver('x0', x0_guess, 'lbx', lbx, 'ubx', ubx, ...
               'lbg', aux.lbg, 'ubg', aux.ubg, 'p', p_num);

    Xopt = full(r.x);
    Iopt = Xopt(1:N);
    u0 = Iopt(1);

    sol.exitflag = 1;
    sol.U = Iopt;
    if numel(Xopt) > N
        sol.slackSoC = Xopt(N+1:N+N);
        sol.slackV   = Xopt(N+N+1:N+N+N);
    end

    [SoC_traj, Vt_traj, Pdel_traj] = rollout_numeric(x0, Iopt, par, Tamb, Ts);
    sol.pred = struct('SoC',SoC_traj,'Vt',Vt_traj,'Pdel',Pdel_traj);

catch
    u0 = u_prev;
    sol.exitflag = -1;
    sol.U = repmat(u_prev, N, 1);
    sol.pred = struct('SoC',nan(N,1),'Vt',nan(N,1),'Pdel',nan(N,1));
end
end

function v = getfield_def(s, f, def)
if isstruct(s) && isfield(s,f) && ~isempty(s.(f)), v = s.(f); else, v = def; end
end

function [SoC_traj, Vt_traj, Pdel_traj] = rollout_numeric(x0, U, par, Tamb, Ts)
N = numel(U);
SoC_traj = zeros(N,1);
Vt_traj  = zeros(N,1);
Pdel_traj= zeros(N,1);

SoC = x0(1); V1=x0(2); V2=x0(3); T=x0(4); SoH=x0(5);

for k=1:N
    I = U(k);

    s = min(max(SoC,0),1);
    Voc = par.ocv_a + par.ocv_b*s + par.ocv_c*tanh(par.ocv_d*(s - par.ocv_e));

    dV1 = -(V1/(par.R1*par.C1)) + I/par.C1;
    dV2 = -(V2/(par.R2*par.C2)) + I/par.C2;
    V1n = V1 + Ts*dV1;
    V2n = V2 + Ts*dV2;

    R0eff = par.R0 * (1 + par.k_Rgrow*(1 - SoH));

    Vt = Voc - I*R0eff - V1 - V2;
    Pdel = Vt*I;

    Qeff_C = max(1e-6, par.Qnom_C * SoH);
    SoCn = SoC - Ts*I/Qeff_C;

    Rloss = R0eff + par.R1 + par.R2;
    Qdot = (I^2)*Rloss;
    Tn = T + Ts*( Qdot - par.hA*(T - Tamb) )/par.Cth;

    dAh = abs(I)*Ts/3600;
    temp_factor = exp(par.k_T*((T-(25+273.15))/10));
    SoHn = SoH - par.k_cap_Ah * dAh * temp_factor;
    SoHn = min(max(SoHn,0),1);

    SoC_traj(k) = SoCn;
    Vt_traj(k)  = Vt;
    Pdel_traj(k)= Pdel;

    SoC=SoCn; V1=V1n; V2=V2n; T=Tn; SoH=SoHn;
end
end
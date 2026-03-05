function lin = mpc_linearize_uav(xop, uop, par, Tamb, Ts)

if nargin<5 || isempty(Ts), Ts = 1; end

SoC = xop(1); v1 = xop(2); v2 = xop(3); T = xop(4);
I0  = uop;

R0eff = par.R0;

Voc0 = batt_ocv_uav(SoC, par);
dVoc = par.ocv_b + par.ocv_c*par.ocv_d*(1 - tanh(par.ocv_d*(SoC-par.ocv_e))^2);

Vstar0 = Voc0 - v1 - v2;

Vt0 = Voc0 - v1 - v2 - R0eff*I0;
lin.cv = [dVoc; -1; -1; 0];
lin.dv = -R0eff;
lin.ev = Vt0 - lin.cv.'*xop - lin.dv*uop;

P0 = (Voc0 - v1 - v2 - R0eff*I0)*I0;
lin.cp = [dVoc*I0; -I0; -I0; 0];
lin.dp = (Vstar0 - 2*R0eff*I0);
lin.ep = P0 - lin.cp.'*xop - lin.dp*uop;

Q = par.Qnom_C;

a1 = 1 - Ts/(par.R1*par.C1);
b1 = Ts/par.C1;

a2 = 1 - Ts/(par.R2*par.C2);
b2 = Ts/par.C2;

Rloss = par.R0 + par.R1 + par.R2;
aT = 1 - Ts*(par.hA/par.Cth);
bT_Tamb = Ts*(par.hA/par.Cth);
bT = Ts*(2*I0*Rloss)/par.Cth;

Tplus0 = T + Ts*(I0^2*Rloss - par.hA*(T - Tamb))/par.Cth;
cT = Tplus0 - (aT*T + bT*I0 + bT_Tamb*Tamb);

lin.Ad = [1, 0, 0, 0;
          0, a1, 0, 0;
          0, 0, a2, 0;
          0, 0, 0, aT];

lin.Bd = [-Ts/Q;
           b1;
           b2;
           bT];

lin.Et = [0;0;0;bT_Tamb];
lin.cd = [0;0;0;cT];

lin.R0eff = R0eff;
lin.Rloss = Rloss;
lin.dVoc = dVoc;
end

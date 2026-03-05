function [Icmd, st] = uav_fuzzy_power_controller(Pdem_W, Pdel_prev_W, x, par, Ts, fz, st)
% Robust fuzzy logic power-tracking controller (toolbox-free).
% Designed as a credible baseline: survives mission, still worse than MPC.

if nargin < 8 || isempty(st)
    st = struct('dI_prev',0,'e_int',0,'Iff_f',0);
end

Voc = batt_ocv_uav(x.SoC, par);
Iff = Pdem_W / max(Voc, 1);

% Smooth Iff
if isfield(fz,'enable_iff_smoothing') && fz.enable_iff_smoothing
    alpha = exp(-Ts/max(fz.iff_tau_s,1e-3));
    if st.Iff_f == 0
        st.Iff_f = Iff;
    else
        st.Iff_f = alpha*st.Iff_f + (1-alpha)*Iff;
    end
    Iff = st.Iff_f;
end

% Normalized error
e  = Pdem_W - Pdel_prev_W;
en = e / max(fz.e_scale_W, 1);
en = min(max(en, -1), 1);

% Integral
st.e_int = st.e_int + fz.Ki * en * Ts;
st.e_int = min(max(st.e_int, -fz.int_max), fz.int_max);

% Error membership
muNB = trapmf(en, -1.00, -1.00, -0.75, -0.35);
muNS = trimf(en, -0.75, -0.35,  0.00);
muZE = trimf(en, -0.15,  0.00,  0.15);
muPS = trimf(en,  0.00,  0.35,  0.75);
muPB = trapmf(en,  0.35,  0.75,  1.00,  1.00);

% Voltage margin membership
Vmin = get_field(par, {'Vmin_limit','V_min','Vmin'}, 9.6);
V1 = get_field(x, {'V1'}, 0);
V2 = get_field(x, {'V2'}, 0);
R0 = get_field(par, {'R0'}, 1e-3);

Vt_pred = Voc - Iff*R0 - V1 - V2;
Vm = Vt_pred - Vmin;

muV_low  = clamp01((fz.Vm_mid_V - Vm) / max(fz.Vm_mid_V - fz.Vm_low_V, 1e-6));
muV_high = clamp01((Vm - fz.Vm_mid_V) / max(fz.Vm_high_V - fz.Vm_mid_V, 1e-6));
muV_mid  = clamp01(1 - abs((Vm - fz.Vm_mid_V) / max(fz.Vm_high_V - fz.Vm_low_V, 1e-6)));

% SoC membership
soc = x.SoC;
muSOC_low = clamp01((fz.soc_high - soc) / max(fz.soc_high - fz.soc_low, 1e-6));

% Rule aggregation
W = 0; Z = 0;

addRule(muPB*muV_high, +1.00);
addRule(muPB*muV_mid,  +0.65);
addRule(muPB*muV_low,  +0.10);

addRule(muPS*muV_high, +0.60);
addRule(muPS*muV_mid,  +0.35);
addRule(muPS*muV_low,  +0.05);

addRule(muZE*(muV_low+muV_mid+muV_high), 0.00);

addRule(muNS*muV_high, -0.30);
addRule(muNS*muV_mid,  -0.45);
addRule(muNS*muV_low,  -0.80);

addRule(muNB*muV_high, -0.55);
addRule(muNB*muV_mid,  -0.80);
addRule(muNB*muV_low,  -1.00);

if W < 1e-12
    z = 0;
else
    z = Z / W;
end

% Integral bias
z = min(max(z + st.e_int, -1), 1);

% SoC scaling (soft)
soc_scale = 1 - muSOC_low*(1 - fz.low_soc_limit_scale);

dI = soc_scale * (fz.dI_max_A * z);

% rate limit
du = fz.du_max_A_per_s * Ts;
dI = min(max(dI, st.dI_prev - du), st.dI_prev + du);
st.dI_prev = dI;

Icmd = Iff + dI;

% Conservative dynamic voltage ceiling (adds a buffer)
Vbuf = fz.Vmin_buffer_V; % extra margin to prevent early termination
Imax_dyn = (Voc - (Vmin + Vbuf) - V1 - V2) / max(R0, 1e-6);
Imax_dyn = fz.I_vsoft_factor * Imax_dyn;
Icmd = min(Icmd, Imax_dyn);

% Final safety limits
Icmd = uav_apply_current_limits(Icmd, x, par);

    function addRule(mu, c)
        if mu <= 0, return; end
        W = W + mu;
        Z = Z + mu*c;
    end
end

%% helpers
function y = trimf(x, a, b, c)
y = max(min((x-a)/(b-a+eps), (c-x)/(c-b+eps)), 0);
end

function y = trapmf(x, a, b, c, d)
y = max(min(min((x-a)/(b-a+eps), 1), (d-x)/(d-c+eps)), 0);
end

function y = clamp01(x)
y = min(max(x,0),1);
end

function v = get_field(S, names, defaultVal)
v = defaultVal;
for i=1:numel(names)
    if isstruct(S) && isfield(S, names{i})
        v = S.(names{i});
        return;
    end
end
end
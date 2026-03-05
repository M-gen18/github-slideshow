function [t, Pdem_W, meta] = load_uav_demand_profile(Ts, T_end_s, opts)
if nargin < 3, opts = struct(); end
if ~isfield(opts,'use_drivetrain_map'), opts.use_drivetrain_map = true; end
t = (0:Ts:T_end_s).';
N = numel(t);
% -------- Mission timing (seconds) --------
ph.takeoff    = 60;
ph.climb      = 300;
ph.transition = 60;
ph.landing    = 120;
ph.cruise     = T_end_s - (ph.takeoff + ph.climb + ph.transition + ph.landing); %dynamic
% -------- Nominal shaft power levels (W) --------
P_takeoff = 260;
P_climb   = 170;
P_cruise  = 75;
P_land    = 55;
% proxy airspeed profile (m/s)
V_to   = 18;
V_cl   = 22;
V_cr   = 24;
V_ld   = 16;
% smoothstep for ramps (step response shock);creates an S-Curve
smoothstep = @(x) (x<=0).*0 + (x>=1).*1 + (x>0 & x<1).*(x.^2.*(3-2*x));%cubic Hermite interpolation polynomial $x^2(3 - 2x)$
Pshaft_W = zeros(N,1);
V_ms     = zeros(N,1);
phase    = strings(N,1);
k0 = 1;
% ---- Takeoff: ramp 0 -> P_takeoff over 15s, hold remainder ----
n = round(ph.takeoff/Ts); idx = k0:min(N,k0+n-1);
tau = (0:numel(idx)-1)'*Ts;
r = smoothstep(tau/15);
Pshaft_W(idx) = P_takeoff*r;
V_ms(idx)     = V_to*r;
phase(idx)    = "takeoff";
k0 = idx(end)+1;
% ---- Climb: ramp P_takeoff -> P_climb over 20s, then hold ----
n = round(ph.climb/Ts); idx = k0:min(N,k0+n-1);
tau = (0:numel(idx)-1)'*Ts;
r = smoothstep(tau/20);
Pshaft_W(idx) = P_takeoff + (P_climb - P_takeoff)*r;
V_ms(idx)     = V_to + (V_cl - V_to)*r;
phase(idx)    = "climb";
k0 = idx(end)+1;
% ---- Transition: ramp P_climb -> P_cruise over 30s ----
n = round(ph.transition/Ts); idx = k0:min(N,k0+n-1);
tau = (0:numel(idx)-1)'*Ts;
r = smoothstep(tau/30);
Pshaft_W(idx) = P_climb + (P_cruise - P_climb)*r;
V_ms(idx)     = V_cl + (V_cr - V_cl)*r;
phase(idx)    = "transition";
k0 = idx(end)+1;
% ---- Cruise: steady with small band-limited gust noise + a few maneuver bursts ----
n = round(ph.cruise/Ts); idx = k0:min(N,k0+n-1);
tau = (0:numel(idx)-1)'*Ts;
% band-limited gust noise: low-pass filtered white noise
rng(1); % repeatable
w = randn(numel(idx),1);
alpha = 0.02; % lower = smoother (time constant ~ Ts/alpha)
gust = zeros(size(w));
for i=2:numel(w)
    gust(i) = (1-alpha)*gust(i-1) + alpha*w(i);
end
gust = gust / max(1e-6, std(gust)); % normalize
gust_amp = 0.02; % 2% power ripple (realistic)
Pshaft_W(idx) = P_cruise*(1 + gust_amp*gust);
% maneuver bursts: +35% for 3 s at a few times
burst_dur_s = 3;
burst_amp   = 0.35;
burst_times_s = [600, 1800, 3000]; % seconds into mission (rough)
for bt = burst_times_s
    kk = round(bt/Ts)+1;
    kk = min(max(kk, idx(1)), idx(end));
    kk2 = min(idx(end), kk + round(burst_dur_s/Ts)-1);
    Pshaft_W(kk:kk2) = Pshaft_W(kk:kk2) * (1 + burst_amp);
end
V_ms(idx)  = V_cr * ones(numel(idx),1);
phase(idx) = "cruise";
k0 = idx(end)+1;
% ---- Landing: ramp P_cruise -> P_land over 40s, then hold ----
n = round(ph.landing/Ts); idx = k0:min(N,k0+n-1);
tau = (0:numel(idx)-1)'*Ts;
r = smoothstep(tau/40);
Pshaft_W(idx) = P_cruise + (P_land - P_cruise)*r;
V_ms(idx)     = V_cr + (V_ld - V_cr)*r;
phase(idx)    = "landing";
% ensure nonnegative shaft power
Pshaft_W = max(Pshaft_W, 0);
% -------- Drivetrain mapping (shaft -> battery) --------
if opts.use_drivetrain_map && exist('drivetrain_efficiency_map','file')==2
    eta_drv = drivetrain_efficiency_map(Pshaft_W);
    eta_drv = max(min(eta_drv, 0.95), 0.6);
else
    % Fallback constant efficiency if no map is provided
    eta_drv = 0.95*ones(size(Pshaft_W));
end
Pdem_W = Pshaft_W ./ eta_drv;
% propulsion-only: no regen
Pdem_W = max(Pdem_W, 0);
meta = struct();
meta.t = t;
meta.Pshaft_W = Pshaft_W;
meta.eta_drv  = eta_drv;
meta.V_ms     = V_ms;
meta.phase    = phase;
end
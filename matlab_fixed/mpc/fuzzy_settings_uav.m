function fz = fuzzy_settings_uav()
% FUZZY_SETTINGS_UAV  Fair, robust fuzzy baseline for comparison with MPC.
%
% Reviewer-facing policy:
%   - No SoC guard tapers or hidden SoC-dependent scaling
%   - No artificial Vmin buffers
%   - Any hard constraint enforcement is done via the same physical clamps
%     as MPC (I bounds + voltage feasibility in simulate_uav_with_fuzzy.m)

fz = struct();

% =========================
% Tracking / aggressiveness
% =========================
% Error normalization (W): larger => less aggressive, more stable
fz.e_scale_W = 150;

% Max magnitude of fuzzy delta-current command (A)
% (Not a guard; it's the controller output range)
fz.dI_max_A = 45;

% Rate limit (A/s): MUST be active for robustness and fairness (matches MPC-like actuation)
fz.du_max_A_per_s = 30;

% Integral action (small, bounded)
fz.Ki = 0.007;
fz.int_max = 0.25;

% =========================
% FAIR BASELINE POLICY
% =========================
% Disable any SoC moderation/guarding completely.
fz.soc_low  = -inf;
fz.soc_high = -inf;
fz.low_soc_limit_scale = 1.00;

% No extra protective voltage buffer (must be 0 for fairness)
fz.Vmin_buffer_V = 0.00;

% No soft scaling of current ceiling
fz.I_vsoft_factor = 1.00;

% =========================
% Voltage membership shaping (soft logic only)
% =========================
% These are NOT hard limits; they only affect fuzzy inference smoothly.
% Hard feasibility is enforced in simulate_uav_with_fuzzy via physics clamp.
fz.Vm_low_V  = 0.30;
fz.Vm_mid_V  = 0.60;
fz.Vm_high_V = 1.10;

% =========================
% Output smoothing (optional, recommended)
% =========================
fz.enable_iff_smoothing = true;
fz.iff_tau_s = 3.0;

end
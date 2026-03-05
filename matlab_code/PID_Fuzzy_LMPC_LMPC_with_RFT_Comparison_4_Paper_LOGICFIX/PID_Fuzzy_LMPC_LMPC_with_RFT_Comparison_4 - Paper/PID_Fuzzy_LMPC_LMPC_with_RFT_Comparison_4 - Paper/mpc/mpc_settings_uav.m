function mpc = mpc_settings_uav()

%MPC_SETTINGS_UAV Linear MPC settings.

mpc = struct();

mpc.use_yalmip = false;

mpc.yalmip_solver = 'quadprog';

% Horizon / sampling

mpc.N  = 30;

mpc.Ts = 1;

% RFT constraint (hard, linear, updated online)

mpc.use_rft_constraint = true;

mpc.RFT_spec_s = 0;     % overwritten in scripts

% ===== RFT strengthening knobs =====

mpc.rft_gain        = 100;    % stronger early RFT activation

mpc.rft_soc_buffer  = 0.10;   % reserve margin above SoC_min

mpc.rft_margin_bias = 0.006;  % compensates observed negative margin bias

% Numerical tightening margins

mpc.soc_eps = 0.003;

mpc.v_eps   = 0.04;

% --- TUNING ---

mpc.qP   = 500;     % tracking weight

mpc.qdI  = 18;      % strong smoothness for feasible trajectories

mpc.qI   = 5e-3;    % stronger magnitude regularization

% --- Constraints ---

mpc.enable_terminal_soc_buffer = false;

mpc.SoC_buffer = 0.0;

mpc.enforce_power_min = true;

mpc.Pcrit_W   = 150;

mpc.Pmargin_W = 0.0;

% Safety Margins

mpc.Vmin_margin = 0.00;

mpc.Tmax_margin = 1.0;

% Terminal pull (optional, mild)

mpc.qSoC_terminal = 200;

mpc.SoC_target    = 0.2;

mpc.quadprog_opts = optimoptions('quadprog','Display','off','Algorithm','interior-point-convex');

end
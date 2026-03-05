function pid = pid_settings_uav()

pid = struct();

% PID on power error (W) -> delta current (A) via division by Voc
pid.Kp_W = 0.18;      % proportional gain (A per (W/Voc)) 0.18
pid.Ki_W = 0.015;     % integral gain  0.015
pid.Kd_W = 0.00;      % derivative gain  00

pid.Iint_max_A = 40;  % anti-windup clamp on integral contribution (A)40

% Optional SoC guard: reduce commanded current as SoC approaches minimum.
pid.enable_soc_guard = false;
pid.soc_guard_start  = 0.22;  % start taper
pid.soc_guard_min    = 0.12;  % fully limit at/under this SoC
pid.soc_guard_scale  = 0.35;  % remaining fraction when fully guarded
end
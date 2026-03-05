function OUT = sweep_phase_adaptive_rft(simfun, par, t1, t2, hi_list_hr, lo_list_hr)
%SWEEP_PHASE_ADAPTIVE_RFT
% simfun: @(mpc_struct)->log
% OUT: table of results

rows = numel(hi_list_hr)*numel(lo_list_hr);
OUT = table('Size',[rows 7], ...
    'VariableTypes',{'double','double','double','double','double','double','double'}, ...
    'VariableNames',{'RFT_hi_hr','RFT_lo_hr','Endurance_min','Unmet_kWh','SoC_end','SoC_min','TradeoffScore'});

k = 0;

for i = 1:numel(hi_list_hr)
    for j = 1:numel(lo_list_hr)

        hi_hr = hi_list_hr(i);
        lo_hr = lo_list_hr(j);

        if lo_hr >= hi_hr
            continue;
        end

        mpc = mpc_settings_uav();
        mpc.use_rft_constraint = true;

        % STRICT in critical window:
        mpc.RFT_schedule = struct('t1',t1,'t2',t2,'RFT_hi_s',hi_hr*3600,'RFT_lo_s',lo_hr*3600);

        log = simfun(mpc);
        S = summarize_log_uav(log, par);
        
% ===== Auto-tune schedule using MC slack statistics (research-grade) =====
optsTune = struct();
optsTune.active_tol = RFT_ACTIVE_TOL;
optsTune.target_crit_active_pct = 60; % aim: constraint meaningfully active in critical
optsTune.max_outer_active_pct   = 40; % avoid too much activity outside critical (save energy)
optsTune.base_hi_hr = 3.0;
optsTune.base_lo_hr = 1.5;
optsTune.step_hr    = 0.25;
optsTune.min_lo_hr  = 1.0;
optsTune.max_hi_hr  = 4.0;

sched_auto = auto_tune_rft_schedule_from_mc(tgrid, S, t_critical_start, t_critical_end, optsTune);

disp('=== Auto-tuned schedule from Monte Carlo slack ===');
disp(sched_auto);
disp(sched_auto.stats);

% ===== Run auto-tuned schedule on the SAME disturbed mission =====
mpc_auto = mpc0;
mpc_auto.use_rft_constraint = true;
mpc_auto.RFT_schedule = sched_auto;

[log_auto, ~] = runMPC(Pdem_true_W, mpc_auto, Tamb_K);

figure('Name','Constant vs Auto-tuned Phase-adaptive RFT'); hold on; grid on;
plot(log_rft.t,  log_rft.SoC,  'LineWidth',1.6);
plot(log_auto.t, log_auto.SoC, 'LineWidth',1.6);
xline(t_critical_start,'--'); xline(t_critical_end,'--');
xlabel('t [s]'); ylabel('SoC');
title('Constant RFT vs Auto-tuned Phase-adaptive RFT');
legend('Constant RFT (3h)','Auto-tuned phase-adaptive','Location','best');
        % score: strong unmet penalty + mild endurance penalty
        wU = 1.0;
        wE = 0.02;
        score = wU*S.Unmet_kWh + wE*(max(0,(50 - S.Endurance_min)));

        k = k + 1;
        OUT{k,'RFT_hi_hr'}     = hi_hr;
        OUT{k,'RFT_lo_hr'}     = lo_hr;
        OUT{k,'Endurance_min'} = S.Endurance_min;
        OUT{k,'Unmet_kWh'}     = S.Unmet_kWh;
        OUT{k,'SoC_end'}       = S.SoC_end;
        OUT{k,'SoC_min'}       = S.SoC_min;
        OUT{k,'TradeoffScore'} = score;
    end
end

OUT = OUT(1:k,:);

% Paper-style plots
figure('Name','Sweep tradeoff score'); grid on;
scatter3(OUT.RFT_hi_hr, OUT.RFT_lo_hr, OUT.TradeoffScore, 40, OUT.TradeoffScore, 'filled');
xlabel('RFT_{hi} [h]'); ylabel('RFT_{lo} [h]'); zlabel('Score');
title('Phase-adaptive sweep: TradeoffScore (lower is better)');

figure('Name','Sweep unmet energy'); grid on;
scatter3(OUT.RFT_hi_hr, OUT.RFT_lo_hr, OUT.Unmet_kWh, 40, OUT.Unmet_kWh, 'filled');
xlabel('RFT_{hi} [h]'); ylabel('RFT_{lo} [h]'); zlabel('Unmet [kWh]');
title('Phase-adaptive sweep: Unmet energy');

figure('Name','Sweep endurance'); grid on;
scatter3(OUT.RFT_hi_hr, OUT.RFT_lo_hr, OUT.Endurance_min, 40, OUT.Endurance_min, 'filled');
xlabel('RFT_{hi} [h]'); ylabel('RFT_{lo} [h]'); zlabel('Endurance [min]');
title('Phase-adaptive sweep: Endurance');

[~,idx] = min(OUT.TradeoffScore);
disp('=== Best schedule from sweep (min TradeoffScore) ===');
disp(OUT(idx,:));
%% ===== Paper figure: (RFT_hi, RFT_lo) sweep =====
hi_list_hr = 2.5:0.25:4.0;
lo_list_hr = 1.0:0.25:2.5;

simfun = @(mpc_in) simulate_uav_with_mpc(Pdem_true_W, t, Ts, par, x0, Tamb_K, mpc_in, cfg_pred);

OUT = sweep_phase_adaptive_rft(simfun, par, t_critical_start, t_critical_end, hi_list_hr, lo_list_hr);

disp('=== Sweep results table (phase-adaptive schedules) ===');
disp(OUT);
end
function plot_prediction_error_lmpc_nmpc(logL, predL, logN, predN, mpc)
% plot_prediction_error_lmpc_nmpc
% Quantitative prediction error comparison LMPC vs NMPC.
% Uses snapshot predictions stored in predlog and compares them to the
% TRUE future evolution from the logged closed-loop trajectory.
%
% Outputs:
% - RMSE vs time for Pdel, Vt, SoC (LMPC dashed blue, NMPC solid red)
% - Bar plot of mean RMSE
% - Command window RMSE summary

if ~isfield(mpc,'N')
    error('plot_prediction_error_lmpc_nmpc: mpc.N is required.');
end
N = mpc.N;

[rmseL, t0L] = compute_rmse(logL, predL, N);
[rmseN, t0N] = compute_rmse(logN, predN, N);

if numel(t0L)==0 || numel(t0N)==0
    warning('plot_prediction_error_lmpc_nmpc: empty prediction logs.');
    return;
end

%% RMSE vs time (Pdel)
figure('Name','Prediction RMSE (Pdel)');
grid on; hold on;
plot(t0L, rmseL.Pdel, 'b--', 'LineWidth', 1.6);
plot(t0N, rmseN.Pdel, 'r-',  'LineWidth', 1.6);
xlabel('t_0 [s]');
ylabel('RMSE of P_{del} prediction [W]');
title('Prediction RMSE over horizon snapshots: Delivered Power');
legend('LMPC','NMPC','Location','best');

%% RMSE vs time (Vt)
figure('Name','Prediction RMSE (Vt)');
grid on; hold on;
plot(t0L, rmseL.Vt, 'b--', 'LineWidth', 1.6);
plot(t0N, rmseN.Vt, 'r-',  'LineWidth', 1.6);
xlabel('t_0 [s]');
ylabel('RMSE over horizon [V]');
title('Prediction RMSE over horizon snapshots: Terminal Voltage');
legend('LMPC','NMPC','Location','best');

%% RMSE vs time (SoC)
figure('Name','Prediction RMSE (SoC)');
grid on; hold on;
plot(t0L, rmseL.SoC, 'b--', 'LineWidth', 1.6);
plot(t0N, rmseN.SoC, 'r-',  'LineWidth', 1.6);
xlabel('t_0 [s]');
ylabel('RMSE over horizon [-]');
title('Prediction RMSE over horizon snapshots: SoC');
legend('LMPC','NMPC','Location','best');

%% Mean RMSE bar plot
meanL = [mean(rmseL.Pdel,'omitnan'), mean(rmseL.Vt,'omitnan'), mean(rmseL.SoC,'omitnan')];
meanN = [mean(rmseN.Pdel,'omitnan'), mean(rmseN.Vt,'omitnan'), mean(rmseN.SoC,'omitnan')];

figure('Name','Mean prediction RMSE (LMPC vs NMPC)');
bar([meanL; meanN]);
grid on;
set(gca,'XTickLabel',{'LMPC','NMPC'});
legend({'Pdel [W]','Vt [V]','SoC [-]'},'Location','best');
title('Mean prediction RMSE over horizon');

%% Print summary
fprintf('\n--- Prediction RMSE over horizon (mean over snapshots) ---\n');
fprintf('LMPC: mean RMSE Pdel = %.3f W, Vt = %.4f V, SoC = %.5f\n', meanL(1), meanL(2), meanL(3));
fprintf('NMPC: mean RMSE Pdel = %.3f W, Vt = %.4f V, SoC = %.5f\n', meanN(1), meanN(2), meanN(3));
end

% =====================================================================
% RMSE computation (returns struct with fields Pdel, Vt, SoC)
% =====================================================================
function [rmse, t0] = compute_rmse(logS, pred, N)

rmse = struct('Pdel',[],'Vt',[],'SoC',[]);
t0   = [];

% ---- get truth signals from log ----
tlog = get_signal(logS, {'t','time'});
Pdel = get_signal(logS, {'Pdel','P_del','Pdel_W'});
Vt   = get_signal(logS, {'Vt','V','Vterm'});
SoC  = get_signal(logS, {'SoC','soc'});

if numel(tlog)==0 || numel(Pdel)==0
    return;
end
tlog = tlog(:);
Pdel = Pdel(:);

if numel(Vt)>0,  Vt  = Vt(:);  end
if numel(SoC)>0, SoC = SoC(:); end

% ---- normalize pred format to array S ----
S = [];
if ~isstruct(pred) || numel(pred)==0
    return;
elseif isfield(pred,'snapshots')
    S = pred.snapshots;
else
    S = pred; % already struct array (your predlog format)
end

if ~isstruct(S) || numel(S)==0
    return;
end

M = numel(S);
rmseP = nan(M,1);
rmseV = nan(M,1);
rmseS = nan(M,1);
t0v   = nan(M,1);

for k = 1:M
    sk = S(k);

    % time origin
    if isfield(sk,'t0') && numel(sk.t0)>0
        t0k = sk.t0;
    elseif isfield(sk,'t') && numel(sk.t)>0
        t0k = sk.t(1);
    else
        continue;
    end

    % predicted horizon time
    if isfield(sk,'t_pred')
        tp = sk.t_pred(:);
    elseif isfield(sk,'t')
        tp = sk.t(:);
    else
        continue;
    end

    % predicted signals (try multiple naming conventions)
    Pp = pick_pred(sk, {'Pdel_pred','Pdel','P_pred','P'});
    Vp = pick_pred(sk, {'Vt_pred','Vt','V_pred','V'});
    Sp = pick_pred(sk, {'SoC_pred','SoC','soc_pred','soc'});

    % At minimum, require P prediction
    if numel(Pp)==0
        continue;
    end

    nh = min([numel(tp), numel(Pp), N]);
    if nh < 2, continue; end
    tp = tp(1:nh);
    Pp = Pp(1:nh);

    % truth interpolated onto predicted times
    Ptr = interp1(tlog, Pdel, tp, 'linear', 'extrap');
    eP = Pp - Ptr;
    rmseP(k) = sqrt(mean(eP.^2));

    % optional Vt RMSE
    if numel(Vp)>0 && numel(Vt)>0
        nhv = min([numel(tp), numel(Vp), numel(tp)]);
        Vp2 = Vp(1:nhv);
        Vtr = interp1(tlog, Vt, tp(1:nhv), 'linear', 'extrap');
        eV = Vp2 - Vtr;
        rmseV(k) = sqrt(mean(eV.^2));
    end

    % optional SoC RMSE
    if numel(Sp)>0 && numel(SoC)>0
        nhs = min([numel(tp), numel(Sp), numel(tp)]);
        Sp2 = Sp(1:nhs);
        Str = interp1(tlog, SoC, tp(1:nhs), 'linear', 'extrap');
        eS = Sp2 - Str;
        rmseS(k) = sqrt(mean(eS.^2));
    end

    t0v(k) = t0k;
end

mask = isfinite(rmseP) & isfinite(t0v);
rmse.Pdel = rmseP(mask);
rmse.Vt   = rmseV(mask);
rmse.SoC  = rmseS(mask);
t0        = t0v(mask);
end

% -------- helper: select first non-empty predicted field ----------
function y = pick_pred(s, names)
y = [];
for i=1:numel(names)
    nm = names{i};
    if isfield(s,nm)
        tmp = s.(nm);
        if isnumeric(tmp) && numel(tmp)>0
            y = tmp(:);
            return;
        end
    end
end
end

% -------- helper: signal getter without isempty ----------
function y = get_signal(s, names)
y = [];
if ~isstruct(s), return; end
for i=1:numel(names)
    nm = names{i};
    if isfield(s,nm)
        tmp = s.(nm);
        if isnumeric(tmp) && numel(tmp)>0
            y = tmp;
            return;
        end
    end
end
end
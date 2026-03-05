
function plot_compare_baseline_lmpc_nmpc(log_base, log_lmpc, log_nmpc, par, meta)
cfg = struct('export', false, 'show_pred', false);
plot_uav_publication_figures(log_base, log_lmpc, log_nmpc, par, meta, cfg);
end

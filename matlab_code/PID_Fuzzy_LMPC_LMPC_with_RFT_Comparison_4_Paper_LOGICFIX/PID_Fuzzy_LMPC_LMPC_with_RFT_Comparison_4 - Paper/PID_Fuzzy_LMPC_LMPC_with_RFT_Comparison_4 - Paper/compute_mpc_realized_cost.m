function J = compute_mpc_realized_cost(log, mpc, varargin)
    p = inputParser;
    p.addParameter('IncludeTerminal', true, @(x)islogical(x) && isscalar(x));
    p.parse(varargin{:});
    incTerminal = p.Results.IncludeTerminal;

    % Guard: align vectors
    n = min([numel(log.Pdem), numel(log.Pdel), numel(log.I), numel(log.SoC)]);
    Pdem = log.Pdem(1:n);
    Pdel = log.Pdel(1:n);
    I    = log.I(1:n);
    SoC  = log.SoC(1:n);

    % Defaults if fields missing
    if ~isfield(mpc,'qP'), mpc.qP = 1; end
    if ~isfield(mpc,'qI'), mpc.qI = 0; end
    if ~isfield(mpc,'qdI'), mpc.qdI = 0; end
    if ~isfield(mpc,'qSoC_terminal'), mpc.qSoC_terminal = 0; end
    if ~isfield(mpc,'SoC_target'), mpc.SoC_target = SoC(end); end

    eP = Pdem - Pdel;
    dI = [I(1); diff(I)];

    J.track  = mpc.qP  * sum(eP.^2);
    J.mag    = mpc.qI  * sum(I.^2);
    J.slew   = mpc.qdI * sum(dI.^2);
    if incTerminal
        J.term = mpc.qSoC_terminal * (SoC(end) - mpc.SoC_target).^2;
    else
        J.term = 0;
    end
    J.total = J.track + J.mag + J.slew + J.term;
end

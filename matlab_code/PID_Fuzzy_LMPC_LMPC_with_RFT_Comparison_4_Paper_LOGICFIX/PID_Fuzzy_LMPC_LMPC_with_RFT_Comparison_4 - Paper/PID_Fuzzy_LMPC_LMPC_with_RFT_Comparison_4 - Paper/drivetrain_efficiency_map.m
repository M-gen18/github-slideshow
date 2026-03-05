function eta = drivetrain_efficiency_map(Pshaft_W)
P = max(Pshaft_W(:),0);
% Parameters (tuneable)
eta_low  = 0.70;   % very low power efficiency
eta_peak = 0.86;   % peak efficiency
eta_high = 0.80;   % near max power efficiency
P1 = 30;           % [W] transition out of low-power region
P2 = 200;          % [W] around peak
P3 = 300;          % [W] high power region start
eta = zeros(size(P));
% Region 1: [0, P1] ramp up
i1 = P <= P1;
eta(i1) = eta_low + (eta_peak-0.02-eta_low)*(P(i1)/max(P1,1));
% Region 2: [P1, P2] approach peak
i2 = (P > P1) & (P <= P2);
eta(i2) = (eta_peak-0.02) + (eta_peak - (eta_peak-0.02))*((P(i2)-P1)/(P2-P1));
% Region 3: [P2, P3] slight drop from peak
i3 = (P > P2) & (P <= P3);
eta(i3) = eta_peak - (eta_peak-eta_high)*((P(i3)-P2)/(P3-P2));
% Region 4: >P3 clamp
i4 = P > P3;
eta(i4) = eta_high;
eta = reshape(eta, size(Pshaft_W));
end
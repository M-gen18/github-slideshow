function eta = drivetrain_efficiency_map_uav(Pshaft_W)
%DRIVETRAIN_EFFICIENCY_MAP_UAV BLDC+ESC+prop efficiency surrogate.
P = max(Pshaft_W(:), 0);
Pnorm = P / 250;  % normalize around ~250 W peak
eta = 0.70 + 0.18*(1 - exp(-3*Pnorm));  % rises with power
eta = eta .* (1 - 0.05*(Pnorm.^2));     % mild roll-off
eta = min(max(eta, 0.60), 0.92);
end

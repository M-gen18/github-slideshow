function id = fit_thermal_least_squares(t, I, T, Tamb, id_elec, Ts)
I=I(:); T=T(:); Tamb=Tamb(:);
Rloss = id_elec.R0 + id_elec.R1 + id_elec.R2;

y = (T(2:end)-T(1:end-1))/Ts;
phi1 = I(1:end-1).^2;
phi2 = (T(1:end-1)-Tamb(1:end-1));
Phi=[phi1, phi2];
theta=Phi\y;

alpha=theta(1); beta=theta(2);
Cth = Rloss/max(1e-12,alpha);
hA  = -beta*Cth;

That=zeros(numel(T),1); That(1)=T(1);
for k=1:numel(T)-1
    Qdot = I(k)^2*Rloss;
    That(k+1)=That(k)+Ts*(Qdot - hA*(That(k)-Tamb(k)))/Cth;
end
rmse=sqrt(mean((T-That).^2));

id=struct('Rloss',Rloss,'Cth',Cth,'hA',hA,'rmse_T_K',rmse);

figure('Name','Thermal fit');
plot(t,T-273.15,'LineWidth',1.0); hold on; plot(t,That-273.15,'LineWidth',1.0);
grid on; xlabel('t [s]'); ylabel('T [C]'); legend('Measured','Fit');
title(sprintf('RMSE=%.3f K',rmse));
end

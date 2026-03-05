function id = fit_2rc_least_squares(t, I, V, Voc, Ts, opt)
if nargin<6, opt=struct(); end
if ~isfield(opt,'verbose'), opt.verbose=false; end
if ~isfield(opt,'maxIter'), opt.maxIter=2000; end
I=I(:); V=V(:); Voc=Voc(:);

R0_0=0.02; a1_0=exp(-Ts/8); a2_0=exp(-Ts/80);
R1_0=0.30*R0_0; R2_0=0.50*R0_0;
b1_0=(1-a1_0)*R1_0; b2_0=(1-a2_0)*R2_0;
theta0=[R0_0,a1_0,b1_0,a2_0,b2_0];

pen=@(th) 1e6*(th(1)<=0) + 1e6*(th(2)<=0||th(2)>=1) + 1e6*(th(4)<=0||th(4)>=1) + 1e6*(th(3)<0) + 1e6*(th(5)<0);
cost=@(th) mean((V - simV(th,I,Voc)).^2) + pen(th);

opts=optimset('MaxIter',opt.maxIter,'MaxFunEvals',opt.maxIter*5,'Display', ternary(opt.verbose,'iter','off'));
theta=fminsearch(cost,theta0,opts);

[Vhat,v1,v2]=simAll(theta,I,Voc);
rmse=sqrt(mean((V-Vhat).^2));

R0=theta(1); a1=theta(2); b1=theta(3); a2=theta(4); b2=theta(5);
R1=b1/max(1e-9,(1-a1)); R2=b2/max(1e-9,(1-a2));
C1=-Ts/max(1e-9,(R1*log(a1))); C2=-Ts/max(1e-9,(R2*log(a2)));

id=struct('R0',R0,'R1',R1,'C1',C1,'tau1',R1*C1,'R2',R2,'C2',C2,'tau2',R2*C2,'rmse_V',rmse,'theta',theta);

figure('Name','2RC voltage fit');
plot(t,V,'LineWidth',1.0); hold on; plot(t,Vhat,'LineWidth',1.0);
grid on; xlabel('t [s]'); ylabel('V [V]'); legend('Measured','Fit');
title(sprintf('RMSE=%.4f V',rmse));

figure('Name','Polarization states'); plot(t,v1,'LineWidth',1.0); hold on; plot(t,v2,'LineWidth',1.0);
grid on; xlabel('t [s]'); ylabel('v [V]'); legend('v1','v2');

    function Vhat = simV(th,I,Voc)
        Vhat = simAll(th,I,Voc);
    end
    function [Vhat,v1,v2]=simAll(th,I,Voc)
        R0=th(1); a1=th(2); b1=th(3); a2=th(4); b2=th(5);
        N=numel(I);
        v1=zeros(N,1); v2=zeros(N,1); Vhat=zeros(N,1);
        for k=1:N
            Vhat(k)=Voc(k)-I(k)*R0 - v1(k)-v2(k);
            if k<N
                v1(k+1)=a1*v1(k)+b1*I(k);
                v2(k+1)=a2*v2(k)+b2*I(k);
            end
        end
    end
    function out=ternary(cond,a,b)
        if cond, out=a; else, out=b; end
    end
end

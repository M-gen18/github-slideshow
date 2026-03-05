function pulse = generate_pulse_profile(Ts)
if nargin<1, Ts=1; end
seg=@(dur,I) struct('dur',dur,'I',I);
I1=10; I2=20; I3=-10;
S=[seg(300,0), seg(60,I1), seg(600,0), seg(60,I2), seg(600,0), seg(30,I3), seg(900,0)];
t=[]; I=[]; tk=0;
for s=1:numel(S)
    n=round(S(s).dur/Ts);
    ts=(tk:Ts:tk+(n-1)*Ts)';
    t=[t;ts]; %#ok<AGROW>
    I=[I;S(s).I*ones(n,1)]; %#ok<AGROW>
    tk=tk+n*Ts;
end
pulse.t_s=t; pulse.I_A=I;
figure('Name','Pulse profile'); plot(t,I,'LineWidth',1.0); grid on;
xlabel('t [s]'); ylabel('I [A]');
end

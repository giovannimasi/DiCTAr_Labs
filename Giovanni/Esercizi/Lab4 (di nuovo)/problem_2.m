clearvars; clc;

%% Definitions

A=[0 1;0 -1];
B=[0;1];
C=[1 0];
D=0;

Ts=0.05;
x0=[0;0];

sys=ss(A,B,C,D);
sys_x=ss(A,B,eye(2),zeros(2,1));
sys_dt=c2d(sys,Ts);
[Ad,Bd,Cd,Dd]=ssdata(sys_dt);

%% Design

Aaug=[1 -Ts*Cd;zeros(2,1) Ad];
Baug=[0;Bd];
Caug=[0 Cd];

Q=diag([100000 1 1]);
R=10000;
Cq=chol(Q);

Mr=ctrb(Aaug,Baug); rho_r=rank(Mr)
Mo=obsv(Aaug,Cq); rho_o=rank(Mo)

K=dlqr(Aaug,Baug,Q,R);

Kq=K(1)
Kx=K(2:3)

% Simulation

t_sim=10;

out=sim("problem_2_sim.slx");

figure; plot(out.y.time,out.y.data,'b'), grid on, xlabel('t'), ylabel('y(t)'),
xline(2+2*0.02,':g'), xline(2-2*0.02,':g'), yline(1+0.01,':r'), yline(1-0.01,':r');

figure; plot(out.u.time,out.u.data,'b'), grid on, xlabel('t'), ylabel('u(t)'),
yline(1,':r'), yline(-1,':r');

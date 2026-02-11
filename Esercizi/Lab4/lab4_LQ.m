clearvars;close all;clc;

%% Definitions

A=[0 1;0 -1];
B=[0;1];
C=[1 0];
D=0;
Ts=0.05;
x0=[0.8;0];

sys=ss(A,B,C,D);
sys_dt=c2d(sys,Ts);
sys_x=ss(A,B,eye(2),zeros(2,1));
[Ad,Bd,Cd,Dd]=ssdata(sys_dt);

%% Design

Mr=ctrb(Ad,Bd); rho_r=rank(Mr)

Q=diag([200 1])
R=1;

Cq=chol(Q);
Mo=obsv(Ad,Cq); rho_o=rank(Mo)

K=dlqr(Ad,Bd,Q,R);

% Simulation

t_sim=6;
out=sim("lab4_LQ_sim.slx");

figure(1); plot(out.x.time, sqrt(out.x.data(:,1) .^2+out.x.data(:,2) .^2),'b'), grid on, hold on,
xlabel('t'), ylabel('||x(t)||_2'), xline(5-0.02*5,':g'), xline(5+0.02*5,':g'),
yline(1e-5,':r'),yline(-1e-5,':r');

%% Problem 2
close all;clc;

Aaug=[1 -Ts*Cd;zeros(2,1) Ad];
Baug=[0;Bd];
Caug=[0 C];
Daug=0;

x0=[0;0];
x0i=0;
Q=diag([100000 1 1]);
R=1;

Cq=chol(Q);
Mo=obsv(Aaug,Cq); rho_o=rank(Mo)

K=dlqr(Aaug,Baug,Q,R)
Kq=K(1)
Kx=K(2:3)

% Simulation

t_sim=6;

out=sim("lab4_LQ_tracking_sim.slx");

figure(1); plot(out.y.time, out.y.data,'b'), grid on, hold on,
xlabel('t'), ylabel('y(t)'), xline(2-0.02*2,':g'), xline(2+0.02*2,':g'),
yline(r+r*0.01,':r'),yline(r-r*0.01,':r');
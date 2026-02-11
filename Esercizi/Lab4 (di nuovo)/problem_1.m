clearvars; close all; clc;

%% Definitions

A=[0 1;0 -1];
B=[0;1];
C=[1 0];
D=0;

Ts=0.05;
x0=[0.8;0];

sys=ss(A,B,C,D);
sys_x=ss(A,B,eye(2),zeros(2,1));
sys_dt=c2d(sys,Ts);
[Ad,Bd,Cd,Dd]=ssdata(sys_dt);
%% Design

Mr=ctrb(Ad,Bd); rho_r=rank(Mr)

Q=diag([100 1]);
R=1;
Cq=chol(Q);

Mo=obsv(Ad,Cq); rho_o=rank(Mo)

K=dlqr(Ad,Bd,Q,R);

% Simulation

t_sim=10;

out=sim("problem_1_sim.slx");

figure; plot(out.x.time,sqrt(out.x.data(:,1) .^2+out.x.data(:,2) .^2),'b'),
grid on, xlabel('t'), ylabel('||x(t)||_2'), 
xline(5-5*0.02,':g'), xline(5+5*0.02,':g'), yline(10e-5,':r'), yline(-10e-5,':r');
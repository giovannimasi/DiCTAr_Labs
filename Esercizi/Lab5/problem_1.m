clearvars; close all; clc;

%% Definitions

A=[0 1;0 -1];
B=[0;1];
C=[1 0];
D=0;
Ts=0.05;
n=2;

sys=ss(A,B,C,D);
sys_x=ss(A,B,eye(n),zeros(n,1));
sys_dt=c2d(sys,Ts);
[Ad,Bd,Cd,Dd]=ssdata(sys_dt);

%% Design
Cy=eye(n); Cz=Cd; Cc=Cd;
Dz=Dd; Dc=Dd;

Q=100;
R=1;

u_max=1;
u_min=-1;
z_max=Inf;
z_min=-Inf;
du_max=Inf;
du_min=-Inf;
zblk=1;
ublk=1;

Hp=40;
Hc=11;
Hw=1;

md = MPCInit(Ad,Bd,Cy,Cz,Dz,Cc,Dc,Hp,Hw,zblk,Hc,ublk, ...
 		      du_max,du_min,u_max,u_min,z_max, ...
 		      z_min,Q,R,[],[],Ts,0,'qp_as');

% simulation

t_sim=5;
r=1;
out=sim("problem_1_sim.slx");

figure(1); plot(out.y.time, out.y.data,'b'), grid on, hold on,
xlabel('t'), ylabel('y(t)'), xline(2-2*0.05,':g'), xline(2+2*0.05,':g'),
yline(1.01,':r'),yline(0.99,':r');
clearvars; close all; clc;

%% Definitions

A=[0 1;0 -0.1];
B=[0;0.1];
C=[1 0];
D=0;
x0=[0;0];

Ts=50e-3;

sys=ss(A,B,C,D);
sys_x=ss(A,B,eye(2),0);
sys_dt=c2d(sys,Ts);
[Ad,Bd,Cd,Dd]=ssdata(sys_dt);

%% Design
Cy=eye(2); Cz=Cd; Cc=Cd;
Dz=0; Dc=0;

Q=100;
R=1;

u_max=10;
u_min=-10;
z_max=1.02;
z_min=-Inf;
du_max=2;
du_min=-2;
zblk=1;
ublk=1;
cmode=0;

Hp=40;
Hc=16;
Hw=1;

md = MPCInit(Ad,Bd,Cy,Cz,Dz,Cc,Dc,Hp,Hw,zblk,Hc,ublk, ...
 		      du_max,du_min,u_max,u_min,z_max, ...
 		      z_min,Q,R,[],[],Ts,cmode,'qp_as');

% Simulation

t_sim=5;
r=1;

out=sim("s360159_sim_MPC.slx");

figure(1);
plot(out.y.time,out.y.data,'b'), grid on, hold on,
xlabel('t'), ylabel('y(t)'),
xline(2-0.1*2,':g'), xline(2+0.1*2,':g'),
yline(r-0.02*r,':r'), yline(r+0.02*r,':r');
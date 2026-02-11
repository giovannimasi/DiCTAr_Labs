clearvars; close all; clc;

%% Definitions

A=[0 1;0 -1];
B=[0;1];
C=[1 0];
D=0;

x0=[1;0];
Ts=0.05;

n=2;

sys=ss(A,B,C,D);
sys_x=ss(A,B,eye(n),zeros(n,1));
sys_dt=c2d(sys,Ts);
[Ad,Bd,Cd,Dd]=ssdata(sys_dt);

%% Design
Cy=eye(n); Cz=Cy; Cc=Cy;
Dz=zeros(n,1); Dc=Dz;

Q=diag([100 1]);
R=1;

u_max=5.5;
u_min=-5.5;
z_max=[Inf Inf];
z_min=-[Inf Inf];
du_max=3;
du_min=-3;
ublk=1;
zblk=1;
cmode=0;

Hp=50;
Hc=5;
Hw=1;

md = MPCInit(Ad,Bd,Cy,Cz,Dz,Cc,Dc,Hp,Hw,zblk,Hc,ublk, ...
 		      du_max,du_min,u_max,u_min,z_max, ...
 		      z_min,Q,R,[],[],Ts,cmode,'qp_as');

% Simulation

t_sim=4;

out=sim("s360159_sim_MPC.slx");

figure(1); plot(out.x.time, sqrt(out.x.data(:,1) .^2+out.x.data(:,2) .^2),'b'),
hold on, grid on, xlabel('t'), ylabel('||x(t)||_2'), 
yline(1e-4,':r'), yline(-1e-4,':r'), xline(2.5-2.5*0.05,':g'),xline(2.5+2.5*0.05,':g');
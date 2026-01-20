clearvars; close all; clc;

%% Part II - MPC

A=[0 1;0 -0.1];
B=[0;0.1];
C=[1 0];
D=0;

Ts=50e-3;

x0=[0;0];

sys=ss(A,B,C,D);
sys_dt=c2d(sys,Ts);
sys_x=ss(A,B,eye(2),0);

[Ad,Bd,Cd,Dd]=ssdata(sys_dt)

%% Design

Cy=Cd; Cz=eye(2); Cc=Cz;
Dz=zeros(2,1); Dc=Dz;

Q=diag([50 1]);
R=1;

u_max=10;
u_min=-10;
z_max=[Inf Inf];
z_min=-[Inf Inf];
du_max=2;
du_min=-2;
zblk=1;
ublk=1;

Hp=50;
Hu=19;
Hw=1;

md=MPCInit(Ad,Bd,Cy,Cz,Dz,Cc,Dc,Hp,Hw,zblk,Hu,ublk,du_max,du_min,u_max,u_min,z_max,z_min,Q,R,[],[],Ts,0,'qp_as');

%% Simulation

t_sim=3;

r=1;
out=sim("s360159_sim_MPC.slx");

y=out.y;
u=out.u;
x=out.x;

figure;
subplot(221), plot(x.time,x.data(:,1),'b'), grid on, xlabel('t'), ylabel('x_1(t)');
subplot(222), plot(x.time,x.data(:,2),'b'), grid on, xlabel('t'), ylabel('x_2(t)');
subplot(223), plot(y.time,y.data-r,'b'), grid on, xlabel('t'), ylabel('e(t)'), xline(1.8,':g'),xline(2.2,':g'),yline(0.02,':r'),yline(-0.02,':r');
subplot(224), plot(u.time,abs(u.data),'b'), grid on, xlabel('t'), ylabel('|u(t)|');
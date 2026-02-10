clearvars; close all; clc;

%% Part II - MPC

A=[0 1;0 -1];
B=[0;1];
C=[1 0];
D=0;

x0=[1;0];
Ts=0.05;

sys=ss(A,B,C,D);
sys_x=ss(A,B,eye(2),0);
sys_dt=c2d(sys,Ts);

[Ad,Bd,Cd,Dd]=ssdata(sys_dt);

treg=2.5;

%% Design

Q=diag([100 1]);
R=1;

Cy=eye(2); Cz=Cy; Cc=Cy;
Dz=zeros(2,1); Dc=Dz;

u_max=5.5;
u_min=-5.5;
z_max=[Inf Inf];
z_min=-[Inf Inf];
du_max=3;
du_min=-3;
ublk=1;
zblk=1;

Hp=22;
Hc=5;
Hw=1;

 md = MPCInit(Ad,Bd,Cy,Cz,Dz,Cc,Dc,Hp,Hw,zblk,Hc,ublk, ...
 		      du_max,du_min,u_max,u_min,z_max, ...
 		      z_min,Q,R,[],[],Ts,0,'qp_as');

% Simulation

t_sim=3;

out=sim("s360159_sim_MPC.slx");

figure(1);
subplot(221), plot(out.x.time,out.x.data(:,1),'b'), grid on,
xlabel('Time (s)'), ylabel('x_1(t)');
subplot(222), plot(out.x.time,out.x.data(:,2),'b'), grid on,
xlabel('Time (s)'), ylabel('x_1(t)');
subplot(223), plot(out.x.time,sqrt(out.x.data(:,1) .^2+out.x.data(:,2) .^2),'b'), grid on,hold on,
xlabel('Time (s)'), ylabel('||x(t||_2(t)'),
xline(treg+treg*5/100,':g'), xline(treg-treg*5/100,':g'),
yline(1e-4,':r'),yline(-1e-4,':r');
subplot(224), plot(out.u.time,out.u.data,'b'), grid on,
xlabel('Time (s)'), ylabel('u(t)');
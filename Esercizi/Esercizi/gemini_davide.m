clearvars; close all; clc;

%% Algebraic

% Definitions

Ts=0.05;
z=tf('z',Ts);
G=minreal(zpk(0.02*(z+0.8)/((z-1)*(z-1.25)*(z-0.4))))

%% Requirements

l1=1; l=3; l2=l-l1;

% first requirement is more restrictive

s_hat=0.05;
ts_1=1.2;

zeta=abs(log(s_hat))/sqrt(pi^2+log(s_hat)^2);
wn=4.6/(zeta*ts_1);

%% pzmap

pzmap(G,'r');
hold on;
zgrid(zeta,wn,Ts);

%% Design

[z_G,p_G,k_G]=zpkdata(G,'v')

A_plus=[1 -p_G(3)];
A_minus=[1 -p_G(1)];

B_plus=1;
B_minus=k_G*[1 -z_G];

deg_A=3; deg_A1=2; deg_B=1;
deg_A_plus=1; deg_A_minus=1;
deg_B_plus=0; deg_B_minus=1;

deg_S1=l+deg_A_minus-1
deg_R1=deg_A1-deg_B_plus+l1-1
deg_Am=l+deg_A1+deg_A_minus+l1-deg_B_plus-1

p1c=-zeta*wn+1j*wn*sqrt(1-zeta^2);
p1=exp(p1c*Ts)
p2=conj(p1)
p3c=-5*zeta*wn;
p3=exp(p3c*Ts)
p4=p3; p5=p3;p6=p3;

Am=poly([p1 p2 p3 p4 p5 p6])

Adioph=conv([1 -1],conv([1 -1], conv([1 -1], A_minus)))
Bdioph=B_minus

M_s=[[Adioph';0;0] [0;Adioph';0] [0;0;Adioph'] [0;0;Bdioph';0;0;0] [0;0;0;Bdioph';0;0] [0;0;0;0;Bdioph';0] [0;0;0;0;0;Bdioph']]

Gamma=Am';

theta=M_s\Gamma

R1=theta(1:3);
S1=theta(4:7);

R=conv([1 -1],conv([1 -1],R1))'
S=conv(A_plus,S1)'

C=minreal(zpk(tf(S,R,Ts)))

W=minreal(zpk(C*G/(1+C*G)))

%% Simulation

t_sim=5;

delta_r=1;
delta_d1=0;
delta_d2=0;
out=sim("gemini_davide_sim.slx");

figure; plot(out.y.time,out.y.data), grid on, xlabel('Time [s]'), ylabel('y(k)')
yline(delta_r+delta_r*s_hat,':r'), xline(ts_1,':g');


delta_r=0;
delta_d1=2;
delta_d2=0;
out=sim("gemini_davide_sim.slx");
figure; plot(out.y.time,out.y.data), grid on, xlabel('Time [s]'), ylabel('y_d1(k)');

delta_r=0;
delta_d1=0;
delta_d2=4;
out=sim("gemini_davide_sim.slx");
figure; plot(out.y.time,out.y.data), grid on, xlabel('Time [s]'), ylabel('y_d2(k)');
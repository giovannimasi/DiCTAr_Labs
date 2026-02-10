clearvars; close all; clc;

%% Algebraic Design

% Definitions

Ts=10e-3;
s=tf('s');
Gcont=zpk(0.48/(1+0.068*s+0.0016*s^2));

G=minreal(zpk(c2d(Gcont,Ts)))

%% Requirements

% requirement 1b more restrictive
nadd=1;
l=1; l1=0; l2=l-l1;

s_hat=0.2;
ts_1=1;

zeta=abs(log(s_hat))/sqrt(pi^2+log(s_hat)^2)*1.2;
wn=4.6/(zeta*ts_1);

[z_G, p_G, k_G]=zpkdata(G,'v')

pzmap(G,'r');
hold on;
zgrid(zeta,wn,Ts);

%% Design

A_plus=conv([1 -p_G(1)],[1 -p_G(2)]);
A_minus=1;
B_plus=1;
B_minus=k_G*[1 -z_G];

deg_A=2; deg_A1=2; deg_A_plus=2; deg_A_minus=0;
deg_B=1; deg_B_plus=0; deg_B_minus=1;

deg_S1=l+deg_A_minus-1+nadd
deg_R1=deg_A1-deg_B_plus+l1-1+nadd
deg_Am=l+deg_A1-deg_A_minus+l1-deg_B_plus-1+nadd

p1c=-zeta*wn+1j*wn*sqrt(1-zeta^2);
p1=exp(p1c*Ts);
p2=conj(p1);
p3c=-5*zeta*wn;
p3=exp(p3c*Ts);

Am=poly([p1 p2 p3])

Adioph=[1 -1]
Bdioph=B_minus

extra_eq=[Ts*ones(1,3) -0.08*dcgain(G)*polyval(A_plus,1)*ones(1,2)];

Gamma=[Am';0];

M_s=[[Adioph';0;0] [0;Adioph';0] [0;0;Adioph'] [0;Bdioph';0] [0;0;Bdioph'];extra_eq]

theta=M_s\Gamma;

R1=theta(1:3);
S1=theta(4:5);

R=conv([1 -1],R1)'
S=conv(A_plus,S1)

C=minreal(zpk(tf(S,R,Ts)))

%% Simulation

t_sim=3;

r=0.5;
d2=0;

out=sim("s360159_sim.slx");

figure; plot(out.y.time,out.y.data), grid on, xlabel('t'), ylabel('y(t)'),
yline(r,':g'), yline(r+r*s_hat,':r'),xline(ts_1,':g'), yline(0.5-0.005,':g'), yline(0.5+0.005,':g');

r=0;
d2=1;

out=sim("s360159_sim.slx");

figure; plot(out.y.time,out.y.data), grid on, xlabel('t'), ylabel('y_d1(t)');


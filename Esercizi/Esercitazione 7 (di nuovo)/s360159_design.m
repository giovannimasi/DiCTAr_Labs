clearvars; close all; clc;

%% Definitions

Ts=5e-3;
s=tf('s');
Gcont=minreal(zpk(40/(s^2+4*s-10)));
G=c2d(Gcont,Ts)

%% Requirements

% 1a is more restrictive

l=1; l1=0; l2=l-l1;
nadd=1;

s_hat=0.25;
ts_1=2.5;

zeta=abs(log(s_hat))/sqrt(pi^2+log(s_hat)^2)*1.2;
wn=4.6/(zeta*ts_1);
tr=1/(wn*sqrt(1-zeta^2))*(pi-acos(zeta))  %0.4764 lower than 0.9 -> satisfied

pzmap(G,'r');
hold on;
zgrid(zeta,wn,1);

[zG,pG,kG]=zpkdata(G,'v')

%% Design

A_plus=[1 -pG(2)];
A_minus=[1 -pG(1)];
B_plus=1;
B_minus=kG*[1 -zG];

deg_A=2; deg_A1=2; deg_A_plus=1; deg_A_minus=1;
deg_B=1; deg_B_plus=0; deg_B_minus=1;

deg_S1=l+deg_A_minus-1+nadd
deg_R1=deg_A1-deg_B_plus+l1-1+nadd
deg_Am=l+deg_A1+deg_A_minus+l1-deg_B_plus-1+nadd

p1c=-zeta*wn+1j*wn*sqrt(1-zeta^2);
p1=exp(p1c*Ts);
p2=conj(p1);
p3c=-22*zeta*wn;
p3=exp(p3c*Ts);
p4=p3;

Am=poly([p1 p2 p3 p4]);
Gamma=[Am';0];
extra_eq=[0.1*Ts*ones(1,3) -dcgain(G)*polyval(A_plus,1)*5e-4*ones(1,3)];

Adioph=conv([1 -1],A_minus)
Bdioph=B_minus

M_s=[[Adioph';0;0] [0;Adioph';0] [0;0;Adioph'] [0;Bdioph';0;0] [0;0;Bdioph';0] [0;0;0;Bdioph'];extra_eq]

theta=M_s\Gamma

R1=theta(1:3);
S1=theta(4:6);

R=conv([1 -1],R1)';
S=conv(A_plus,S1)';

C=minreal(zpk(tf(S,R,Ts)))

%% Simulation

t_sim=4;

r=0;
d1=0.7;
d2=0;

out=sim("s360159_sim.slx");

figure; plot(out.y.time,out.y.data,'b'), grid on, xlabel('t'), ylabel('y_d1(t)');

r=0;
d1=0;
d2=0.1;

out=sim("s360159_sim.slx");

figure; plot(out.y.time,out.y.data,'b'), grid on, xlabel('t'), ylabel('y_d2(t)');

r=1;
d1=0;
d2=0;

out=sim("s360159_sim.slx");

figure; plot(out.y.time,out.y.data,'b'), grid on, xlabel('t'), ylabel('y(t)'),
yline(r+r*s_hat,':r'), xline(0.9,':g'), xline(2.5,':g'), yline(r+r*0.01,':r'), yline(r-r*0.01,':r');
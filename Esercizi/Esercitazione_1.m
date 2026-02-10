    clearvars; close all; clc;

%% Problem 1 - stability, response computation, steady state analysis of discrete time 1dof control system
Ts=1;
z=tf('z',Ts);

C=zpk((z-0.2)/(z-1));
G=zpk(0.1/(z-0.5));
% 1. study stability properties
L=series(C,G);
W=minreal(zpk(feedback(L,1,-1)));

%asyntotically stable

% 2. Wd2 and output with d2
d2=5*z/(z-0.1);

Wd2=minreal(zpk(1/(1+L)));

y_d2=tf(Wd2*d2);
[yd2_num,yd2_den]=tfdata(y_d2,'v');
[rd2,pd2,kd2]=residuez(yd2_num,yd2_den) %-2.1429*0.8^k+2*0.6^k+5.1429*0.1^k

% 3. W3 and output with r
r=0.1*z/(z-1);

W3=minreal(zpk(C/(1+L)));

y_u=tf(W3*r);
[yu_num,yu_den]=tfdata(y_u,'v');
[ru,pu,ku]=residuez(yu_num,yu_den) %0.5-0.45*0.8^k+0.05*0.6^k

%4. Complete example

r=0.2*z/(z-1);
d1=z/(z-1);
d2=0.8*z/(z-1);

Wd1=minreal(zpk(G/(1+L)));

Y=zpk(minreal(W*r+Wd1*d1+Wd2*d2));
y_inf=dcgain((z-1)*Y)

%% Problem 2


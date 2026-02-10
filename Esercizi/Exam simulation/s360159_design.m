clearvars; close all; clc;

%% Part II - Algebraic Design
s=tf('s');

Gcont=zpk(0.48/(1+0.068*s+0.0016*s^2));
Ts=10e-3;
G=minreal(zpk(c2d(Gcont,Ts,'zoh')))

l=1;
l1=0;
l2=l-l1;

nadd=1;

% requirement 1b is more restrictive than requirement 1c

s_hat=0.2;
ts_1=1;

zeta=abs(log(s_hat))/sqrt(pi^2+log(s_hat)^2)*1.2;
wn=4.6/(ts_1*zeta);

[z_G,p_G,k_G] = zpkdata(G,'v')

figure;
pzmap(G,'r');
axis('equal');
hold on;
zgrid(zeta,wn,Ts);

%% Design

A_plus=conv([1 -p_G(1)],[1 -p_G(2)]);
A_minus=1;
B_plus=1;
B_minus=k_G*[1 -z_G];

deg_A=2; deg_A1=2;
deg_A_plus=2; deg_A_minus=0;
deg_B=1;
deg_B_plus=0; deg_B_minus=1;

deg_S1=l+deg_A_minus-1+nadd;
deg_R1=deg_A1-deg_B_plus+l1-1+nadd;
deg_Am=l+deg_A1+deg_A_minus+l1-deg_B_plus-1+nadd;

A_dioph=conv(A_minus, [1 -1])
B_dioph=B_minus

p1c=-zeta*wn+1j*wn*sqrt(1-zeta^2);
p1=exp(p1c*Ts);
p2=conj(p1);
p3c=-5*zeta*wn;
p3=exp(p3c*Ts);

Am=poly([p1 p2 p3]);

extra_eq=[-Ts/0.08*ones(1,3) dcgain(G)*polyval(A_plus,1)*ones(1,2)];

M_s=[[A_dioph';0;0] [0;A_dioph';0] [0;0;A_dioph'] [0;B_dioph';0] [0;0;B_dioph'];extra_eq];

Gamma=[Am';0];

theta=M_s\Gamma;

R1=theta(1:3,:);
S1=theta(4:5,:);

R=conv(R1,[1 -1])';
S=conv(S1,A_plus);

C=zpk(tf(S,R,Ts))

figure;
pzmap(minreal(C*G/(1+C*G)));
hold on;
zgrid(zeta,wn,Ts);

%% Simulation

t_sim=2;
delta_r=0;
delta_d2=1;

out=sim("s360159_sim.slx");

figure; plot(out.y.time, out.y.data,'b'), grid on, hold on,
xlabel('kTs'), ylabel('yd2(k)'), yline(0.08,':r');


delta_r=0.5;
delta_d2=0;

out=sim("s360159_sim.slx");

figure; 
subplot(211), plot(out.y.time, out.y.data-out.r.data,'b'), grid on, hold on,
xlabel('kTs'), ylabel('e(k)');

subplot(212), plot(out.y.time, out.y.data,'b'), grid on, hold on,
xlabel('kTs'), ylabel('y(k)'),
xline(1-1/100,':g'), xline(1+1/100,':g')
yline(delta_r+delta_r*s_hat,':r');


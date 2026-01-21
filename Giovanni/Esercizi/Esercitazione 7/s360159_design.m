clearvars; close all; clc;

%% Part II - Algebraic design

s=tf('s');
Ts = 5e-3;
Gcont=zpk(40/(s^2+4*s-10));
G=c2d(Gcont,Ts)

l=1;
l1=0;
l2=1;

nadd=1;

%requirement 1a is more restrictive

s_hat=0.25;
ts_1=2.5;
zeta=abs(log(s_hat))/sqrt(pi^2+log(s_hat)^2)
wn=4.6/(zeta*ts_1)
tr=1/(wn*sqrt(1-zeta^2))*(pi-acos(zeta))

[z_G,p_G,k_G]=zpkdata(G,'v')
%pzmap(G,'r');
%hold on; axis equal;
%zgrid(zeta,wn,Ts);

%% Design

Aplus=[1 -p_G(2)]
Aminus=[1 -p_G(1)]
Bplus=1
Bminus=k_G*[1 -z_G]

deg_A=2; deg_A1=2;
deg_A_plus=1; deg_A_minus=1;
deg_B_plus=0; deg_B_minus=1;


deg_S1=l+deg_A_minus-1+nadd
deg_R1=deg_A1-deg_B_plus+l1-1+nadd
deg_Am=l+deg_A1+deg_A_minus+l1-deg_B_plus-1+nadd

p1c=-zeta*wn+1j*wn*sqrt(1-zeta^2);
p1=exp(p1c*Ts);
p2=conj(p1);
p3c=-30*zeta*wn;
p3=exp(p3c*Ts);
p4=p3;

Am=poly([p1 p2 p3 p4])

extra_eq=[0.1*Ts/5e-4*ones(1,3) -dcgain(G)*polyval(Aplus,1)*ones(1,3)];

A_dioph=conv([1 -1],Aminus)
B_dioph=Bminus

Gamma=[Am';0]
M_s=[[A_dioph';0;0] [0;A_dioph';0] [0;0;A_dioph'] [0;B_dioph';0;0] [0;0;B_dioph';0] [0;0;0;B_dioph'];extra_eq]
theta=M_s\Gamma

R1=theta(1:3,:);
S1=theta(4:6,:);

R=conv(R1,[1 -1])'
S=conv(S1,Aplus)'

C=minreal(zpk(tf(S,R,Ts)))

%% Simulation
z=tf('z',Ts);

t_sim=10;

d1=0.7;
d2=0;
r=0;

out=sim("s360159_sim.slx");

y=out.y;

figure;
plot(y.time,y.data,'b'), grid on, xlabel('Time(s)'), ylabel('y(t)')

d1=0;
d2=0.1;
r=0;

out=sim("s360159_sim.slx");

y=out.y;

figure;
plot(y.time,y.data,'b'), grid on, xlabel('Time(s)'), ylabel('y(t)')

d1=0;
d2=0;
r=1;

out=sim("s360159_sim.slx");

y=out.y;

figure;
plot(y.time,y.data,'b'), grid on, xlabel('Time(s)'), ylabel('y(t)')
xline(2.5,':g'), xline(0.9,':r'), yline(r+r*s_hat,':r');

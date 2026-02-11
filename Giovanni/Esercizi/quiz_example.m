s_hat=0.1;

zeta=abs(log(s_hat))/sqrt(pi^2+log(s_hat)^2);

poles=[0.9+j*0.3 0.3+j*0.9 0.3+j*0.5 0.1+j*0.3];

plot(poles,'bo');
zgrid(zeta,1,1);
%% 

A=0.732;
B=0.219;
x0=0.5;

Hp=3;
Q=1;
R=1;

Acal=[A;A^2;A^3];
Bcal=[B 0 0;A*B B 0;A^2*B A*B B];
Qcal=blkdiag(Q,Q,Q);
Rcal=blkdiag(R,R,R);

H=2*(Bcal'*Qcal*Bcal+Rcal);
F=2*Acal'*Qcal*Bcal;
U=-H\F'*x0

%% 
Ts=1;
z=tf('z',Ts);
C=minreal(zpk(0.2*(z+0.1)/(z-0.2)));
G=minreal(zpk(1/(z-1)));

Wd1=minreal(G/(1+C*G));
d1=2*z/(z-1);

yd1=Wd1*d1;

[ynum,yden]=tfdata(yd1,'v');
[r p k]=residuez(ynum,yden)
%% 
clear; clc;

z=tf('z',1);
A=[0.05 0 -0.25;0 -0.7 0;-0.25 0 0.5];
B=[1 0 3]';
eig(A)

Q=diag([1 1 1]); R=1;

Mr=ctrb(A,B); rho_r=rank(Mr)

Cq=chol(Q);
Mo=obsv(A,Cq); rho_o=rank(Mo)

%% 
clear;
A=[-0.6 0.8;-0.5 -0.2]; B=[1;1];
x0=[-10;-8];
Ts=50e-3;

Hp=4;
Q=diag([5 1]); R=1;

Acal=[A;A^2;A^3;A^4];
Bcal=[B zeros(2,3);A*B B zeros(2,2);A^2*B A*B B zeros(2,1);A^3*B A^2*B A*B B];
Qcal=blkdiag(Q,Q,Q,Q);
Rcal=blkdiag(R,R,R,R);
H=2*(Bcal'*Qcal*Bcal+Rcal); H=(H+H')/2;
F=2*Acal'*Qcal*Bcal;
G=[eye(4);-eye(4)];
h=3*ones(2*Hp,1);

x_traj(:,1)=x0;
x_k=x0;
steps=200;

for kk=1:steps
    U=quadprog(H,x_k'*F,G,h);
    u_traj(:,kk)=U(1);
    x_traj(:,kk+1)=A*x_k+B*U(1);
    x_k=x_traj(:,end);
end

disp(u_traj(:,1));
figure;
x_norm=sqrt(x_traj(1,:) .^2+x_traj(2,:) .^2)';
plot(0:Ts:Ts*steps,x_norm,'b'),
yline(1e-4,':g'), yline(-1e-4,':g');

%% 
clear;

Ts=1;
z=tf('z',Ts);

C=minreal(zpk(1.5*(z-0.7)/(z-0.9)));
G=minreal(zpk(1.1/(z-1.2)));

W=minreal(C*G/(1+C*G));

r=z/(z-1);
d1=r;
Wd1=minreal(G/(1+C*G));

y=minreal(Wd1*d1+W*r)

[ynum,yden]=tfdata(y,'v');
[r p k] = residuez(ynum,yden)
%% 

clear;

Kl=0.15/0.5

Kc=Kl/((1-0.4)/(1-0.2))

%% 

clear;

Ts=2e-3;

Kl_d2=Ts^2/0.5
Kl_d1=Ts/0.05
%% 

A=0.732;
B=0.219;
x0=0.5;

Hp=3;
Q=1;
R=1;

Acal=[A;A^2;A^3];
Bcal=[B 0 0;A*B B 0; A^2*B A*B B];
Qcal=blkdiag(Q,Q,Q);
Rcal=blkdiag(R,R,R);

H=2*(Bcal'*Qcal*Bcal+Rcal);
F=2*Acal'*Qcal*Bcal;
U=-H\F'*x0

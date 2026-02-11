%% Problem 3

clearvars; close all; clc;

A=[0.9616 0.1878;-0.3756 0.8677];
B=[0.0192;0.1878];
x0=[-0.3;0.4];
Ts=0.2;
n=2;

Q=diag([5 1]);
S=Q;
R=0.1;

Hp=3;

Acal=[A;A^2;A^3];
Bcal=[B zeros(n,1) zeros(n,1);A*B B zeros(n,1);A^2*B A*B B];
Qcal=blkdiag(Q,Q,Q);
Rcal=blkdiag(R,R,R);

H=2*(Bcal'*Qcal*Bcal+Rcal)
H=(H'+H)/2
F=2*Acal'*Qcal*Bcal
G=[eye(Hp);-eye(Hp)]
h=0.5*ones(Hp*2,1)

x_traj(:,1)=x0;
x_k=x0;

steps=40;

for kk=1:steps
    U=quadprog(H,x_k'*F,G,h);
    x_traj(:,kk+1)=A*x_k+B*U(1);
    u_traj(:,kk)=U(1);
    x_k=x_traj(:,end);
end

figure;
subplot(221), stairs(0:Ts:Ts*steps,x_traj(1,:),'b'), grid on, ylabel('x_1(k)');
subplot(222), stairs(0:Ts:Ts*steps,x_traj(1,:),'b'), grid on, ylabel('x_2(k)');
subplot(224), stairs(0:Ts:Ts*(steps-1),u_traj(1,:),'b'), grid on, ylabel('u(k)');
subplot(223), stairs(0:Ts:Ts*steps,sqrt(x_traj(1,:) .^2+x_traj(2,:) .^2),'b'), grid on, 
yline(1e-4,':g'), yline(-1e-4,':g'), ylabel('||x(k)||_2');

%% Problem 4

clearvars; close all; clc;

A=[0.9616 0.1878;-0.3756 0.8677];
B=[0.0192;0.1878];
x0=[-0.3;0.4];
Ts=0.2;
n=2;

Q=diag([5 1]);
S=Q;
R=0.1;

Hp=3;

Acal=[A;A^2;A^3];
Bcal=[B zeros(n,1) zeros(n,1);A*B B zeros(n,1);A^2*B A*B B];
Qcal=blkdiag(Q,Q,Q);
Rcal=blkdiag(R,R,R);

H=2*(Bcal'*Qcal*Bcal+Rcal);
H=(H'+H)/2;
F=2*Acal'*Qcal*Bcal;

Gu=[eye(Hp);-eye(Hp)];
hu=0.5*ones(Hp*2,1);

Gx=Bcal;

x_traj(:,1)=x0;
x_k=x0;
x_max=[0;0.5];

steps=40;

for kk=1:steps
    hx=-Acal*x_k+repmat(x_max,Hp,1);
    G=[Gu;Gx];
    h=[hu;hx];
    U=quadprog(H,x_k'*F,G,h);
    x_traj(:,kk+1)=A*x_k+B*U(1);
    u_traj(:,kk)=U(1);
    x_k=x_traj(:,end);
end

figure;
subplot(221), stairs(0:Ts:Ts*steps,x_traj(1,:),'b'), grid on, ylabel('x_1(k)');
subplot(222), stairs(0:Ts:Ts*steps,x_traj(1,:),'b'), grid on, ylabel('x_2(k)');
subplot(224), stairs(0:Ts:Ts*(steps-1),u_traj(1,:),'b'), grid on, ylabel('u(k)');
subplot(223), stairs(0:Ts:Ts*steps,sqrt(x_traj(1,:) .^2+x_traj(2,:) .^2),'b'), grid on, 
yline(1e-4,':g'), yline(-1e-4,':g'), ylabel('||x(k)||_2');
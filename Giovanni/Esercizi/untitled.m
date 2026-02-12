clear;
A=[-0.6 0.8;-0.5 -0.2]; B=[1;1];
x0=[-10;-8];
Ts=50e-3;
Hp=4;
Q=diag([5 1])
R=1;
Acal=[A;A^2;A^3;A^4]
n=2;
Bcal=[B zeros(n,3);A*B B zeros(n,2);A^2*B A*B B zeros(n,1);A^3*B A^2*B A*B B]
Qcal=blkdiag(Q,Q,Q,Q)
Rcal=blkdiag(R,R,R,R)
H=2*(Bcal'*Qcal*Bcal+Rcal)
F=2*Acal'*Qcal*Bcal
steps=200;
G=[eye(Hp);-eye(Hp)]
h=3*ones(2*Hp,1)
x_traj(:,1)=x0;
x_k=x0;
H=(H+H')/2
for kk=1:steps
U=quadprog(H,x_k'*F,G,h);
u_traj(:,kk)=U(1);
x_traj(:,kk+1)=A*x_k+B*U(1);
x_k=x_traj(:,end);
end
figure
stairs(0:Ts:Ts*(steps-1),u_traj(:),'b')
figure
stairs(0:Ts:Ts*steps,sqrt(x_traj(1,:) .^2+x_traj(2,:) .^2),'b'),
grid on, yline(1e-4,'g'), yline(-1e-4,'g');
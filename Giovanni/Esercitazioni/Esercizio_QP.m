%clear; %importantissimo...nosense
A = [-0.6 0.8; -0.5 -0.2];
B = [1;1];
x0 = [-10;-8];
Ts = 50e-3;
Hp = 4;
Q=diag([5 1]);
R=1;
G=[eye(4);-eye(4)];
h=3*ones(8,1);
Acal=[A;A^2;A^3;A^4];
Bcal=[B zeros(2,3);A*B B zeros(2,2); A^2*B A*B B zeros(2,1); A^3*B A^2*B A*B B];
Qcal=blkdiag(Q,Q,Q,Q);
Rcal=blkdiag(R,R,R,R);
H=2*(Bcal'*Qcal*Bcal+Rcal);
H=(H+H')/2;
F=2*Acal'*Qcal*Bcal;

xtraj(:,1)=x0;
x_k=x0;
steps=200;

for kk=1:steps
    U=quadprog(H,x_k'*F,G,h);
    x_traj(:,kk+1)=A*x_k+B*U(1);
    u_traj(kk)=U(1);
    x_k=x_traj(:,end);
end

figure; stairs(0:Ts:Ts*steps, sqrt(x_traj(1,:) .^2+x_traj(2,:) .^2)), grid;
figure; stairs(0:Ts:Ts*(steps-1),u_traj), grid;
%% Davide
A = [-0.6 0.8; -0.5 -0.2];
B = [1;1];
x0 = [-10;-8];
Ts = 50e-3;
Hp = 4;
Q = [5 0; 0 1];
S = Q;
R = 1;
n = 2;
p = 1;

Acal = [A;A^2;A^3; A^4];
Bcal = [B zeros(n,1) zeros(n,1) zeros(n,1); A*B B zeros(n,1) zeros(n,1); 
    A^2*B A*B B zeros(n,1); A^3*B A^2*B A*B B];
Qcal = blkdiag(Q,Q,Q,Q);
Rcal = blkdiag(R,R,R,R);

H = 2*(Bcal'*Qcal*Bcal + Rcal);
H = (H+H')/2;
F = 2*Acal'*Qcal*Bcal;

% input constraints
G = [eye(Hp); -eye(Hp)];
u_max = 3;
h = u_max*ones(2*Hp,1);

x_traj(:,1) = x0;
x_k = x0;
steps = 200;
for kk=1:steps
    U = quadprog(H,x_k'*F, G, h);
    x_traj(:, kk+1) = A*x_k + B*U(1);
    u_traj(kk) = U(1);
    x_k = x_traj(:, end);
end

figure
stairs(0:Ts:Ts*steps, sqrt(x_traj(1,:).^2+x_traj(2,:).^2))

figure
stairs(0:Ts:Ts*(steps-1), u_traj)
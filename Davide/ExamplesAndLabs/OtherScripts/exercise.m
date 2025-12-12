clear
clc
A = [0.5 -1; 0 1];
B = [2; 0];
C = [2 4];

% analytical expression of the output response y with initial condition: 
x0 = [2; 1];
% and input:
z = tf('z',1);
U = z/(z-0.9); % z-transform of 0.9^k

% the total output will be the sum of zero-input output response plus
% zero-state output response

% Y(z) = Y_zi(z) + Y_zs(z) = zC*(zI-A)^-1*x0 + (C*(zI-A)^-1*B+D)*U(z)
% (nb D = 0)

Y = zpk(minreal(C*inv(z*eye(2)-A)*(z*x0 + B*U), 1e-2))

% based on this result we're able to do PFE ->

[num, den] = tfdata(Y, 'v');
[ry,py,ky] = residuez(num, den)


% Y(z) = 10z/(z-0.9) - 2z/(z-0.5)
% y(k) = 10*0.9^k - 2+0.5^k


% INTERNAL STABILITY
lambda = eig(A);

% lambda_1 = 0.5 -> convergent 
% lambda_2 = 1 -> bounded

% stability -> all eigs have mag <= 1. lambda_2 has unitary algebraic mult
% so it's stable


% BIBO STABILITY
% I have to evaluate the poles of H
H = zpk(minreal(C*inv(z*eye(2)-A)*B, 1e-2))

% alternative procedure
sys = ss(A,B,C,0);
H1 = tf(sys) 

% pole is 0.5 => it's bibo stable
% in general, the poles of tf are a subset of the eigs 




%% ex 2
clear
clc

A = [-0.2 0 0; 0 1 0; 0 0 1];
% eigs of A lie on the diagonal since it's a diagonal matrix
lambda = eig(A);
% eig = 1 has alg mult = 2

z = tf('z',1);
inv(z*eye(3)-A) % nb matlab shows the result column by column

minpoly(A)
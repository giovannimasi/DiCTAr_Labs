%% es 1
clear
clc

z = tf('z',1);
C = (z-0.2)/(z-1);
G = 0.1/(z-0.5);
L = C*G;
% stability problem: compute stability for just one tf
% I study W = L/(1+L)
W = zpk(minreal(feedback(C*G,1),1e-2));

[zw, pw, kw] = zpkdata(W(1), 'v');


W2 = 1/(1+L);
D2 = 5*z/(z-0.1);
Y_D2 = zpk(minreal(W2*D2,1e-2))

[nY2, dY2] = tfdata(Y_D2(1), 'v');
[rY2, pY2, kY2] = residuez(nY2, dY2);


W3 = C/(1+L);
R = 0.1*z/(z-1);
U = zpk(minreal(W3*R,1e-2));
[nYr, dYr] = tfdata(U(1), 'v');
[rYr, pYr, kYr] = residuez(nYr, dYr);

r = 0.2*z/(z-1);
d1 = z/(z-1);
d2 = 0.8*z/(z-1);
W1 = G/(1+L);
Y = zpk(minreal(W*r + W1*d1 + W2*d2, 1e-2))
Y_temp = minreal((z-1)*Y,1e-2)
y_inf = dcgain(Y_temp)
% [nY, dY] = tfdata(Y(1), 'v');
% [rY, pY, kY] = residuez(nY, dY)


%% es 2
clear
clc
z = tf('z',1e-3);
G = 0.1*(z+0.8)/((z-0.1)*(z-0.5));
dcgain(G)
% goodnotes 


%% es 3
clear
clc
T = 1;
z = tf('z',T);
zeta = 0.6;
G = (z+1.1)/((z-1)*(z-0.4));
% figure;
% pzmap(G);
% axis('equal');
% zgrid(zeta, [], T);

Ms = [1 0 0; -1 1 1; 0 -1 1.1];
% Gamma = [1;-1;0.29];
Gamma = [1;-1.2;0.45];
Theta = Ms\Gamma;
R1 = Theta(1:2); % coeff of R'
S1 = Theta(end); % coeff of S'

% build the controller
R = R1(1)*z+R1(2); % since there are no poles in 1 for C
S = (z-0.4)*S1;
C = zpk(S/R)

% check the result
W = minreal(zpk(feedback(C *G,1)),1e-2);
pole(W) 


%% es 4
clear
clc
T = 1;
z = tf('z',T);
zeta = 0.6;
G = (z+1.1)/((z-1)*(z-0.4));

n_add = 1;
degS1 = 2;
degR1 = 2;

r = [0.09 0.1 0.11 0.12];
Am = poly(r)'; % già trasposto
% Am = [Am ; zeros(length(n_add),1)];

Aplus = [1 -0.4];
Aminus = 1;
Bplus = 1;
Bminus = [1 1.1];
extra_eq_matrix = [1 1 1 -0.06 -0.06 -0.06]; % solo una riga poiché solo un eq extra
Adioph = conv([1 -2 1], Aminus)';
Bdioph = Bminus';
% 
% Ms = zeros(length(Am));
% 
% for ii=1:length(Adioph)
%     Ms(ii:ii+length(Adioph)-1,ii) = Adioph;
% end
% 
% for jj=1:length(Am)-ii
%     Ms(jj+1:jj+length(Bdioph),ii+jj) = Bdioph;
% end
% 
% if n_add>0
%     for kk=1:length(n_add)
%         Ms(end-kk+1,:) = extra_eq_matrix(kk,:);
%     end
% end
% 
% theta = Ms \ Am;
% R1 = theta(1:degR1+1);
% S1 = theta(degR1+2:end);
[R1,S1] = diop_solver(Adioph,Bdioph,Am,degR1,n_add,extra_eq_matrix);
S = conv(Aplus,S1);
R = conv([1 -1], R1);
C = zpk(tf(S',R',1))

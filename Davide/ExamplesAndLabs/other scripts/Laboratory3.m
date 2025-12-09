% Laboratory 3
clc
clear all
close all

% Defining the parameter and the continuous time plant
Ts = 0.01; % defining the sampling time
s = tf('s'); % defining the s variable
Gcont = zpk(10/(1+s/50));

% let's get the plant in discrete time
z = tf('z',Ts); % defining the z variable
G = zpk(c2d(Gcont,Ts,'zoh')) % this is our plant 
[zg,pg,kg] = zpkdata(G,'v')

% define A and B polynomial
B = kg;
A = [1 -pg];

% Steady state analisy by hand

% Transient response parameters
s_hat = 0.15; % maximum overshoot
ts_1 = 0.8; % settling time
zeta = abs(log(s_hat))/(sqrt(pi^2+(log(s_hat))^2))*1.32 % we have to increase this value to suddisfy the requirements (after simulink simulation)
wn = 4.6/(ts_1*zeta)

% plot the cardioid region to understand wich zero-pole cancellation can be
% done => must be stable
figure(1)
pzmap(G)
hold on
grid on
zgrid(zeta,[],Ts) % we can cancel the pole inside the cardioid area

% Define the polinomial that can be cancelled
B_plus = 1;
B_minus = B;
A_plus = A;
A_minus = 1;

% By solving the degree condition for the solution by hand we get that the
% degree Am is 3 we have to choose 3 poles
pc_1 = -zeta*wn + 1j*wn*sqrt(1-zeta^2);
pc_2 = -zeta*wn - 1j*wn*sqrt(1-zeta^2);
pc_3 = -5*zeta*wn;

% convert them in descrete time
pd_1 = exp(pc_1*Ts);
pd_2 = exp(pc_2*Ts);
pd_3 = exp(pc_3*Ts);
Am = poly([pd_1 pd_2 pd_3])

% define the diophantine polinomial
A_dioph = conv(conv([1 -1],[1 -1]),A_minus)
B_dioph = B_minus

% Define the parameters for the additional equation
delta1 = 1;
B_plus1 = polyval(B_plus,1);
A_plus1 = polyval(A_plus,1);
yd1 = 0.05;

% Define the silvester matrix
M_S = [[[A_dioph(:);0],[0;A_dioph(:)],[0;B_dioph;0;0],[0;0;B_dioph;0],[0;0;0;B_dioph]];[delta1*Ts^2*B_plus1,delta1*Ts^2*B_plus1,-yd1*A_plus1,-yd1*A_plus1,-yd1*A_plus1]]

% Solve the diophantine problem
Gamma = [Am';0];
Theta = M_S\Gamma;
R1 = Theta(1:2)';
S1 = Theta(3:5)';

% Define the controller
S = conv(A_plus,S1);
R = conv(conv(conv([1 -1], [1 -1]),B_plus),R1);
C = zpk(tf(S,R,Ts)) % Double check it has 2 poles in one and also as zero the pole of the G that we want to cancel

% To test the behaviour we have to create the simulink project
% We have to test the output response in the presence of two disturb d1 and
% d2. Let's start by testing the output response for yd1

t_sim = 10;
delta_1 = 1;
delta_2 = 0;
delta = 0;
out = sim("SimulinkLaboratory3.slx");
figure(2)
plot(out.y.time,out.y.data,'b')
xlabel('t')
ylabel('y_d1(inf)')
% We can say that it is exactly respected because the output value is 0.05

%% Let's move one with the output response for yd2
t_sim = 10;
delta_1 = 0;
delta_2 = 1;
delta = 0;
out = sim("SimulinkLaboratory3.slx");
figure(3)
plot(out.y.time,out.y.data,'b')

xlabel('t')
ylabel('y_d2(inf)')
% The condition of yd1 was the most stringent infact we can that the output
% response ad steady state is 0.004 

%% To test the overshoot and settling time we have to see the output response obtained with a unitary step ference signal
t_sim = 10;
delta_1 = 0;
delta_2 = 0;
delta = 1;
out = sim("SimulinkLaboratory3.slx");
figure(4)
plot(out.y.time,out.y.data,'b')
hold on
plot(out.r.time,out.r.data,'r')
xlabel('t')
ylabel('y(r(t))')

% In the first plot so without modifing the zeta value we got an overhoot
% that is about 20% so that condition is not respected instead the settling
% time is around 0.65s so in respected

% To vary the overshoot in such condition to suddisfy our requirements we
% must increase the damping!!
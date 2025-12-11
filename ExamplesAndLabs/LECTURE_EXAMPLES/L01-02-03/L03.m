clear
clc
T = 0.01;
zgrid(T); 
axis('square'); % proportion between vertical and orizontal = 1
%% 
% it is possible to customize for plotting special case of damp
% if I want a particular damping and a particular natural frequency, I can
% customize my statement as:
ksi = 0.4;
omega = 50;
zgrid(ksi, omega, T); 
axis('square');
%%
% if I want to draw the constant damping diagram => in the spot of omega 
% in zgrid I have to put 0 
zgrid(ksi, 0, T); 
axis('square');
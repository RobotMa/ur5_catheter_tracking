% Fitting Trigger Count - Distance (mm) into a Gasussian Distribution
% "Data" is acquired from Xiaoyu's paper "Active Ultrasound Pattern
% Injection System for Interventional Tool Guidance", Figure 6
clc
close all
clear all

%%
x = [ -8; -4; -2; -0.5;  0; -1; 0.5;  1;  2;  6;  8   ]; % Distance (mm)
y = [0.5; 11; 25;   39; 40; 36;  39; 37; 25;  1; 0.3  ]; % Trigger Count (int)
f = fit(x, y, 'gauss1');
plot(f,x,y)
axis([-10, 10, 0, 45] )
xlabel('Distance (mm)')
ylabel('Trigger Count')
grid on
title('The trigger count versus the offset between the active element and the mid-plane')

%%
clear x y

syms x

a = f.a1;
b = f.b1;
c = f.c1;
eqn = a*exp(-((x-b)/c)^2) == 4;
xsol = solve(eqn, x);
vpa(xsol,4)

%% 
ygiv = 4;
xsol2 = sqrt(-c^2*log(ygiv/a)) + b



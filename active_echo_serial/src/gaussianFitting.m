% Fitting Trigger Count - Distance (mm) into a Gasussian Distribution
% "Data" is acquired from Xiaoyu's paper "Active Ultrasound Pattern
% Injection System for Interventional Tool Guidance", Figure 6
clc
close all;
clear all;

%% Get the Gaussian distribution between tc and x
x1  = [ -8; -4; -2; -0.5;  0; -1; 0.5;  1;  2;  6;   8]; % Distance (mm)
tc1 = [0.5; 11; 25;   39; 40; 36;  39; 37; 25;  1; 0.3]; % Trigger Count (int)
f1 = fit(x1, tc1, 'gauss1');

figure
plot(f1, x1, tc1)
axis([-10, 10, 0, 45] )
xlabel('Distance (mm)')
ylabel('Trigger Count')
grid on
title('The trigger count versus the offset between the active element and the mid-plane')

a1 = f1.a1;
b1 = f1.b1;
c1 = f1.c1;


%% Get the Gaussian distribution between tc and x
x2 = [-451.77; -449.06; -450.35; -451.43; -452.16; -453.8; 
      -455.36; -456.27; -457.31; -458.39; -459.94; -460.38];
x2 = x2 + ones(size(x2))*455;

tc2 = [26;  2; 13; 27; 31; 41;
       70; 80; 42; 14;  4;  2];
f2 = fit(x2, tc2, 'gauss1');

figure
plot(f2, x2, tc2)
xlabel('Distance (mm)')
ylabel('Trigger Count')
grid on
title('The trigger count versus the offset between the active element and the mid-plane') 

a2 = f2.a1;
b2 = f2.b1;
c2 = f2.c1;

%% Get the Gaussian distribution between tc and x
clear x tc
x3  = [-460.38; -457.34; -458.43; -456.01; -455.27; -454.61;
       -453.53; -452.7; -451.77; -450.19; -448.83];
x3  = x3 + ones(size(x3))*455;
tc3 = [ 1; 37; 14; 73; 70; 62; 41; 41; 37; 28;  3];
f3  = fit(x3, tc3, 'gauss1');

figure
plot(f3, x3, tc3)
xlabel('Distance (mm)')
ylabel('Trigger Count')
grid on
title('The trigger count versus the offset between the active element and the mid-plane')

a3 = f3.a1;
b3 = f3.b1;
c3 = f3.c1;


%% Get the Gaussian distribution between tc and x
clear x tc
x4  = [-438.12; -439.21; -440.76; -441.82; -442.21; -443.91; 
       -444.85; -445.74; -446.73];
x4  = x4 + ones(size(x4))*443;

tc4 = [ 2;  9; 17; 22; 22; 18; 15; 11; 13];
f4  = fit(x4, tc4, 'gauss1');

figure
plot(f4, x4, tc4)
xlabel('Distance (mm)')
ylabel('Trigger Count')
grid on
title('The trigger count versus the offset between the active element and the mid-plane')

a4 = f4.a1;
b4 = f4.b1;
c4 = f4.c1;

%%
syms x
eqn = a3*exp(-((x-b3)/c3)^2) == 4;
xsol = solve(eqn, x);
vpa(xsol,4)

ygiv = 4;
xsol2 = sqrt(-c3^2*log(ygiv/a3)) + b3;

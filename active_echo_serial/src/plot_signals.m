clc
close all
clear all

%% Depth = 292.15 - 273.07
x = [-451.77; -449.06; -450.35; -451.43; -452.16; -453.8; 
     -455.36; -456.27; -457.31; -458.39; -459.94; -460.38];

l_ta = [68; 75; 69; 69; 75; 75;
        91; 88; 77; 62; 64; 63];
    
dly = [1213; 1255; 1222; 1210; 1209; 1207; 
       1204; 1197; 1198; 1202; 1223; 3747];

tc = [26;  2; 13; 27; 31; 41;
      70; 80; 42; 14;  4;  2];

%%
figure
scatter(x, l_ta)
xlabel('x/mm')
ylabel('l_{ta}')

figure
scatter(x, dly)
xlabel('x/mm')
ylabel('dly')

figure
scatter(x, tc)
xlabel('x/mm')
ylabel('tc')

%% Depth = 298.32 - 273.07
clear x l_ta dly tc
x = [-460.38; -457.34; -458.43; -456.01; -455.27; -454.61;
     -453.53; -452.7; -451.77; -450.19; -448.83];

l_ta = [64; 83; 66; 90; 90; 84; 
        73; 75; 75; 68; 75];

dly = [3300; 1530; 1546; 1521; 1524; 1532;
       1546; 1546; 1545; 1554; 1582];

tc = [ 1; 37; 14; 73; 70; 62;
       41; 41; 37; 28;  3];

%%
figure
scatter(x, l_ta)
xlabel('x/mm')
ylabel('l_{ta}')

figure
scatter(x, dly)
xlabel('x/mm')
ylabel('dly')

figure
scatter(x, tc)
xlabel('x/mm')
ylabel('tc')

%% Depth = 298.32 - 273.07
clear x l_ta dly tc
x = [-438.12; -439.21; -440.76; -441.82; -442.21; -443.91;
     -444.85; -445.74; -446.73; -447.87; -448.54];

l_ta = [59; 65; 65; 63; 64; 64;
        64; 65; 64; 63; 73];

dly = [2096; 2083; 2066; 2064; 2064; 2046; 
       2042; 2042; 2032; 2020; 2019];

tc = [ 2;  9; 17; 22; 22; 18;
      15; 11; 13; 13; 19];

%%
figure
scatter(x, l_ta)
xlabel('x/mm')
ylabel('l_{ta}')

figure
scatter(x, dly)
xlabel('x/mm')
ylabel('dly')

figure
scatter(x, tc)
xlabel('x/mm')
ylabel('tc')


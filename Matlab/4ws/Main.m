
%Main
clc
%% init
s0 = [ 0 0 0 0 0];
s1 = [ 8 4 0 0 0];
Dmax = pi/4;
%% test1
% fprintf(' 1) Sicliano polinomio cubico   \n');
% pause;
% poly_pathMiche2(s0,s1,1,1,0.01);

%% Test1
fprintf('1) CrabMode\n')
%pause;
[Pc] = CrabMode(s0,s1,100,Dmax,0.5,0.5);


%% test2
fprintf('2) Car-Like adattato Wang + shevat \n');
pause;
Pb = BiwsVehicleSolver(1,s0,s1,1,1);

%% test3 
%fprintf('3) primitive a mano (TesiTorino) \n');
%pause
%TestMove
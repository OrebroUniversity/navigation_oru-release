
%Main
clc
%% init
s0 = [ 0 0 0.7854 0];
s1 = [ 0 -15 -0.7854 0];
%Dmax = pi/4;
%% test1
% fprintf(' 1) Sicliano polinomio cubico   \n');
% pause;
% poly_pathMiche2(s0,s1,1,1,0.01);

%% Test1
% fprintf('1) CrabMode\n')
% pause;
% %[Pc] = CrabMode(1,s0,s1,100,Dmax,0.5,0.5);
%  [Pc] = CrabMode(1, s0, s1, 100, 1,1);


%% test2
%fprintf('2) Car-Like  \n');
%pause(2);

%Pb = CarLikeVehicleSolver(0.5,s0,s1,1,1);

%% test3
%fprintf('3) 4ws - Car-Like  \n');
%pause(2);
%Pb = BiwsVehicleSolver(1,s0,s1,1,1);

%% test4 
fprintf('4) Rear \n');
%pause(2)
% s0(1)=s0(1)-0.5*cos(s0(3));
% s0(2)=s0(2)-0.5*sin(s0(3));
% s1(1)=s1(1)-0.5*cos(s1(3));
% s1(2)=s1(2)-0.5*sin(s1(3));

Pb = BiwsVehicleSolverRear(3.2,s0,s1,1,1);
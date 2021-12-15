%% test
tic;
L = 3.2;
start = [0 0 pi/2 0 0];
goal = [4 4 pi 0 0];

[tr, m] = casadi(L, start, goal, 0, 1);
total_time = toc;

%% cusp
%load('trj');

cusp = cuspidi(tr(:,1),tr(:,2),tr(:,3),1);

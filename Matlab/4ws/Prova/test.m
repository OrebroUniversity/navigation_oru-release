%TEST
s0 = [ 0 0 0 0 0];
s1 = [6 6 -pi/6 0 0];
lf =1;lr=1; L = 2;
time = 15;
drawFlag = 1;
%% four wheel steering
fwsSolver(s1,time,lf,lr,drawFlag);

%% Car solver
k = 0;
carSolver(s1,time,k,lf,lr,drawFlag);

k = 1;
carSolver(s1,time,k,lf,lr,drawFlag);

k = -1;
carSolver(s1,time,k,lf,lr,drawFlag);
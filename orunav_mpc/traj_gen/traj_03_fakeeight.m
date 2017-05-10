%
% generate data and verify the result of the C++ implementation
%


%clear;clc

% =========================================================================
% plan
% =========================================================================
%dt = 0.05;        % time step
%tf = 5;           % final time
dt = 0.06;        % time step
tf = 7.2;           % final time
tspan = 0:dt:tf;  

L = 0.68;            % length of the car

s0 = [0;0;0;0];   % initial state
s1 = [0;-1.5;-pi;0]; % final state
[state1,control1] = plan_car_trajectory(L,s0,s1,[3;3],tspan,dt);

s0 = state1(:,end);   % initial state
s1 = [0;-3;0;0]; % final state
[state2,control2] = plan_car_trajectory(L,s0,s1,[3;3],tspan,dt);

s0 = state2(:,end);   % initial state
s1 = [0;-1.5;pi;0]; % final state
[state3,control3] = plan_car_trajectory(L,s0,s1,[3;3],tspan,dt);

s0 = state3(:,end);   % initial state
s1 = [0;0;0;0]; % final state
[state4,control4] = plan_car_trajectory(L,s0,s1,[3;3],tspan,dt);

state = [state1 state2 state3 state4];
control = [control1 control2 control3 control4];

output_trajectory([0; 0; 0; 0], state, control, 'trajectory_03.txt');

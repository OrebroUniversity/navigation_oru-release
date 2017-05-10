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

%s0 = [0;0;0;0];   % initial state
%s1 = [50;20;0;0]; % final state
s0 = [0;0;0;0];   % initial state
s1 = [2;1;0;0]; % final state

L = 0.68;            % length of the car

[state,control] = plan_car_trajectory(L,s0,s1,[4;4],tspan,dt);

output_trajectory(s0, state, control, 'trajectory_02.txt');

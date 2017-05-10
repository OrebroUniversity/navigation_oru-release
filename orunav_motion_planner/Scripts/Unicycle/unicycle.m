function dx = unicycle(t,x,u)
%
% unicycle model (continuous time) 
%
% Input:
% -------
% x  - state x = [x1;x2;x3]
%      x1 - 'x' coordinate
%      x2 - 'y' coordinate
%      x3 - angle [rad] (around the 'z' axis)
% u  - control input
%      u(1) - linear velocity v
%      u(2) - angular velocity w
%
% Output:
% -------
% dx - derivative of the state 
%

dx = zeros(3,1);

dx(1) = cos(x(3))*u(1);
dx(2) = sin(x(3))*u(1);
dx(3) = u(2);

%%%EOF
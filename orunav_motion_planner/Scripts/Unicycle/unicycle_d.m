function xp = unicycle_d(x,u,tau)
%
% unicycle model (discrete time) 
%
% Input:
% -------
% x   - state x = [x1;x2;x3]
%       x1 - 'x' coordinate
%       x2 - 'y' coordinate
%       x3 - angle [rad] (around the 'z' axis)
% u   - control input
%       u(1) - linear velocity v
%       u(2) - angular velocity w
% tau - sampling interval
%
% Output:
% -------
% xp  - state tau seconds after applying control input u from state x
%

[T,X] = ode45(@unicycle,[0,tau],x,[],u);
xp = X(end,:)'; 

%%%EOF
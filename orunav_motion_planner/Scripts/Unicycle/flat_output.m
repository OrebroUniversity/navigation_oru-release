function [u,theta,phi] = flat_output(P,dP,ddP,dddP,motion_direction,L)
%
% Flat outputs for a unicycle or a car-like robot
%
% Note:
% ------
% If L is specified, then car-like robot is assumed
% i.e., the function has 6 input parameters
% If L is not specified, then a unicycle is assumed
% i.e., the function has only 5 input parameters
%
% Input:
% ------
% P,dP,ddP,dddP - position, velocity, acceleration and jerk motion profile
%                 (formed in function poly_path.m)
%
% motion_direction =  1 for forward motion
% motion_direction = -1 for reverse motion
%
% L - distance between the rear and front axes
%
% Output:
% -------
% u     - evolution of the control inptus
%         in case of a unicycle: u(1) - linear velocity, u(2) - angular velocity
%         in case of a car-like robot: u(1) - linear velocity, u(2) - steering rate
%
% theta - evolution of the angle
% phi   - evolution of the steering angle
%         (in case of a unicycle phi is set equal to [])
%


if nargin < 6 % unicycle
    case_flag = 0;
    phi = [];
else % car-like robot
    case_flag = 1;
end

N = size(P,2);

if motion_direction == 1
    k = 0;
elseif motion_direction == -1
    k = 1;
else
    disp('ERROR: motion_direction can be either -1 or 1')
end

u = [];
for i=1:N
    
    x = P(1,i); % not used
    y = P(2,i); % not used
    dx = dP(1,i);
    dy = dP(2,i);
    ddx = ddP(1,i);
    ddy = ddP(2,i);
    dddx = dddP(1,i);
    dddy = dddP(2,i);
    
    v =  motion_direction*sqrt(dx^2 + dy^2);
    
    if case_flag == 1 % car-like robot
    
        num = (dddy*dx - dddx*dy)*v^2 - 3*(ddy*dx - ddx*dy)*(dx*ddx + dy*ddy);
        den = v^6 + L^2*(ddy*dx - ddx*dy)^2;
        w = L*v*num/den;
        
        theta(i) = atan2(dy/v,dx/v);% + k*pi;
    
        % if v > 0, atan2(dy/v,dx/v) = atan2(dy,dx)
        
        num = L*(ddy*dx - ddx*dy);
        den = v^3;
        phi(i) = atan(num/den);
    
    else % unicycle
        
        theta(i) = atan2(dP(2,i),dP(1,i)) + k*pi;
        w = (cos(theta(i))*ddy - sin(theta(i))*ddx)/v;
        
    end
    
    tmp = [v;w];
    u = [u, tmp];
end

%%%EOF
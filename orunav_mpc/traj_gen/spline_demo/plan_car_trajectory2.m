function [state,control] = plan_car_trajectory2(L,s,kGains,tspan,dt,tsplit)
%
% Plan a trajectory for a car-like vehicle
%
% Input:
% ------
% L     - distance between the rear and front axes
% s     - states
% k     - free parameters (k is a vector with two positive entries)
% tspan - time span (e.g., tspan = 0:0.1:10)
%
% Output:
% -------
% control - evolution of the control inptus
%           u(1) - linear velocity 
%           u(2) - steering rate
%
% state   - evolution of the state variables
%

[P,dP,ddP,dddP] = path_poly3(s, kGains, tspan, tsplit);
[state,control]      = flat_outputs(P,dP,ddP,dddP,L);

control(1,:) = control(1,:); % derivative w.r.t. time
control(2,:) = control(2,:); % derivative w.r.t. time

control(2, 1) = ((state(4,1) - s(1,4))/dt); % workaround for the bug in trajectory generation


% --------------------------------------------------------------------------------

function [state,control]  = flat_outputs(P,dP,ddP,dddP,L)
%
% Flat outputs for a car-like vehicle (only forward motion is considered)
%
% Input:
% ------
% P,dP,ddP,dddP - motion profile: position, velocity, acceleration and jerk
%                 (formed in function poly_path.m)
%
% L             - distance between the rear and front axes
%
% Output:
% -------
% control - evolution of the control inptus
%           u(1) - linear velocity 
%           u(2) - steering rate
%
% state   - evolution of the state variables
%

N = size(P,2); % number of points

control = zeros(2,N);
state = zeros(4,N);
for i=1:N
    
    % ------------------------------
    % get input
    % ------------------------------
       x =    P(1,i); 
       y =    P(2,i); 
      dx =   dP(1,i);
      dy =   dP(2,i);
     ddx =  ddP(1,i);
     ddy =  ddP(2,i);
    dddx = dddP(1,i);
    dddy = dddP(2,i);
    % ------------------------------
    
    v = sqrt(dx^2 + dy^2);
    
    num = (dddy*dx - dddx*dy)*v^2 - 3*(ddy*dx - ddx*dy)*(dx*ddx + dy*ddy);
    den = v^6 + L^2*(ddy*dx - ddx*dy)^2;
    w = L*v*num/den;
        
    num = L*(ddy*dx - ddx*dy);
    den = v^3;

    % ------------------------------
    % form output
    % ------------------------------
    state(1,i) = x;
    state(2,i) = y;
    state(3,i) = atan2(dy/v,dx/v);
    state(4,i) = atan(num/den);
%keyboard
    control(1,i) = v;
    control(2,i) = w;
    % ------------------------------
    
end

%%%EOF flat_outputs

% --------------------------------------------------------------------------------

function [P, dP, ddP, dddP, q, dq] = path_poly3(s, k, tspan, tsplit)
%
% Path planning for a car-like vehicle using cubic polynomials
%
% Input:
% ------
% s     - states
% k     - free parameters (k is a vector with two positive entries)
% tspan - time span (e.g., tspan = 0:0.1:10)
% 
% Output:
% -------
% P,dP,ddP,dddP - motion profile: position, velocity, acceleration and jerk 
%  q            - arc length profile (between 0:1)
% dq            - speed on the path
%
% Note: 
% -----
% s = (x,y,theta,phi), but phi is not considered in the analysis
%

vxi = k(1)*cos(s(1,3));
vyi = k(1)*sin(s(1,3));

vxf = k(2)*cos(s(end,3));
vyf = k(2)*sin(s(end,3));


outx = spline(tspan(tsplit), [vxi s(:,1)' vxf]);
outy = spline(tspan(tsplit), [vyi s(:,2)' vyf]);

px = outx.coefs(:,4:-1:1);
py = outy.coefs(:,4:-1:1);


   x = [];
  dx = [];
 ddx = [];
dddx = [];

   y = [];
  dy = [];
 ddy = [];
dddy = [];


dq = [];

N = size(s, 1);

for i = 1:N-1
    if (i == 1)
        tt = tspan(1:tsplit(i+1));
    else
        tt = tspan(tsplit(i)+1:tsplit(i+1)) - tspan(tsplit(i)+1);
    end

       x = [   x px(i,1) + px(i,2)*tt +   px(i,3)*tt.^2 +   px(i,4)*tt.^3];
      dx = [  dx           px(i,2)    + 2*px(i,3)*tt    + 3*px(i,4)*tt.^2];
     ddx = [ ddx                         2*px(i,3)      + 6*px(i,4)*tt];
    dddx = [dddx                                          6*px(i,4)*ones(1,length(tt))];

       y = [   y py(i,1) + py(i,2)*tt +   py(i,3)*tt.^2 +   py(i,4)*tt.^3];
      dy = [  dy           py(i,2)    + 2*py(i,3)*tt    + 3*py(i,4)*tt.^2];
     ddy = [ ddy                         2*py(i,3)      + 6*py(i,4)*tt];
    dddy = [dddy                                          6*py(i,4)*ones(1,length(tt))];
end

   P = [   x;   y];
  dP = [  dx;  dy];
 ddP = [ ddx; ddy];
dddP = [dddx;dddy];

%%%EOF path_poly3


%%%EOF plan_car_trajectory

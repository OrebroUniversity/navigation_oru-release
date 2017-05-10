function [state,control] = plan_car_trajectory(L,s0,s1,kGains,tspan,dt)
%
% Plan a trajectory for a car-like vehicle
%
% Input:
% ------
% L     - distance between the rear and front axes
% s0    - initial state
% s0    - final state 
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

[P,dP,ddP,dddP,q,dq] = path_poly3(s0,s1,kGains, tspan);
[state,control]      = flat_outputs(P,dP,ddP,dddP,L);

control(1,:) = control(1,:).*dq; % derivative w.r.t. time
control(2,:) = control(2,:).*dq; % derivative w.r.t. time

control(2, 1) = ((state(4,1) - s0(4))/dt); % workaround for the bug in trajectory generation


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

function [P, dP, ddP, dddP, q, dq] = path_poly3(s0, s1, k, tspan)
%
% Path planning for a car-like vehicle using cubic polynomials
%
% Input:
% ------
% s0    - initial state
% s0    - final state 
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

[q,dq] = poly3(0, 1, 0, 0, tspan); % generate speed profile on the path
q = q(:)'; dq = dq(:)';

t0 = 0; t1 = 1;
A = [ 1 t0    t0^2    t0^3;
      1 t1    t1^2    t1^3;
      0  1  2*t0    3*t0^2;
      0  1  2*t1    3*t1^2];

bx = [         s0(1);
               s1(1);
      k(1)*cos(s0(3));
      k(2)*cos(s1(3))];

by = [         s0(2);
               s1(2);
      k(1)*sin(s0(3));
      k(2)*sin(s1(3))];

tmp = A\[bx,by];
px = tmp(:,1);
py = tmp(:,2);

   x = px(1) + px(2)*q +   px(3)*q.^2 +   px(4)*q.^3;
  dx =         px(2)   + 2*px(3)*q    + 3*px(4)*q.^2;
 ddx =                   2*px(3)      + 6*px(4)*q;
dddx =                                + 6*px(4)*ones(1,length(q));

   y = py(1) + py(2)*q +   py(3)*q.^2 +   py(4)*q.^3;
  dy =         py(2)   + 2*py(3)*q    + 3*py(4)*q.^2;
 ddy =                   2*py(3)      + 6*py(4)*q;
dddy =                                + 6*py(4)*ones(1,length(q));

   P = [   x;   y];
  dP = [  dx;  dy];
 ddP = [ ddx; ddy];
dddP = [dddx;dddy];

%%%EOF path_poly3

% --------------------------------------------------------------------------------

function [q,dq,ddq] = poly3(qi,qf,vi,vf,t)
%
% Using 3rd order polynomial generate position, velocity and acceleration profiles  
% that satisfy the initial and final position and velocity constraints. 
% 
% Syntax:
% -------
% [q,dq,ddq] = poly3(qi,qf,vi,vf,t)
%
% Input:
% ------
% qi   - initial position
% qf   - final position
% vi   - initial velocity
% vf   - final velocity
% t    - [ti : time_step : tf]
%
% Output:
% -------
% q    - position profile
% dq   - velocity profile
% ddq  - acceleration profile
%
% Example: 
% --------
% [q, dq, ddq] = poly3(5, 30, 0, 0, [0:0.01:10])
%

t = t(:); ti = t(1); tf = t(end); 
N = length(t);
I = ones(N,1);
Z = zeros(N,1);

b = [qi;vi;qf;vf];
A = [  ti^3    ti^2   ti  1;   
     3*ti^2  2*ti     1   0;   
       tf^3    tf^2   tf  1;   
     3*tf^2  2*tf     1   0];  
p = A\b;                     % p = [a b c d]'

tt = [t.^3 t.^2 t I]; 
q = tt*p;                    % position

tt = [3*t.^2 2*t I Z];
dq = tt*p;                   % velocity

tt = [6*t 2*I Z Z];
ddq = tt*p;                  % acceleration

%%%EOF poly3

% --------------------------------------------------------------------------------

%%%EOF plan_car_trajectory

function [P,dP,ddP,dddP,ds] = poly_path(xi,xf,yi,yf,Ti,Tf,ki,kf,dt,t_f)
%
% Implements the approach in Siciliano pp. 492 
% (path planning using cubic polynomials)
%
% Input:
% ------
% (xi,xy) - initial position 
% (xf,yf) - final position
% Ti,Tf   - initial and final angles of the shasi
% ki,kf   - free parameters
% dt      - time sampling
% t_f     - final time
% 
% Output:
% -------
% P,dP,ddP,dddP - position, velocity, acceleration and jerk motion profile
%
% Note: 
% -----
% dddP is used with the model of a car-like robot
%

if 1
  [s,ds] = poly3(0, 1, 0, 0, [0:dt:t_f]);
  s = s(:)';
  ds = ds(:)';
else
  s = 0:dt:1;
  ds = diff(s);
end

a1 =  kf*cos(Tf) + ki*cos(Ti) + 2*(xi-xf);
a2 = -kf*cos(Tf) - 2*ki*cos(Ti) + 3*(xf-xi);
a3 =  ki*cos(Ti);
a4 =  xi;

x   = a1*s.^3 + a2*s.^2 + a3*s + a4;
dx  = 3*a1*s.^2 + 2*a2*s + a3;
ddx = 6*a1*s + 2*a2;
dddx = 6*a1.*ones(1,length(s));

b1 =  kf*sin(Tf) + ki*sin(Ti) + 2*(yi-yf);
b2 = -kf*sin(Tf) - 2*ki*sin(Ti) + 3*(yf-yi);
b3 =  ki*sin(Ti);
b4 =  yi;

y   = b1*s.^3 + b2*s.^2 + b3*s + b4;
dy  = 3*b1*s.^2 + 2*b2*s + b3;
ddy = 6*b1*s + 2*b2;
dddy = 6*b1.*ones(1,length(s));

P = [x;y];
dP = [dx;dy];
ddP = [ddx;ddy];
dddP = [dddx;dddy];

%%%EOF
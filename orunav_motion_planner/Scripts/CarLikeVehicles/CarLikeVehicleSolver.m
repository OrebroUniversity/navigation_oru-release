function [path motion] = CarLikeVehicleSolver(distance_between_axes, ...
    start_x, start_y, start_theta, start_phi, ...
    goal_x, goal_y, goal_theta, goal_phi, ...
    movie_flag, plot_flag)

% Implementation of the approach in:
%
% Yongji Wang
% "Nonholonomic motion planning: a polynomail fitting approach" - ICRA 1996
%
% Note on approach: Express y as a function of x.
%                   theta \in (-pi/2, pi/2) because of tan(theta)
%                         (furthermore, we divide by cos(theta))
%
%                   Using a 5-th order polynomial.

% Note: start_theta \in (0, pi/4)

% initi return varlables
path = 0;
motion = 1;

s0 = zeros(4,1); s1 = zeros(4,1);

% -----------------------------------------------
% user input
% -----------------------------------------------

L = distance_between_axes;  % length of the car-like vehicle
t = 5;                      % final time (the initial time is assumed to be 0)
N = 100;                    % number of sampling times

% initial state
s0(1) = start_x;        % x
s0(2) = start_y;        % y
s0(3) = start_theta;    % theta
s0(4) = start_phi;      % phi

% final state
s1(1) = goal_x;         % x
s1(2) = goal_y;         % y
s1(3) = goal_theta;     % theta
s1(4) = goal_phi;       % phi


% -----------------------------------------------
% rotations to ensure theta \in (-pi/2, pi/2)
% -----------------------------------------------

z = s1(3) - s0(3);
z = atan2(sin(z), cos(z));

% not safely computable
if abs(z) > (3/4 * pi)
    %fprintf(1, 'No solution');
    return;
end

rMatrix = [1 0;
           0 1];
rotAngle = 0;

% if one of the angle is outside the bounds, we have to rotate
if abs(s0(3)) > (3/8 * pi) || abs(s1(3)) > (3/8 * pi)

    if abs(s0(3)) > abs(s1(3))
        rotAngle = sign(s0(3))*(abs(s0(3)) - 3/8 * pi);
    else
        rotAngle = sign(s1(3))*(abs(s1(3)) - 3/8 * pi);
    end
    rotAngle = -atan2(sin(rotAngle), cos(rotAngle));
    rMatrix = [cos(rotAngle) -sin(rotAngle);
        sin(rotAngle) cos(rotAngle)];
end

if rotAngle ~= 0
    startPoint = rMatrix*[s0(1); s0(2)];
    s0(1) = startPoint(1);
    s0(2) = startPoint(2);
    goalPoint = rMatrix*[s1(1); s1(2)];
    s1(1) = goalPoint(1);
    s1(2) = goalPoint(2);
    s0(3) = s0(3) + rotAngle;
    s0(3) = atan2(sin(s0(3)), cos(s0(3)));
    s1(3) = s1(3) + rotAngle;
    s1(3) = atan2(sin(s1(3)), cos(s1(3)));
end


if s1(1) > s0(1)
    motion = 1;
else
    motion =-1;
end


% -----------------------------------------------
% form boundary conditions
% -----------------------------------------------

x0 = s0(1);
x1 = s1(1);

y0 = s0(2);
y1 = s1(2);

dy0 = tan(s0(3));
dy1 = tan(s1(3));

ddy0 = tan(s0(4))/(L*cos(s0(3))^3);
ddy1 = tan(s1(4))/(L*cos(s1(3))^3);

% -----------------------------------------------
% find parameters
% -----------------------------------------------

A = [1 x0   x0^2   x0^3    x0^4    x0^5;
    1 x1   x1^2   x1^3    x1^4    x1^5;
    0  1 2*x0   3*x0^2  4*x0^3  5*x0^4;
    0  1 2*x1   3*x1^2  4*x1^3  5*x1^4;
    0  0    2   6*x0   12*x0^2 20*x0^3;
    0  0    2   6*x1   12*x1^2 20*x1^3];

b = [  y0;
    y1;
    dy0;
    dy1;
    ddy0;
    ddy1];

% matrix ill conditioned
if cond(A) > 1.0e+20
    %fprintf(1, 'No solution');
    return;
end
p = A\b;

% -----------------------------------------------
% generate profile
% -----------------------------------------------

x = linspace(s0(1),s1(1),N);

for i=1:N
    y(i) = p(1) + p(2)*x(i) +   p(3)*x(i)^2 +   p(4)*x(i)^3 +    p(5)*x(i)^4 +    p(6)*x(i)^5;
    dy(i) =        p(2)      + 2*p(3)*x(i)   + 3*p(4)*x(i)^2 +  4*p(5)*x(i)^3 +  5*p(6)*x(i)^4;
    ddy(i) =                    2*p(3)        + 6*p(4)*x(i)   + 12*p(5)*x(i)^2 + 20*p(6)*x(i)^3;
end

% find theta and phi
for i=1:N
    theta(i) = atan(dy(i));
    phi(i) = atan(ddy(i)*L*cos(theta(i))^3);
end

s = [x;y;theta;phi];


% -----------------------------------------------
% reverse rotation
% -----------------------------------------------

if rotAngle ~= 0
    for i=1:size(s,2)
        point = rMatrix'*[s(1,i); s(2,i)];
        s(1,i) = point(1);
        s(2,i) = point(2);
        s(3,i) = s(3,i) - rotAngle;
        s(3,i) = atan2(sin(s(3,i)), cos(s(3,i)));
    end
end

path =  [[start_x; s(1,2:size(s,2)-1)'; goal_x] [start_y; s(2,2:size(s,2)-1)'; goal_y] [start_theta; s(3,2:size(s,2)-1)'; goal_theta] [start_phi; s(4,2:size(s,2)-1)'; goal_phi]];

if ~plot_flag
    return
end



% -----------------------------------------------
% plot
% -----------------------------------------------

figure(1)
subplot(2,2,1);
plot(s(1,:),s(2,:),'b');grid on; axis equal
title('(x,y) [m]')

subplot(2,2,2);
plot(x,s(3,:),'b');grid on
title('theta [rad]')

subplot(2,2,[3,4]);
plot(x,s(4,:),'b');grid on
title('phi [rad]')

if ~movie_flag
    return
end

% -----------------------------------------------
% movie
% -----------------------------------------------

figure(2)
plot(s(1,:),s(2,:),'b');grid on; axis equal
hold on;

% refline(slopeStart, s0(2));
% refline(slopePerpendicular, s1(2));
% plot(xBeforeSymmetry, yBeforeSymmetry, 'x');
% plot(xSymmetric, ySymmetric, 'x');

for i=1:N
    h = draw_car(L,s(:,i));
    drawnow
    %pause
    
    if i>1 && i<N
        for k=1:length(h)
            set(h{k},'Visible','off');
        end
    end
end



%%%EOF
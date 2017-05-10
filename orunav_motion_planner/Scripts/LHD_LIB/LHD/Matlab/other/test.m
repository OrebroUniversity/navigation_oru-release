clear;clc;cla

options = odeset('RelTol',1e-6);

% ---------------------------------------------------
[T,Y] = ode45(@ode,[0 12],[0 1 1],options);

[Tb,Yb] = ode45(@ode_back,[-12 0],Y(end,:),options);

hold on
plot(T,Y)
plot(-Tb,Yb,'k--')
grid on


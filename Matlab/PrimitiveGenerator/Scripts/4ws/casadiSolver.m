function [path,motion] = casadiSolver(wheel_base, s0, s1, movie_flag, plot_flag)
addpath('/home/ubuntu18/casadi')
% ----------------------
% An optimal control problem (OCP),
% solved with direct multiple-shooting.
%
% For more information see: http://labs.casadi.org/OCP
import casadi.*
N = 100; % number of control intervals

opti = casadi.Opti(); % Optimization problem

% ---- decision variables ---------
X = opti.variable(5,N+1); % state trajectory
x       = X(1,:);
y       = X(2,:);
th      = X(3,:);
phi     = X(4,:);
phiRear = X(5,:);

U = opti.variable(3,N);   % control trajectory (throttle)
v = U(1,:);
w = U(2,:);
wr = U(3,:);

T = opti.variable();
% ---- objective          ---------

opti.minimize( U(1) ); % race in minimal time

% ---- dynamic constraints --------
f = @(x,u) [cos(x(3)+x(5))*u(1);
            sin(x(3)+x(5))+u(1);
            u(1)*sin(x(4)-x(5))/(wheel_base*cos(x(4)));
            u(2);
            u(3)]; % dx/dt = f(x,u)
    


dt = T/N; % length of a control interval
for k=1:N % loop over control intervals
   % Runge-Kutta 4 integration
   k1 = f(X(:,k),         U(:,k));
   k2 = f(X(:,k)+dt/2*k1, U(:,k));
   k3 = f(X(:,k)+dt/2*k2, U(:,k));
   k4 = f(X(:,k)+dt*k3,   U(:,k));
   x_next = X(:,k) + dt/6*(k1+2*k2+2*k3+k4); 
   opti.subject_to(X(:,k+1)==x_next); % close the gaps
end

% ---- path constraints -----------

opti.subject_to(-0.6<=x(4)<=0.6);           % control is limited
opti.subject_to(-0.6<=x(5)<=0.6); 
% ---- boundary conditions --------
opti.subject_to(x(1)== s0(1));   % start at position 0 ...
opti.subject_to(y(1)== s0(2));
opti.subject_to(th(1)== s0(3));
opti.subject_to(phi(1)== s0(4));
opti.subject_to(phiRear(1)== s0(5));

opti.subject_to(x(N+1)==s1(1)); % finish line at position 1
opti.subject_to(y(N+1)==s1(2));
opti.subject_to(th(N+1)==s1(3));
opti.subject_to(phi(N+1)==s1(4));
opti.subject_to(phiRear(N+1)==s1(5));
% ---- misc. constraints  ----------
opti.subject_to(T>=0); % Time must be positive

% ---- initial values for solver ---

opti.set_initial(T, 1);
% ---- solve NLP              ------
opti.solver('ipopt'); % set numerical backend
sol = opti.solve();   % actual solve

% ---- post-processing        ------

figure
hold on
plot(sol.value(speed));
plot(sol.value(pos));
plot(limit(sol.value(pos)),'r--');
stairs(1:N,sol.value(U),'k');
legend('speed','pos','speed limit','throttle','Location','northwest')


figure
spy(sol.value(jacobian(opti.g,opti.x)))
figure
spy(sol.value(hessian(opti.f+opti.lam_g'*opti.g,opti.x)))

path = 1;
motion =1 ;

end


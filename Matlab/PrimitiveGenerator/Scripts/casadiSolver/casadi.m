function [sol] = casadi(start,goal)
addpath('/home/ubuntu18/catkin_ws/src/navigation_oru/Matlab/PrimitiveGenerator/Scripts/casadiSolver');
import casadi.*;
N = 100; % number of control intervals

 opti = casadi.Opti();

 X  =   opti.variable(6,N+1);
     x      =   S(0,all);
     y      =   S(1,all);
     th     =   S(2,all);
     phi1   =   S(3,all);
     phi2   =   S(4,all);
     v      =   S(5,all);

    
U  =   opti.variable(3,N);
     dv   =   U(0,all);
     dw   =   U(1,all);
     dwr  =   U(2,all);
 
 obj = dv*dv + dw*dw + dwr*dwr ;
 
 opti.minimize(obj);
  dt = 0.2;
  for k=1:N % loop over control intervals
   % Runge-Kutta 4 integration
   k1 = dynamic(X(:,k),         U(:,k));
   k2 = dynamic(X(:,k)+dt/2*k1, U(:,k));
   k3 = dynamic(X(:,k)+dt/2*k2, U(:,k));
   k4 = dynamic(X(:,k)+dt*k3,   U(:,k));
   x_next = X(:,k) + dt/6*(k1+2*k2+2*k3+k4); 
   opti.subject_to(X(:,k+1)==x_next); % close the gaps
  end
 
    opti.subject_to( -max_steering_ <= phi1 <= max_steering_);
    opti.subject_to( -max_steering_ <= phi2 <= max_steering_);
    opti.subject_to( -max_vel_      <= v    <= max_vel_);

    % --- velocity constraints ---
    opti.subject_to( -2*max_dv_ <= dv  <= max_dv_);
    opti.subject_to( -max_dw_   <= dw  <= max_dw_);
    opti.subject_to( -max_dw_   <= dwr <= max_dw_);

  
    % ---  boundary start conitions ---
    opti.subject_to( x(1) == start(1)  );
    opti.subject_to( y(1) == start(2) );
    opti.subject_to( th(1)== start(3) );
    opti.subject_to( phi1(1) == start(4) );
    opti.subject_to( phi2(1) == start(5) );
    opti.subject_to( v(1)== 0);
    
     % ---  boundary goal conitions ---
    opti.subject_to( x(N) == goal(1)  );
    opti.subject_to( y(N) == goal(2) );
    opti.subject_to( th(N)== goal(3) );
    opti.subject_to( phi1(N) == goal(4) );
    opti.subject_to( phi2(N) == goal(5) );
    opti.subject_to( v(N)== 0);
    
    opti.solver('ipopt'); % set numerical backend
    sol = opti.solve();
  
 
end


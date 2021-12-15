function [solution, motion] = casadi(L, start, goal, movie_flag, plot_flag)
addpath('/home/ubuntu18/catkin_ws/src/navigation_oru/Matlab/PrimitiveGenerator/Scripts/casadiSolver')
import casadi.*
%N = 100; % number of control intervals
load('bounds.mat');
% N=200;
if goal(1)>0
    max_vel_low = 0;%0
    max_vel_up = max_vel;
    mt = 1;
else
    max_vel_low = max_vel;
    max_vel_up = 0;%0
    mt = -1;
end

solution = 0;
motion = 0;
z = goal(3) - start(3);
z = atan2(sin(z), cos(z));

% not safely computable
if abs(z) > (2/4 * pi) %3/4
    %fprintf(1, 'No solution');
    return;
end

% if  (abs(start(3)-goal(3)) > 1.58)
%     fprintf("angolo");
%     return
% end

opti = casadi.Opti();

 X  =   opti.variable(6,N+1);
     x      =   X(1,:);
     y      =   X(2,:);
     th     =   X(3,:);
     phi1   =   X(4,:);
     phi2   =   X(5,:);
     v      =   X(6,:);

    
U  =   opti.variable(3,N);
     dv   =   U(1,:);
     dw   =   U(2,:);
     dwr  =   U(3,:);
 obj = 0;
 for k=1:N
    obj = obj + 2*(dv(k)*dv(k)) + 1*( dw(k)*dw(k) + dwr(k)*dwr(k) ) ;
 end
 
 
 f = @(x,u,L) [
     x(6)*cos(x(3) + x(5));
     x(6)*sin(x(3) + x(5));
     atan2(sin( x(6)*sin(x(4)-x(5))/(L*cos(x(4))) ), cos( x(6)*sin(x(4)-x(5))/(L*cos(x(4))) ) );
     u(2);
     u(3);
     u(1)];

 
 opti.minimize(obj );
  dt = 0.3;
  for k=1:N % loop over control intervals
   % Runge-Kutta 4 integration
   k1 = f(X(:,k),         U(:,k), L);
   k2 = f(X(:,k)+dt/2*k1, U(:,k), L);
   k3 = f(X(:,k)+dt/2*k2, U(:,k), L);
   k4 = f(X(:,k)+dt*k3,   U(:,k), L);
   x_next = X(:,k) + dt/6*(k1+2*k2+2*k3+k4); 
   opti.subject_to(X(:,k+1)==x_next); % close the gaps
  end
 
    opti.subject_to( -max_steering <= phi1 <= max_steering);
    opti.subject_to( -max_steering <= phi2 <= max_steering);
    opti.subject_to( -max_vel_low     <= v    <= max_vel_up);
    
   % for k = 2:N
        %opti.subject_to( v(k)/abs(v(k) ) == v(k-1)/abs(v(k-1)) );
        %diff = v(k)/abs(v(k) ) - v(k-1)/abs(v(k-1));
        %diff = abs(diff);
        %obj = obj + 1000*diff;
   % end

    % --- velocity constraints ---
    opti.subject_to( -max_dv <= dv  <= max_dv);
    opti.subject_to( -max_dw   <= dw  <= max_dw);
    opti.subject_to( -max_dw  <= dwr <= max_dw);

  
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
    
    try
        %opts =struct;
        %opts.print_time = 'false';
    opti.solver('ipopt'); % set numerical backend
    sol = opti.solve();
    
    solution = [sol.value(x)',sol.value(y)',sol.value(th)',sol.value(phi1)',sol.value(phi2)'];
    solution = round(solution,3);
    sol_v = sol.value(v);
    catch
     %   opti.debug;
        fprintf("NO SOLUTION");
        return;
    end
    for i = 1:N
        motion(i) = mt;
    end
    %motion = sol_v;
    
    if plot_flag
        plot(solution(:,1),solution(:,2));
        grid on; axis equal;
    end
    
    if movie_flag
        for i=1:N
            %h = draw_car4w(L,s(:,i));
            h = draw_4wsRear(L/2,L/2,solution(i,:)');
            drawnow
            pause(0.05)

            if i>1 && i<N
                for k=1:length(h)
                    set(h{k},'Visible','off');
                end
            end
        end
    end
 
end


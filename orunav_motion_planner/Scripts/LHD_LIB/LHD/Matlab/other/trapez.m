function [T,q,dq,ddq,err] = trapez(q_i,q_f,dq_c,t_f,Ts);
%TRAPEZ Trapezoidal velocity profile.
%       [T,q,dq,ddq,err] = TRAPEZ(q_i,q_f,dq_c,t_f,Ts) returns
%       trajectory with trapezoidal velocity profile of maximum velocity dq_c
%       from initial position q_i to final position q_f, where:
%
%       t_f and t_s are respectively trajectory duration and sample time
%       q, dq and ddq are respectively vectors of position, velocity and
%         acceleration
%       T is vector of time base
%
%       If err = 1 then dq_c is not compatible with other trajectory parameters

% L. Villani, G. Oriolo, B. Siciliano
% February 2009

% time base vector
  T = (0:Ts:t_f)';
  n = size(T,1);

% variables initialization
  q = zeros(n,1);
  dq = zeros(n,1);
  ddq = zeros(n,1);
  err = 0;
  delta = q_f - q_i;
  dq_c = sign(delta)*abs(dq_c);

 if (delta ~= 0),

 % constraint verification
   dq_r = abs(delta/t_f);
   err = (abs(dq_c) <= dq_r)|(abs(dq_c) > 2*dq_r);

   if (err==0)

   % evaluates t_c
     t_c = t_f - delta/dq_c;

   % evaluates transition indices of velocity vector
     n1 = max(1,fix(n*t_c/t_f));
     n2 = n - n1;

   % evaluates ddq_c
     ddq_c = dq_c/t_c;

   % trajectory generation
     q   = [q_i + 0.5*ddq_c*T(1:n1).^2
            q_i + dq_c*(T(n1 + 1:n2)-0.5*t_c);
            q_f - 0.5*ddq_c*(T(n2 + 1:n) - t_f).^2];
     dq  = [ddq_c*T(1:n1);
            dq_c*ones(n2 - n1,1);
            ddq_c*(t_f - T(n2 + 1:n))];
     ddq(1:n1) = ddq_c*ones(1,n1);
     ddq(n2 + 1:n) = - ddq_c*ones(1,n1);

   end;

 end;

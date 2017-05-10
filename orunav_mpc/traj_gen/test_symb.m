%
%
%

syms l dt
syms t1 v1 p1
syms t2 v2 p2
syms t3 v3 p3
syms t4 v4 p4

A1 = [1, 0, -sin(t1)*v1*dt,                   0; 
      0, 1,  cos(t1)*v1*dt,                   0; 
      0, 0,              1, v1*dt/(l*cos(p1)^2);  
      0, 0,              0,                   1];

A2 = [1, 0, -sin(t2)*v2*dt,                   0; 
      0, 1,  cos(t2)*v2*dt,                   0; 
      0, 0,              1, v2*dt/(l*cos(p2)^2);  
      0, 0,              0,                   1];

A3 = [1, 0, -sin(t3)*v3*dt,                   0; 
      0, 1,  cos(t3)*v3*dt,                   0; 
      0, 0,              1, v3*dt/(l*cos(p3)^2);  
      0, 0,              0,                   1];

A4 = [1, 0, -sin(t4)*v4*dt,                   0; 
      0, 1,  cos(t4)*v4*dt,                   0; 
      0, 0,              1, v4*dt/(l*cos(p4)^2);  
      0, 0,              0,                   1];

B1 = [cos(t1)*dt,   0;
      sin(t1)*dt,   0;
      dt*tan(p1)/l, 0;
      0,           dt];

B2 = [cos(t2)*dt,   0;
      sin(t2)*dt,   0;
      dt*tan(p2)/l, 0;
      0,           dt];

B3 = [cos(t3)*dt,   0;
      sin(t3)*dt,   0;
      dt*tan(p3)/l, 0;
      0,           dt];

B4 = [cos(t4)*dt,   0;
      sin(t4)*dt,   0;
      dt*tan(p4)/l, 0;
      0,           dt];

% ---------------------------------------------------

a1 = A1;
a2 = A2*A1;
a3 = A3*A2*A1;
a4 = A4*A3*A2*A1;

%%%EOF
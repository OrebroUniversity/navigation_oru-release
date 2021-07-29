% 4ws 
syms x y phi df dr thf thr real
syms beta v vf vr r lf lr real

%q = [ x y phi df dr thf thr ]';
%  x,y centro del robot
%  phi orientazione
%  df angolo di sterzo ruota anteriore
%  dr angolo di sterzo ruota posteriore
%  thf posizione angolare ruota anteriore  (?)
%  thr posizione angolare ruota posteriore (?)


% cinematica 
%[Danwei Wang 2001 Trajectory Planning for a Four-Wheel-Steering Vehicle]


beta = atan( ( lf*tan(dr) + lr*tan(df) )/(lf+lr) );
v = (vf*cos(df) + vr*cos(dr))/2*cos(beta);

x_d = v*cos(phi+beta);
y_d = v*sin(phi+beta);
phi_d = v*cos(beta)*(tan(df) - tan(dr))/(lf+lr);



%vincoli 
% Robert Grepl, 2011 ,Development of 4WS/4WD Experimental Vehicle: platform for research and education in mechatronics
% A. De Luca and G. Oriolo, Kinematics and Dynamics of Multi-Body Systems, CISM Courses and Lectures, vol. 360. Springer Verlag, Wien, 1995, ch. Modeling and Control of Nonholonomic Mechanical Systems,p. pp. 277342.

%vincoli anolonomi
eq1 = x_d*sin(phi+df) - y_d*cos(phi+df) - phi_d*lf*cos(df) == 0;
eq2 = x_d*sin(phi+dr) - y_d*cos(phi+dr) + phi_d*lr*cos(df) == 0;

%relazioni velocità
eq3 = x_d*cos(phi+df) + y_d*sin(phi+df) + phi_d*lf*sin(df) - thf_d*r == 0;
eq4 = x_d*cos(phi+dr) + y_d*sin(phi+dr) - phi_d*lr*sin(dr) - thr_d*r == 0;

%A(q)
q = [ x y phi df dr thf thr ]';

A = [ sin(phi+df) -cos(phi+df) -lf*cos(df) 0 0 0 0;
         sin(phi+dr) -cos(phi+dr) +lr*cos(dr) 0 0 0 0;
         cos(phi+df) +sin(phi+df) +lf*sin(df) 0 0 -r 0;
         cos(phi+dr) +sin(phi+dr) -lr*sin(dr) 0 0 0 -r];
 
     
%% forma affine 
syms l real

A = subs(A,lr,l); %suppngo lf = lr;
A = subs(A,lf,l);

s31 = -r*( sin(phi)*cos(dr)*sin(df) + sin(phi)*sin(dr)*cos(df) - 2*cos(dr)*cos(phi)*cos(df))/(2*cos(dr));
s32 = r* ( 2*cos(df)*cos(dr)*sin(phi) + cos(phi)*cos(dr)*sin(df) + cos(phi)*sin(dr)*cos(df)) /(2*cos(dr));
s33 = r* ( cos(dr)*sin(df) - sin(dr)*cos(df) ) / ( 2*(l)*cos(dr));
s37 = cos(df)/cos(dr);


s41 = -r*( sin(phi)*cos(dr)*sin(df) + sin(phi)*sin(dr)*cos(df) - 2*cos(dr)*cos(phi)*cos(df))/(2*cos(df));
s42 = r* ( 2*cos(df)*cos(dr)*sin(phi) + cos(phi)*cos(dr)*sin(df) + cos(phi)*sin(dr)*cos(df)) /(2*cos(df));
s43 = r* ( cos(dr)*sin(df) - sin(dr)*cos(df) ) / ( 2*(l)*cos(df));
s46 = cos(dr)/cos(df);


s1 = [ 0 0 0 1 0 0 0]';
s2 = [ 0 0 0 0 1 0 0]';
s3 = [ s31 s32 s33 0 0 1 s37]';
s4 = [ s41 s42 s43 0 0 s46 1]';

s = [s1 s2 s3 s4];
%simplify(A*s) % A*s = 0
%%
syms x_d y_d phi_d df_d dr_d thf_d thr_d real
u = [df_d dr_d thf_d thr_d];
q_d = s1*df_d + s2*dr_d + s3*thf_d + s4*thr_d  %??

%%
% Implementation of the approach in:
%
% Yongji Wang
% "Nonholonomic motion planning: a polynomail fitting approach" - ICRA 1996
% 
% ( ???????????????????? 
%verificare accessibilità con la chiusura involutiva

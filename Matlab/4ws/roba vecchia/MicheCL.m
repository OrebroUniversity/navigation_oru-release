clear all
clc
syms x y th phi df dr   th  real
vf=1; vr=1; l=0.5; lf=1;lr=1;
syms u1 u2 real
%%
A = [ sin(phi+df) -cos(phi+df) -lf*cos(df) 0 0 ;
         sin(phi+dr) -cos(phi+dr) +lr*cos(dr) 0 0 ];


beta = atan( ( lf*tan(dr) + lr*tan(df) )/(lr+lf) );     
v = (vf*cos(df) + vr*cos(dr))/(2*cos(beta));

S1 = [0 0 0 1 0]';
S2 =  [0 0 0 0 1]';
S3 = [ cos(phi+beta), sin(phi+beta) , cos(beta)*(tan(df)-tan(dr))/(lf+lr), 0, 0]';
test1 = A * [ v*cos(phi+beta), v*sin(phi+beta) , v*cos(beta)*(tan(df)-tan(dr))/(lf+lr), 0, 0]';
simplify(test1)

f1 = S1*u1 + S2*u2 + S3*v;

%%
% A = [ sin(phi+df) -cos(phi+df) -l*cos(df) 0 0 ;
%          sin(phi+dr) -cos(phi+dr) +l*cos(dr) 0 0 ];
% 
% v = (vf*cos(df) + vr*cos(dr))/(lf+lr);
% beta = atan( ( lf*tan(df) + lr*tan(dr) )/(lf+lr) ); 
% 
% s31 = -( sin(phi)*cos(dr)*sin(df) + sin(phi)*sin(dr)*cos(df) - 2*cos(dr)*cos(phi)*cos(df))/(2*cos(dr));
% s32 = ( 2*cos(df)*cos(dr)*sin(phi) + cos(phi)*cos(dr)*sin(df) + cos(phi)*sin(dr)*cos(df)) /(2*cos(dr));
% s33 =( cos(dr)*sin(df) - sin(dr)*cos(df) ) / ( 2*(l)*cos(dr));
% 
% 
% s1 = [0 0 0 1 0]';
% s2 =  [0 0 0 0 1]';
% s3 = [ s31, s32 , s33, 0, 0]';
% test2 = A * [ s31, s32 , s33, 0, 0]';
% simplify(test2)


%% forma affine 


s31 = -( sin(phi)*cos(dr)*sin(df) + sin(phi)*sin(dr)*cos(df) - 2*cos(dr)*cos(phi)*cos(df))/(2*cos(dr));
s32 = ( 2*cos(df)*cos(dr)*sin(phi) + cos(phi)*cos(dr)*sin(df) + cos(phi)*sin(dr)*cos(df)) /(2*cos(dr));
s33 = ( cos(dr)*sin(df) - sin(dr)*cos(df) ) / ( 2*(l)*cos(dr));
s37 = cos(df)/cos(dr);


s41 = -( sin(phi)*cos(dr)*sin(df) + sin(phi)*sin(dr)*cos(df) - 2*cos(dr)*cos(phi)*cos(df))/(2*cos(df));
s42 = ( 2*cos(df)*cos(dr)*sin(phi) + cos(phi)*cos(dr)*sin(df) + cos(phi)*sin(dr)*cos(df)) /(2*cos(df));
s43 = ( cos(dr)*sin(df) - sin(dr)*cos(df) ) / ( 2*(l)*cos(df));
s46 = cos(dr)/cos(df);


s1 = [ 0 0 0 1 0 0 0]';
s2 = [ 0 0 0 0 1 0 0]';
s3 = [ s31 s32 s33 0 0 1 s37]';
s4 = [ s41 s42 s43 0 0 s46 1]';

f2 = s1*u1 + s2*u2 + s3*vf/2 + s4*vr/2;
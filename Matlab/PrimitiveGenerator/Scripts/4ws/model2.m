% modello veicolo
clear all
%clc
syms x y  phi df dr   th  real
%vf=1; vr=1; 
l=0.5; lf=1;lr=1; L = lf+lr;
syms u1 u2 u3 vr vf real
%% CAR
A = [ sin(phi+df) -cos(phi+df) -L*cos(df) 0  ;
         sin(phi) -cos(phi) 0 0 ];
s = [x y  phi df ]';

g1c = [ cos(phi), sin(phi) , tan(df)/L, 0]';
g2c = [0 0 0 1]';
test0 = A*(g1c*u1 + g2c*u2)
%% 4WS
A = [ sin(phi+df) -cos(phi+df) -lf*cos(df) 0 0 ;
         sin(phi+dr) -cos(phi+dr) +lr*cos(dr) 0 0 ];

s = [x y  phi df dr]';
beta = atan( ( lf*tan(dr) + lr*tan(df) )/(lr+lf) );     
u1 = (vf*cos(df) + vr*cos(dr))/(2*cos(beta));
v = u1;

g1 = [ cos(phi+beta), sin(phi+beta) , cos(beta)*(tan(df)-tan(dr))/(lf+lr), 0, 0]';
g2 = [0 0 0 1 0]';
g3 =  [0 0 0 0 1]';
test1 = A * [ u1*cos(phi+beta), u1*sin(phi+beta) , u1*cos(beta)*(tan(df)-tan(dr))/(lf+lr), u2, u3]';
simplify(test1)
F1 = g1*u1 + g2*u2 + g3*u3 

%% 4ws 2

A = [ sin(phi+df) -cos(phi+df) -L*cos(df) 0  ;
         sin(phi+dr) -cos(phi+dr) 0 0 ];
s = [x y  phi df ]';

g1c = [ cos(phi+dr), sin(phi+dr) , sin(df-dr)/(L*cos(df)), 0]';
g2c = [0 0 0 1]';
F = (g1c*u1 + g2c*u2)
test0 = A*F;
simplify(test0)
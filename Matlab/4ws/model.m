% modello veicolo
clear all
%clc
syms x y  phi df dr   th  real
%vf=1; vr=1; 
l=0.5; lf=1;lr=1;
syms u1 u2 u3 vr vf real
%%
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
F = g1*u1 + g2*u2 + g3*u3 + g4*u1d
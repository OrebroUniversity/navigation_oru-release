function s = move4ws(s0,df,dr,vf,vr,lf,lr)
x0 = s0(1);
y0 = s0(2);
phi0 = s0(3);
beta = atan( ( lf*tan(dr) + lr*tan(df) )/(lf+lr) );
v = (vf*cos(df) + vr*cos(dr))/2*cos(beta);

x = x0 + v*cos(phi0+beta);
y = y0 + v*sin(phi0+beta);
phi = phi0 + v*cos(beta)*(tan(df) - tan(dr))/(lf+lr);
s = [x,y,phi,df,dr];
end


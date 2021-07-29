%problema ottimo con funzione MATLB bvp5c...
% vincoli dipendono dalle variabili di stato.. non applicabile?

%xs = [v df dr];
%s = [x y phi];
xmesh = [0 0 0 0 0 0];

%%non funziona nullaaaaaaa


yInit = [0;0;0];
xmesh = [0 0.25 0.5 ];

yinit = [0; 0;0];
sol = bvpinit(xmesh,yinit);


sol = bvp5c(@(x,y) f(x,y), @bc, sol);
function dydx = f(x,y)
lf = 0.2; lr = 0.2;
df = x(2);
dr = x(3);
beta = atan( ( lf*tan(dr) + lr*tan(df) )/(lf+lr) );

dydx(1) = v*cos(y(3)+beta);
dydx(2) = v*sin(y(3)+beta);
dydx(3) =  v*cos(beta)*(tan(df) - tan(dr))/(lf+lr);

end

function res = bc(x_d,y_d,phi_d)
    
res  = [ x_d*sin(phi+df)-y_d*cos(phi+df)-phi_d*lf*cos(df)  ;
         x_d*sin(phi+dr)-y_d*cos(phi+dr)+phi_d*lr*cos(dr)  ];
end






function [P,dP,ddP,dddP,ds] = poly_pathMiche2(s0,s1,ki,kf,step)
%%
% Implements the approach in Siciliano pp. 492 
% (path planning using cubic polynomials)
%
% Input:
% ------
% s0 - initial position 
% s1 - final position
% 
% ki,kf   - free parameters
% s = 0:step:1;
% 
% Output:
% -------
% P,dP,ddP,dddP - position, velocity, acceleration and jerk motion profile
%
% Note: 
% -----
% dddP is used with the model of a car-like robot
%
xi = s0(1); xf = s1(1);
yi = s0(2); yf = s1(2);
phii = s0(3); phif = s1(3);
dfi = s0(4); dff = s1(4);
dri = s0(5); drf = s0(5);


lf = 0.5;
lr = 0.5;


s = 0:step:1;
ds = diff(s);
N=1/0.01;
beta_i = atan( ( lf*tan(dfi) + lr*tan(dri) )/(lf+lr) );
beta_f = atan( ( lf*tan(dff) + lr*tan(drf) )/(lf+lr) );

Ti = phii + beta_i;
Tf = phif + beta_f;

Li = (tan(dfi)+tan(dri))/(lf+lr);
Lf = (tan(dff)+tan(drf))/(lf+lr);


%x(s) = s^3*xf - (s-1)^3*xi + a_x*s^2*(s-1) + b_x*s*(s-1)^2

c1 =  kf*cos(beta_f)*Lf + ki*cos(beta_i)*Li + 2*(phii-phif);
c2 = -kf*cos(beta_f)*Lf - 2*ki*cos(beta_i)*Li + 3*(phif-phii);
c3 =  ki*cos(beta_i)*Li ;
c4 =  phii;

phi   = c1*s.^3 + c2*s.^2 + c3*s + c4;
dphi  = 3*c1*s.^2 + 2*c2*s + c3;
ddphi = 6*c1*s + 2*c2;
dddphi = 6*c1.*ones(1,length(s));


a1 =  kf*cos(Tf) + ki*cos(Ti) + 2*(xi-xf);
a2 = -kf*cos(Tf) - 2*ki*cos(Ti) + 3*(xf-xi);
a3 =  ki*cos(Ti);
a4 =  xi;

x   = a1*s.^3 + a2*s.^2 + a3*s + a4;
dx  = 3*a1*s.^2 + 2*a2*s + a3;
ddx = 6*a1*s + 2*a2;
dddx = 6*a1.*ones(1,length(s));

b1 =  kf*sin(Tf) + ki*sin(Ti) + 2*(yi-yf);
b2 = -kf*sin(Tf) - 2*ki*sin(Ti) + 3*(yf-yi);
b3 =  ki*sin(Ti);
b4 =  yi;

y   = b1*s.^3 + b2*s.^2 + b3*s + b4;
dy  = 3*b1*s.^2 + 2*b2*s + b3;
ddy = 6*b1*s + 2*b2;
dddy = 6*b1.*ones(1,length(s));




df = 0*ones(1,length(s));
dr = 0*ones(1,length(s));

P = [x;y;phi;df;dr];
dP = [dx;dy;dphi];
ddP = [ddx;ddy;ddphi];
dddP = [dddx;dddy;dddphi];

%%
figure(1)
subplot(2,2,1);
plot(P(1,:),P(2,:),'b');grid on; axis equal
title('(x,y) [m]')

subplot(2,2,2);
plot(s,P(3,:),'b');grid on
title('theta [rad]')


figure(2)
plot(P(1,:),P(2,:),'b');grid on; axis equal
hold on;

for i=1:N
    h = draw_4ws(lf,lr,P(:,i));
    drawnow
    %pause
    
    if i>1 && i<N
        for k=1:length(h)
            set(h{k},'Visible','off');
        end
    end
end


end




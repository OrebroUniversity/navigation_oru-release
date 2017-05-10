%
%
%

clear;clc

a = [-98.81481481481480955154 5.21111111111111036109 -0.08777777777777778789 0.00048148148148148150];

p = inline('a(1) + a(2)*t + a(3)*t^2 + a(4)*t^3','a','t');
dp = inline('a(2) + 2*a(3)*t + 3*a(4)*t^2','a','t');

N = 100;
t = linspace(40,70,N);

for i=1:N
    z(i) = p(a,t(i));
    dz(i) = dp(a,t(i));
end

figure(1);
subplot(2,1,1);plot(t,z,'r--'); grid on
subplot(2,1,2);plot(t,dz,'r--'); grid on

return
     
%%%EOF
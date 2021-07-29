% tecnica per unire waypoint usando un certo grado
%dovrebbe generare curve carine, ma servono più punti??
%genera solo curve senza considerare la geometria del problema


function B = Bezier(s0,s1)
si = [s0(1), s0(2)];
ti = [5*cos(s0(3)),5*sin(s0(3))];

sf = [s1(1), s1(2)];
tf = [5*cos(s1(3)),5*sin(s1(3))];

ki = -6*si - 4*ti -2*tf +6*sf;



P0 = si;
P1 = si+  1/5*ti;

P4 = sf-1/5*tf;
P5 = sf;

P2 = 1/20*ki+2*P1 -si;
P3 =1/20*ki +2*P4 -sf;
i=1;
for t = 0:0.01:1
B(i,:) = (1-t)^5*P0 + 5*t*(1-t)^4*P1 + 10*t^2*(1-t)^3*P2 + 10*t^3*(1-t)^2*P3 + 5*t^4*(1-t)*P4 + t^5*P5;
i = i+1;
end


% figure(1)
% subplot(2,2,1);
% plot(B(1,:),B(2,:),'b');grid on; axis equal
% title('(x,y) [m]')
% 
% subplot(2,2,2);
% plot(s,B(3,:),'b');grid on
% title('theta [rad]')
% 
% 
figure(2)
plot(B(:,1),B(:,2),'b');grid on; axis equal
% hold on;
% 
% for i=1:N
%     h = draw_4ws(lf,lr,B(:,i));
%     drawnow
%     %pause
%     
%     if i>1 && i<N
%         for k=1:length(h)
%             set(h{k},'Visible','off');
%         end
%     end
% end


end



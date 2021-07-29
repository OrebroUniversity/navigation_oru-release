function [h] = draw2(out,L,k,flag)

P = out(:,2:5);
P(:,5) = -k*P(:,4);
N = size(P,1);
color = 'g';
if k == -1 
    color = 'r'; 
elseif k== 1
    color = 'b';
end
figure(3)
hold on;
plot(P(:,1),P(:,2),color);grid on; axis equal

plot(P(end,1),P(end,2),'pg')

if flag
for i=1:N
    
    h = draw_car4w(L,P(i,:)');
    drawnow
    pause(0.5)
    
    if i>1 && i<N
        for k=1:length(h)
            set(h{k},'Visible','off');
        end
    end
end
end
end
function [h] = draw(out,lf,lr,flag)

P = out(:,2:6);
N = size(P,1);
figure(1)
hold on;
plot(P(:,1),P(:,2),'b');grid on; axis equal

plot(P(end,1),P(end,2),'pr')

if flag
for i=1:N

    h = draw_4ws(lf,lr,P(i,:)');
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


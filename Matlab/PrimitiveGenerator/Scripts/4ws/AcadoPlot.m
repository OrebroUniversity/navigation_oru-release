N = length(out.STATES);
L=3.2;
round(out.STATES,4);
plot(out.STATES(:,2),out.STATES(:,3))
for i=1:N
    %h = draw_car4w(L,s(:,i));
    h = draw_4wsRear(L/2,L/2,out.STATES(i,2:6)');
    drawnow
    pause(0.05)
    
    if i>1 && i<N
        for k=1:length(h)
            set(h{k},'Visible','off');
        end
    end
end
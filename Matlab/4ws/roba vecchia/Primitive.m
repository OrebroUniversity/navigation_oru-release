function primitives = Primitive(s0,s1,N,v)
D = [-pi/6 -pi/12 0 pi/12 pi/6]; % 0.706];
primitives = zeros(N,5);
S = zeros(10,5);
vf = v; vr = v;
lf = 0.2; lr =  0.2;
primitives(1,:) = s0;

for i=2:N
    for j = 1:size(D,2)
        df = D(j);
        dr = D(j);
        S(j,:) = move4ws(primitives(i-1,:),df,dr,vf,vr,lf,lr);
    end
    for jj = 1:size(D,2)
        df = D(jj);
        dr = -D(jj);
        S(j+jj,:) = move4ws(primitives(i-1,:),df,dr,vf,vr,lf,lr);
    end
        s = primitives(i-1,:);
        primitives(i,:) = s;
        mod0 = sqrt((s(1)-s1(1))^2 + (s(2)-s1(2))^2 );
        mod1 = mod0;
        for k = 1:size(S,1)
            s = S(k,:);
            mod = sqrt((s(1)-s1(1))^2 + (s(2)-s1(2))^2 );
            if mod < mod1
                primitives(i,:) = s;
                mod1 = mod;
            end
        end
        
      if primitives(i,:) == s1
          for ii=i:N
              primitives(ii,:) = s1;
          end
          break
      end


end
P = primitives;
figure(2)
plot(P(:,1),P(:,2),'b');grid on; axis equal
hold on;

for i=1:N

    h = draw_4ws(lf,lr,P(i,:)');
    drawnow
    %pause
    
    if i>1 && i<N
        for k=1:length(h)
            set(h{k},'Visible','off');
        end
    end
end

end



function [P, motion] = CrabMode(distance_between_axes, s0, s1, Dmax, movieFlag,drawFlag)

step = 100;
lf = distance_between_axes/2;
lr= distance_between_axes/2;

xi = s0(1);     xf = s1(1);
yi = s0(2);     yf = s1(2);
phii = s0(3);   phif = s1(3);
dfi = s0(4);    dff = s1(4);
dri = s0(5);

cost = 0;
if phii ~= phif
    fprintf('ERROR: sliding mode -> phii must be equal to phif');
    return
end

%refering system rotation
R = [cos(phii) -sin(phii);
       sin(phii) cos(phii)];
   
 si = R'*[xi,yi]'; 
 si = [si(1) si(2) 0 s0(4) s0(5)];
 xir = si(1); yir = si(2);
 
 sf = R'*[xf,yf]';
 sf = [sf(1) sf(2) 0 s1(4) s1(5)];
 xfr = sf(1); yfr = sf(2);
   
d = (yir-yfr)/(xir-xfr);

%split into two segments.
if abs(d) > abs(Dmax)
    cost = 1;
    s=1;
    
    % computing best way:
    if (sf(1)<0 && sf(2)>0) || ((sf(1)>0 && sf(2)<0))
        s =-1;
    end
    
    %intemidian point
    xm = (yfr-yir)/(s*Dmax);
    ym = yfr;
    sm = [xm ym 0 dfi dri];
    
    %same step size
    A = sqrt(xm^2 + ym^2);
    B = sqrt((xfr-xm)^2 + (yfr-ym)^2);
    delta = 1 /(A+B); da = delta*A;  db = delta*B;

    P1 = slide(si,sm,da*step-5,Dmax);
    P2 = slide(sm,sf,db*step-4,Dmax);
   
    
    for i = 1:4
        ang = (Dmax - dfi)/4;
        Ps(i,:) = [si(1) si(2) phii dfi+ang*i dfi+ang*i];
    end
    
     for i = 1:4
        ang = (dff - Dmax)/4;
        Pm(i,:) = [sm(1) sm(2) phii Dmax+ang*i Dmax+ang*i];
     end
    
      Ptot = [si; Ps;  P1(2:end,:) ; Pm; P2(2:end,:);sf];
    
else
    
        for i = 1:4
            ang = (d - dfi)/4;
            Ps(i,:) = [si(1) si(2) phii dfi+ang*i dfi+ang*i];
        end

         for i = 1:4
            ang = (dff - d)/4;
            Pm(i,:) = [sf(1) sf(2) phii d+ang*i d+ang*i];
         end

    P1 = slide(si,sf,step-9,Dmax);
    Ptot = [si;Ps; P1(2:end,:) ; Pm];
    
end

if cost == 0
    motion = 2;
else
    motion = 3;
end
Ptot(:,1:2) = (R*Ptot(:,1:2)')';
Ptot(:,3) = phii;
P = Ptot;
%P = [s0;Ptot;s1] ;

%plot
if drawFlag 
    s = 0:(1/(step+2)):1;
    figure(1)
    %subplot(2,2,1);
    plot(P(:,1),P(:,2),'b');grid on; axis equal
    title('(x,y) [m]')
   % pause

%     subplot(2,2,2);
%     plot(s,P(:,3),'b');grid on
%     title('theta [rad]')

    if movieFlag
        figure(2)
        plot(P(:,1),P(:,2),'b');grid on; axis equal
        hold on;
        N = size(P,1);
        for i=1:N
            h = draw_4wsRear(lf,lr,P(i,:)');
            drawnow
            %pause

            if i>1 && i<N
                for k=1:length(h)
                    set(h{k},'Visible','off');
                end
            end
        end
    end
end

end






function [P] = slide(s0,s1,step,Dmax)

xi = s0(1);     xf = s1(1);
yi = s0(2);     yf = s1(2);
phii = s0(3);   phif = s1(3);
dfi = s0(4);    dff = s1(4);
dri = s0(5);    drf = s0(5);

if phii ~= phif
    fprintf('ERROR: sliding mode -> phii must be equal to phif');
    return
end

d =  (yi-yf)/(xi-xf);

if abs(d) > abs(Dmax + 0.01)
    fprintf('mi dovrò spostare')
    return
end

df = d;
dr = d;

s = 0:(1/step):1;

x = s*xf - (s-1)*xi;
y = s*yf - (s-1)*yi;

phi = phii*ones(1,length(s));
df = df*ones(1,length(s));
dr = dr*ones(1,length(s));


P = [x' y' phi' df' dr'];



end


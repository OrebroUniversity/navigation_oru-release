%% test movimenti
clc
l = 0.5; v =  0.2;

s0 = [0 0 0 ];
df = 0; dr = 0;
N = 50;
s = zeros(50,5);
s(1,:) = [s0, df, dr];

figure(2)
plot(s0(1),s0(2), 'rx');
grid on; axis([-1 10 -1 10]);hold on;
h = draw_4ws(l,l,s(1,:)');
drawnow
fprintf( 'use keyboard:\nTrasL: a s TurnL: d f \nStreight g \nTurnRt: h j TrasLeft: k l \nexit: e');
for i = 1:N
   str1 = string(i);str2 = string(N); str = str1+'/'+ str2+' - ' ;
   k = input(str,'s');
  [dr,df,e] = key(k,dr,df);
   if e == 1 
       break; 
   end
   s(i+1,:) = move4ws(s(i,:),df,dr,v,v,l,l);
   plot([s(i,1);s(i+1,1)],[s(i,2);s(i+1,2)],'b');
   
   disp(s(i+1,1:3));
   
   for n=1:length(h) 
       set(h{n},'Visible','off');
   end
   
   h = draw_4ws(l,l,s(i+1,:)');
   drawnow
end

function [dr,df,e] = key(k,dri,dfi)
    dr = dri; df = dfi;e = 0;
    switch k
        case 'a' 
            df = pi/6; dr = df;
        case 's' 
            df = pi/12; dr = df;
        case 'd' 
            df = pi/6; dr = -df;
        case 'f' 
            df = pi/12; dr = -df;
        case 'g' 
            df = 0; dr = 0;
        case 'h' 
            df = -pi/12; dr = -df;
        case 'j' 
            df = -pi/6; dr = -df;
        case 'k' 
            df = -pi/12; dr = df;
        case 'l' 
            df = -pi/6; dr = df;
        case 'e'
            e = 1;
            disp('exit')
            
    end
end

            
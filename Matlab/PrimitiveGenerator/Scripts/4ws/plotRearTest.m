%%
x1 = out.x1;
y1 = out.y1;

x2 = out.x2;
y2 = out.y2;
plot(x1,y1,'b');
hold on
plot(x2,y2,'r');
%%
L = x1(1) - x2(1);
for i = 1:size(x1)
    LT(i) =  x1(i) - x2(i);
    r = (L - LT(i));
    if r ~=0
        fprintf("NO");
    end
    
end
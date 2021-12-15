function [direction] =findDir(pre,next)

s_x = pre(1);  s_y = pre(2);  s_o = pre(3);
g_x = next(1); g_y = next(2); g_o = next(3);

if (s_x == g_x && s_y == g_y && s_o == g_o)
    alfa = s_o;
    beta = 0;
    direction = 0;
else

    x = g_x - s_x;
    y = g_y - s_y;
    alfa = atan2(y,x);
    beta = alfa - s_o;
    beta = atan2(sin(beta),cos(beta));
    if abs(beta) < 1.58
        direction = 1;
    else
        direction = -1;
    end
end

s = "alfa: " + alfa + " - " + s_o + " - " +beta + " == " + direction;
disp(s);


end


function [dyn] = dynamic(s,u,L)
    dyn{1} = s(6)*cos(s(3) + s(5));
    dyn{2} = s(6)*sin(s(3) + s(5));
    dyn{3} = s(6)*sin(s(4)-s(5))/(L*cos(s(4)));
    dyn{4} = u(2);
    dyn{5} = u(3);
    dyn{6} = u(1);

    dyn{3} = atan2(sin(dyn{3}), cos(dyn{3}));
end
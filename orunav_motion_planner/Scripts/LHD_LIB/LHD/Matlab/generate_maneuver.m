function m = generate_maneuver(Nv,ax)
%
% Nv - number of via regions
%

disp('Choose initial and final states')
disp('-------------------------------')
h = figure; axis(ax);
[m.x, m.y] = getpts(h);

plot(m.x,m.y,'ko')
axis(ax);
grid on

disp('Choose via regions')
disp('------------------')
m.A = [eye(2);-eye(2)];
for i=1:Nv
    r = getrect(h);

    lb = [r(1);r(2)];
    ub = [r(1)+r(3);r(2)+r(4)];
    m.b{i} = [ub;-lb];
    
    plot_constraints(m.A,m.b{i},'b')
end

%%%EOF
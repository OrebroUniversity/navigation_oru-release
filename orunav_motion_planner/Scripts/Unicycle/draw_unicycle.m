function h = draw_unicycle(x)
%
% Draw a unicycle
%
% Input:
% ------
% x(1)  - x coordinate of middle of rear axis
% x(2)  - y coordinate of middle of rear axis
% x(3)  - angle of car [rad.] (theta)
%

scale = 2;
sx = 0.1*scale;
sy = 0.05*scale;

p = [sx,  sx, -sx, -sx  sx
     sy, -sy, -sy,  sy  0];

p = [cos(x(3)) -sin(x(3));
     sin(x(3))  cos(x(3))]*p + repmat([x(1);x(2)],1,5);

hold on;
h{1} = plot(x(1),x(2),'ko','MarkerSize',2,'MarkerFaceColor','k'); 
h{2} = plot([p(1,5) p(1,3)],[p(2,5) p(2,3)],'k','LineWidth',0.5);
h{3} = plot([p(1,3) p(1,4)],[p(2,3) p(2,4)],'k','LineWidth',0.5);
h{4} = plot([p(1,5) p(1,4)],[p(2,5) p(2,4)],'k','LineWidth',0.5);

%%%EOF
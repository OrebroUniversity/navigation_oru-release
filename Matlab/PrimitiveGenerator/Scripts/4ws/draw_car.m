function h = draw_car(L,x)
%
% Draw a car-like vehicle
%
% Input:
% ------
% L     - distance from the rear axis to the front axis
% x(1)  - x coordinate of middle of rear axis
% x(2)  - y coordinate of middle of rear axis
% x(3)  - angle of car [rad.] (theta)
% x(4)  - steering angle [rad.] (phi)
%

% used only for plotting purposes
wL = 0.06; % (wheel length)/2
wS = 0.13; % (wheel separation)/2

% -------------------------------------------------------------------------
% shasi
% -------------------------------------------------------------------------
sx = L/5;
sy = 0.2;

p = [ 6*sx, 6*sx, -sx, -sx, 6*sx
       sy,   -sy, -sy,  sy,   sy];

R1 = [cos(x(3)) -sin(x(3));
      sin(x(3))  cos(x(3))];   

p = R1*p + repmat(x(1:2),1,5);

hold on;
h{1} = fill(p(1,:),p(2,:),[0.93,0.93,0.93]);

% -------------------------------------------------------------------------
% rear wheels
% -------------------------------------------------------------------------

p1 = [ wL, -wL;
       wS,  wS];
  
p2 = [ wL, -wL;
      -wS, -wS];  

p3 = [  0,   0;
       wS, -wS];
   
p1 = R1*p1 + repmat(x(1:2),1,2);
p2 = R1*p2 + repmat(x(1:2),1,2);
p3 = R1*p3 + repmat(x(1:2),1,2); 
 
h{2} = plot([p1(1,1), p1(1,2)],[p1(2,1), p1(2,2)],'k','LineWidth',4);
h{3} = plot([p2(1,1), p2(1,2)],[p2(2,1), p2(2,2)],'k','LineWidth',4);
h{4} = plot([p3(1,1), p3(1,2)],[p3(2,1), p3(2,2)],'k','LineWidth',1);
h{5} = plot(x(1),x(2),'ro');

% -------------------------------------------------------------------------
% front wheels
% -------------------------------------------------------------------------

p0 = [ wL, -wL;
       0,   0];
   
R2 = [cos(x(3)+x(4)) -sin(x(3)+x(4));
      sin(x(3)+x(4))  cos(x(3)+x(4))];   
   
p0 = R2*p0 + repmat(x(1:2)+[L*cos(x(3));L*sin(x(3))],1,2);

%h{6} = plot([p0(1,1), p0(1,2)],[p0(2,1), p0(2,2)],'k','LineWidth',4);

p1(:,1) = p0(:,1) + R1*[0;wS];
p1(:,2) = p0(:,2) + R1*[0;wS];

p2(:,1) = p0(:,1) + R1*[0;-wS];
p2(:,2) = p0(:,2) + R1*[0;-wS];

p3(:,1) = (p1(:,1)+p1(:,2))/2;
p3(:,2) = (p2(:,1)+p2(:,2))/2;

h{7} = plot([p1(1,1), p1(1,2)],[p1(2,1), p1(2,2)],'k','LineWidth',4);
h{8} = plot([p2(1,1), p2(1,2)],[p2(2,1), p2(2,2)],'k','LineWidth',4);
h{9} = plot([p3(1,1), p3(1,2)],[p3(2,1), p3(2,2)],'k','LineWidth',1);

%%%EOF
function [h,X] = draw_LHD_2(x,L,scale_factor,wL,wS,bxb,bxf,fxb,fxf,cy)
%
% Draw an LHD vehicle
%
% Input:
% ------
% x(1) - x coordinate of middle of front axis
% x(2) - y coordinate of middle of front axis
% x(3) - angle of front part [rad.] (theta)
% x(4) - joint angle [rad.] (phi)
% L(1) - distance from the front axis to the joint
% L(2) - distance from the rear axis to the joint
%
% Optional input (plotting purposes):
% ---------------
% scale_factor - plot multiplier (depends on figure resolution)
% wL   - wheel length
% wS   - wheel separation
% bxb  - back chassis, from back axel to its back along the x axis
% bxf  - back chassis, from back axel to its front along the x axis
% fxb  - front chassis, from front axel to its back along the x axis
% fxf  - front chassis, from front axel to its front along the x axis
% cy   - chassis, max width
%
% Output:
% --------
% h    - figure handle
% X    - [2x3] matrix
%        X(:,1) position of front part
%        X(:,2) position of rear part
%        X(:,3) position of joint
%
% Notes:
% -------
% temporary variables
%
% x1 - position of the middle of the front axis
% x2 - position of the middle of the rear axis
% T1 - angle (in the world frame) of the front part
% T2 - angle (in the world frame) of the rear part
% J  - position (in the world frame) of the joint

if nargin < 6
    bxb = 3.385;
    bxf = 1.9; % slightly reduced for visualization purposes
    fxb = 1.7; % slightly reduced for visualization purposes
    fxf = 3.585;
    cy = 2.640/2;
    
    if nargin < 4
        wL = 0.7; % (wheel length)/2
        wS = 2.30 / 2; % (wheel separation)/2
        
        if nargin < 3
            scale_factor = 1;
        end
        
    end
end

% adjust all plot variables to scale
bxb = bxb * scale_factor;
bxf = bxf * scale_factor;
fxb = fxb * scale_factor;
fxf = fxf * scale_factor;
cy = cy * scale_factor;
wL = wL * scale_factor;
wS = wS * scale_factor;
L = L * scale_factor;


x1 = x(1:2) * scale_factor;
T1= x(3);
q = x(4);
R1 = [cos(T1) -sin(T1);sin(T1) cos(T1)];

J = x1 - R1*[L(1);0];
T2 = T1 + q;
R2 = [cos(T2) -sin(T2);sin(T2) cos(T2)];

x2 = J - R2*[L(2);0];

X = [x1,x2,J];

% -------------------------------------------------------------------------
% chassis
% -------------------------------------------------------------------------

h{1} = chassis(x1,T1,[0.7,0.7,0.7],1,fxb,fxf,cy);
h{2} = chassis(x2,T2,[1,1,1],1,bxb,bxf,cy);

h{3} = plot([x1(1) J(1)],[x1(2) J(2)],'k');
h{4} = plot([x2(1) J(1)],[x2(2) J(2)],'k');

h{5} = plot(x1(1),x1(2),'ko','MarkerSize',3,'MarkerFaceColor','k');
h{6} = plot(x2(1),x2(2),'ko','MarkerSize',3,'MarkerFaceColor','k');
h{7} = plot(J(1),J(2),'ko','MarkerSize',3,'MarkerFaceColor','k');

% -------------------------------------------------------------------------
% wheels
% -------------------------------------------------------------------------

p1 = [ wL, -wL;
    wS,  wS];

p2 = [ wL, -wL;
    -wS, -wS];

p3 = [  0,   0;
    wS, -wS];

% front wheels
% -------------

x1_r = repmat(x1,1,2);

p11 = R1*p1 + x1_r;
p12 = R1*p2 + x1_r;
p13 = R1*p3 + x1_r;

h{8}  = plot([p11(1,1), p11(1,2)],[p11(2,1), p11(2,2)],'k','LineWidth',5);
h{9}  = plot([p12(1,1), p12(1,2)],[p12(2,1), p12(2,2)],'k','LineWidth',5);
h{10} = plot([p13(1,1), p13(1,2)],[p13(2,1), p13(2,2)],'k','LineWidth',1);

% rear wheels
% -------------

x2_r = repmat(x2,1,2);

p21 = R2*p1 + x2_r;
p22 = R2*p2 + x2_r;
p23 = R2*p3 + x2_r;

h{11} = plot([p21(1,1), p21(1,2)],[p21(2,1), p21(2,2)],'k','LineWidth',5);
h{12} = plot([p22(1,1), p22(1,2)],[p22(2,1), p22(2,2)],'k','LineWidth',5);
h{13} = plot([p23(1,1), p23(1,2)],[p23(2,1), p23(2,2)],'k','LineWidth',1);

% -------------------------------------------------------------------------

function h = chassis(x,T,color,FA,xb,xf,cy)

p = [ xf,  xf, -xb, -xb, xf
    cy, -cy, -cy,  cy, cy];

R1 = [cos(T) -sin(T);
    sin(T)  cos(T)];

p = R1*p + repmat(x(1:2),1,5);

hold on;
h = fill(p(1,:),p(2,:),color,'FaceAlpha',FA);

%%%EOF
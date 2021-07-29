function draw_arrow(p1, p2, hW, hL, lC, hC, head_case, ls)
%
% Draws an arrow
%
% Input:
% ------
% p1 [2x1]  - point 1
% p2 [2x1]  - point 2
% hW [1x1]  - head width
% hL [1x1]  - head length
% lC [1x3]  - line color (RGB)
% hC [1x3]  - head color (RGB)
% head_case - if head_case = 1 plot arrow sides only
%             if head_case ~= 1 fill the arrow with hC
% ls        - line style
%
% Example:
% ---------
% draw_arrow([0;0], [1;1], 0.05, 0.06, [0,1,1], [0.01,1,1], 0, '-')
%

% edge color
eC = [0 0 0];

p1 = p1(:);
p2 = p2(:);

white = [1 1 1];

if nargin < 8
    ls = '-';
    if nargin < 7
        head_case = 0;
        if nargin < 5
            lC = white*0.95;
            hC = white*0.95;
            if nargin < 3
                hW = 0.1;
                hL = 0.2;
            end
        end
    end
end

p = p2-p1;
a = atan2(p(2),p(1));
R = [cos(a) -sin(a);
    sin(a)  cos(a)];

c1(1) = -hL;
c1(2) = -hW/2;

c2(1) = -hL;
c2(2) =  hW/2;

c1 = R*c1(:);
c2 = R*c2(:);

c = [p2(1)+c1(1), p2(2)+c1(2);
     p2(1)+c2(1), p2(2)+c2(2);
     p2'];

hold on;
h1 = plot([p1(1),p2(1)],[p1(2),p2(2)],'k', 'LineStyle', ls);

set(h1,'Color',lC);
if head_case
    h2 = plot([c(3,1) c(2,1)],[c(3,2) c(2,2)],'k',[c(3,1) c(1,1)],[c(3,2) c(1,2)],'k');
    set(h2,'Color',eC);
else
    h2 = patch( c(:,1),c(:,2),hC);
    set(h2,'EdgeColor',eC);
    set(h2,'FaceColor',hC);
end

%%%EOF
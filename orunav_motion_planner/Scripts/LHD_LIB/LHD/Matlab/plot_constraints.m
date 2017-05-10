function plot_constraints(A,b,color)
%
% plot constraints A*x <= b (in 2D)
%

if nargin < 3
    color = 'b';
end

plot_ConvHull(Constraints2Vert(A,b),color);

function p = Constraints2Vert(A,b)
%
% Compute the vertices of a polygon from the 
% constraints (A*x <= b)
%
% Inputs:
% --------
% A, b    - form the linear constraint inequalities in 2D
%
% Outputs:
% ---------
% p       - all vertex points
% dim     - dimensions of each constraint
%

p = [0;0];

dim = size(A,1);

for i = 1:1:dim-1
  p(:,i+1) = A(i:i+1,:)\b(i:i+1);
end

p(:,1) = A([1,size(A,1)],:)\b([1,size(A,1)]);
p = p';


function plot_ConvHull(c,color)
%
% Function to display a convex hull defined by n 2D points.
%
% Inputs:
% --------
% c     - points [n x 2]
% color - color. For example 'b' 
%

% Row size of c (the points defining the convex hull) 
n = size(c,1); 

% Plots the convex hull
hold on;
for i=1:n-1
  plot([c(i,1) c(i+1,1)], [c(i,2) c(i+1,2)], color);
end
plot([c(1,1) c(n,1)], [c(1,2) c(n,2)], color);

%%%EOF
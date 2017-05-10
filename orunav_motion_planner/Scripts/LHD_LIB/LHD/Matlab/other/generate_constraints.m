%
%
%

clear;clc

color = {'b', 'r', 'g', 'k', 'c', 'm', 'b--', 'r--', 'g--', 'k--', 'c--', 'm--'}; 

A = [eye(2);-eye(2)];
b = [0.5;0.5;0.5;0.5];

N = 2;

F{1} = [8;6];

%F{1} = [1;1];
%F{2} = [4;1];
%F{3} = [7;1];
%F{4} = [11;1];
%F{5} = [16;1];

for i=1:N-1
    bF = b + A*F{i};
    plot_constraints(A,bF,color{i});

    x_bounds(i,:) = [-bF(3), bF(1)];
    y_bounds(i,:) = [-bF(4), bF(2)];
end

L = [0.36;0.4];
s0 = [-0.5;-1;pi/4;0];
s1 = [ 21;-2;0;0];

plot(s0(1),s0(2),'gs')
plot(s1(1),s1(2),'rs')

draw_LHD(s0,L);
draw_LHD(s1,L);

%[x_bounds, y_bounds]

fprintf('ctr.set_dimensions( % f, % f );\n\n',L(1),L(2));

fprintf('ctr.set_initial_state( % f, % f, % f, % f );\n',s0(1),s0(2),s0(3),s0(4));
fprintf('ctr.set_final_state( % f, % f, % f, % f );\n\n',s1(1),s1(2),s1(3),s1(4));

for i=1:N-1
   fprintf('ctr.vr[%d].set_xy( % f, % f, % f, % f );\n',i-1,x_bounds(i,1),x_bounds(i,2),y_bounds(i,1),y_bounds(i,2));
end

axis equal; grid on; box on

%%%EOF
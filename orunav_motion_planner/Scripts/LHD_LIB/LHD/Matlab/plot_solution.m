function plot_solution(motion_flag, function_name)
%
% Display the results
%
% Input:
% ======
% motion_flag   : 0 - display boundary configurations only
%               : k - display vehicle every k data points
% function_name : name of Matlab function to execute
%

if nargin < 2
    function_name = 'info_maneuvers';
    if nargin < 1
        motion_flag = 0;
    end
end

% add more colors if there are more than 12 stages
color = {'b', 'r', 'g', 'k', 'c', 'm', 'b--', 'r--', 'g--', 'k--', 'c--', 'm--'}; 

% ---------------------------------------------------
% load data
% ---------------------------------------------------

[maneuver,NM,NT,L,data_opt,data_sim] = eval(function_name);

% ---------------------------------------------------

stages_opt = data_opt(:,1);
stages_sim = data_sim(:,1);

t_sim = data_sim(:,2);
l_sim = data_sim(:,3);
l_opt = data_opt(:,2);

xy = data_sim(:,[4,5]);
theta_sim = data_sim(:,6);
phi_sim = data_sim(:,7);

valid_primitive = true;
step = 10;
hinge_error_counter = ceil(size(phi_sim,1) * 0.005);
for ind = step*2:step:size(phi_sim,1)
    partial_distance = sqrt((xy(ind,1) - (xy(ind-step,1)))^2 + (xy(ind,2) - (xy(ind-step,2)))^2);
    dt = partial_distance / 1;
    current_rate = (abs(atan2(sin(phi_sim(ind,1) - phi_sim(ind-step,1)), cos(phi_sim(ind,1) - phi_sim(ind-step,1))))) / dt;
    if current_rate > 0.22 * 2.5
        valid_primitive = false;
    else
        if current_rate > 0.22
            hinge_error_counter = hinge_error_counter - 1;
        end
        if hinge_error_counter <= 0
            valid_primitive = false;
        end
    end
    if valid_primitive == false
        fprintf('rate: %2.2f (ind: %d)\n', current_rate, ind);
        fprintf('violations: %d \n', ceil(size(phi_sim,1) * 0.005) - hinge_error_counter);
        break;
    end
end



phi_opt = data_opt(:,3);

% ---------------------------------------------------
% plot x-y motion
% ---------------------------------------------------

figure('Position',[0 0 1200 900]);

hold on

grid on; axis equal; box on

for i=1:NM
   draw_LHD_2(maneuver(i).s0,L);
   draw_LHD_2(maneuver(i).s1,L);
%   draw_LHD(maneuver(i).s0,L);
%   draw_LHD(maneuver(i).s1,L);
end

% print(gcf,'-djpeg','-r300',['./jpg/plot',sprintf('%04d',0),'.jpg'])

for i=1:NT
  I = find( stages_sim == i-1 );
  plot(xy(I,1),xy(I,2),color{i},'LineWidth',2)
end

A = [eye(2);-eye(2)]; % Matrix of constraints
for k=1:NM
    for i=1:length(maneuver(k).b)
        plot_constraints(A,maneuver(k).b{i},'k');
        p = maneuver(k).p{i};
        plot(p(1),p(2),'kd','MarkerSize',10,'MarkerFaceColor','r');
    end
end

if (motion_flag>0)   
  NS = length(t_sim);    
  s = zeros(4,1);
  for i = 1:motion_flag:NS
    s = data_sim(i,[4,5,6,7])';
    h = draw_LHD_2(s,L);
    %h = draw_LHD(s,L);
    drawnow
    
    %print(gcf,'-djpeg','-r300',['./jpg/plot',sprintf('%04d',i),'.jpg'])
    
    for k = 1:length(h)
      set(h{k},'Visible','off');
    end
  end
  
  xlabel('x')
  ylabel('y')
  grid on; box on
end

% ---------------------------------------------------

figure;subplot(2,2,[1,2]);cla;hold on
for i = 1:NT
  I = find( stages_sim == i-1 );
  plot(l_sim(I)+i-1,phi_sim(I),color{i},'LineWidth',2)
end

for i = 1:NT
  I = find( stages_opt == i-1 );
  plot(l_opt(I)+i-1,phi_opt(I),'ko','LineWidth',1)
end

plot([0 NT], [-pi/4 -pi/4],'k')
plot([0 NT], [ pi/4  pi/4],'k')
axis([0 NT -1 1])

title('SIMULATION vs. OPTIMIZATION')
xlabel('arc lenght')
ylabel('phi')
grid on; box on

% ---------------------------------------------------

subplot(2,2,3);cla;hold on
for i = 1:NT
  I = find( stages_sim == i-1 );
  plot(t_sim(I),phi_sim(I),color{i},'LineWidth',2)
end

plot([0 t_sim(end)], [-0.6 -0.6],'k')
plot([0 t_sim(end)], [ 0.6  0.6],'k')
axis([0 t_sim(end) -1 1])

title('SIMULATION')
xlabel('time')
ylabel('phi')
grid on; box on

% ---------------------------------------------------

subplot(2,2,4);cla;hold on

for i = 1:NT
    I = find( stages_sim == i-1 );
    plot(t_sim(I),theta_sim(I),color{i},'LineWidth',2)
end
axis([0,t_sim(end),-4,4])

title('SIMULATION')
xlabel('time')
ylabel('theta')
grid on
box on

% ---------------------------------------------------

%%%EOF
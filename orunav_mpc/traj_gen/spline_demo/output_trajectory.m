function output_trajectory(state_init, state, control, file_name)
%
% Output state and control trajectories to ASCII file
% 
% Input:
% ------
% state     - state trajectory
% control   - control trajectory
% file_name - file name where to store the trajectory
%

%      X           Y           theta       phi         v             w
OUT = [state(1,:)' state(2,:)' state(3,:)' state(4,:)' control(1,:)' control(2,:)'];
OUT = [state_init' 0 0;
       OUT]; 

fileID = fopen(file_name,'w');

fprintf(fileID,'%2.14f %2.14f %2.14f %2.14f %2.14f %2.14f\n',  OUT');

fclose(fileID);

%%%EOF

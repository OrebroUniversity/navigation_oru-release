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


%fileID = fopen(file_name,'w');

%fprintf(fileID,'\n#N\n');
%fprintf(fileID,'%d ', size(state,2));

%fprintf(fileID,'\n\n#x\n');
%fprintf(fileID,'%2.14f ', state(1,:));

%fprintf(fileID,'\n\n#y\n');
%fprintf(fileID,'%2.14f ', state(2,:));

%fprintf(fileID,'\n\n#theta\n');
%fprintf(fileID,'%2.14f ', state(3,:));

%fprintf(fileID,'\n\n#phi\n');
%fprintf(fileID,'%2.14f ', state(4,:));

%fprintf(fileID,'\n\n#v\n');
%fprintf(fileID,'%2.14f ', control(1,:));

%fprintf(fileID,'\n\n#w\n');
%fprintf(fileID,'%2.14f ', control(2,:));
    
%fclose(fileID);

%%%EOF

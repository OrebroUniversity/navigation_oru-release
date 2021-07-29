output_filename =               'fws';

car_width =                     0.5;
lf =               1.5;  %car_length_front
lr =              1.5; % car_length_back
distance_between_axes =         3;

max_steering_radians =          0.7;
numberofangles =                1;

steeringanglepartitions =       1;
steeringanglecardinality =      1;

distance =                      12;
cell_size =                     6;

plotting =                      0;

% -------------------------------------------------------------------------
% ============================= PARAMETERS ================================
% -------------------------------------------------------------------------

% Cost multipliers
% forwardcostmult = 1.0;
% forwardturncostmult = 1.0;
% backwardcostmult = 1.0;
% backwardturncostmult = 1.0;

% approximation error tolerance
approxError = 10e-05;

% max hinge angle rate in rad/s
max_steering_angle_rate = 1;

% assumed longitudinal speed to calculate the hinge angle rate (m/s)
assumed_longitudinal_speed = 0.4;



%
% -------------------------------------------------------------------------
% ======================== PRIMITIVE GENERATION ===========================
% -------------------------------------------------------------------------

base_primitives_filename = sprintf('%s_all_baseprims.mat', output_filename);

orientation_angle_granularity = pi/(numberofangles/2);

% we have already generated all the base primitives
if exist(base_primitives_filename,'file') ~= 0
    load(base_primitives_filename);
else
    i = 1;
    primitive = cell(2,((distance/cell_size)*numberofangles*3)^2);
    % orientation and steering
    for start_orient_ID = 1:1
            
            start_o = (start_orient_ID - 1) * pi/2;
            
            fprintf('\nStart pose: %2.4f %2.4f %2.4f %2.4f\n', 0,0,start_o);
            
            % goal x and y
            for goal_x = 0:cell_size:distance
                for goal_y = -distance:cell_size:distance
                    % we do not consider the origin
                    if ~(goal_x == 0 && goal_y == 0)
                        
                        
                        % goal orientation and steering
                        for goal_orient_ID = 1:numberofangles 

                                goal_o = (goal_orient_ID - 1) * orientation_angle_granularity;
                                goal_o = atan2(sin(goal_o),cos(goal_o));
                                
 
                                
                                fprintf('\nGoal pose: %2.4f %2.4f %2.4f %2.4f\n', goal_x,goal_y,goal_o);
                                s1 =[ goal_x,goal_y,goal_o,0,0];
                                %[prim,control] = fwsSolver(s1,15,lr,lf,0);
                                for k= -1:1; %0 car, 1 steering, -1 crab
                                [prim,control] = carSolver(s1,15,k,lr,lf,0);
                                
                                 primitive{1,i} = control;
                                 primitive{2,i} = prim; 
                                i = i+1;
                                end
                         end
                     end
                end
            end
    end
end    



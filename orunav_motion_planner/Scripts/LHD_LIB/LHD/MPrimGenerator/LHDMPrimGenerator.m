function[] = LHDMPrimGenerator(output_filename, lhd_width, lhd_length_back, ...
    lhd_length_front, lhd_length_back_from_axle, lhd_length_front_from_axle, ...
    bound_phi, numberofangles, distance, cell_size)

% LHDMPrimGenerator : Generates a primitive set for an LHD vehicle
%   The set guarantees 8-axis simmetry
%
%   output_filename :           the file to which the results are saved
%   lhd_width :                 max width of the LHD
%   lhd_length_back :           distance from central joint to back axel (meters)
%   lhd_length_front :          distance from central joint to front axel (meters)
%   lhd_length_back_from_axle : back distance from axle (meters)
%   lhd_length_front_from_axle :front distance from axle (meters)
%   bound_phi :                 the bound on the joint steering angle
%   numberofangles :            angle granularity
%   distance :                  the max distance for the area of the primitives (meters)
%   cell_size :                 the size of the side of each cell (meters)
%
%   Two outputs: all primitives and primitive set


% -------------------------------------------------------------------------
% ============================= PARAMETERS ================================
% -------------------------------------------------------------------------

% Cost multipliers
forwardcostmult = 1.0;
forwardturncostmult = 1.2;
backwardcostmult = 1.1;
backwardturncostmult = 1.3;

% approximation error tolerance
approxError = 10e-05;

% max hinge angle rate in rad/s
max_hinge_angle_rate = 0.22;
% assumed longitudinal speed to calculate the hinge angle rate (m/s)
assumed_longitudinal_speed = 1;

% primitives connecting point A and B should not be longer than
% allowed_primitive_length_multiplier * linear_distance(A,B)
allowed_primitive_length_multiplier = 3;

% parameter to avoid primitives which curve too much on one side
max_side_angle = pi / 2;


% -------------------------------------------------------------------------
% ======================== PRIMITIVE GENERATION ===========================
% -------------------------------------------------------------------------

base_primitives_filename = sprintf('%s_all_baseprims.mat', output_filename);

% we have already generated all the base primitives
if exist(base_primitives_filename,'file') ~= 0
    load(base_primitives_filename);
else
    % generate the base primitives for this model, that is, the ones in the
    % First sector  [0  pi/4]
    start_x = 0;
    start_y = 0;
    start_phi = 0;
    goal_phi = 0;
    
    % open multiple pools for parallel computation
    matlabpool open 8
    
    all_base_primitives = cell(numberofangles/8+1,1);
    for i = 1:size(all_base_primitives,1)
        all_base_primitives{i} = cell(1,0);
    end
    
    for start_angle_counter = 1:numberofangles/8+1
        start_o = (start_angle_counter - 1) * (pi/(numberofangles/2));
        start_o = atan2(sin(start_o),cos(start_o));
        
        fprintf('\nStart pose: %2.4f %2.4f %2.4f %2.4f\n', start_x,start_y,start_o,start_phi);
        
        for goal_x = -distance:cell_size:distance
            for goal_y = -distance:cell_size:distance
                % we do not consider the origin
                if ~(goal_x == 0 && goal_y == 0)
                    temp_primitives = cell(numberofangles,1);
                    parfor goal_angle_counter = 1:numberofangles
                        goal_o = (goal_angle_counter - 1) * (pi/(numberofangles/2));
                        goal_o = atan2(sin(goal_o),cos(goal_o));
                        
                        fprintf('Goal pose: %2.4f %2.4f %2.4f %2.4f\n', goal_x,goal_y,goal_o,goal_phi);
                        
                        if ~exist('data', 'dir')
                            mkdir('data');
                        end
                        filename = (sprintf('./data/output_%d.txt', goal_angle_counter));
                        if exist(filename, 'file')
                            delete(filename); % remove old files (just in case)
                        end
                        
                        % check if the motion is forward or backward
                        if dot([1 tan(start_o)], [goal_x goal_y]) >= 0
                            % forward motion
                            motion = 1;
                        else
                            % backward motion
                            motion = -1;
                        end
                        arguments = [num2str(lhd_length_front)  , ' ', ...
                            num2str(lhd_length_back)            , ' ', ...
                            num2str(bound_phi)                  , ' ', ...
                            num2str(start_x)                    , ' ', ...
                            num2str(start_y)                    , ' ', ...
                            num2str(start_o)                    , ' ', ...
                            num2str(start_phi)                  , ' ', ...
                            num2str(goal_x)                     , ' ', ...
                            num2str(goal_y)                     , ' ', ...
                            num2str(goal_o)                     , ' ', ...
                            num2str(goal_phi)                   , ' ', ...
                            num2str(1)                          , ' ', ...
                            num2str(motion)                     , ' ', ...
                            filename                            , ' ', ...
                            '>> /dev/null'];
                        
                        fprintf('\n Execute command: %s %s \n\n', './SingleStageTimeout.sh', arguments);
                        system(['./', 'SingleStageTimeout.sh', ' ', arguments]);
                        
                        traj_x = [];
                        traj_y = [];
                        traj_o = [];
                        traj_phi = [];
                        if exist(filename, 'file')
                            posture    = load(filename);
                            traj_x = posture(:,1);
                            traj_y = posture(:,2);
                            traj_o = posture(:,3);
                            traj_phi = posture(:,4);
                        end
                        
                        if size(traj_x,1) > 1
                            traj_x = [start_x; traj_x; goal_x];
                            traj_y = [start_y; traj_y; goal_y];
                            traj_o = [start_o; traj_o; goal_o];
                            traj_phi = [start_phi; traj_phi; goal_phi];
                            traj = [traj_x traj_y traj_o traj_phi];
                            
                            % remove -0
                            traj(logical(abs(traj) < approxError)) = 0;
                            % we do not want -3.14 but only 3.14
                            traj(logical(traj(:,3) - approxError < -pi), 3) = pi;
                            traj(logical(traj(:,4) - approxError < -pi), 4) = pi;
                            
                            temp_primitives{goal_angle_counter} = traj;
                        end
                        if exist(filename, 'file')
                            delete(filename); % clean up
                        end
                        
                    end %end parfor
                    
                    % get the primitives I saved
                    for i = 1:numberofangles
                        if size(temp_primitives{i},1) > 1
                            all_base_primitives{start_angle_counter}{size(all_base_primitives{start_angle_counter},2)+1} = temp_primitives{i};
                        end
                    end
                end
            end
        end
    end
    % save the results for future use
    save(base_primitives_filename, 'all_base_primitives');
    
    matlabpool close
    
end % end check if all base prims exist



% -------------------------------------------------------------------------
% ================ FILTERING & ADDING COST MULTIPLIERS ====================
% -------------------------------------------------------------------------

% filtering: eliminate unwanted primitives
primitives = cell(numberofangles,1);
temp_primitives = cell(numberofangles,1);
primitives_cost_multipliers = cell(numberofangles,1);
for i = 1:size(primitives,1)
    primitives{i} = cell(1,0);
    temp_primitives{i} = cell(1,0);
    primitives_cost_multipliers{i} = cell(1,0);
end

for angleind = 1:numberofangles/8+1
    
    %----------------------------------------------------------------------
    % first filtering: primitives outside the scope or too long
    %----------------------------------------------------------------------
    for primind = 1:size(all_base_primitives{angleind},2)
        
        fprintf(1,'First filtering: primitive #%d (%d) -- angle %d (%d) \n', ...
            primind, size(all_base_primitives{angleind},2), angleind, numberofangles/8+1);
        
        original = all_base_primitives{angleind}{primind};
        traj_x = original(:,1);
        traj_y = original(:,2);
        traj_o = original(:,3);
        
        start_x = traj_x(1,1);
        goal_x = traj_x(size(traj_x,1),1);
        start_y = traj_y(1,1);
        goal_y = traj_y(size(traj_y,1),1);
        start_o = traj_o(1,1);
        goal_o = traj_o(size(traj_o,1),1);
        
        valid_primitive = true;
        
        % eliminate primitives outside the radius of the model
        if sqrt(goal_x^2 + goal_y^2) + approxError > distance
            valid_primitive = false;
        end
        
        % prune primitives which are too long
        if valid_primitive
            % calculate primitive length
            primitive_length = 0;
            for index = 2:size(traj_x,1)
                primitive_length = primitive_length + sqrt((traj_x(index,1) - (traj_x(index-1,1)))^2 + (traj_y(index,1) - (traj_y(index-1,1)))^2);
            end
            if primitive_length > sqrt((goal_x - start_x)^2 + (goal_y - start_y)^2) * allowed_primitive_length_multiplier
                valid_primitive = false;
            end
        end
        
        % remove primitives that curve too much on the sides
        if valid_primitive && ...
                (pi - abs(abs(start_o-atan2(goal_y,goal_x))-pi)) + approxError >= max_side_angle && ...
                (pi - abs(abs(start_o+pi-atan2(goal_y,goal_x))-pi)) + approxError >= max_side_angle
            valid_primitive = false;
        end
        
        if valid_primitive
            temp_primitives{angleind}{size(temp_primitives{angleind},2)+1} = original;
        end
    end
    
    fprintf(1,'\nPrimitives after first filtering: %d (%d) \n\n', size(temp_primitives{angleind},2), size(all_base_primitives{angleind},2));
    
    %----------------------------------------------------------------------
    % second filtering: remove when hinge angle rate is too high
    %----------------------------------------------------------------------
    
    for primind = 1:size(temp_primitives{angleind},2)
        
        fprintf(1,'Checking primitive #%d (%d) -- angle %d (%d) \n', primind, size(temp_primitives{angleind},2), angleind, numberofangles/8+1);
        
        original = temp_primitives{angleind}{primind};
        traj_x = original(:,1);
        traj_y = original(:,2);
        traj_o = original(:,3);
        traj_phi = original(:,4);
        
        start_x = traj_x(1,1);
        goal_x = traj_x(size(traj_x,1),1);
        start_y = traj_y(1,1);
        goal_y = traj_y(size(traj_y,1),1);
        start_o = traj_o(1,1);
        goal_o = traj_o(size(traj_o,1),1);
        start_phi = traj_phi(1,1);
        goal_phi = traj_phi(size(traj_phi,1),1);
        
        % reduce number of points to 100
        traj_x_reduced = [];
        traj_y_reduced = [];
        traj_o_reduced = [];
        traj_phi_reduced = [];
        for i = 1:size(traj_x,1)
            if mod(i, 10) == 0
                traj_x_reduced = [traj_x_reduced traj_x(i,1)]; %#ok<AGROW>
                traj_y_reduced = [traj_y_reduced traj_y(i,1)]; %#ok<AGROW>
                traj_o_reduced = [traj_o_reduced traj_o(i,1)]; %#ok<AGROW>
                traj_phi_reduced = [traj_phi_reduced traj_phi(i,1)]; %#ok<AGROW>
            end
        end
        traj_reduced = [[start_x traj_x_reduced(1,2:size(traj_x_reduced,2)-1) goal_x]', ...
            [start_y traj_y_reduced(1,2:size(traj_y_reduced,2)-1) goal_y]', ...
            [start_o traj_o_reduced(1,2:size(traj_o_reduced,2)-1) goal_o]', ...
            [start_phi traj_phi_reduced(1,2:size(traj_phi_reduced,2)-1) goal_phi]'];
        
        
        
        valid_primitive = true;
        % HERE WE ELIMINATE WITH THE HINGE ANGLE RATE
        for index = 2:size(traj_x_reduced,2)
            partial_distance = sqrt((traj_x_reduced(1,index) - (traj_x_reduced(1,index-1)))^2 + ...
                (traj_y_reduced(1,index) - (traj_y_reduced(1,index-1)))^2);
            dt = partial_distance / assumed_longitudinal_speed;
            current_rate = (abs(atan2(sin(traj_phi_reduced(1,index) - traj_phi_reduced(1,index-1)), cos(traj_phi_reduced(1,index) - traj_phi_reduced(1,index-1))))) / dt;
            if current_rate > max_hinge_angle_rate
                valid_primitive = false;
            end
              if valid_primitive == false
                  break
              end
        end
        
        if valid_primitive
            
            primitives{angleind}{size(primitives{angleind},2)+1} = traj_reduced;
            
            % ---------------------------------------------------------
            % add cost multipliers
            % ---------------------------------------------------------
            
            goal_x = traj_reduced(size(traj_reduced, 1), 1);
            goal_y = traj_reduced(size(traj_reduced, 1), 2);
            goal_o = traj_reduced(size(traj_reduced, 1), 3);
            start_o = traj_reduced(1, 3);
            
            % check if the motion is forward or backward and if we turn
            if dot([1 tan(start_o)], [goal_x goal_y]) >= 0
                % forward motion
                if (pi - abs(abs(start_o - atan2(goal_y,goal_x))-pi)) < pi/32 && (pi - abs(abs(start_o - goal_o)-pi)) < pi/32
                    % straight
                    primitives_cost_multipliers{angleind}{size(primitives_cost_multipliers{angleind},2)+1} = forwardcostmult;
                else
                    primitives_cost_multipliers{angleind}{size(primitives_cost_multipliers{angleind},2)+1} = forwardturncostmult;
                end
            else
                % backward motion
                if (pi - abs(abs((start_o - pi) - atan2(goal_y,goal_x))-pi)) < pi/16 && (pi - abs(abs(start_o - goal_o)-pi)) < pi/16
                    % straight
                    primitives_cost_multipliers{angleind}{size(primitives_cost_multipliers{angleind},2)+1} = backwardcostmult;
                else
                    primitives_cost_multipliers{angleind}{size(primitives_cost_multipliers{angleind},2)+1} = backwardturncostmult;
                end
            end
            
        else % not a valid primitive
            fprintf(1,'........ primitive #%d discarded \n', primind);
        end
    end
end


%--------------------------------------------------------------------------
% Second sector  (pi/4 - pi/2]
%--------------------------------------------------------------------------
% iterate over angles
for angleind = ((numberofangles/8)+2):numberofangles/4+1
    % iterate over primitives
    for primind = 1:size(primitives{numberofangles/4 + 2 - angleind},2)
        
        original = primitives{numberofangles/4 + 2 - angleind}{primind};
        % invert x,y
        primitives{angleind}{primind}(:,1) = original(:,2);
        primitives{angleind}{primind}(:,2) = original(:,1);
        % invert the angles
        primitives{angleind}{primind}(:,3) = atan2(cos(original(:,3)), sin(original(:,3)));
        primitives{angleind}{primind}(:,4) = -original(:,4);
        
        % remove -0
        primitives{angleind}{primind}(logical(abs(primitives{angleind}{primind}) < approxError)) = 0;
        % we do not want -3.14 but only 3.14
        primitives{angleind}{primind}((logical(primitives{angleind}{primind}(:,3) - approxError < -pi)), 3) = pi;
        primitives{angleind}{primind}((logical(primitives{angleind}{primind}(:,4) - approxError < -pi)), 4) = pi;
        
        % add the cost multipliers
        primitives_cost_multipliers{angleind}{primind} = primitives_cost_multipliers{numberofangles/4 + 2 - angleind}{primind};
    end
end

%--------------------------------------------------------------------------
% 90 degrees rotation  (pi/2 - pi]
%--------------------------------------------------------------------------
% iterate over angles
for angleind = ((numberofangles/4)+2):numberofangles/2+1
    % iterate over primitives
    for primind = 1:size(primitives{numberofangles/2 + 2 - angleind},2)
        
        original = primitives{numberofangles/2 + 2 - angleind}{primind};
        
        primitives{angleind}{primind}(:,1) = -original(:,1);
        primitives{angleind}{primind}(:,2) = original(:,2);
        % invert the angles
        primitives{angleind}{primind}(:,3) = atan2(sin(original(:,3)), -cos(original(:,3)));
        primitives{angleind}{primind}(:,4) = -original(:,4);
        
        % remove -0
        primitives{angleind}{primind}(logical(abs(primitives{angleind}{primind}) < approxError)) = 0;
        % we do not want -3.14 but only 3.14
        primitives{angleind}{primind}(logical(primitives{angleind}{primind}(:,3) - approxError < -pi), 3) = pi;
        primitives{angleind}{primind}(logical(primitives{angleind}{primind}(:,4) - approxError < -pi), 4) = pi;
        
        % add the cost multipliers
        primitives_cost_multipliers{angleind}{primind} = primitives_cost_multipliers{numberofangles/2 + 2 - angleind}{primind};
    end
end

%--------------------------------------------------------------------------
% final rotation (pi - 2*pi)
%--------------------------------------------------------------------------
% iterate over angles
for angleind = numberofangles/2+2:numberofangles
    % iterate over primitives
    for primind = 1:size(primitives{numberofangles + 2 - angleind},2)
        
        original = primitives{numberofangles + 2 - angleind}{primind};
        
        primitives{angleind}{primind}(:,1) = original(:,1);
        primitives{angleind}{primind}(:,2) = -original(:,2);
        % invert the angles
        primitives{angleind}{primind}(:,3) = atan2(-sin(original(:,3)), cos(original(:,3)));
        primitives{angleind}{primind}(:,4) = -original(:,4);
        
        % remove -0
        primitives{angleind}{primind}(logical(abs(primitives{angleind}{primind}) < approxError)) = 0;
        % we do not want -3.14 but only 3.14
        primitives{angleind}{primind}(logical(primitives{angleind}{primind}(:,3) - approxError < -pi), 3) = pi;
        primitives{angleind}{primind}(logical(primitives{angleind}{primind}(:,4) - approxError < -pi), 4) = pi;
        
        % add the cost multipliers
        primitives_cost_multipliers{angleind}{primind} = primitives_cost_multipliers{numberofangles + 2 - angleind}{primind};
    end
end


% -------------------------------------------------------------------------
% ======================== OUTPUT AND PLOTTING ============================
% -------------------------------------------------------------------------

fout = fopen(output_filename, 'w');

% write the header
fprintf(fout, 'lhd_width: %f\n', lhd_width);
fprintf(fout, 'lhd_length_front: %f\n', lhd_length_front);
fprintf(fout, 'lhd_length_back: %f\n', lhd_length_back);
fprintf(fout, 'lhd_length_front_from_axle: %f\n', lhd_length_front_from_axle);
fprintf(fout, 'lhd_length_back_from_axle: %f\n', lhd_length_back_from_axle);
fprintf(fout, 'bound_phi: %f\n', bound_phi);
fprintf(fout, 'resolution_m: %f\n', cell_size);
fprintf(fout, 'numberofangles: %d\n', numberofangles);

for angleind = 1:numberofangles
    % do we have primitives?
    for primind = 1:size(primitives{angleind},2)
        
        fprintf(fout, 'primID: %d\n', primind-1);
        fprintf(fout, 'startangle_c: %d\n', angleind-1);
        
        intermcells_m = primitives{angleind}{primind};
        
        % add the cost multipliers and motion directions
        fprintf(fout, 'additionalactioncostmult: %1.1f\n', primitives_cost_multipliers{angleind}{primind});
        if (primitives_cost_multipliers{angleind}{primind} == forwardcostmult || ...
                primitives_cost_multipliers{angleind}{primind} == forwardturncostmult)
            fprintf(fout, 'motiondirection: %d\n', 1);
        else
            fprintf(fout, 'motiondirection: %d\n', -1);
        end
        
        % write out
        fprintf(fout, 'intermediateposes: %d\n', size(intermcells_m,1));
        
        % print to file
        for interind = 1:size(intermcells_m, 1)
            fprintf(fout, '%.4f %.4f %.4f %.4f\n', intermcells_m(interind,1), intermcells_m(interind,2), intermcells_m(interind,3), intermcells_m(interind,4));
        end;
        
        
        %------------------------------------------------------------------
        % plotting
        figure(1); cla
        % PATH with arrows
        hold on;
        grid on;
        text(0, 0, int2str(angleind));
        hold on;
        plot(primitives{angleind}{primind}(:,1), primitives{angleind}{primind}(:,2), '-');
        axis([-12 12 -12 12]);
        hold on;
        d = [cos(primitives{angleind}{primind}(1,3));sin(primitives{angleind}{primind}(1,3))]; d=d/norm(d)*1;
        draw_arrow([primitives{angleind}{primind}(1,1) primitives{angleind}{primind}(1,2)]', [primitives{angleind}{primind}(1,1) primitives{angleind}{primind}(1,2)]'+d, 0.2, 0.2, [0;0;0], [0;0;0]);
        hold on;
        d = [cos(primitives{angleind}{primind}(size(primitives{angleind}{primind}, 1),3));sin((primitives{angleind}{primind}(size(primitives{angleind}{primind}, 1),3)))]; d=d/norm(d)*0.5;
        draw_arrow([primitives{angleind}{primind}(size(primitives{angleind}{primind}, 1),1) primitives{angleind}{primind}(size(primitives{angleind}{primind}, 1),2)], [primitives{angleind}{primind}(size(primitives{angleind}{primind}, 1),1) primitives{angleind}{primind}(size(primitives{angleind}{primind}, 1),2)]'+d, 0.2, 0.2, [0;0;0], [0;0;0]);
        for i=1:5:size(primitives{angleind}{primind}, 1)
            hold on;
            d = [cos(primitives{angleind}{primind}(i,4)+primitives{angleind}{primind}(i,3));sin(primitives{angleind}{primind}(i,4)+primitives{angleind}{primind}(i,3))]; d=d/norm(d)*1;
            draw_arrow([primitives{angleind}{primind}(i,1) primitives{angleind}{primind}(i,2)]', [primitives{angleind}{primind}(i,1) primitives{angleind}{primind}(i,2)]'+d, 0.05, 0.05, [0;0;0], [0;0;0]);
        end
        
        figure(2)
        hold on;
        axis([-12 12 -12 12]);
        plot(primitives{angleind}{primind}(:,1), primitives{angleind}{primind}(:,2),'b')
        hold on;
        grid on;
        %pause;
    end
    %pause;
    %clf;
    size(primitives{angleind},2)
end




%%% EOF
end


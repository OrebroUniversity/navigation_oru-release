% LHDMPrimGeneratorMPI : Generates a primitive set for an LHD vehicle
% The set guarantees 8-axis simmetry
%
%   output_filename :           the file to which the results are saved
%   lhd_width :                 max width of the LHD
%   lhd_length_back :           distance from central joint to back axel (meters)
%   lhd_length_front :          distance from central joint to front axel (meters)
%   lhd_length_back_from_axle : back distance from axle (meters)
%   lhd_length_front_from_axle :front distance from axle (meters)
%   bound_phi :                 the bound on the joint steering angle
%   numberofangles :            the number of orientation angles for this vehicle -- MUST BE MULTIPLE OF 8
%   steeringanglepartitions :   the number of steering angle partitions for this vehicle, calculated as (2*pi)/steeringAngleGranularity
%   steeringanglecardinality :  the number of steering angles supported by the vehicle -- MUST BE ODD AND <= steeringanglepartitions
%   distance :                  the max distance for the area of the primitives (meters) -- MUST BE A MULTIPLE OF cell_size
%   cell_size :                 the size of the side of each cell (meters)
%
%   all measures are in radians and meters
%   Two outputs: all primitives and primitive set
%
% Note:  no steering -> steeringanglepartitions == steeringanglecardinality == 1


% -------------------------------------------------------------------------
% ========================== SCRIPT PARAMETERS ============================
% -------------------------------------------------------------------------

% output_filename =               'AtlasCopco_ST14_16_3.16_12_0.2';
% lhd_width =                     2.640;
% lhd_length_back =               2.02;
% lhd_length_front =              1.835;
% lhd_length_back_from_axle =     3.385;
% lhd_length_front_from_axle =    3.585;
% bound_phi =                     0.767;
% numberofangles =                16;
% steeringanglepartitions =       16;
% steeringanglecardinality =      3;
% distance =                      12;
% cell_size =                     0.2;

output_filename =               'Volvo_L120F_8_3.32_6_0.5';
lhd_width =                     2.6;
lhd_length_back =               1.6;
lhd_length_front =              1.6;
lhd_length_back_from_axle =     1.0;
lhd_length_front_from_axle =    1.0;
bound_phi =                     0.6;
numberofangles =                8;
steeringanglepartitions =       32;
steeringanglecardinality =      3;
distance =                      6;
cell_size =                     0.5;



% -------------------------------------------------------------------------
% ============================= PARAMETERS ================================
% -------------------------------------------------------------------------

[status, result] = system('hostname');
hostname_str = mat2str(result);

if size(strfind(hostname_str, 'Celsius'),1) > 0
    datadir = './data/';
    executable_dir = './';
else
    datadir = '/home/mco/data/';
    executable_dir = '/home/mco/';
    P = path;
    path(P, '/share/apps/LHD_lib/MPrimGenerator');
end


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
allowed_primitive_length_multiplier = 1.5;

% parameter to avoid primitives which curve too much on one side
max_side_angle = pi/4;

% parameter to avoid primitives which change too much the orientation of
% the vehicle
max_orient_change = (pi/2);

% plot the final results: 1: plot; 0: do not plot
plot_results = 1;


% -------------------------------------------------------------------------
% ======================== PRIMITIVE GENERATION ===========================
% -------------------------------------------------------------------------

base_primitives_filename = sprintf('%s_all_baseprims.mat', output_filename);

% orientation and steering granularity + the initial steering ID
steering_angle_granularity = 2*pi / steeringanglepartitions;
orientation_angle_granularity = pi/(numberofangles/2);
initial_steer_ID = mod((steeringanglepartitions - ((steeringanglecardinality - 1) / 2)),steeringanglepartitions) + 1;

% Initialize MPI
MPI_Init;
% Create communicator object
comm = MPI_COMM_WORLD;
warning off 'MATLAB:fileparts:VersionToBeRemoved'

comm_size = MPI_Comm_size(comm);
my_rank = MPI_Comm_rank(comm);


% we have already generated all the base primitives
if exist(base_primitives_filename,'file') ~= 0
    % only the host needs to load the file. The SLAVES should die
    if (my_rank == 0)
        load(base_primitives_filename);
    else
        exit
    end
else
    % we go MASSIVELY PARALLEL
    
    % HOST: coordinate the ranks:
    % 1. prepare the jobs
    % 2. poll the ranks
    % 3. assign remaining jobs
    % 4. collect return data
    if my_rank == 0
        
        % Check if there had been previous iterations
        workspace_filename = sprintf('%s_workspace.mat', output_filename);
        
        % we already have a workspace: we need to load it and keep working
        if exist(workspace_filename,'file') ~= 0
            fprintf('Opening previously saved workspace\n');
            load(workspace_filename);
        else
            
            % generate the base primitives for this model, that is, the ones in the
            % First sector  [0  pi/4]
            start_x = 0;
            start_y = 0;
            
            % motion primitive orientationID, motion primitive steeringID
            all_base_primitives = cell(numberofangles/8+1, steeringanglecardinality, 1);
            for i = 1:size(all_base_primitives,1)
                for j = 1:size(all_base_primitives,2)
                    all_base_primitives{i,j} = cell(1,0);
                end
            end
            
            jobs = cell(1,0);
            % it contains start angle counter of the job and completion flag
            jobs_additional_info = [];
            
            % counter of pre-discarded primitives
            pre_discarded_counter = 0;
            
            % orientation and steering
            for start_orient_ID = 1:numberofangles/8+1
                for start_steer_ID = 1:steeringanglecardinality
                    
                    real_start_steer_ID = initial_steer_ID + start_steer_ID - 1;
                    if real_start_steer_ID > steeringanglepartitions
                        real_start_steer_ID = real_start_steer_ID - steeringanglepartitions;
                    end
                    
                    start_o = (start_orient_ID - 1) * orientation_angle_granularity;
                    start_o = atan2(sin(start_o),cos(start_o));
                    
                    start_phi = (real_start_steer_ID - 1) * steering_angle_granularity;
                    start_phi = atan2(sin(start_phi),cos(start_phi));
                    
                    fprintf('\nStart pose: %2.4f %2.4f %2.4f %2.4f\n', start_x,start_y,start_o,start_phi);
                    
                    % prepare all the requests for a starting angle
                    for goal_x = -distance:cell_size:distance
                        for goal_y = -distance:cell_size:distance
                            % we do not consider the origin
                            if ~(goal_x == 0 && goal_y == 0)
                                
                                % goal orientation and steering
                                for goal_orient_ID = 1:numberofangles
                                    for goal_steer_ID = 1:steeringanglecardinality
                                        
                                        real_goal_steer_ID = initial_steer_ID + goal_steer_ID - 1;
                                        if real_goal_steer_ID > steeringanglepartitions
                                            real_goal_steer_ID = real_goal_steer_ID - steeringanglepartitions;
                                        end
                                        
                                        goal_o = (goal_orient_ID - 1) * orientation_angle_granularity;
                                        goal_o = atan2(sin(goal_o),cos(goal_o));
                                        
                                        goal_phi = (real_goal_steer_ID - 1) * steering_angle_granularity;
                                        goal_phi = atan2(sin(goal_phi),cos(goal_phi));
                                        
                                        % check if the motion is forward or backward
                                        if dot([1 tan(start_o)], [goal_x goal_y]) >= 0
                                            % forward motion
                                            motion = 1;
                                        else
                                            % backward motion
                                            motion = -1;
                                        end
                                        
                                        valid_job = true;
                                        
                                        % remove jobs that curve too much on the sides
                                        if (pi - abs(abs(start_o-atan2(goal_y,goal_x))-pi)) >= max_side_angle + approxError  && ...
                                                (pi - abs(abs(start_o+pi-atan2(goal_y,goal_x))-pi)) >= max_side_angle + approxError
                                            valid_job = false;
                                        end
                                        
                                        % eliminate jobs that change too much the pose orientation
                                        if abs(atan2(sin(start_o-goal_o), cos(start_o-goal_o))) > max_orient_change + approxError
                                            valid_job = false;
                                        end
                                        
                                        if valid_job
                                            
                                            args = [lhd_length_front, lhd_length_back ,bound_phi, ...
                                                start_x, start_y, start_o, start_phi, ...
                                                goal_x,  goal_y,  goal_o,  goal_phi, ...
                                                1, motion];
                                            
                                            jobs{size(jobs,2)+1} = args;
                                            % jobs_additional_info:
                                            % start_orient_ID, start_steer_ID, completion_flag
                                            jobs_additional_info = [jobs_additional_info ; [start_orient_ID, start_steer_ID, 0]];
                                        else
                                            pre_discarded_counter = pre_discarded_counter +1;
                                        end
                                    end
                                end
                            end
                        end
                    end
                end
            end
            
            fprintf('Pre-discarded jobs: %d\n', pre_discarded_counter);
            % save all the jobs
            fprintf('Saving pre-computed jobs\n');
            save(workspace_filename, 'jobs', 'jobs_additional_info', 'all_base_primitives', '-v7.3');
        end
        
        
        
        % output directory for all ranks
        fprintf('Checking existence of directory %s\n', datadir);
        if ~exist(datadir, 'dir')
            fprintf('Creating %s\n', datadir);
            mkdir(datadir);
        else
            fprintf('Directory %s found\n', datadir);
        end
        
        % now we have all the jobs
        % check the ranks which are alive (with a timeout)
        for k = 1:comm_size-1
            fprintf('Polling rank %d \n', k);
            MPI_Send(k, k, comm, 1);
        end
        timeoutcounter = 240;
        % ranks running: [[rank,tag]; [rank,tag]]
        ranks_running  = [];
        pause(2);
        while timeoutcounter > 0 && size(ranks_running,1) ~= comm_size-1
            message_rank = []; %#ok<NASGU>
            message_tag = []; %#ok<NASGU>
            [message_rank, message_tag] = MPI_Probe('*', '*', comm);
            % Get all the results back
            for k = 1:size(message_rank,2)
                dummy_response = MPI_Recv(message_rank(k), message_tag(k), comm);
                ranks_running = [ranks_running; [message_rank(k), message_tag(k)]];
                fprintf('Rank %d up and running \n', message_rank(k));
            end
            pause(1);
            timeoutcounter = timeoutcounter - 1;
        end
        pause(3);
        
        % sort on rows
        ranks_running = sortrows(ranks_running,1);
        fprintf('%d ranks up and running \n', size(ranks_running,1));
        
        % while there are jobs which are not successfully executed and ranks running
        while size(find(jobs_additional_info(:,3) == 0),1) > 0 && size(ranks_running,1) >= 1
            
            % get all the jobs to assign
            jobs_to_assign = find(jobs_additional_info(:,3) == 0);
                        
            job_to_assign_per_rank = size(jobs_to_assign, 1);
            % if there is more than one rank running, divide the load
            if size(ranks_running,1) > 1
                job_to_assign_per_rank = floor(size(jobs_to_assign, 1) / (size(ranks_running,1) - 1));
            end 
            % put a limit on the number of jobs per rank
            if job_to_assign_per_rank > 1000
                job_to_assign_per_rank = 1000;
            end
            
            fprintf('Jobs to assign: %d\n', size(jobs_to_assign, 1));
            
            % assign jobs to the available pool
            job_assignment_counter = 1;
            for k = 1:(size(ranks_running,1)-1)
                % tag
                ranks_running(k,2) = job_assignment_counter;
                jobs_for_this_rank = cell(1,0);
                for index = 1:job_to_assign_per_rank
                    jobs_for_this_rank{size(jobs_for_this_rank,2)+1} = jobs{jobs_to_assign(job_assignment_counter)};
                    job_assignment_counter = job_assignment_counter + 1;
                end
                
                % do we have jobs for this rank?
                if size(jobs_for_this_rank,2) > 0
                    MPI_Send(ranks_running(k,1), ranks_running(k,2), comm, jobs_for_this_rank);
                    fprintf('Sent %d (%d) jobs [tag: %d] to rank %d \n', size(jobs_for_this_rank,2), size(jobs_to_assign, 1), ranks_running(k,2), ranks_running(k,1));
                end
            end
            % complete the assignment: last tag
            ranks_running(size(ranks_running,1),2) = job_assignment_counter;
            jobs_for_this_rank = cell(1,0);
            
            % last rank: limit on the number of jobs here as well
            if (size(jobs_to_assign, 1) - job_assignment_counter) > 50
                last_job_limit = (job_assignment_counter+49);
            else
                last_job_limit = size(jobs_to_assign, 1);
            end
            while job_assignment_counter <= last_job_limit
                jobs_for_this_rank{size(jobs_for_this_rank,2)+1} = jobs{jobs_to_assign(job_assignment_counter)};
                job_assignment_counter = job_assignment_counter + 1;
            end
            
            % do we have jobs for this rank?
            if size(jobs_for_this_rank,2) > 0
                MPI_Send(ranks_running(size(ranks_running,1),1), ranks_running(size(ranks_running,1),2), comm, jobs_for_this_rank);
                fprintf('Sent %d (%d) jobs [tag: %d] to rank %d \n', size(jobs_for_this_rank,2), size(jobs_to_assign, 1), ranks_running(size(ranks_running,1),2),ranks_running(size(ranks_running,1),1));
            end
            
            % now receive back the data with a timeout
            received = 0;
            new_ranks_running = [];
            response_timeout = job_to_assign_per_rank * 15;
            if response_timeout < (50*15)
                response_timeout = (50*15);
            end
            fprintf('Response timeout: %d \n', response_timeout);
            
            % DEBUG
            primitives_before_hinge_check = 0;
            primitives_after_hinge_check = 0;
            
            while received < size(ranks_running,1) && response_timeout > 0
                message_rank = []; %#ok<NASGU>
                message_tag = []; %#ok<NASGU>
                [message_rank, message_tag] = MPI_Probe('*', '*', comm);
                % Get all the results back
                for k = 1:size(message_rank,2)
                    % Receive data. It must be a rank to which we assigned
                    % a job in the last iteration and the tag must be the same
                    rank_index = find(ranks_running(:,1) == message_rank(k));
                    if message_rank(k) ~= my_rank && size(rank_index, 1) == 1 && ranks_running(rank_index, 2) == message_tag(k) 
                        fprintf('...receiving from %d...\n', message_rank(k));
                        rank_result = MPI_Recv(message_rank(k), message_tag(k), comm);
                        fprintf('Received message from rank %d [tag: %d]\n', message_rank(k),  message_tag(k));
                        % update the list of running ranks
                        new_ranks_running = [new_ranks_running; [message_rank(k),0]]; %#ok<*AGROW>
                        received = received + 1;
                        
                        % extract the result
                        for index = 1:size(rank_result,2)
                            offset = message_tag(k) - 1;
                            
                            % mark the job as executed
                            jobs_additional_info(jobs_to_assign(offset + index),3) = 1;
                            
                            % FILTERING and REDUCTION
                            if size(rank_result{index},1) > 0
                                
                                traj_x = rank_result{index}(:,1);
                                traj_y = rank_result{index}(:,2);
                                traj_o = rank_result{index}(:,3);
                                traj_phi = rank_result{index}(:,4);
                                
                                valid_primitive = true;
                                
                                % 1. prune primitives which are too long
                                % calculate primitive length
                                primitive_length = 0;
                                for ind = 2:size(traj_x,1)
                                    primitive_length = primitive_length + sqrt((traj_x(ind,1) - (traj_x(ind-1,1)))^2 + (traj_y(ind,1) - (traj_y(ind-1,1)))^2);
                                end
                                if primitive_length > sqrt((traj_x(size(traj_x,1),1) - traj_x(1,1))^2 + (traj_y(size(traj_y,1),1) - traj_y(1,1))^2) * allowed_primitive_length_multiplier
                                    valid_primitive = false;
                                end
                                
                                % 2. remove when hinge angle rate is too high
                                if valid_primitive
                                    
                                    % DEBUG
                                    primitives_before_hinge_check = primitives_before_hinge_check + 1;
                                    
                                    % HERE WE ELIMINATE WITH THE HINGE ANGLE RATE
                                    % allow for a small percentage of outliers
                                    step = 10;
                                    hinge_error_counter = ceil(size(traj_x,1) * 0.01);
                                    for ind = step*2:step:size(traj_x,1)
                                        partial_distance = sqrt((traj_x(ind,1) - (traj_x(ind-step,1)))^2 + (traj_y(ind,1) - (traj_y(ind-step,1)))^2);
                                        dt = partial_distance / assumed_longitudinal_speed;
                                        current_rate = (abs(atan2(sin(traj_phi(ind,1) - traj_phi(ind-step,1)), cos(traj_phi(ind,1) - traj_phi(ind-step,1))))) / dt;
                                        if current_rate > max_hinge_angle_rate * 2
                                            valid_primitive = false;
                                        else
                                            if current_rate > max_hinge_angle_rate
                                                hinge_error_counter = hinge_error_counter - 1;
                                            end
                                            if hinge_error_counter <= 0
                                                valid_primitive = false;
                                            end
                                        end
                                        if valid_primitive == false
                                            break
                                        end
                                    end
                                end
                                
                                % primitive still valid? -> reduce number
                                % of points to 100 and save it
                                if valid_primitive
                                    
                                    % DEBUG
                                    fprintf('Primitive ready to be reduced\n');
                                    primitives_after_hinge_check = primitives_after_hinge_check + 1;
                                    
                                    traj_x_reduced = [];
                                    traj_y_reduced = [];
                                    traj_o_reduced = [];
                                    traj_phi_reduced = [];
                                    for i = 10:10:size(traj_x,1)
                                        traj_x_reduced = [traj_x_reduced traj_x(i,1)];
                                        traj_y_reduced = [traj_y_reduced traj_y(i,1)];
                                        traj_o_reduced = [traj_o_reduced traj_o(i,1)];
                                        traj_phi_reduced = [traj_phi_reduced traj_phi(i,1)];
                                    end
                                    
                                    local_args = jobs{jobs_to_assign(offset + index)};
                                    traj_x_reduced = [local_args(4) traj_x_reduced(1,2:size(traj_x_reduced,2)-1) local_args(8)]';
                                    traj_y_reduced = [local_args(5) traj_y_reduced(1,2:size(traj_y_reduced,2)-1) local_args(9)]';
                                    traj_o_reduced = [local_args(6) traj_o_reduced(1,2:size(traj_o_reduced,2)-1) local_args(10)]';
                                    traj_phi_reduced = [local_args(7) traj_phi_reduced(1,2:size(traj_phi_reduced,2)-1) local_args(11)]';
                                    traj = [traj_x_reduced traj_y_reduced traj_o_reduced traj_phi_reduced];
                                    
                                    % remove -0
                                    traj(logical(abs(traj) < approxError)) = 0;
                                    % we do not want -3.14 but only 3.14
                                    traj(logical(traj(:,3) - approxError < -pi), 3) = pi;
                                    traj(logical(traj(:,4) - approxError < -pi), 4) = pi;
                                    
                                    % OK: extra check! DEBUG
                                    if abs(traj_x(1,1) - local_args(4)) > 0.1 || abs(traj_x(size(traj_x,1),1) - local_args(8)) > 0.1 || ...
                                            abs(traj_y(1,1) - local_args(5)) > 0.1 || abs(traj_y(size(traj_y,1),1) - local_args(9)) > 0.1 || ...
                                            abs(atan2(sin(traj_o(1,1) - local_args(6)), cos(traj_o(1,1) - local_args(6)))) > 0.1 || ...
                                            abs(atan2(sin(traj_o(size(traj_o,1),1) - local_args(10)), cos(traj_o(size(traj_o,1),1) - local_args(10)))) > 0.1 || ...
                                            abs(atan2(sin(traj_phi(1,1) - local_args(7)), cos(traj_phi(1,1) - local_args(7)))) > 0.1 || ...
                                            abs(atan2(sin(traj_phi(size(traj_phi,1),1) - local_args(11)), cos(traj_phi(size(traj_phi,1),1) - local_args(11)))) > 0.1
                                        
                                        fprintf('MISMATCH\n');
                                        
                                        fprintf('offset: %d \nindex: %d\njob_index:%d\n', offset, index, jobs_to_assign(offset + index));
                                        fprintf('jobs_additional_info (start angle and steer IDs): %d\t%d\n', jobs_additional_info(jobs_to_assign(offset + index), 1),jobs_additional_info(jobs_to_assign(offset + index), 2));
                                        
                                        fprintf('jobs: \n');
                                        jobs{jobs_to_assign(offset + index)}
                                        fprintf('Local args (start):\n');
                                        fprintf('%.4f\t%.4f\t%.4f\t%.4f\n', local_args(4), local_args(5), local_args(6), local_args(7));
                                        
                                        fprintf('-----------------------------\n');
                                        fprintf('%.4f\t%.4f\t%.4f\t%.4f\n', traj_x(1,1), traj_y(1,1), traj_o(1,1), traj_phi(1,1));
                                        fprintf('%.4f\t%.4f\t%.4f\t%.4f\n', traj_x(2,1), traj_y(2,1), traj_o(2,1), traj_phi(2,1));
                                        fprintf('.............................\n');
                                        fprintf('%.4f\t%.4f\t%.4f\t%.4f\n', traj_x(size(traj_x,1)-1,1), traj_y(size(traj_y,1)-1,1), traj_o(size(traj_o,1)-1,1), traj_phi(size(traj_phi,1)-1,1));
                                        fprintf('%.4f\t%.4f\t%.4f\t%.4f\n', traj_x(size(traj_x,1),1), traj_y(size(traj_y,1),1), traj_o(size(traj_o,1),1), traj_phi(size(traj_phi,1),1));
                                        fprintf('-----------------------------\n');
                                        
                                        fprintf('Local args (goal):\n');
                                        fprintf('%.4f\t%.4f\t%.4f\t%.4f\n', local_args(8), local_args(9), local_args(10), local_args(11));
                                    end
                                    
                                    % save the primitive -- recover the info
                                    % for this job from jobs_additional_info
                                    angle_id = jobs_additional_info(jobs_to_assign(offset + index), 1);
                                    steer_id = jobs_additional_info(jobs_to_assign(offset + index), 2);
                                    all_base_primitives{angle_id, steer_id}{size(all_base_primitives{angle_id, steer_id},2)+1} = traj;
                                    
                                    fprintf('Primitive stored\n');
                                end
                            end
                        end
                    else
                        % receive and discard the message that is out of time
                        dummy_result = MPI_Recv(message_rank(k), message_tag(k), comm);
                        fprintf('Discarded message from rank %d [tag: %d]\n', message_rank(k),  message_tag(k));
                    end
                end
                pause(0.1);
                response_timeout = response_timeout - 0.1;
                if mod(response_timeout, 10) < 0.1
                    fprintf('Timeout counter: %d\n', round(response_timeout));
                end
            end
            
            % DEBUG
            fprintf('Primitives after hinge check %d(%d)', primitives_after_hinge_check, primitives_before_hinge_check);
            
            % security save
            fprintf('Saving workspace\n');
            save(workspace_filename, 'jobs', 'jobs_additional_info', 'all_base_primitives', '-v7.3');
            
            % cleaning unwanted files
            fprintf('Cleaning leftover buffer and lock files\n');
            dir_sep = '/';
            if (ispc) dir_sep = '\'; end %#ok<SEPEX>
            comm_dir = './MatMPI';
            % Delete buffer and lock files in this directory.
            delete([comm_dir dir_sep 'p*_p*_t*_buffer.mat']);
            delete([comm_dir dir_sep 'p*_p*_t*_lock.mat']);
            
            % update the running ranks
            ranks_running = new_ranks_running;
            ranks_running = sortrows(ranks_running,1);
            fprintf('UPDATE: %d ranks up and running \n', size(ranks_running,1));
        end
        
        % send termination signal to all ranks
        for k = 1:comm_size-1
            MPI_Send(k, k, comm, 0);
        end
        
        % clean up the data dir
        if exist(datadir, 'dir')
            rmdir(datadir, 's');
        end
        
        % the primitives have been generated or loaded, and the ranks have been
        % terminated.
        % Finalize MatlabMPI.
        MPI_Finalize;
        
        
    else % rank > 0 - SLAVES
        
        while 1
            message_rank = [];
            message_tag = [];
            jobs_assigned = cell(1,0);
            while isempty(message_rank)
                [message_rank, message_tag] = MPI_Probe(0, '*', comm);
                pause(1);
            end
            
            fprintf('Message tag received: %d\n', size(message_rank,2));
            
            % Get all the results back
            for k = size(message_rank,2)
                jobs_assigned =  MPI_Recv(0,message_tag(k),comm);
                if iscell(jobs_assigned)
                    rank_output = cell(1,0);
                    fprintf('[Rank %d] - Received message from %d [tag: %d]\n', my_rank, message_rank(k), message_tag(k));
                    for i = 1:size(jobs_assigned,2)
                        args = jobs_assigned{i};
                        filename = sprintf('%s%d_%d.txt', datadir, my_rank, message_tag(k) + i);
                        if exist(filename, 'file')
                            delete(filename); % remove old files (just in case)
                        end
                        arguments = [num2str(args(1)), ' ', ...
                            num2str(args(2)) , ' ', num2str(args(3)) , ' ', ...
                            num2str(args(4)) , ' ', num2str(args(5)) , ' ', ...
                            num2str(args(6)) , ' ', num2str(args(7)) , ' ', ...
                            num2str(args(8)) , ' ', num2str(args(9)) , ' ', ...
                            num2str(args(10)), ' ', num2str(args(11)), ' ', ...
                            num2str(args(12)), ' ', num2str(args(13)), ' ', ...
                            filename         , ' ', '>> /dev/null'];
                        
                        command = sprintf('%sSingleStageTimeout.sh %s', executable_dir, arguments);
                        fprintf('\nExecute command: %s \n\n', command);
                        system(command);
                        
                        posture = [];
                        if exist(filename, 'file')
                            posture = load(filename);
                            delete(filename); % clean up
                        end
                        rank_output{i} = posture; %#ok<*SAGROW>
                        fprintf('[Rank %d] - Job %d (%d) completed\n', my_rank, i, size(jobs_assigned,2));
                    end
                    fprintf('[Rank %d] - Execution completed [tag: %d]\n', my_rank,message_tag(k));
                    MPI_Send(0, message_tag(k), comm, rank_output);
                    fprintf('[Rank %d] - Message sent [tag: %d]\n', my_rank,message_tag(k));
                else
                    % shut down the rank
                    if jobs_assigned == 0
                        fprintf('[Rank %d] - Shutting down\n', my_rank);
                        exit
                    else
                        % answer the probing
                        fprintf('[Rank %d] - Answering to polling\n', my_rank);
                        MPI_Send(0,message_tag(k),comm, 1);
                        fprintf('[Rank %d] - Message sent [tag: %d]\n', my_rank,message_tag(k));
                    end
                end
            end
        end
    end
    
    save(base_primitives_filename, 'all_base_primitives', '-v7.3');
end

% -------------------------------------------------------------------------
% ============== FINAL FILTERING & ADDING COST MULTIPLIERS ================
% -------------------------------------------------------------------------

% filtering: eliminate primitives outside the radius
primitives = cell(numberofangles,steeringanglecardinality,1);
primitives_cost_multipliers = cell(numberofangles,steeringanglecardinality,1);
for i = 1:size(primitives,1)
    for j = 1:size(primitives,2)
        primitives{i,j} = cell(1,0);
        primitives_cost_multipliers{i,j} = cell(1,0);
    end
end

for angleind = 1:numberofangles/8+1
    for steerind = 1:steeringanglecardinality
        for primind = 1:size(all_base_primitives{angleind,steerind},2)
            
            original = all_base_primitives{angleind,steerind}{primind};
            traj_x = original(:,1);
            traj_y = original(:,2);
            traj_o = original(:,3);
            
            % eliminate primitives outside the radius of the model
            if sqrt(traj_x(size(traj_x,1),1)^2 + traj_y(size(traj_y,1),1)^2) < distance + approxError
                primitives{angleind,steerind}{size(primitives{angleind,steerind},2)+1} = original;
                % ---------------------------------------------------------
                % add cost multipliers
                % ---------------------------------------------------------
                goal_x = original(size(original, 1), 1);
                goal_y = original(size(original, 1), 2);
                goal_o = original(size(original, 1), 3);
                start_o = original(1, 3);
                
                % check if the motion is forward or backward and if we turn
                if dot([1 tan(start_o)], [goal_x goal_y]) >= 0
                    % forward motion
                    if (pi - abs(abs(start_o - atan2(goal_y,goal_x))-pi)) < pi/32 && (pi - abs(abs(start_o - goal_o)-pi)) < pi/32
                        % straight
                        primitives_cost_multipliers{angleind,steerind}{size(primitives_cost_multipliers{angleind,steerind},2)+1} = forwardcostmult;
                    else
                        primitives_cost_multipliers{angleind,steerind}{size(primitives_cost_multipliers{angleind,steerind},2)+1} = forwardturncostmult;
                    end
                else
                    % backward motion
                    if (pi - abs(abs((start_o - pi) - atan2(goal_y,goal_x))-pi)) < pi/16 && (pi - abs(abs(start_o - goal_o)-pi)) < pi/16
                        % straight
                        primitives_cost_multipliers{angleind,steerind}{size(primitives_cost_multipliers{angleind,steerind},2)+1} = backwardcostmult;
                    else
                        primitives_cost_multipliers{angleind,steerind}{size(primitives_cost_multipliers{angleind,steerind},2)+1} = backwardturncostmult;
                    end
                end
                
            end
        end
        fprintf(1,'\nPrimitives: %d (%d) \n\n', size(primitives{angleind,steerind},2), size(all_base_primitives{angleind,steerind},2));
    end
end


% -------------------------------------------------------------------------
% ============================= SYMMETRIES ================================
% -------------------------------------------------------------------------

%--------------------------------------------------------------------------
% Second sector  (pi/4 - pi/2]
%--------------------------------------------------------------------------
% iterate over angles -- floor is for numberofangles = 4
for angleind = floor(((numberofangles/8)+2)):numberofangles/4+1
    for steerind = 1:steeringanglecardinality
        % iterate over primitives
        for primind = 1:size(primitives{numberofangles/4 + 2 - angleind, steeringanglecardinality - steerind + 1},2)
            
            original = primitives{numberofangles/4 + 2 - angleind, steeringanglecardinality - steerind + 1}{primind};
            % invert x,y
            primitives{angleind,steerind}{primind}(:,1) = original(:,2);
            primitives{angleind,steerind}{primind}(:,2) = original(:,1);
            % invert the angles
            primitives{angleind,steerind}{primind}(:,3) = atan2(cos(original(:,3)), sin(original(:,3)));
            primitives{angleind,steerind}{primind}(:,4) = -original(:,4);
            
            % remove -0
            primitives{angleind,steerind}{primind}(logical(abs(primitives{angleind,steerind}{primind}) < approxError)) = 0;
            % we do not want -3.14 but only 3.14
            primitives{angleind,steerind}{primind}((logical(primitives{angleind,steerind}{primind}(:,3) - approxError < -pi)), 3) = pi;
            primitives{angleind,steerind}{primind}((logical(primitives{angleind,steerind}{primind}(:,4) - approxError < -pi)), 4) = pi;
            
            % add the cost multipliers
            primitives_cost_multipliers{angleind,steerind}{primind} = primitives_cost_multipliers{numberofangles/4 + 2 - angleind, steeringanglecardinality - steerind + 1}{primind};
        end
    end
end

%--------------------------------------------------------------------------
% 90 degrees rotation  (pi/2 - pi]
%--------------------------------------------------------------------------
% iterate over angles
for angleind = ((numberofangles/4)+2):numberofangles/2+1
    for steerind = 1:steeringanglecardinality
        % iterate over primitives
        for primind = 1:size(primitives{numberofangles/2 + 2 - angleind, steeringanglecardinality - steerind + 1},2)
            
            original = primitives{numberofangles/2 + 2 - angleind, steeringanglecardinality - steerind + 1}{primind};
            
            primitives{angleind,steerind}{primind}(:,1) = -original(:,1);
            primitives{angleind,steerind}{primind}(:,2) = original(:,2);
            % invert the angles
            primitives{angleind,steerind}{primind}(:,3) = atan2(sin(original(:,3)), -cos(original(:,3)));
            primitives{angleind,steerind}{primind}(:,4) = -original(:,4);
            
            % remove -0
            primitives{angleind,steerind}{primind}(logical(abs(primitives{angleind,steerind}{primind}) < approxError)) = 0;
            % we do not want -3.14 but only 3.14
            primitives{angleind,steerind}{primind}(logical(primitives{angleind,steerind}{primind}(:,3) - approxError < -pi), 3) = pi;
            primitives{angleind,steerind}{primind}(logical(primitives{angleind,steerind}{primind}(:,4) - approxError < -pi), 4) = pi;
            
            % add the cost multipliers
            primitives_cost_multipliers{angleind,steerind}{primind} = primitives_cost_multipliers{numberofangles/2 + 2 - angleind, steeringanglecardinality - steerind + 1}{primind};
        end
    end
end

%--------------------------------------------------------------------------
% final rotation (pi - 2*pi)
%--------------------------------------------------------------------------
% iterate over angles
for angleind = numberofangles/2+2:numberofangles
    for steerind = 1:steeringanglecardinality
        % iterate over primitives
        for primind = 1:size(primitives{numberofangles + 2 - angleind, steeringanglecardinality - steerind + 1},2)
            
            original = primitives{numberofangles + 2 - angleind, steeringanglecardinality - steerind + 1}{primind};
            
            primitives{angleind,steerind}{primind}(:,1) = original(:,1);
            primitives{angleind,steerind}{primind}(:,2) = -original(:,2);
            % invert the angles
            primitives{angleind,steerind}{primind}(:,3) = atan2(-sin(original(:,3)), cos(original(:,3)));
            primitives{angleind,steerind}{primind}(:,4) = -original(:,4);
            
            % remove -0
            primitives{angleind,steerind}{primind}(logical(abs(primitives{angleind,steerind}{primind}) < approxError)) = 0;
            % we do not want -3.14 but only 3.14
            primitives{angleind,steerind}{primind}(logical(primitives{angleind,steerind}{primind}(:,3) - approxError < -pi), 3) = pi;
            primitives{angleind,steerind}{primind}(logical(primitives{angleind,steerind}{primind}(:,4) - approxError < -pi), 4) = pi;
            
            % add the cost multipliers
            primitives_cost_multipliers{angleind,steerind}{primind} = primitives_cost_multipliers{numberofangles + 2 - angleind, steeringanglecardinality - steerind + 1}{primind};
        end
    end
end


% -------------------------------------------------------------------------
% ======================== OUTPUT AND PLOTTING ============================
% -------------------------------------------------------------------------

save('primitives', 'primitives', '-v7.3');

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
fprintf(fout, 'steeringanglepartitions: %d\n', steeringanglepartitions);
fprintf(fout, 'steeringanglecardinality: %d\n', steeringanglecardinality);


for angleind = 1:numberofangles
    for steerind = 1:steeringanglecardinality
        % do we have primitives?
        for primind = 1:size(primitives{angleind,steerind},2)
            
            real_start_steer_ID = initial_steer_ID + steerind - 1;
            if real_start_steer_ID > steeringanglepartitions
                real_start_steer_ID = real_start_steer_ID - steeringanglepartitions;
            end
            
            fprintf(fout, 'primID: %d\n', primind-1);
            fprintf(fout, 'startangle_c: %d\n', angleind-1);
            fprintf(fout, 'startsteer_c: %d\n', real_start_steer_ID-1);
            
            intermcells_m = primitives{angleind,steerind}{primind};
            
            % add the cost multipliers and motion directions
            fprintf(fout, 'additionalactioncostmult: %1.1f\n', primitives_cost_multipliers{angleind,steerind}{primind});
            if (primitives_cost_multipliers{angleind,steerind}{primind} == forwardcostmult || ...
                    primitives_cost_multipliers{angleind,steerind}{primind} == forwardturncostmult)
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
            %------------------------------------------------------------------
            if size(strfind(hostname_str, 'Celsius'),1) > 0 && plot_results == 1
                figure(1); cla
                % PATH with arrows
                hold on;
                grid on;
                text(0, 0, int2str(angleind));
                hold on;
                plot(primitives{angleind,steerind}{primind}(:,1), primitives{angleind,steerind}{primind}(:,2), '-');
                axis([-12 12 -12 12]);
                hold on;
                d = [cos(primitives{angleind,steerind}{primind}(1,3));sin(primitives{angleind,steerind}{primind}(1,3))]; d=d/norm(d)*1;
                draw_arrow([primitives{angleind,steerind}{primind}(1,1) primitives{angleind,steerind}{primind}(1,2)]', [primitives{angleind,steerind}{primind}(1,1) primitives{angleind,steerind}{primind}(1,2)]'+d, 0.2, 0.2, [0;0;0], [0;0;0]);
                hold on;
                d = [cos(primitives{angleind,steerind}{primind}(size(primitives{angleind,steerind}{primind}, 1),3));sin((primitives{angleind,steerind}{primind}(size(primitives{angleind,steerind}{primind}, 1),3)))]; d=d/norm(d)*0.5;
                draw_arrow([primitives{angleind,steerind}{primind}(size(primitives{angleind,steerind}{primind}, 1),1) primitives{angleind,steerind}{primind}(size(primitives{angleind,steerind}{primind}, 1),2)], [primitives{angleind,steerind}{primind}(size(primitives{angleind,steerind}{primind}, 1),1) primitives{angleind,steerind}{primind}(size(primitives{angleind,steerind}{primind}, 1),2)]'+d, 0.2, 0.2, [0;0;0], [0;0;0]);
                for i=1:5:size(primitives{angleind,steerind}{primind}, 1)
                    hold on;
                    d = [cos(primitives{angleind,steerind}{primind}(i,4)+primitives{angleind,steerind}{primind}(i,3));sin(primitives{angleind,steerind}{primind}(i,4)+primitives{angleind,steerind}{primind}(i,3))]; d=d/norm(d)*1;
                    draw_arrow([primitives{angleind,steerind}{primind}(i,1) primitives{angleind,steerind}{primind}(i,2)]', [primitives{angleind,steerind}{primind}(i,1) primitives{angleind,steerind}{primind}(i,2)]'+d, 0.05, 0.05, [0;0;0], [0;0;0]);
                end
                
                figure(2)
                hold on;
                axis([-12 12 -12 12]);
                plot(primitives{angleind,steerind}{primind}(:,1), primitives{angleind,steerind}{primind}(:,2),'b')
                hold on;
                grid on;
                pause;
            end
        end
        size(primitives{angleind,steerind},2)
        %pause;
        %clf;
    end
    
end

%%% EOF


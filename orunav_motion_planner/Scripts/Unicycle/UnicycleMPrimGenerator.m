% UnicycleMPrimGenerator : Generates a primitive set for a
% Unicycle-like. The set guarantees 8-axis simmetry
%
%   output_filename :           the file to which the results are saved
%   uni_width :                 max width of the vehicle
%   uni_length :                max length of the vehicle   
%   numberofangles :            angle granularity (how many angles in 2pi)
%   cell_size :                 the size of the side of each cell (meters)
%
%   all measures are in radians and meters
%   Two outputs: all primitives and primitive set


% -------------------------------------------------------------------------
% ========================== SCRIPT PARAMETERS ============================
% -------------------------------------------------------------------------

output_filename =               'MyUnicycle';
uni_width =                     0.8;
uni_length =                    1.2;
numberofangles =                16;
cell_size =                     0.2;


% -------------------------------------------------------------------------
% ============================= PARAMETERS ================================
% -------------------------------------------------------------------------

% Cost multipliers
forwardcostmult = 1.0;
forwardandturncostmult = 1.2;
backwardcostmult = 1.1;
backwardandturncostmult = 1.3;

% number of primitives per angle
numberofprimsperangle = 5;

% approximation error tolerance
approxError = 10e-05;

resolution = 1.

% -------------------------------------------------------------------------
% ======================== PRIMITIVE GENERATION ===========================
% -------------------------------------------------------------------------

if (numberofangles ~= 16 && numberofangles ~= 32)
    fprintf(1, 'ERROR: undefined mprim type\n');
end


% note, what is shown x,y,theta changes (not absolute numbers)
% x aligned with the heading of the robot, angles are positive
% counterclockwise

% Prepare the primitives for the first sector (0--pi/4)

% 0 degreees
basemprimendpts0_c = zeros(numberofprimsperangle, 4); % x,y,theta,costmult
% 0 theta change
basemprimendpts0_c(1,:) = [1 0 0 forwardcostmult];
basemprimendpts0_c(2,:) = [8 0 0 forwardcostmult];
basemprimendpts0_c(3,:) = [-1 0 0 backwardcostmult];
% 1/16 - 1/32 theta change
basemprimendpts0_c(4,:) = [8 1 1 forwardandturncostmult];
basemprimendpts0_c(5,:) = [8 -1 -1 forwardandturncostmult];

if (numberofangles > 16)
    % 11.25 degrees
    basemprimendpts11p25_c = zeros(numberofprimsperangle, 4); %x,y,theta,costmult
    % 0 theta change
    basemprimendpts11p25_c(1,:) = [3 1 0 forwardcostmult];
    basemprimendpts11p25_c(2,:) = [6 2 0 forwardcostmult];
    basemprimendpts11p25_c(3,:) = [-3 -1 0 backwardcostmult];
    % 1/32 theta change
    basemprimendpts11p25_c(4,:) = [5 3 1 forwardandturncostmult];
    basemprimendpts11p25_c(5,:) = [7 1 -1 forwardandturncostmult];
end

% 22.5 degrees
basemprimendpts22p5_c = zeros(numberofprimsperangle, 4); %x,y,theta,costmult
% 0 theta change
basemprimendpts22p5_c(1,:) = [2 1 0 forwardcostmult];
basemprimendpts22p5_c(2,:) = [6 3 0 forwardcostmult];
basemprimendpts22p5_c(3,:) = [-2 -1 0 backwardcostmult];
% 1/16 - 1/32 theta change
basemprimendpts22p5_c(4,:) = [5 4 1 forwardandturncostmult];
basemprimendpts22p5_c(5,:) = [7 2 -1 forwardandturncostmult];

if (numberofangles > 16)
    % 33.75 degrees
    basemprimendpts33p75_c = zeros(numberofprimsperangle, 4); %x,y,theta,costmult
    % 0 theta change
    basemprimendpts33p75_c(1,:) = [3 2 0 forwardcostmult];
    basemprimendpts33p75_c(2,:) = [6 4 0 forwardcostmult];
    basemprimendpts33p75_c(3,:) = [-3 -2 0 backwardcostmult];
    % 1/32 theta change
    basemprimendpts33p75_c(4,:) = [5 5 1 forwardandturncostmult];
    basemprimendpts33p75_c(5,:) = [7 3 -1 forwardandturncostmult];
end

% 45 degrees
basemprimendpts45_c = zeros(numberofprimsperangle, 4); % x,y,theta,costmult
% 0 theta change
basemprimendpts45_c(1,:) = [1 1 0 forwardcostmult];
basemprimendpts45_c(2,:) = [6 6 0 forwardcostmult];
basemprimendpts45_c(3,:) = [-1 -1 0 backwardcostmult];
% 1/8 - 1/16 - 1/32 theta change
basemprimendpts45_c(4,:) = [5 7 1 forwardandturncostmult];
basemprimendpts45_c(5,:) = [7 5 -1 forwardandturncostmult];


% open the output file
fout = fopen(output_filename, 'w');

%write the header
fprintf(fout, 'unicycle_width: %f\n', uni_width);
fprintf(fout, 'unicycle_length: %f\n', uni_length);
fprintf(fout, 'resolution_m: %f\n', resolution);
fprintf(fout, 'numberofangles: %d\n', numberofangles);
fprintf(fout, 'totalnumberofprimitives: %d\n', numberofprimsperangle*numberofangles);

% iterate over angles
for angleind = 1:numberofangles
    
    %current angle
    currentangle = (angleind-1)*2*pi/numberofangles;
    currentangle_36000int = round((angleind-1)*36000/numberofangles);

    % graphic output
    figure(1);
    hold off;
    text(0, 0, int2str(angleind));


    % iterate over primitives
    for primind = 1:numberofprimsperangle
        fprintf(fout, 'primID: %d\n', primind-1);
        fprintf(fout, 'startangle_c: %d\n', angleind-1);
        
        % compute which template to use - a bit crude, but it works
        if (rem(currentangle_36000int, 9000) == 0)
            basemprimendpts_c = basemprimendpts0_c(primind,:);
            angle = currentangle;
        elseif (rem(currentangle_36000int, 4500) == 0)
            basemprimendpts_c = basemprimendpts45_c(primind,:);
            angle = currentangle - 45*pi/180;
        elseif (rem(currentangle_36000int-7875, 9000) == 0)
            basemprimendpts_c = basemprimendpts11p25_c(primind,:);
            basemprimendpts_c(1) = basemprimendpts11p25_c(primind, 2); %reverse x and y
            basemprimendpts_c(2) = basemprimendpts11p25_c(primind, 1);
            basemprimendpts_c(3) = -basemprimendpts11p25_c(primind, 3); %reverse the angle as well
            angle = currentangle - 78.75*pi/180;
            fprintf(1, '78p75\n');
        elseif (rem(currentangle_36000int-6750, 9000) == 0)
            basemprimendpts_c = basemprimendpts22p5_c(primind,:);
            basemprimendpts_c(1) = basemprimendpts22p5_c(primind, 2); %reverse x and y
            basemprimendpts_c(2) = basemprimendpts22p5_c(primind, 1);
            basemprimendpts_c(3) = -basemprimendpts22p5_c(primind, 3); %reverse the angle as well
            angle = currentangle - 67.5*pi/180;
            fprintf(1, '67p5\n');
        elseif (rem(currentangle_36000int-5625, 9000) == 0)
            basemprimendpts_c = basemprimendpts33p75_c(primind,:);
            basemprimendpts_c(1) = basemprimendpts33p75_c(primind, 2); %reverse x and y
            basemprimendpts_c(2) = basemprimendpts33p75_c(primind, 1);
            basemprimendpts_c(3) = -basemprimendpts33p75_c(primind, 3); %reverse the angle as well
            angle = currentangle - 56.25*pi/180;
            fprintf(1, '56p25\n');
        elseif (rem(currentangle_36000int-3375, 9000) == 0)
            basemprimendpts_c = basemprimendpts33p75_c(primind,:);
            angle = currentangle - 33.75*pi/180;
            fprintf(1, '33p75\n');
        elseif (rem(currentangle_36000int-2250, 9000) == 0)
            basemprimendpts_c = basemprimendpts22p5_c(primind,:);
            angle = currentangle - 22.5*pi/180;
            fprintf(1, '22p5\n');
        elseif (rem(currentangle_36000int-1125, 9000) == 0)
            basemprimendpts_c = basemprimendpts11p25_c(primind,:);
            angle = currentangle - 11.25*pi/180;
            fprintf(1, '11p25\n');
        else
            fprintf(1, 'ERROR: invalid angular resolution. angle = %d\n', currentangle_36000int);
            return;
        end;
        
        % now figure out what action will be
        baseendpose_c = basemprimendpts_c(1:3);
        additionalactioncostmult = basemprimendpts_c(4);
        endx_c = round(baseendpose_c(1)*cos(angle) - baseendpose_c(2)*sin(angle));
        endy_c = round(baseendpose_c(1)*sin(angle) + baseendpose_c(2)*cos(angle));
        endtheta_c = rem(angleind - 1 + baseendpose_c(3), numberofangles);
        endpose_c = [endx_c endy_c endtheta_c];
        
        fprintf(1, 'rotation angle=%f\n', angle*180/pi);
        
        if baseendpose_c(2) == 0 && baseendpose_c(3) == 0
            fprintf(1, 'endpose=%d %d %d\n', endpose_c(1), endpose_c(2), endpose_c(3));
        end;
        
        % calculate the initial and final poses
        startpt = [0 0 currentangle];
        endpt = [endpose_c(1)*resolution endpose_c(2)*resolution ... 
            rem(angleind - 1 + baseendpose_c(3), numberofangles)*2*pi/numberofangles];
        
       
        
        % -------------------------------------------------------------
        % Mitko's functions: input parameters
        % -------------------------------------------------------------
        dt = 1; % time discretization
        t_f = 10; % final time
        ki = 2; kf = 2; % free parameters
        
        % generate intermediate poses (remember they are w.r.t 0,0 (and not centers of the cells)
        numofsamples = t_f/dt + 1;
        intermcells_m = zeros(numofsamples,3);
        
        % and now let's calculate the path
        
        % calculating the orientation of the line connecting startpt-endpt
        if forwardcostmult == basemprimendpts_c(4)
            line_orient = atan2(endpt(2), endpt(1));
        else
            line_orient = atan2(-endpt(2), -endpt(1));
        end
        if line_orient < 0
            line_orient = 2*pi - abs(line_orient)
        end
        % are we on a straight line?
        if abs(startpt(3) - endpt(3)) < approxError && abs(line_orient - startpt(3)) < approxError
            intermcells_m = [linspace(startpt(1), endpt(1), numofsamples)' ...
                linspace(startpt(2), endpt(2), numofsamples)' ...
                linspace(startpt(3), startpt(3), numofsamples)'];
            fprintf(1, 'here we are\n');
        else
            [P,dP,ddP,dddP,ds] = poly_path(startpt(1),endpt(1),startpt(2),endpt(2),startpt(3),endpt(3),ki,kf,dt,t_f);
            if (backwardcostmult == basemprimendpts_c(4))
                % we go backwards
                [u,theta,phi] = flat_output(P,dP,ddP,dddP,-1);
            else
                [u,theta,phi] = flat_output(P,dP,ddP,dddP,1);
            end
            
            % remove -0
            P(logical(abs(P) < approxError)) = 0;
            theta(logical(abs(theta) < approxError)) = 0;
            % we do not want -3.14 but only 3.14
            theta(logical(theta - approxError < -pi)) = pi;
            
            N = size(P,2);
            for i = 1:N
                intermcells_m = [P', theta'];
            end
        end
        
        
        %%%%%% Validity check!
        
        
        % -------------------------------------------------------------
        
        % write out
        fprintf(fout, 'endpose_c: %d %d %d\n', endpose_c(1), endpose_c(2), endpose_c(3));
        fprintf(fout, 'additionalactioncostmult: %d\n', additionalactioncostmult);
        if (backwardcostmult == additionalactioncostmult)
            % we go backwards
            fprintf(fout, 'motiondirection: %d\n', -1);
        else
            fprintf(fout, 'motiondirection: %d\n', 1);
        end
        fprintf(fout, 'intermediateposes: %d\n', size(intermcells_m,1));
        for interind = 1:size(intermcells_m, 1)
            % MCO modification for normalizing the angles
            fprintf(fout, '%.4f %.4f %.4f\n', intermcells_m(interind,1), intermcells_m(interind,2), atan2(sin(intermcells_m(interind,3)), cos(intermcells_m(interind,3))));
            %fprintf(fout, '%.4f %.4f %.4f\n', intermcells_m(interind,1), intermcells_m(interind,2), intermcells_m(interind,3));
        end;
        
        plot(intermcells_m(:,1), intermcells_m(:,2));
        axis([-4 4 -4 4]);
        text(intermcells_m(numofsamples,1), intermcells_m(numofsamples,2), int2str(endpose_c(3)));
        hold on;
        
    end;
    grid;
    pause;
end;

fclose('all');

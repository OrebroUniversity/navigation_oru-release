function [path,motion] = ReedsSheepSolver(L,s0,s1,delta_max, plot_flag,movie_flag)
     
    path=0;
    motion = 1;
    
    l = L/2;
    minR = l/tan(delta_max);
    
    s0(1) = s0(1)+l*cos(s0(3));
    s0(2) = s0(2)+l*sin(s0(3));
    
    s1(1) = s1(1)+l*cos(s1(3));
    s1(2) = s1(2)+l*sin(s1(3));
    
    z = s1(3) - s0(3);
    z = atan2(sin(z), cos(z));

    % not safely computable
    if abs(z) > (3/4 * pi)
        fprintf(1, 'No solution1\n');
        return;
    end

    
    %%reedsSheep
    
    ConnObj = reedsSheppConnection;
    ConnObj.MinTurningRadius = minR;
    startPose = s0(1:3);
    goalPose = s1(1:3);
    [pathSegObj,pathCosts] = connect(ConnObj,startPose,goalPose);
    lenght = pathSegObj{1}.Length;
    poses = interpolate(pathSegObj{1},0:lenght/97:lenght);
    poses2 = interpolate(pathSegObj{1});
    
    if size(poses2,1)<=2
        poses2(3,:) = poses2(2,:);
        poses2(2,:) = poses2(1,:);
    end
    for i = 1:length(poses2)-1
        POSE{i,1} = poses2(i+1,:);
        POSE{i,2} = pathSegObj{1}.MotionTypes{1,i};
        POSE{i,3} = pathSegObj{1}.MotionLengths(i);     
    end
    
    path = zeros(100,5);
    k = 1;
    p=0;
    phi_old = 0;
    phi=0;
    steer = POSE{1,2};
    for j = 1:length(poses)
        if ( j == 1 || j == length(poses)) 
            phi_old=phi;
            phi = 0;   
        elseif (steer == "S") phi = 0;
            elseif (steer == "L" ) phi = delta_max;
                elseif (steer == "R" ) phi = -delta_max;
        end
        
        if phi_old ~= phi
            for f = 1:4
                ang = (abs(phi_old) + abs(phi) )/5;
                s = sign(phi);
                if (s == 0 ) 
                    s=-sign(phi_old);
                end
                path(p+f,:)=path(p,:);
                path(p+f,4) = path(p,4) + s*ang*f;
                path(p+f,5) = -path(p+f,4); 
            end
            p = p + 4;
            phi_old = phi;
        end
                
        p=p+1;
        path(p,3) = poses(j,3);
        path(p,1) = poses(j,1)-l*cos(poses(j,3));
        path(p,2) = poses(j,2)-l*sin(poses(j,3));
        path(p,4) = phi;
        path(p,5) = -phi;
        
        if (k<length(poses2)-1 )
            if(poses(j,1:3) == POSE{k,1})
                steer = POSE{k+1,2};
                phi_old = phi;
                k = k+1;
            end
        end
    end

      

    % -----------------------------------------------
    % plot
    % -----------------------------------------------
    if ~plot_flag
        return
    end
    show(pathSegObj{1,1});
    s = path;

    N = 100;

    if ~movie_flag
    return
    end

    % -----------------------------------------------
    % movie
    % -----------------------------------------------

    figure(2)
    plot(s(1,:),s(2,:),'r');grid on; axis equal

    hold on;

    for i=1:N
        %h = draw_car4w(L,s(:,i));
        h = draw_4wsRear(L/2,L/2,s(i,:)');
        drawnow
        pause(0.05)

        if i>1 && i<N
            for k=1:length(h)
                set(h{k},'Visible','off');
            end
        end
    end
end


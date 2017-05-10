
for x = -1:0.5:1
    for y = -1:0.5:1
        for theta = -pi:pi/8:pi
            fprintf(1, '\ngoal: %1.4f\t%1.4f\t%1.4f\n', x, y, theta);
            test = CarLikeVehicleSolver(1,1,0,0,0,0,x,y,theta,0,1);
            pause;
            close all
        end
    end
end



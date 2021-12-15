function [cusp] = cuspidi(traj_x,traj_y,traj_o,print)
      incr = 5;
      mot_array = [];
      motion_old3 = 0;
      motion_old2 = 0;
      motion_old = 0;
      motion = 0;
      cusp = 0;
      inc = 1 + incr;
      for  i = 1:length(traj_x)-inc
        if (motion  ~= 0)
            motion_old3 = motion_old2;
            motion_old2 = motion_old;
            motion_old = motion;
        end
        
        prev = [traj_x(i),traj_y(i),traj_o(i)];
        next = [traj_x(i+inc),traj_y(i+inc),traj_o(i+inc)];
       
        
        %motion = findDirection(prev , next);
        motion = findDir(prev,next);
        mot_array = [mot_array;motion];
        

        if (motion_old3 ~= 0 && motion_old ~= motion_old2 && motion_old == motion && motion_old2 == motion_old3)
            if (prev(1) ~= next(1) || prev(2) ~= next(2) ||  prev(3) ~= next(3) )
                cusp = cusp +1;
            end
        end
      end
        if (print) 
            display(mot_array);
            plot(traj_x,traj_y);
            grid on; axis equal;
        end
       
      
end

function [r] =  wrap_rads( r )
     
    while ( r > pi )  
        r = r - 2 * pi; 
    end
    while ( r <= -pi ) 
        r = r+ 2 * pi; 
    end

end

function direction = findDirection(prev, next)
      x0 = prev(1); y0 = prev(2);  th = prev(3);
      x1 = next(1); y1 = next(2);
      
      th = wrap_rads(th);
      
      
      if (abs(cos(th+pi/2)) <= 0.01 || th == 0 || abs(th) == 3.14159)
          m=1000;
          
         if (abs(th) < 3)
            if ((y1-y0) < m*(x1-x0))
                direction = 1;
                a = 'a'
                return
            else
                direction = -1;
                b = 'b'
                return 
            end
          
        else
            if ((y1-y0) > m*(x1-x0)) 
                direction = 1;
                c = 'c'
                return
            else
                direction = -1;
                d = 'd'
                return 
            end
        end
      
      else
          m = tan(th+pi/2);
      end

      if (sign(th) == 1)
            if ((y1-y0) < m*(x1-x0))
                direction = 1;
                e = 'e'
                return
            end
      
      else 
        if ((y1-y0) > m*(x1-x0))
            direction = 1;
            f = 'f'
            return 
            
        end
      end
      direction = -1;
      g = 'g'
      return
end

    
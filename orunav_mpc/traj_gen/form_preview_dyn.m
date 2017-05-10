function [S,T] = form_preview_dyn(stage, L, dt)
%
% form S and T in X = S*x0 + T*U
%

Nx = 4;
Nu = 2;

% --------------------------------------------------------------------------------------------

N = length(stage);

% form S
S = zeros(Nx*N,Nx);
for i = 1:N
    ind_x = Nx*(i-1)+1:Nx*i;
    S(ind_x,:) = eye(Nx);
    for k = 1:i
        S(ind_x,:) = A(dt,stage(k).theta,stage(k).phi,stage(k).v,L)*S(ind_x,:);
    end
end

% form T
T = zeros(Nx*N,Nu*N);
for i = 1:N
    ind_u = Nu*(i-1)+1:Nu*i;
    for j = i:N
        ind_x = Nx*(j-1)+1:Nx*j;
        T(ind_x,ind_u) = B(dt,stage(i).theta,stage(i).phi,L);
        for k = i+1:j
            T(ind_x,ind_u) = A(dt,stage(k).theta,stage(k).phi,stage(k).v,L)*T(ind_x,ind_u);
        end
    end
end

end % EOF form_preview_dyn


function out = A(dt, theta, phi, v, l)

out = [1, 0, -sin(theta)*v*dt,                   0; 
       0, 1,  cos(theta)*v*dt,                   0; 
       0, 0,                1, v*dt/(l*cos(phi)^2);  
       0, 0,                0,                   1];

end % EOF A


function out = B(dt, theta, phi, l)

out = [cos(theta)*dt,  0;
       sin(theta)*dt,  0;
       dt*tan(phi)/l,  0;
       0,              dt];

end % EOF B


%%%EOF

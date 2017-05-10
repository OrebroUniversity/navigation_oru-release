%
%

load ../test/test_05_data/trajectory.txt
load ../test/test_05_data/sensors.txt
outfile = '../test/test_05_data/solution.txt';

sensors(:,3) = atan2(sin(sensors(:,3)), cos(sensors(:,3)));

theta0 = sensors(1,3);
RR = [cos(theta0) sin(theta0); -sin(theta0) cos(theta0)];
sensors = [(sensors(:,1:3) - repmat (sensors(1,1:3), size(sensors, 1), 1)) sensors(:,4)]';
sensors(:,3) = atan2(sin(sensors(:,3)), cos(sensors(:,3)));
sensors(1:2,:) = RR*sensors(1:2,:);

state = trajectory(:,1:4)';
control = trajectory(:,5:6)';

%               ________(2,1)
%              /       /
%             /_______/
%  (-0.2, -0.2)
pos_G = [
    0 1 0 0; 
    -1 0 0 0;
    1 -1 0 0;
    -1 1 0 0];
pos_g = [1; 0.2; 0.9; 0.2];


% =========================================================================
% generate preview data
% =========================================================================

Nx = 4; % dimension of state vector
Nu = 2; % dimension of control vector

Np = 5; % number of sampling times in a preview window

dt = 0.06;        % time step

L = 0.68;            % length of the car

max_steer_angle = pi/3; % radian
max_vel = 1;            % m/s
max_steer_vel = pi/2;   % rad/s


% Penalties
StateGains         = [1,1,1,1]; 
TerminalStateGains = [1,1,1,1]; 
ControlGains       = [1,1]; 
Qk = diag(StateGains); 
QN = diag(TerminalStateGains); 
Rk = diag(ControlGains); 


% --------------------------------------------------------

solution = [];
control_phi = [];
control_v = [];
control_w = [];
state_sol = [];

for k = 2:size(state, 2) - Np + 1
    k
    % index of initial sampling point on trajectory 
    PreviewIndex = k:k+Np-1;

    % initial state error
    x0 = sensors(:,k-1) - state(:,k-1);


    counter = 1;
    for i=PreviewIndex
        stage(counter).x     = state(1,i);
        stage(counter).y     = state(2,i);
        stage(counter).theta = state(3,i);
        stage(counter).phi   = state(4,i); 

        stage(counter).v = control(1,i); 
        stage(counter).w = control(2,i); 

        counter = counter+1;
    end


    [S,T] = form_preview_dyn(stage, L, dt);

    Q = zeros(Np*Nx,Np*Nx); 
    R = zeros(Np*Nu,Np*Nu);
    for i=1:Np-1
        ind_x = Nx*(i-1)+1:Nx*i;
        Q(ind_x, ind_x) = Qk;
    end
    ind_x = Nx*(Np-1)+1:Nx*Np;
    Q(ind_x, ind_x) = QN;
        
    for i=1:Np
        ind_u = Nu*(i-1)+1:Nu*i;
        R(ind_u, ind_u) = Rk;
    end

    P = T'*Q*T+R;
    p = T'*Q*S*x0;




    % =========================================================================
    % bounds on phi
    % =========================================================================

    M = zeros(1,4);
    M(1,4) = 1;
    phi_G = [];
    control_bounds = [max_vel, max_steer_vel];
    lb = [];
    ub = [];

    for i=1:Np
        phi_G = blkdiag(phi_G, M);

        lb = [lb, -control_bounds - [control(1,k+i-1) control(2,k+i-1)]];
        ub = [ub,  control_bounds - [control(1,k+i-1) control(2,k+i-1)]];
    end

    phi_GT = phi_G*T;   % constant
    phi_g_l = -max_steer_angle*ones(Np,1) - state(4, PreviewIndex)' - phi_G*S*x0;
    phi_g_u =  max_steer_angle*ones(Np,1) - state(4, PreviewIndex)' - phi_G*S*x0;
    phi_GT = [phi_GT; -phi_GT];
    phi_g_u = [phi_g_u; - phi_g_l];
   

    % =========================================================================
    % Constraints on position
    % =========================================================================
    pos_G_Np = [];
    for i=1:Np
        pos_G_Np = blkdiag(pos_G_Np, pos_G);
    end
    pos_g_Np = repmat(pos_g, Np, 1);
    pos_GT = pos_G_Np * T;
% Octave    
    pos_g_u = pos_g_Np - pos_G_Np*S*x0 - pos_G_Np*vec(state(:,PreviewIndex));
% Matlab
%    tmp = state(:,PreviewIndex);
%    pos_g_u = pos_g_Np - pos_G_Np*S*x0 - pos_G_Np*tmp(:);


    % =========================================================================
    % Solution
    % =========================================================================
    GT = [phi_GT; pos_GT];
    g_u = [phi_g_u; pos_g_u];
    [X, FVAL, OUTPUT, LAMBDA] = qp ([], P, p, [], [], lb, ub, [], GT, g_u);
%    [X, FVAL, OUTPUT, LAMBDA] = qp ([], P, p);
% Octave
    if OUTPUT.info != 0
% Matlab
%    if OUTPUT.info ~= 0
         keyboard
    end

    solution = [solution; X];
    control_v = [control_v; X(1) + control(1,k)];
    control_w = [control_w; X(2) + control(2,k)];
    control_phi = [control_phi; state(4,k) + dt*X(2)];
    state_sol = [state_sol; vec(state(:,PreviewIndex)) + S*x0 + T*X];
end

fileID = fopen(outfile,'w');
fprintf(fileID,'%2.14f\n',  solution');
fclose(fileID);


%control_v - control(1,2:117)'
%control_phi - state(4,2:117)'

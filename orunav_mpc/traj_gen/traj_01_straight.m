%
% generate data and verify the result of the C++ implementation
%


%clear;clc

% =========================================================================
% plan
% =========================================================================
%dt = 0.05;        % time step
%tf = 5;           % final time
dt = 0.06;        % time step
tf = 3;           % final time
tspan = 0:dt:tf;  

%s0 = [0;0;0;0];   % initial state
%s1 = [50;20;0;0]; % final state
s0 = [0;0;0;0];   % initial state
s1 = [1;0;0;0]; % final state

L = 0.68;            % length of the car



%[state,control] = plan_car_trajectory(L,s0,s1,[110;110],tspan);
[state,control] = plan_car_trajectory(L,s0,s1,[1;1],tspan,dt);

output_trajectory(s0, state, control, 'trajectory_01.txt');

return

% =========================================================================
% generate preview data
% =========================================================================
Np = 15; % number of sampling times in a preview window

Nx = 4; % dimension of state vector
Nu = 2; % dimension of control vector


% Penalties
StateGains         = [1,2,3,4]; 
TerminalStateGains = [5,6,7,8]; 
ControlGains       = [9,10]; 

% initial state error
x0 = [0.1;0.2;-0.1;0.04];

% index of initial sampling point on trajectory 
index_0 = 17+1; 

% --------------------------------------------------------

Qk = diag(StateGains); 
QN = diag(TerminalStateGains); 
Rk = diag(ControlGains); 

PreviewIndex = index_0:index_0+Np-1;

counter = 1;
for i=PreviewIndex
    stage(counter).x     = state(1,i);
    stage(counter).y     = state(2,i);
    stage(counter).theta = state(3,i);
    stage(counter).phi   = state(4,i); 
    counter = counter+1;
end

counter = 1;
for i=PreviewIndex
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
% compare C++ vs Matlab 
% =========================================================================

%P1 = load('../data/H.txt');
%p1 = load('../data/g.txt');

%norm(P1-P)
%norm(p1-p)

% =========================================================================
% bounds on phi
% =========================================================================

phi_G = zeros(2*Np,Nx*Np);
M = zeros(2,4);
M(1,4) = 1; M(2,4) = -1;

for i=1:Np
    ind1 = 2*(i-1)+1:2*i;
    ind2 = Nx*(i-1)+1:Nx*i;
    
    phi_G(ind1,ind2) = M;
end

phi_GT = phi_G*T;   % constant
phi_g = phi_G*S*x0; % only the last element of x0 is important

phi_g = pi/4*ones(2*Np,1) - phi_g; % assuming phi \in [-pi/4, pi/4]

%phi_GT1 = load('../data/phi_GT.txt');
%phi_g1 = load('../data/phi_g.txt');

%[X, FVAL, EXITFLAG, OUTPUT, LAMBDA] = quadprog(P, p, phi_GT, phi_g);

[X, FVAL, OUTPUT, LAMBDA] = qp ([], P, p, [], [], [], [], [], phi_GT, phi_g);

return
% =========================================================================
% test bounds on theta
% =========================================================================

theta_G = zeros(2*Np,Nx*Np);
M = zeros(2,4);
M(1,3) = 1; M(2,3) = -1;

for i=1:Np
    ind1 = 2*(i-1)+1:2*i;
    ind2 = Nx*(i-1)+1:Nx*i;
    
    theta_G(ind1,ind2) = M;
end

theta_GT = theta_G*T;   
theta_g = theta_G*S*x0; 

%%%EOF

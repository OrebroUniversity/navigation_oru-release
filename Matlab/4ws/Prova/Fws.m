clear;

BEGIN_ACADO;                                % Always start with "BEGIN_ACADO". 
    
    acadoSet('problemname', 'FourWheelSteering');% Set your problemname. If you 
                                            % skip this, all files will
                                            % be named "myAcadoProblem"

    DifferentialState x;                    
    DifferentialState y;                   
    DifferentialState psi;                    
    DifferentialState df;
    DifferentialState dr;
    

 
    
    Control vw; 
    Control wf wr; 
                          
    lf = 1;
    lr = 1;
    
    beta = atan((lf*tan(dr)+lr*tan(df))/(lr+lf));
    v = (vw*cos(df)+vw*cos(dr))/(lf+lr);

    
    %% Diferential Equation
    f = acado.DifferentialEquation();

    
    f.add(dot(x) == v*cos(psi+beta));                     % Write down your ODE. 
    f.add(dot(y) == v*sin(psi+beta));      %
    f.add(dot(psi) == (v*cos(beta)*(tan(df)-tan(dr)))/(lf+lr));             %
    f.add(dot(df) == wf);     
    f.add(dot(dr) == wr); 

    
    %% Optimal Control Problem
    ocp = acado.OCP(0.0, 15,50);         % Set up the Optimal Control Problem (OCP)
                                            % Start at 0s, control in 20
                                            % intervals upto 10s
                                            
    ocp.minimizeMayerTerm((dr)^2);
    ocp.subjectTo( f );                     % Optimize with respect to your differential equation
    ocp.subjectTo( 'AT_START', x ==  0.0 ); 
    ocp.subjectTo( 'AT_START', y ==  0.0 ); 
    ocp.subjectTo( 'AT_START', psi ==  0.0 );
    ocp.subjectTo( 'AT_START', df ==  0.0 );
    ocp.subjectTo( 'AT_START', dr ==  0.0 );
    
    ocp.subjectTo( 'AT_END', x ==  5.0 ); 
    ocp.subjectTo( 'AT_END', y ==  5.0 ); 
    ocp.subjectTo( 'AT_END', psi ==  0 ); 
    ocp.subjectTo( 'AT_END', df ==  0.0 );
    ocp.subjectTo( 'AT_END', dr ==  0.0 );
    
     
    ocp.subjectTo( -0.8 <= vw <= 0.8 ); 
    
    ocp.subjectTo( -0.3 <= wf <= 0.3 );
    ocp.subjectTo( -0.3 <= wr <= 0.3 );
    
    ocp.subjectTo( -pi/4 <= df <= pi/4 );
    ocp.subjectTo( -pi/4 <= dr <= pi/4 );
    


    
    %% Optimization Algorithm
    algo =acado.OptimizationAlgorithm(ocp); % Set up the optimization algorithm, link it to your OCP
    algo.set( 'KKT_TOLERANCE', 1e-6 );      % Set a custom KKT tolerance
    
    
END_ACADO;           % Always end with "END_ACADO".
                     % This will generate a file problemname_ACADO.m. 
                     % Run this file to get your results. You can
                     % run the file problemname_ACADO.m as many
                     % times as you want without having to compile again.

% Run the test
out = FourWheelSteering_RUN();

draw(out.STATES,lf,lr,1);
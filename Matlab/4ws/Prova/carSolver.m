function [pr,control] = carSolver(s1,time,k,lf,lr,drawFlag)


BEGIN_ACADO;                                % Always start with "BEGIN_ACADO". 
    
    acadoSet('problemname', 'FourWheelSteering');% Set your problemname. If you 
                                            % skip this, all files will
                                            % be named "myAcadoProblem"

    DifferentialState x;                    
    DifferentialState y;                   
    DifferentialState ps;                    
    DifferentialState df;


    


    
    Control v; 
    Control wf; 
                          
    
    L=lr+lf;
    dr = -k*df;
    %% Diferential Equation
    f = acado.DifferentialEquation();

    
    f.add(dot(x) == v*cos(ps - k*df ));                     % Write down your ODE. 
    f.add(dot(y) == v*sin(ps  - k*df));      %
    f.add(dot(ps) == v*sin(df + k*df)/(L*cos(df)));             %
    f.add(dot(df) == wf);  
    


    
    %% Optimal Control Problem
    ocp = acado.OCP(0.0, time,time*3);         % Set up the Optimal Control Problem (OCP)
                                            % Start at 0s, control in 20
                                            % intervals upto 10s
                                            
    ocp.minimizeMayerTerm( x^2+y^2);
    ocp.subjectTo( f );                     
    ocp.subjectTo( 'AT_START', x ==  0.0 ); 
    ocp.subjectTo( 'AT_START', y ==  0.0 ); 
    ocp.subjectTo( 'AT_START', ps ==  0.0 );
    ocp.subjectTo( 'AT_START', df ==  0.0 );

    
    ocp.subjectTo( 'AT_END', x ==  s1(1) ); 
    ocp.subjectTo( 'AT_END', y ==  s1(2)); 
    ocp.subjectTo( 'AT_END', ps ==  s1(3) ); 
    ocp.subjectTo( 'AT_END', df ==  s1(4) );

    
     
    ocp.subjectTo( -0.8 <= v <= 0.8 );  %velocity
    
    ocp.subjectTo( -0.3 <= wf <= 0.3 );  %steering angular velocity
    
    ocp.subjectTo( -pi/4 <= df <= pi/4 );  %dteering angle

    


    
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
pr = out.STATES(:,2:5);
pr(:,5) = -k*pr(:,4);
control = out.CONTROLS;

    draw2(out.STATES,L,k,drawFlag);
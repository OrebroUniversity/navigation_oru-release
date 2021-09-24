function [out,ocp] = acadoSolver(wheel_base,s0,s1,plot)

%fare prima make su cartella acado/interfaces/matlab

BEGIN_ACADO;                                % Always start with "BEGIN_ACADO". 
    
    acadoSet('problemname', 'FourWheelSteering');% Set your problemname. If you 
                                            % skip this, all files will
                                            % be named "myAcadoProblem"
    Parameter T;
    DifferentialState x;                    
    DifferentialState y;                   
    DifferentialState th;                    
    DifferentialState phi;
    DifferentialState phiRear;
    
    Control v wf wr; 
                          
    

    
    %% Diferential Equation
    f = acado.DifferentialEquation();

    
    f.add(dot(x) == cos(th+phiRear)*v);                     % Write down your ODE. 
    f.add(dot(y) == sin(th+phiRear)*v);      %
    f.add(dot(th) == v*sin(phi-phiRear)/(wheel_base*cos(phi)));             %
    f.add(dot(phi) == wf);     
    f.add(dot(phiRear) == -wf); 
    
    
    
    %% Optimal Control Problem
    ocp = acado.OCP(0,15,100);         % Set up the Optimal Control Problem (OCP)
                                            % Start at 0s, control in 20
                                            % intervals upto 10s
                                            
    ocp.minimizeMayerTerm(2*v^2);
    ocp.subjectTo( f );                     % Optimize with respect to your differential equation
    ocp.subjectTo( 'AT_START', x ==  s0(1) ); 
    ocp.subjectTo( 'AT_START', y ==  s0(2) ); 
    ocp.subjectTo( 'AT_START', th ==  s0(3) );
    ocp.subjectTo( 'AT_START', phi ==  s0(4) );
    ocp.subjectTo( 'AT_START', phiRear ==  s0(5) );
    
    ocp.subjectTo( 'AT_END', x ==  s1(1) ); 
    ocp.subjectTo( 'AT_END', y ==  s1(2) ); 
    ocp.subjectTo( 'AT_END', th ==  s1(3) ); 
    ocp.subjectTo( 'AT_END', phi ==  s1(4) );
    ocp.subjectTo( 'AT_END', phiRear ==  s1(5) );
    
     
    ocp.subjectTo( -0.8 <= v <= 0.8 );
    ocp.subjectTo( -0.6<= wf <= 0.6 );
    ocp.subjectTo( -0.6 <= wr <= 0.6);
    ocp.subjectTo( -0.6 <= phi <= 0.6 );
    ocp.subjectTo( -0.6 <= phiRear <= 0.6 );
    ocp.subjectTo(    2 <= T <= 20 ); 


    
    %% Optimization Algorithm
    algo =acado.OptimizationAlgorithm(ocp); % Set up the optimization algorithm, link it to your OCP
    algo.set( 'KKT_TOLERANCE', 1e-4 );      % Set a custom KKT tolerance
    
    
END_ACADO;           % Always end with "END_ACADO".
                     % This will generate a file problemname_ACADO.m. 
                     % Run this file to get your results. You can
                     % run the file problemname_ACADO.m as many
                     % times as you want without having to compile again.

% Run the test
out = FourWheelSteering_RUN();

%plot(out.STATES(:,2),out.STATES(:,3))
%AcadoPlot
end


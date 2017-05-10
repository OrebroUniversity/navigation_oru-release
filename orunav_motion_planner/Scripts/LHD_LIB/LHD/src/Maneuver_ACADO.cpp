// Time-stamp: <2012-07-02 10:07:04 (drdv)>

#include "../include/Maneuver.h"
#include "ODE_opt.cpp"
#include "ODE_sim.cpp"

void Maneuver::integrate()
{
  USING_NAMESPACE_ACADO;

  TIME                t;    
  DifferentialState   phi;
  
  std::vector<VariablesGrid> states;
  states.resize(N);
    
  double phi_IC[1];
  for (int k=0; k<N; k++)
    {      
      choose_ode(k);

      DifferentialEquation f_sim;  
      ODE_sim( *this, t, phi, f_sim );
  
      IntegratorRK78 integrator( f_sim );  
      integrator.set( INTEGRATOR_PRINTLEVEL, NONE );
      integrator.set( INTEGRATOR_TOLERANCE, 1.0e-6 );
  
      // ========================================================
      // set initial conditions
      // ========================================================
      if (N == 1) // a single stage
	{
	  if (sp[0].direction == 1) // forward motion
	    phi_IC[0] = s0.phi;
	  else // backward motion
	    phi_IC[0] = s1.phi;
	}
      else // multiple stages
	{
	  if (k==0)
	    {
	      if (sp[0].direction == 1) // forward motion
		phi_IC[0] = s0.phi;
	      else // backward motion
		phi_IC[0] = vr[0].phi.solution;
	    }

	  if ( (k > 0) && (k < N-1) )
	    {
	      if (sp[k].direction == 1) // forward motion
		phi_IC[0] = vr[k-1].phi.solution;
	      else // backward motion
		phi_IC[0] = vr[k].phi.solution;
	    }

	  if (k == N-1)
	    {
	      if (sp[k].direction == 1) // forward motion
		phi_IC[0] = vr[k-1].phi.solution;
	      else // backward motion
		phi_IC[0] = s1.phi;
	    }
	}
      // ========================================================
      
      // grid of arc lengths (from 0 to 1) where, 
      // 0 corresponds to start of segment
      // 1 corresponds to end of segment
      Grid sigma_grid(options.NumbPointsSim); 
      for (int i=0; i<options.NumbPointsSim; i++)
	sigma_grid.setTime(i,poly[k].p[i]);

      integrator.integrate( sigma_grid, phi_IC );
      integrator.getX( states[k] );

      // output profile of phi (as well as time and arc length) from simulation
      
      if (sp[k].direction == 1) // forward motion
	{
	  for (int i=0; i<options.NumbPointsSim; i++) 
	    {
	      out[k].set_phi_sim(i, states[k](i,0));
	      out[k].set_time_sim(i, poly[k].grid[i]);
	      out[k].set_ArcLength_sim(i, poly[k].p[i]);
	    }
	}
      else // backward motion
	{
	  for (int i=0; i<options.NumbPointsSim; i++) 
	    {
	      out[k].set_phi_sim(i, states[k](options.NumbPointsSim-1-i,0));
	      out[k].set_time_sim(i, poly[k].grid[0] + poly[k].grid[options.NumbPointsSim-1] - poly[k].grid[options.NumbPointsSim-1-i]);
	      out[k].set_ArcLength_sim(i, 1 - poly[k].p[options.NumbPointsSim-1-i]);
	    }
	}
    }

  choose_ode(-1); // set back the default value
  
  // clear static counters
  phi.clearStaticCounters(); 

}

void Maneuver::form_pose()
{
  double a0_x, a0_y, aT_x, aT_y, k_0, k_T;
  double x_0, y_0, theta_0, x_T, y_T, theta_T;
  double x, y, dx, dy, theta, s;
  double tmp[12], px[6], py[6];  
  
  for (int k=0; k<N; k++)
    {
      a0_x = sp[k].a0_x.solution;
      a0_y = sp[k].a0_y.solution;
      aT_x = sp[k].aT_x.solution;
      aT_y = sp[k].aT_y.solution;
      k_0  = sp[k].k_0.solution;
      k_T  = sp[k].k_T.solution;

      if (N == 1) // a single stage
	{
	  if (sp[0].direction == 1) // forward motion
	    {
	      x_0     = s0.x;
	      y_0     = s0.y;
	      theta_0 = s0.theta;
	
	      x_T     = s1.x;
	      y_T     = s1.y;
	      theta_T = s1.theta;
	    }
	  else // backward motion
	    {
	      x_0     = s1.x;
	      y_0     = s1.y;
	      theta_0 = s1.theta;
	
	      x_T     = s0.x;
	      y_T     = s0.y;
	      theta_T = s0.theta;
	    }
	}
      else // multiple stages
	{
	  if (k == 0)
	    {
	      if (sp[0].direction == 1) // forward motion
		{
		  x_0     = s0.x;
		  y_0     = s0.y;
		  theta_0 = s0.theta;
	    
		  x_T     = vr[k].x.solution;
		  y_T     = vr[k].y.solution;
		  theta_T = vr[k].theta.solution;
		}
	      else // backward motion
		{
		  x_0     = vr[k].x.solution;
		  y_0     = vr[k].y.solution;
		  theta_0 = vr[k].theta.solution;

		  x_T     = s0.x;
		  y_T     = s0.y;
		  theta_T = s0.theta;
		}
	    }
	  if ( (k > 0) && (k < N-1) )
	    {
	      if (sp[k].direction == 1) // forward motion
		{
		  x_0     = vr[k-1].x.solution;
		  y_0     = vr[k-1].y.solution;
		  theta_0 = vr[k-1].theta.solution;
	      
		  x_T     = vr[k].x.solution;
		  y_T     = vr[k].y.solution;
		  theta_T = vr[k].theta.solution;
		}
	      else // backward motion
		{	      
		  x_0     = vr[k].x.solution;
		  y_0     = vr[k].y.solution;
		  theta_0 = vr[k].theta.solution;
		 
		  x_T     = vr[k-1].x.solution;
		  y_T     = vr[k-1].y.solution;
		  theta_T = vr[k-1].theta.solution;
		}
	    }
      
	  if (k == N-1)
	    {
	      if (sp[k].direction == 1) // forward motion
		{
		  x_0     = vr[k-1].x.solution;
		  y_0     = vr[k-1].y.solution;
		  theta_0 = vr[k-1].theta.solution;
	      
		  x_T     = s1.x;
		  y_T     = s1.y;
		  theta_T = s1.theta;
		}
	      else // backward motion
		{
		  x_0     = s1.x;
		  y_0     = s1.y;
		  theta_0 = s1.theta;

		  x_T     = vr[k-1].x.solution;
		  y_T     = vr[k-1].y.solution;
		  theta_T = vr[k-1].theta.solution;	      
		}
	    }
	}

      get_parameters_xy(a0_x, a0_y, aT_x, aT_y, k_0, k_T,
			x_0, y_0, theta_0, 
			x_T, y_T, theta_T,
			tmp);
  
      // do this for convenience
      for (int i=0; i<6; i++)
	{
	  px[i] = tmp[i];
	  py[i] = tmp[6+i];
	}
      
      for (int i=0; i<options.NumbPointsSim; i++)
	{
	  s = poly[k].p[i];

	  x = px[0]*pow(s,5) + px[1]*pow(s,4) + px[2]*pow(s,3) + px[3]*pow(s,2) + px[4]*s + px[5];
	  y = py[0]*pow(s,5) + py[1]*pow(s,4) + py[2]*pow(s,3) + py[3]*pow(s,2) + py[4]*s + py[5];
      
	  dx = 5*px[0]*pow(s,4) + 4*px[1]*pow(s,3) + 3*px[2]*pow(s,2) + 2*px[3]*s + px[4];
	  dy = 5*py[0]*pow(s,4) + 4*py[1]*pow(s,3) + 3*py[2]*pow(s,2) + 2*py[3]*s + py[4];
      
	  theta = atan2(dy,dx); // assuming that v>0

	  // output profile of pose from simulation     
	  if (sp[k].direction == 1) // forward motion
	    out[k].set_pose_sim(i, x, y, theta);
	  else // backward motion
	    out[k].set_pose_sim(options.NumbPointsSim-1-i, x, y, theta);
	}
    }
}

void Maneuver::plan()
{
  USING_NAMESPACE_ACADO;

  const int n = NUM_PARAM*N;

  // ==============================================================================  
  // DEFINE VARIABLES: 
  // ==============================================================================  
  DifferentialState     phi(N);            // phi (for all N stages)
  Parameter             p(n);              // a0_x, a0_y, aT_x, aT_y, k_0, k_T (for all N stages)
  Parameter             pv(NUM_VIA*(N-1)); // x, y, theta, phi (via region)
  DifferentialEquation  f_opt;
  TIME                  t;

  // ==========================================================================
  // DEFINE A DIFFERENTIAL EQUATION:
  // ==========================================================================
  ODE_opt( *this, t, phi, p, pv, f_opt );
  
  // ==============================================================================  
  // DEFINE AN OPTIMAL CONTROL PROBLEM:
  // ==============================================================================  
  OCP ocp(0,1,options.NumbIntervalsOpt); //NumbIntervalsOpt+1 points 
  
  // ==============================================================================  
  // DEFINE OBJECTIVE FUNCTION:
  // ==============================================================================  
#ifdef OPTIMIZATION_ALGORITHM
  if (options.ObjFun == 1)
    {
      /*
      Function h;
      h << p; // phi
      ocp.minimizeLSQ(h);
      */
      //ocp.minimizeLagrangeTerm( p.transpose()*p );
      ocp.minimizeLagrangeTerm( phi.transpose()*phi );
      //ocp.minimizeLagrangeTerm( p.transpose()*p + phi.transpose()*phi );
    }
  else  
    ocp.minimizeMayerTerm(1);
#else
  Function h;
  h << 0;
  ocp.minimizeLSQ(h);
#endif

  // ==============================================================================
  // DEFINE CONSTRAINTS DUE TO THE ODE
  // ==============================================================================
  ocp.subjectTo( f_opt );
  
  // ==============================================================================
  // DEFINE BOUNDARY CONSTRAINTS FOR STAGES
  // ==============================================================================

  if (N == 1) // a single stage
    {
      if (sp[0].direction == 1) // forward motion
	{
	  ocp.subjectTo( AT_START, phi(0) == s0.phi );  
	  ocp.subjectTo( AT_END  , phi(0) == s1.phi );  
	}
      else // backward motion
	{
	  ocp.subjectTo( AT_START, phi(0) == s1.phi );  
	  ocp.subjectTo( AT_END  , phi(0) == s0.phi );  
	}
    }
  else // multiple stages
    {
      // -------------------------------------------------
      // first stage (0)
      // -------------------------------------------------
      if (sp[0].direction == 1) // forward motion
	{
	  ocp.subjectTo( AT_START, phi(0) == s0.phi );  
	  ocp.subjectTo( AT_END  , phi(0) - pv(3) == 0 );  
	}
      else // backward motion
	{
	  ocp.subjectTo( AT_START, phi(0) - pv(3) == 0 );  
	  ocp.subjectTo( AT_END  , phi(0) == s0.phi );  
	}

      // -------------------------------------------------
      // stag 1 to stage N-2
      // -------------------------------------------------
      int ind_start, ind_end;
      for (int i=1; i<N-1; i++)
	{
	  ind_start = NUM_VIA*(i-1) + 3;
	  ind_end   = NUM_VIA*i     + 3;

	  if (sp[i].direction == 1) // forward motion
	    {
	      ocp.subjectTo( AT_START, phi(i) - pv(ind_start) == 0 );
	      ocp.subjectTo( AT_END  , phi(i) - pv(ind_end)   == 0 );  
	    }
	  else // backward motion
	    {
	      ocp.subjectTo( AT_START, phi(i) - pv(ind_end)   == 0 );
	      ocp.subjectTo( AT_END  , phi(i) - pv(ind_start) == 0 );  
	    }
	}

      // -------------------------------------------------
      // last stage (N-1)
      // -------------------------------------------------
      if (sp[N-1].direction == 1) // forward motion
	{
	  ocp.subjectTo( AT_START, phi(N-1) - pv(NUM_VIA*(N-2) + 3) == 0 );
	  ocp.subjectTo( AT_END  , phi(N-1) == s1.phi                    );  
	}
      else // backward motion
	{
	  ocp.subjectTo( AT_START, phi(N-1) == s1.phi                   );  
	  ocp.subjectTo( AT_END  , phi(N-1) - pv(NUM_VIA*(N-2) + 3) == 0);  
	}
    }
 
  // ==============================================================================
  // IMPOSE (APPROXIMATE) DIFFERENTIABILITY OF phi AT TRANSISTION BETWEEN STAGES
  // ============================================================================== 
  // MCO: offset originally 1
  int ind, offset = 2;
  for (int i=0; i<N-1; i++) // loop over all via regions
    { 
      // if both stages "around" the i-th via region are with forward motion
      // * ----> * ----> * 
      //   end-1 = 1
      if ((sp[i].direction == 1) && (sp[i+1].direction == 1))  
	{
	  ind = NUM_VIA*i + 3;
	  ocp.subjectTo( options.NumbIntervalsOpt - offset, phi(i)   - pv(ind) == 0 );
	  ocp.subjectTo(                            offset, phi(i+1) - pv(ind) == 0 );
	}
            
      // if both stages "around" the i-th via region are with backward motion
      // * <---- * <---- *
      //       1 = end-1
      if ((sp[i].direction == -1) && (sp[i+1].direction == -1))  
	{
	  ind = NUM_VIA*i + 3;
	  ocp.subjectTo(                            offset, phi(i)   - pv(ind) == 0 );
	  ocp.subjectTo( options.NumbIntervalsOpt - offset, phi(i+1) - pv(ind) == 0 );
	}
    }
  
  // ==============================================================================  
  // DEFINE INEQUALITY CONSTRAINTS
  // ==============================================================================  
  for (int i=0; i<N; i++) // phi
    ocp.subjectTo(phiAllStages.lb <= phi(i) <= phiAllStages.ub);

  for (int i=0; i<N; i++) // stage parameters
    {
      ocp.subjectTo(sp[i].a0_x.lb <= p(NUM_PARAM*i+0) <= sp[i].a0_x.ub);
      ocp.subjectTo(sp[i].a0_y.lb <= p(NUM_PARAM*i+1) <= sp[i].a0_y.ub);
      ocp.subjectTo(sp[i].aT_x.lb <= p(NUM_PARAM*i+2) <= sp[i].aT_x.ub);
      ocp.subjectTo(sp[i].aT_y.lb <= p(NUM_PARAM*i+3) <= sp[i].aT_y.ub);
      ocp.subjectTo(sp[i].k_0.lb  <= p(NUM_PARAM*i+4) <= sp[i].k_0.ub);
      ocp.subjectTo(sp[i].k_T.lb  <= p(NUM_PARAM*i+5) <= sp[i].k_T.ub);
    }

  for (int i=0; i<N-1; i++) // via parameters
    {
      ocp.subjectTo(vr[i].x.lb     <= pv(NUM_VIA*i+0) <= vr[i].x.ub); 
      ocp.subjectTo(vr[i].y.lb     <= pv(NUM_VIA*i+1) <= vr[i].y.ub); 
      ocp.subjectTo(vr[i].theta.lb <= pv(NUM_VIA*i+2) <= vr[i].theta.ub);
      ocp.subjectTo(vr[i].phi.lb   <= pv(NUM_VIA*i+3) <= vr[i].phi.ub);
    }

  // ==============================================================================  
  // DEFINE AN OPTIMIZATION ALGORITHM AND SOLVE THE OCP:
  // ==============================================================================  
#ifdef OPTIMIZATION_ALGORITHM
  OptimizationAlgorithm algorithm(ocp);
#else
  ParameterEstimationAlgorithm algorithm(ocp);
#endif

  // ==============================================================================  
  // DEFINE PARAMETERS FOR THE OPTIMIZATION ALGORITHM:
  // ==============================================================================  
  algorithm.set( INTEGRATOR_TYPE, INT_RK78 );
  algorithm.set( KKT_TOLERANCE, options.KKT_tol);
  
  if (options.DisplayFlag)
    {  
      algorithm.set( PRINTLEVEL, HIGH );
      //algorithm.set( PRINT_SCP_METHOD_PROFILE, YES);
    }
  else
    {
      algorithm.set( PRINTLEVEL, NONE );
    }

  algorithm.set( MAX_NUM_ITERATIONS, options.MaxNumbIter );
  algorithm.set (PRINT_COPYRIGHT, NO);
  
  algorithm.set( GLOBALIZATION_STRATEGY, GS_FULLSTEP );
  algorithm.set( LEVENBERG_MARQUARDT, 1e-6 );

  // ==============================================================================  
  // DEFINE INITIAL GUESS:
  // ==============================================================================  
  if (options.InitialGuess == 0)
    {
      printf("\n --- using auto initialization --- \n\n");
    }
  else if (options.InitialGuess == 1)
    {
      printf("\n --- initialize using ACADO data structures --- \n\n");

      Grid TimeGrid(0, 1, options.NumbPointsOpt);
      VariablesGrid ParametersInit(n+NUM_VIA*(N-1), TimeGrid);

      for (int i=0; i<N; i++)
	{
	  ParametersInit(0,NUM_PARAM*i+0) = sp[i].a0_x.guess; 
	  ParametersInit(0,NUM_PARAM*i+1) = sp[i].a0_y.guess; 
	  ParametersInit(0,NUM_PARAM*i+2) = sp[i].aT_x.guess; 
	  ParametersInit(0,NUM_PARAM*i+3) = sp[i].aT_y.guess; 
	  ParametersInit(0,NUM_PARAM*i+4) = sp[i].k_0.guess; 
	  ParametersInit(0,NUM_PARAM*i+5) = sp[i].k_T.guess; 
	}

      for (int i=0; i<N-1; i++)
	{
	  ParametersInit(0,n+NUM_VIA*i+0) = vr[i].x.guess;
	  ParametersInit(0,n+NUM_VIA*i+1) = vr[i].y.guess;
	  ParametersInit(0,n+NUM_VIA*i+2) = vr[i].theta.guess;
	  ParametersInit(0,n+NUM_VIA*i+3) = vr[i].phi.guess;
	}
      algorithm.initializeParameters(ParametersInit);
    }
  else
    {
      printf("ERROR: we should not be here. \n");
    } 

  // ==============================================================================  
  // SOLVE THE OCP:
  // ==============================================================================  
  double clock_start = clock();
  status = algorithm.solve();
  double clock_end = clock();

  if (options.DisplayFlag == 1)
    printf(" status = %i (time = %f)\n\n", status,(clock_end-clock_start)/CLOCKS_PER_SEC);

  // ==============================================================================  
  // OUTPUT SOLUTION:
  // ==============================================================================  
  VariablesGrid parameters, states;
  algorithm.getParameters(parameters);
  algorithm.getDifferentialStates(states);
  
  
  for (int k=0; k<N; k++) // output profile from optimization 
    {
      if (sp[k].direction == 1) // forward motion
	{
	  for (int i=0; i<options.NumbPointsOpt; i++) 
	    {
	      out[k].set_phi_opt(i, states(i,k));
	      out[k].set_ArcLength_opt(i, states.getTime(i));
	    }
	}
      else // backward motion
	{
	  for (int i=0; i<options.NumbPointsOpt; i++) 
	    {
	      out[k].set_phi_opt(i, states(options.NumbPointsOpt-1-i,k));
	      out[k].set_ArcLength_opt(i, 1 - states.getTime(options.NumbPointsOpt-1-i));
	    }	  
	}
    }
  
  for (int i=0; i<N; i++) // stage parameters
    {    
      sp[i].a0_x.set_solution(parameters(0,NUM_PARAM*i+0));
      sp[i].a0_y.set_solution(parameters(0,NUM_PARAM*i+1));
      sp[i].aT_x.set_solution(parameters(0,NUM_PARAM*i+2));
      sp[i].aT_y.set_solution(parameters(0,NUM_PARAM*i+3));
      sp[i].k_0.set_solution (parameters(0,NUM_PARAM*i+4));
      sp[i].k_T.set_solution (parameters(0,NUM_PARAM*i+5));
    }

  for (int i=0; i<N-1; i++) // via regions
    {
      vr[i].x.set_solution    (parameters(0,n + NUM_VIA*i + 0));
      vr[i].y.set_solution    (parameters(0,n + NUM_VIA*i + 1));
      vr[i].theta.set_solution(parameters(0,n + NUM_VIA*i + 2));
      vr[i].phi.set_solution  (parameters(0,n + NUM_VIA*i + 3));
    }

  // ==============================================================================  
  // CLEAR STATIC COUNTERS:
  // ==============================================================================  
  phi.clearStaticCounters();
  p.clearStaticCounters();
  pv.clearStaticCounters();
  // ==============================================================================  
}

void Maneuver::solve(int exit_flag)
{
  out.resize(N, StageOutput(options.NumbPointsOpt,options.NumbPointsSim));

#ifdef OPTIMIZATION_ALGORITHM
  printf("\n --- Using ACADO::OptimizationAlgorithm --- \n");
#else
  printf("\n --- Using ACADO::ParameterEstimationAlgorithm --- \n");
#endif
  
  plan();
  
  if (status)
    printf("ERROR while solving the OCP\n");
  
  if (status && exit_flag)
    {
      printf(" --- Exit ---\n");
      exit(0);
    }

  if (status && !exit_flag)
    printf(" --- Continue anyway ---\n");      

  integrate();
  form_pose();
}

//EOF


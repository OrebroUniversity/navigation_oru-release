#include "ODE_OneStage_sim.cpp"

/** \brief Forms the ODE used for simulation

    \param[in] m A maneuver
    \param[in] t TIME
    \param[in] phi differential states
    \param[out] f RHS of ODE
    \return void
*/
void ODE_sim( Maneuver &m, 
	      ACADO::TIME &t,
	      ACADO::DifferentialState &phi,
	      ACADO::DifferentialEquation &f )
{
  ACADO::IntermediateState a, b, c; 

  int i = m.ode_index;
  
  if (m.N == 1) // a single stage
    {	  
      if (m.sp[0].direction == 1) // forward motion
	{
	  ODE_OneStage_sim( t, 
			    m.sp[i].a0_x.solution,
			    m.sp[i].a0_y.solution,
			    m.sp[i].aT_x.solution,
			    m.sp[i].aT_y.solution,
			    m.sp[i].k_0.solution,
			    m.sp[i].k_T.solution,
			    m.s0.x,             
			    m.s0.y,
			    m.s0.theta,
			    m.s1.x,
			    m.s1.y,
			    m.s1.theta,
			    m.L1, m.L2, 
			    a, b, c);
	}
      else // backward motion
	{
	  ODE_OneStage_sim( t, 
			    m.sp[i].a0_x.solution,
			    m.sp[i].a0_y.solution,
			    m.sp[i].aT_x.solution,
			    m.sp[i].aT_y.solution,
			    m.sp[i].k_0.solution,
			    m.sp[i].k_T.solution,
			    m.s1.x,             
			    m.s1.y,
			    m.s1.theta,
			    m.s0.x,
			    m.s0.y,
			    m.s0.theta,
			    m.L1, m.L2, 
			    a, b, c);
	}
      f << dot(phi) == a*sin(phi) + b*cos(phi) + c;
    }
  else // multiple stages
    {
      if (i == 0)
	{
	  if (m.sp[0].direction == 1) // forward motion
	    {
	      ODE_OneStage_sim( t, 
				m.sp[i].a0_x.solution,
				m.sp[i].a0_y.solution,
				m.sp[i].aT_x.solution,
				m.sp[i].aT_y.solution,
				m.sp[i].k_0.solution,
				m.sp[i].k_T.solution,
				m.s0.x, 
				m.s0.y, 
				m.s0.theta, 
				m.vr[i].x.solution,
				m.vr[i].y.solution,
				m.vr[i].theta.solution,
				m.L1, m.L2, 
				a, b, c);
	    }
	  else // backward motion
	    {
	      ODE_OneStage_sim( t, 
				m.sp[i].a0_x.solution,
				m.sp[i].a0_y.solution,
				m.sp[i].aT_x.solution,
				m.sp[i].aT_y.solution,
				m.sp[i].k_0.solution,
				m.sp[i].k_T.solution,
				m.vr[i].x.solution,
				m.vr[i].y.solution,
				m.vr[i].theta.solution,				     
				m.s0.x, 
				m.s0.y, 
				m.s0.theta, 
				m.L1, m.L2, 
				a, b, c);
	    }
	  f << dot(phi) == a*sin(phi) + b*cos(phi) + c;
	}
      
      if ( (i > 0) && (i < m.N-1) )
	{
	  if (m.sp[i].direction == 1) // forward motion
	    {
	      ODE_OneStage_sim( t, 
				m.sp[i].a0_x.solution,
				m.sp[i].a0_y.solution,
				m.sp[i].aT_x.solution,
				m.sp[i].aT_y.solution,
				m.sp[i].k_0.solution,
				m.sp[i].k_T.solution,
				m.vr[i-1].x.solution,
				m.vr[i-1].y.solution,
				m.vr[i-1].theta.solution,
				m.vr[i].x.solution,
				m.vr[i].y.solution,
				m.vr[i].theta.solution,
				m.L1, m.L2, 
				a, b, c);
	    }
	  else // backward motion
	    {
	      ODE_OneStage_sim( t, 
				m.sp[i].a0_x.solution,
				m.sp[i].a0_y.solution,
				m.sp[i].aT_x.solution,
				m.sp[i].aT_y.solution,
				m.sp[i].k_0.solution,
				m.sp[i].k_T.solution,
				m.vr[i].x.solution,
				m.vr[i].y.solution,
				m.vr[i].theta.solution,
				m.vr[i-1].x.solution,
				m.vr[i-1].y.solution,
				m.vr[i-1].theta.solution,
				m.L1, m.L2, 
				a, b, c);
	    }
	  f << dot(phi) == a*sin(phi) + b*cos(phi) + c;
	}
      
      if (i == m.N-1)
	{
	  if (m.sp[i].direction == 1) // forward motion
	    {
	      ODE_OneStage_sim( t, 
				m.sp[i].a0_x.solution,
				m.sp[i].a0_y.solution,
				m.sp[i].aT_x.solution,
				m.sp[i].aT_y.solution,
				m.sp[i].k_0.solution,
				m.sp[i].k_T.solution,
				m.vr[i-1].x.solution,
				m.vr[i-1].y.solution,
				m.vr[i-1].theta.solution,
				m.s1.x, 
				m.s1.y,
				m.s1.theta, 
				m.L1, m.L2, 
				a, b, c);
	    }
	  else
	    {
	      ODE_OneStage_sim( t, 
				m.sp[i].a0_x.solution,
				m.sp[i].a0_y.solution,
				m.sp[i].aT_x.solution,
				m.sp[i].aT_y.solution,
				m.sp[i].k_0.solution,
				m.sp[i].k_T.solution,
				m.s1.x, 
				m.s1.y,
				m.s1.theta, 
				m.vr[i-1].x.solution,
				m.vr[i-1].y.solution,
				m.vr[i-1].theta.solution,
				     
				m.L1, m.L2, 
				a, b, c);
	    }
	  f << dot(phi) == a*sin(phi) + b*cos(phi) + c;
	}
    }
}

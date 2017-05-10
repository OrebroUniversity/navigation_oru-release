#include "ODE_OneStage_opt.cpp"

/** \brief Forms the ODE used for optimization 

    \param[in] m A maneuver
    \param[in] t TIME
    \param[in] phi differential states
    \param[in] p stage parameters
    \param[in] pv parameters of the vaia regions
    \param[out] f RHS of ODE
    \return void
*/
void ODE_opt( Maneuver &m, 
	      ACADO::TIME &t,
	      ACADO::DifferentialState &phi,
	      ACADO::Parameter &p,
	      ACADO::Parameter &pv,
	      ACADO::DifferentialEquation &f )
{
  ACADO::IntermediateState a, b, c; 

  for (int i=0; i<m.N; i++)
    {
      if (m.N == 1) // a single stage
	{
	  if (m.sp[0].direction == 1) // forward motion
	    {
	      ODE_OneStage_opt( t, 
				p(NUM_PARAM*i + 0), 
				p(NUM_PARAM*i + 1), 
				p(NUM_PARAM*i + 2), 
				p(NUM_PARAM*i + 3),
				p(NUM_PARAM*i + 4),
				p(NUM_PARAM*i + 5),
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
	      ODE_OneStage_opt( t, 
				p(NUM_PARAM*i + 0), 
				p(NUM_PARAM*i + 1), 
				p(NUM_PARAM*i + 2), 
				p(NUM_PARAM*i + 3),
				p(NUM_PARAM*i + 4),
				p(NUM_PARAM*i + 5),
				m.s1.x,             
				m.s1.y,
				m.s1.theta,
				m.s0.x,
				m.s0.y,
				m.s0.theta,
				m.L1, m.L2, 
				a, b, c);
	    }
	  
	  f << dot(phi(i)) == a*sin(phi(i)) + b*cos(phi(i)) + c;
	}
      else // multiple stages
	{
	  if (i == 0)
	    {
	      if (m.sp[0].direction == 1) // forward motion
		{
		  ODE_OneStage_opt( t, 
				    p(NUM_PARAM*i + 0), 
				    p(NUM_PARAM*i + 1), 
				    p(NUM_PARAM*i + 2), 
				    p(NUM_PARAM*i + 3), 
				    p(NUM_PARAM*i + 4), 
				    p(NUM_PARAM*i + 5), 
				    m.s0.x, 
				    m.s0.y, 
				    m.s0.theta, 
				    pv(NUM_VIA*i + 0), 
				    pv(NUM_VIA*i + 1),
				    pv(NUM_VIA*i + 2), 
				    m.L1, m.L2, 
				    a, b, c);
		}
	      else // backward motion
		{
		  ODE_OneStage_opt( t, 
				    p(NUM_PARAM*i + 0), 
				    p(NUM_PARAM*i + 1), 
				    p(NUM_PARAM*i + 2), 
				    p(NUM_PARAM*i + 3), 
				    p(NUM_PARAM*i + 4), 
				    p(NUM_PARAM*i + 5), 
				    pv(NUM_VIA*i + 0), 
				    pv(NUM_VIA*i + 1),
				    pv(NUM_VIA*i + 2), 
				    m.s0.x, 
				    m.s0.y, 
				    m.s0.theta, 
				    m.L1, m.L2, 
				    a, b, c);
		}

	      f << dot(phi(i)) == a*sin(phi(i)) + b*cos(phi(i)) + c;
	    }
	  
	  if ( (i > 0) && (i < m.N-1) )
	    {
	      if (m.sp[i].direction == 1) // forward motion
		{
		  ODE_OneStage_opt( t, 
				    p(NUM_PARAM*i + 0),     // a0_x
				    p(NUM_PARAM*i + 1),     // a0_y
				    p(NUM_PARAM*i + 2),     // aT_x
				    p(NUM_PARAM*i + 3),     // aT_y
				    p(NUM_PARAM*i + 4),     // k_0
				    p(NUM_PARAM*i + 5),     // k_T
				    pv(NUM_VIA*(i-1) + 0),  // x via     (stage start)
				    pv(NUM_VIA*(i-1) + 1),  // y via     (stage start)
				    pv(NUM_VIA*(i-1) + 2),  // theta via (stage start)
				    pv(NUM_VIA*i + 0),      // x via     (stage end)
				    pv(NUM_VIA*i + 1),      // y via     (stage end)
				    pv(NUM_VIA*i + 2),      // theta via (stage end)
				    m.L1, m.L2, 
				    a, b, c);
		}
	      else // backward motion
		{
		  ODE_OneStage_opt( t, 
				    p(NUM_PARAM*i + 0),     
				    p(NUM_PARAM*i + 1),     
				    p(NUM_PARAM*i + 2),     
				    p(NUM_PARAM*i + 3),     
				    p(NUM_PARAM*i + 4),     
				    p(NUM_PARAM*i + 5),     
				    pv(NUM_VIA*i + 0),      
				    pv(NUM_VIA*i + 1),      
				    pv(NUM_VIA*i + 2),      
				    pv(NUM_VIA*(i-1) + 0),  
				    pv(NUM_VIA*(i-1) + 1),  
				    pv(NUM_VIA*(i-1) + 2),  
				    m.L1, m.L2, 
				    a, b, c);
		}
	      
	      f << dot(phi(i)) == a*sin(phi(i)) + b*cos(phi(i)) + c;
	    }
	  
	  if (i == m.N-1)
	    {
	      if (m.sp[i].direction == 1) // forward motion
		{
		  ODE_OneStage_opt( t, 
				    p(NUM_PARAM*i + 0), 
				    p(NUM_PARAM*i + 1), 
				    p(NUM_PARAM*i + 2), 
				    p(NUM_PARAM*i + 3), 
				    p(NUM_PARAM*i + 4), 
				    p(NUM_PARAM*i + 5), 
				    pv(NUM_VIA*(i-1) + 0), 
				    pv(NUM_VIA*(i-1) + 1), 
				    pv(NUM_VIA*(i-1) + 2), 
				    m.s1.x, 
				    m.s1.y,
				    m.s1.theta, 
				    m.L1, m.L2, 
				    a, b, c);
		}
	      else // backward motion
		{
		  ODE_OneStage_opt( t, 
				    p(NUM_PARAM*i + 0), 
				    p(NUM_PARAM*i + 1), 
				    p(NUM_PARAM*i + 2), 
				    p(NUM_PARAM*i + 3), 
				    p(NUM_PARAM*i + 4), 
				    p(NUM_PARAM*i + 5), 
				    m.s1.x, 
				    m.s1.y,
				    m.s1.theta, 
				    pv(NUM_VIA*(i-1) + 0), 
				    pv(NUM_VIA*(i-1) + 1), 
				    pv(NUM_VIA*(i-1) + 2), 
				    m.L1, m.L2, 
				    a, b, c);
		}
	      f << dot(phi(i)) == a*sin(phi(i)) + b*cos(phi(i)) + c;
	    }
	}
    }
}

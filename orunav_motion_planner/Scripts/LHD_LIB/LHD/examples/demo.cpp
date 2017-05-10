// Time-stamp: <2012-06-04 11:43:21 (drdv)>

#include "../include/LHD_utility.h"

// end headers

int main()
{ 

  const int NM = 5; // number of maneuvers

  // ----------------------------------------------------------
  // Define NM maneuvers
  // ----------------------------------------------------------
  vector<Maneuver> m(NM, Maneuver(1.8,2));    
  
  // Maneuver 0 (one stage)
  // =======================
  m[0].set_options("NumbIntervalsOpt", 50);

  m[0].set_N(1);
  m[0].set_bounds_sp(-50, 50, 0.1, 50);
  m[0].set_initial_state( 96.0,  65.7,  M_PI,   0.0 );
  m[0].set_final_state  ( 70.0,  55.0,  M_PI,   0.0 );

  m[0].set_poly_default(); // should not set more options for m[0] after this

  // Maneuver 1 (one stage)
  // =======================
  m[1].set_options("NumbIntervalsOpt", 50);

  m[1].set_N(1); 
  m[1].set_bounds_sp(-50, 50, 0.1, 50);
  m[1].set_initial_state( 70.0,  55.0,  M_PI,   0.0 );
  m[1].set_final_state  ( 50.0,  55.0,  M_PI,   0.0 );

  m[1].set_poly_default(); 

  // Maneuver 2 (one stage)
  // =======================
  m[2].set_options("NumbIntervalsOpt", 50);

  m[2].set_N(1); 
  m[2].set_bounds_sp(-50, 50, 0.1, 50);
  m[2].set_initial_state( 50.0,  55.0,  M_PI,   0.0 );
  m[2].set_final_state  ( 30.0,  80.0,  M_PI_2, 0.0 );

  m[2].set_poly_default(); 

  // Maneuver 3 (two stages)
  // =======================
  m[3].set_options("NumbIntervalsOpt", 50);

  m[3].set_N(2); 
  m[3].set_bounds_sp(-50, 50, 0.1, 50);
  m[3].set_initial_state( 30.0,  80.0,  M_PI_2, 0.0 );
  m[3].set_final_state  (  0.0, 110.0,  M_PI,   0.0 );

  // define via region
  m[3].vr[0].set_xy( 15.0, 16.0, 91, 94 );

  m[3].set_poly_default(); 

  // Maneuver 4 (one stage)
  // =======================
  m[4].set_options("NumbIntervalsOpt", 50);

  m[4].set_N(1); 
  m[4].set_bounds_sp(-50, 50, 0.1, 50);
  m[4].set_initial_state(  0.0, 110.0, M_PI, 0.0 );
  m[4].set_final_state  (-10.0, 110.0, M_PI, 0.0 );

  m[4].set_poly_default(); 

  // end problem definition

  // ----------------------------------------------------------
  // Solve
  // ----------------------------------------------------------
  for (int k=0; k<NM; k++)
    m[k].solve();
  
  // end problem solution

  // ----------------------------------------------------------
  // Output
  // ----------------------------------------------------------
  ManeuversOutput(NM, m, 
		  "../Matlab",
		  "phi_opt.txt",
		  "phi_sim.txt",
		  "info_maneuvers");
  
  return 0;
}

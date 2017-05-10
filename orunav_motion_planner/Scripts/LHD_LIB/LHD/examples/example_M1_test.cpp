// Time-stamp: <2012-07-01 07:23:51 (drdv)>

#include "../include/LHD_utility.h"

int main()
{ 
    
  Maneuver m(1.6, 1.6); // Define NM maneuvers
  
  m.set_N(1);
  m.set_bounds_sp(-50, 50, 0.1, 20);
  m.set_initial_state( 5, 5, 0.0, 0.0 );  
  m.set_final_state  ( 15, 5, 0.0, 0.0 ); 


  m.set_options("DisplayFlag", 1);

  m.set_poly_default(); 
  //m.set_poly(0,  0, 4, 0.0, 0.0);
  
  m.solve(); // Solve
  
  // Output
  ManeuverOutput(m,
		 "../Matlab",
		 "phi_opt.txt",
		 "phi_sim.txt",
		 "info_maneuvers");

  return 0;
}

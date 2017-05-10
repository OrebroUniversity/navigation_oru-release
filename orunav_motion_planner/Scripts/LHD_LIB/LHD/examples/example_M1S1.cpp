// Time-stamp: <2012-07-01 07:23:51 (drdv)>

#include "../include/LHD_utility.h"

int main()
{ 
    
  Maneuver m(0.36, 0.4); // Define NM maneuvers
  
  m.set_N(1);
  m.set_bounds_sp(-50, 50, 0.1, 20);
  m.set_initial_state( 0.2, -1.0, 0.2, 0.7 );  
  m.set_final_state  ( 4.0, -3.0, 0.4, 0.0 ); 

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

// Time-stamp: <2012-07-10 08:39:17 (drdv)>

#include "../include/LHD_utility.h"

int main()
{ 
    
  Maneuver m(0.36, 0.4); // Define NM maneuvers

  //m.set_options("ObjFun", 1);
  //m.set_options("MaxNumbIter", 1000);
  //m.set_options("NumbPointsSim", 50);

  m.set_N(2);
  m.set_bounds_sp(-50, 50, 0.1, 20);

  // ------------------------------------------------------------

  m.set_initial_state(  3.0,  3.0,  M_PI/4,  0.0 );  
  m.set_final_state  ( -3.0,  -3.0,  M_PI/4,  0.0 ); 

  m.vr[0].set_xy( 2.5, -2.5, 0.5 );
  m.vr[0].set_theta(M_PI/4, 0.4);

  // ------------------------------------------------------------

  m.sp[0].direction = -1; // B
  m.sp[1].direction = -1; // B

  m.set_poly_default(); 
 
  // ------------------------------------------------------------

  m.solve(); // Solve
  
  ManeuverOutput(m); // Output

  // ------------------------------------------------------------
  
  return 0;
}

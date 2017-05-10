// Time-stamp: <2012-07-02 14:51:43 (drdv)>

#include "../include/LHD_utility.h"

int main()
{ 
    
  Maneuver m(0.36, 0.4); // Define NM maneuvers

  //m.set_options("ObjFun", 1);
  //m.set_options("MaxNumbIter", 1000);

  m.set_N(3);
  m.set_bounds_sp(-50, 50, 0.1, 20);

  // ------------------------------------------------------------

  m.set_initial_state( 0.2, -1.0,  0.2,  0.7 );  
  m.set_final_state  ( 6.0, -2.0,  0.4,  0.0 ); 

  m.vr[0].set_xy( 5.0, 6.0, -1.0, 0.0 );
  m.vr[0].set_theta(M_PI/4, 0.4);

  m.vr[1].set_xy( 2.0, 3.0, -3.0, -2.0 );
  m.vr[1].set_theta(0, 0.01);

  // ------------------------------------------------------------

  m.sp[0].direction =  1; // F
  m.sp[1].direction = -1; // B
  m.sp[2].direction =  1; // F

  m.set_poly_default(); 
 
  // ------------------------------------------------------------

  m.solve(); // Solve
  
  ManeuverOutput(m); // Output

  // ------------------------------------------------------------
  
  return 0;
}

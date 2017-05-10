// Time-stamp: <2012-07-02 14:46:07 (drdv)>

#include "../include/LHD_utility.h"

int main()
{ 
  // define the maneuver, with the dimension of the truck
  Maneuver m(0.36, 0.4); // Define NM maneuvers

  //m.set_options("ObjFun", 1);
  //m.set_options("MaxNumbIter", 1000);

  // number of stages: 3 means initial, goal and 2 via regions 
  m.set_N(3);
  // bounds on stage maneuvers parameters
  m.set_bounds_sp(-50, 50, 0.1, 20);

  // ------------------------------------------------------------

  m.set_initial_state( 0.2, -1.0,  0.2,   0.7 );  
  m.set_final_state  ( 3.0, -1.0,  M_PI, -0.4 ); 

  // via regions
  m.vr[0].set_xy( 3.0, 4.0, 3.0, 4.0 );
  m.vr[0].set_theta(M_PI_2, 0.4);

  m.vr[1].set_xy( 7.0, 8.0, 0.0, 1.0 );
  m.vr[1].set_theta(M_PI, 0.4);

  // ------------------------------------------------------------

  m.sp[0].direction =  1; // F
  m.sp[1].direction = -1; // B
  m.sp[2].direction =  1; // F

  /*
  m.set_poly(0,  0, 10, 0.0, 0.0);
  m.set_poly(1, 10, 60, 0.05, 0.0);
  m.set_poly(2, 60, 90, 0.0, 0.0);
  */
  m.set_poly_default(); 
 
  // ------------------------------------------------------------

  m.solve(); // Solve
  
  ManeuverOutput(m); // Output

  // ------------------------------------------------------------
  
  return 0;
}

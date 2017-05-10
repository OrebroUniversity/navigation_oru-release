// Time-stamp: <2012-06-29 14:04:16 (drdv)>

#include "../include/LHD_utility.h"

int main()
{ 
    
  Maneuver m(0.36, 0.4); // Define NM maneuvers
  
  m.set_N(3);
  m.set_bounds_sp(-50, 50, 0.1, 20);
  m.set_initial_state(  0.2, -1.0,  0.2,  0.7 );  
  m.set_final_state  (  5.0, -1.0,  M_PI, 0.0 ); 

  m.vr[0].set_xy( 6.0, 7.0, 3.0, 4.0 );
  m.vr[1].set_xy( 8.0, 9.0, 0.0, 1.0 );

  // ------------------------------------------------------------
  //m.set_poly_default(); 

  m.set_poly(0,  0, 10, 0.0, 0.1);
  m.set_poly(1, 10, 30, 0.1, 0.1);
  m.set_poly(2, 30, 40, 0.1, 0.0);

  /*
  for (int k=0; k<3; k++)
    cout << m.sp[k].direction << endl;
  */

  // ------------------------------------------------------------

  m.solve(); // Solve
  
  ManeuverOutput(m); // Output
  
  return 0;
}

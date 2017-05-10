/*
   SINGLE STAGE MANEUVER - PARALLELIZABLE

   --------------------------------------------------------
   The following command line arguments should be provided:
   --------------------------------------------------------
   argv[1]  - L1 (front axel -- joint)	(double)
   argv[2]  - L2 (back axel -- joint)	(double)

   argv[3]  - phi_max					(double)

   argv[4]  - x     initial    			(double)
   argv[5]  - y     initial    			(double)
   argv[6]  - theta initial    			(double)
   argv[7]  - phi   initial    			(double)

   argv[8]  - x     desired    			(double)
   argv[9]  - y     desired    			(double)
   argv[10] - theta desired    			(double)
   argv[11] - phi   desired    			(double)

   argv[12] - ObjFun           			(int)
   argv[13] - Direction (1:Forw;-1:Back)	(int)

   argv[14] - File output    			(char*)
   --------------------------------------------------------
   */

#include "../include/LHD_utility.h"
#include <cstdlib>

int main(int argc, char** argv) {

  Maneuver m(1.6, 1.6); // Define NM maneuvers

  m.set_options("ObjFun", 1);
  m.set_options("NumbPointsSim", 1000);
  m.set_options("DisplayFlag", 1);

  m.phiAllStages.set(-0.6,0.6);
  m.set_N(1);

  m.set_initial_state(  0, 0,  0,  0.0  );  
  // m.set_final_state  (  9, 9,  1.57,  -0.39 ); 
  // m.set_final_state  (  10, 10,  1.57,  -0.39 ); 
  m.set_final_state  (  10, 1,  0.0,  0.0 ); 

  m.sp[0].direction = 1;

  // set the stage parameters' bounds
  double a_low = -50;
  double a_up = 50;
  double k_low = 0.1;
  double k_up = 20;

  // max iteration counter
  int counter = 20;

  // ----------------- Solve cycle -----------------
  while (m.status != 0 && counter > 0) {
    m.set_bounds_sp(a_low, a_up, k_low, k_up);
    m.set_poly_default();

    m.solve(0); // Solve

    // change the stage parameters' bounds
    a_low -= 10;
    a_up += 10;
    k_up += 10;
    counter --;
  }

  // ----------------- Output -----------------
  // Output, only if the problem has been solved
  if (m.status == 0)
    ManeuverOutput(m,
	"../Matlab",
	"phi_opt.txt",
	"phi_sim.txt",
	"info_maneuvers");
  return 0;
}

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

  Maneuver m(atof(argv[1]), atof(argv[2])); // Define NM maneuvers

  m.set_options("ObjFun", atoi(argv[12]));
  m.set_options("NumbPointsSim", 1000);
  m.set_options("DisplayFlag", 0);

  m.phiAllStages.set(-atof(argv[3]),atof(argv[3]));
  m.set_N(1);

  // ------------------------------------------------------------

  m.set_initial_state( atof(argv[4]), atof(argv[5]), atof(argv[6]),  atof(argv[7]));
  m.set_final_state  ( atof(argv[8]), atof(argv[9]), atof(argv[10]), atof(argv[11]));

  // ------------------------------------------------------------

  m.sp[0].direction = atoi(argv[13]);

  // output file
  char outputFile[200];
  strcpy(outputFile, "");
  strcat(outputFile, argv[14]);

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
    k_up += 20;
    counter --;
  }

  // ----------------- Output -----------------
  if (m.status == 0) {
    FILE *file_op = fopen(outputFile, "w");
    if (!file_op) {
      std::cerr << "Cannot open file (for writing) " << outputFile << std::endl;
      std::exit(1);
    }
    for (int i = 0; i < m.out[0].NumbPointsSim; i++)
      fprintf(
	  file_op,
	  "%4.4f  %4.4f  %4.4f  %4.4f \n",
	  m.out[0].state[i].x,
	  m.out[0].state[i].y,
	  m.out[0].state[i].theta,
	  m.out[0].state[i].phi);
    fclose(file_op);
  }

  return 0;

}

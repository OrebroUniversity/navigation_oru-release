// Time-stamp: <2012-06-27 14:56:09 (drdv)>

#include "../include/LHD_utility.h"

int main()
{ 

  const int NM = 5; // number of maneuvers

  // bounds on stage parameters
  double lb_a = -50;
  double ub_a =  50;
  double lb_k =  0.1;
  double ub_k =  50;

  // ----------------------------------------------------------
  // Define NM maneuvers (with 6 stages in total)
  // ----------------------------------------------------------
  vector<Maneuver> m(NM, Maneuver(1.8,2));    
  
  // Maneuver 0 (one stage)
  // =======================
  m[0].set_N(1);
  m[0].set_bounds_sp(lb_a, ub_a, lb_k, ub_k);
  m[0].set_initial_state( 96.0,  65.7,  M_PI,   0.0 );
  m[0].set_final_state  ( 70.0,  55.0,  M_PI,   0.0 );

  m[0].set_poly_default(); 

  // Maneuver 1 (one stage)
  // =======================
  m[1].set_N(1); 
  m[1].set_bounds_sp(lb_a, ub_a, lb_k, ub_k);
  m[1].set_initial_state( 70.0,  55.0,  M_PI,   0.0 );
  m[1].set_final_state  ( 50.0,  55.0,  M_PI,   0.0 );

  m[1].set_poly_default(); 

  // Maneuver 2 (one stage)
  // =======================
  m[2].set_N(1); 
  m[2].set_bounds_sp(lb_a, ub_a, lb_k, ub_k);
  m[2].set_initial_state( 50.0,  55.0,  M_PI,   0.0 );
  m[2].set_final_state  ( 30.0,  80.0,  M_PI_2, 0.0 );

  m[2].set_poly_default(); 

  // Maneuver 3 (two stages)
  // =======================
  m[3].set_N(2); 
  m[3].set_bounds_sp(lb_a, ub_a, lb_k, ub_k);
  m[3].set_initial_state( 30.0,  80.0,  M_PI_2, 0.0 );
  m[3].set_final_state  (  0.0, 110.0,  M_PI,   0.0 );

  // define via region
  m[3].vr[0].set_xy( 13.0, 17.0, 88, 92 );

  m[3].set_poly_default(); 

  // Maneuver 4 (one stage)
  // =======================
  m[4].set_N(1); 
  m[4].set_bounds_sp(lb_a, ub_a, lb_k, ub_k);
  m[4].set_initial_state(  0.0, 110.0, M_PI, 0.0 );
  m[4].set_final_state  (-10.0, 110.0, M_PI, 0.0 );

  m[4].set_poly_default(); 

  /*
  for (int i=0; i<NM; i++)
    {
      m[i].print();
      getchar();
    }
  */
  // ----------------------------------------------------------

  // Solve
  for (int k=0; k<NM; k++)
    {
      m[k].solve(); 
      //m[k].print_solution();
    }

  ManeuversOutput(NM, m); // Output

  // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  // plan all stages in one maneuver
  // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  
  cout << "=================================================" << endl;
  cout << "ALL AT ONCE" << endl;
  cout << "=================================================" << endl;

  Maneuver ma(1.8,2);    
  
  /*
  ma.set_options("NumbIntervalsOpt", 20    );
  ma.set_options("ObjFun"          ,  1    );
  ma.set_options("KKT_tol"         , 1e-5 );
  ma.set_options("MaxNumbIter"     , 1000  );
  */

  //ma.options.print();

  ma.set_N(6);
  ma.set_bounds_sp(lb_a, ub_a, lb_k, ub_k);
  ma.set_initial_state( 96.0,  65.7,  M_PI,  0.0 );
  ma.set_final_state  (-10.0, 110.0,  M_PI,  0.0 );

  ma.vr[0].set_xy(70, 55, 2);
  ma.vr[0].set_theta(M_PI, 0.4);

  ma.vr[1].set_xy(50, 55, 2);
  ma.vr[1].set_theta(M_PI, 0.4);

  ma.vr[2].set_xy(30, 80, 2);
  ma.vr[2].set_theta(M_PI_2, 0.4);

  ma.vr[3].set_xy(15, 90, 2);
  ma.vr[3].set_theta(M_PI, 0.4);

  ma.vr[4].set_xy(0, 110, 2);
  ma.vr[4].set_theta(M_PI, 0.4);

  ma.set_poly_default(); 

  // ------------------
  
  GuessStageParameters(NM, m, ma); // set initial guess     
  ma.solve(); // solve

  //ma.print_solution();
  
  ManeuverOutput(ma, 
		 "../Matlab",
		 "phi_opt_all.txt",
		 "phi_sim_all.txt",
		 "info_maneuvers_all");
  
  return 0;
}

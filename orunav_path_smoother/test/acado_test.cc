#include <boost/program_options.hpp>
#include <iostream>
#include <fstream>

#include <acado_toolkit.hpp>
#include <acado/bindings/acado_gnuplot/gnuplot_window.hpp>

namespace po = boost::program_options;
using namespace std;

int main(int argc, char **argv)
{
  USING_NAMESPACE_ACADO
  po::options_description desc("Allowed options");
  double sx, sy, sth, sphi, ex, ey, eth, ephi;
  desc.add_options()
    ("help", "produce help message")
    ("sx", po::value<double>(&sx)->default_value(0.0), "start pose x")
    ("sy", po::value<double>(&sy)->default_value(0.0), "start pose y")
    ("sth", po::value<double>(&sth)->default_value(0.0), "start pose th")
    ("sphi", po::value<double>(&sphi)->default_value(0.0), "start pose phi")
    ("ex", po::value<double>(&ex)->default_value(10.0), "end pose x")
    ("ey", po::value<double>(&ey)->default_value(1.0), "end pose y")
    ("eth", po::value<double>(&eth)->default_value(-0.2), "end pose th")
    ("ephi", po::value<double>(&ephi)->default_value(0.0), "end pose phi")

    ;
  
     po::variables_map vm;
     po::store(po::parse_command_line(argc, argv, desc), vm);
     
     if (vm.count("help")) {
	  cout << desc << "\n";
	  
	  return 1;
     }

     po::notify(vm);    

     DifferentialState        x,y,th,phi;     // the differential states
     Control                  v, w;     // the control input u

     //     Parameter T;
     //     DifferentialEquation f(0.0, T);
     DifferentialEquation f(0.0, 10.0);

     //     OCP ocp(0.0, T);
     OCP ocp(0.0, 10.0, 50);
     //     ocp.minimizeMayerTerm(T);
     ocp.minimizeMayerTerm(w*w + v*v);
     //     ocp.minimizeMayerTerm(phi*phi);


     f << dot(x) == cos(th)*v;
     f << dot(y) == sin(th)*v;
     f << dot(th) == tan(phi)*v/0.68; // 0.68 = L
     f << dot(phi) == w;
   
     ocp.subjectTo(f);
     ocp.subjectTo(AT_START, x == sx);
     ocp.subjectTo(AT_START, y == sy);
     ocp.subjectTo(AT_START, th == sth);
     ocp.subjectTo(AT_START, phi == sphi);

     ocp.subjectTo(AT_END, x == ex);
     ocp.subjectTo(AT_END, y == ey);
     ocp.subjectTo(AT_END, th == eth);
     ocp.subjectTo(AT_END, phi == ephi);

     //     ocp.subjectTo( 0.0 <= y <= 10 );
     ocp.subjectTo( -0.1 <= v <= 10.1 );
     ocp.subjectTo( -1.1 <= w <= 1.1 );
     ocp.subjectTo( -1.1 <= phi <= 1.1 );
     //     ocp.subjectTo( 1.0  <= T <= 100.0 );
     //          ocp.subjectTo( 0.1 <= v(3) <= 0.2);


     // How to encode constraints along the trajectory?
     //     IntermediateState X(2);
     //     X(0) = x; X(1) = y;
     

    GnuplotWindow window;
    window.addSubplot( x, y,  "xy" );
    window.addSubplot( th, "th" );
    window.addSubplot( phi,"phi" );
    window.addSubplot( v,  "v" );
    window.addSubplot( w,  "w" );

    OptimizationAlgorithm algorithm(ocp);     // the optimization algorithm
    algorithm << window;
    algorithm.solve();                        // solves the problem.

    return 0;
}

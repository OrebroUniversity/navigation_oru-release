#include <orunav_generic/utils.h>
#include <string>
#include <sstream>
#include <vector>
#include <iostream>
#include <fstream>
#include <boost/foreach.hpp>
#include <boost/program_options.hpp>
#include <stdlib.h>

namespace po = boost::program_options;
using namespace std;

void split(const string &s, char delim, vector<string> &elems) {
    stringstream ss;
    ss.str(s);
    string item;
    while (getline(ss, item, delim)) {
        elems.push_back(item);
    }
}

vector<string> split(const string &s, char delim) {
    vector<string> elems;
    split(s, delim, elems);
    return elems;
}

std::vector<double> loadDoubleVecTextFile(const std::string &fileName, int column_nb, char delim) {

    std::vector<double> vec;
    std::ifstream ifs;
    ifs.open(fileName.c_str());
    if (!ifs.is_open()) {
      std::cerr << __FILE__ << ":" << __LINE__ << " cannot open file : " << fileName << std::endl;
    }
    
    while (!ifs.eof())
      {
	std::string line;
	getline(ifs, line);
	
        std::vector<std::string> subs = split(line, delim);
        if (subs.size() > column_nb) {
          vec.push_back(atof(subs[column_nb].c_str()));
        }
    }
    return vec;
  }


int main(int argc, char **argv){

  po::options_description desc("Allowed options");
  std::string infile;
  int column_nb;

  desc.add_options()
    ("help", "produce help message")
    ("infile", po::value<std::string>(&infile)->default_value(std::string("/rosout")), "input text file, space separated")
    ("column_nb", po::value<int>(&column_nb)->default_value(0), "which column to extract")
    ("debug", "debug flag")
    ;
  
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  
  if (vm.count("help")) {
    cout << desc << "\n";
    
    return 1;
  }
  po::notify(vm);
  bool debug = vm.count("debug");
    
  if (debug) {
    cout << "infile : " << infile << endl;
    cout << "column_nb : " << column_nb << std::endl;
  }

  char delim = ' ';

  std::vector<double> data = loadDoubleVecTextFile(infile, column_nb, delim);
  double mean, stdev, min, max;
  orunav_generic::getVectorMeanStdev(data, mean, stdev);
  orunav_generic::getVectorMinMax(data, min, max);
  if (debug) {
    std::cout << "mean : " << mean << ", stdev : " << stdev << ", min : " << min << ", max : " << max << std::endl;
    std::cout << "-------------------------------------" << std::endl;
  }
  std::cout << mean << " " << stdev << " " << min << " " << max << std::endl;

  return 0;
}

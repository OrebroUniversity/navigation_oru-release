
#include <iostream>
#include <string>


#include "orunav_motion_planner/CarModel.h";
#include "orunav_motion_planner/PathFinder.h";



int main() {
std::cout << "Test 1 avviato... \n" << std::endl;
CarModel* car_model_;
const char* str="CiTiTruck_Expanded10+20_16_1_4.0_0.2.reduced ";
std::string model = str;
//param_nh.param<std::string>("model", model, "");
//car_model_ = new CarModel(model);
std::cout << "Test 1 Fine... " << std::endl;

 PathFinder* pf;
 pf = new PathFinder(20, 20);
     VehicleMission vm(car_model_,
                      0, 0, 0, 0,
                      10,5, 0, 0);
 pf->addMission(&vm);
// std::vector<std::vector<Configuration*> > solution = pf->solve(false);
std::cout << "Test 1 Fine... " << std::endl;
return 0;
}

//	WP::setLogLevel(2);
//	WP::setLogFile("/home/mco/Desktop/test");
//	WP::SAVE_FINAL_VISUALIZATION_TO_FILE = true;
////	CarModel* m = new CarModel("CiTiTruck_16_1_4.0_0.2.reduced.1.2.exp.40.pallet");
//
//	PathFinder* p = new PathFinder(100,100);
//	p->setTimeBound(2);
//
//	VehicleMission* m1 = new VehicleMission(m, 8, 10, 0, 0, 25, 32, 2, 0.4);
//
//	p->addMission(m1);
//
//	std::vector<std::vector<Configuration*> > sol = p->solve(true);
//	p->printPaths(sol);
//
//	// cleanup
//	for(std::vector<std::vector<Configuration*>>::iterator it = sol.begin(); it != sol.end(); it++) {
//		for(std::vector<Configuration*>::iterator confit = (*it).begin(); confit != (*it).end(); confit++) {
//			delete *confit;
//		}
//	}
//
//	delete m1;
//	delete p;
//	delete m;
//	return 0;

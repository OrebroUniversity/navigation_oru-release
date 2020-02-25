#pragma once

#include <orunav_generic/functions.h>
#include <fstream>
#include <dirent.h>
#include <sys/stat.h>
#include <iomanip>      // std::setprecision

// Io operations to simple text formats, useful for doing gnuplotting etc. debugging.
namespace orunav_generic
{
  inline int strcasecmp_withNumbers(const char *void_a, const char *void_b) {
    const char *a = void_a;
    const char *b = void_b;

    if (!a || !b) { // if one doesn't exist, other wins by default
	return a ? 1 : b ? -1 : 0;
    }
    if (isdigit(*a) && isdigit(*b)) { // if both start with numbers
	char *remainderA;
	char *remainderB;
	long valA = strtol(a, &remainderA, 10);
	long valB = strtol(b, &remainderB, 10);
	if (valA != valB)
	  return valA - valB;
	// if you wish 7 == 007, comment out the next two lines
	else if (remainderB - b != remainderA - a) // equal with diff lengths
	  return (remainderB - b) - (remainderA - a); // set 007 before 7
	else // if numerical parts equal, recurse
	  return strcasecmp_withNumbers(remainderA, remainderB);
    }
    if (isdigit(*a) || isdigit(*b)) { // if just one is a number
	return isdigit(*a) ? -1 : 1; // numbers always come first
    }
    while (*a && *b) { // non-numeric characters
	if (isdigit(*a) || isdigit(*b))
	  return strcasecmp_withNumbers(a, b); // recurse
	if (tolower(*a) != tolower(*b))
	  return tolower(*a) - tolower(*b);
	a++;
	b++;
    }
    return *a ? 1 : *b ? -1 : 0;
  }

  inline bool natural_less(const std::string& lhs, const std::string& rhs)
  {
    return strcasecmp_withNumbers(lhs.c_str(), rhs.c_str()) < 0;
  }

  inline bool getFilenamesFromDir(std::vector<std::string> &out, const std::string &directory) {
    DIR *dir = NULL;
    class dirent *ent;
    class stat st;

    dir = opendir(directory.c_str());
    if (dir == NULL) {
        std::cerr << "--- failed to open dir : " << directory << std::endl;
        return false;
    }
    while ((ent = readdir(dir)) != NULL) {
      const std::string file_name = ent->d_name;
      const std::string full_file_name = directory + "/" + file_name;

      if (file_name[0] == '.') {
          continue;
      }

      if (stat(full_file_name.c_str(), &st) == -1) {
          continue;
      }

      const bool is_directory = (st.st_mode & S_IFDIR) != 0;
      
      if (is_directory) {
          continue;
      }
      
      out.push_back(full_file_name);
    }
    closedir(dir);

    std::sort(out.begin(), out.end(), natural_less);
    return true;
  }
  
  inline std::vector<double> loadDoubleVecTextFile(const std::string &fileName) {

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
	
	double val;
	if (sscanf(line.c_str(), "%lf",
		   &val) == 1)
	{
	  vec.push_back(val);
	}
    }
    return vec;
  }

  inline void saveDoubleVecTextFile(const std::vector<double> &vec, const std::string &fileName) {
    std::stringstream st;
    st << fileName;
    std::string file_name = st.str();
    std::ofstream ofs(file_name.c_str());
    
    for (unsigned int i = 0; i < vec.size(); i++)
      {
	ofs << vec[i] << std::endl;
      }	  
    ofs.close();
    }

  inline std::vector<int> loadIntVecTextFile(const std::string &fileName) {

    std::vector<int> vec;
    std::ifstream ifs;
    ifs.open(fileName.c_str());
    if (!ifs.is_open()) {
      std::cerr << __FILE__ << ":" << __LINE__ << " cannot open file : " << fileName << std::endl;
    }
    
    while (!ifs.eof())
      {
	std::string line;
	getline(ifs, line);
	
	int val;
	if (sscanf(line.c_str(), "%d",
		   &val) == 1)
	{
	  vec.push_back(val);
	}
    }
    return vec;
  }

  inline void saveIntVecTextFile(const std::vector<int> &vec, const std::string &fileName) {
    
   std::stringstream st;
   st << fileName;
   std::string file_name = st.str();
   std::ofstream ofs(file_name.c_str());
   
   for (unsigned int i = 0; i < vec.size(); i++)
     {
       ofs << vec[i] << std::endl;
     }	  
   ofs.close();
  }


  inline Path loadPathTextFile(const std::string &fileName) {
    orunav_generic::Path path;
    std::ifstream ifs;
    ifs.open(fileName.c_str());
    if (!ifs.is_open()) {
      std::cerr << __FILE__ << ":" << __LINE__ << " cannot open file : " << fileName << std::endl;
    }

    unsigned int nb_steps;
    while (!ifs.eof())
      {
	std::string line;
	getline(ifs, line);

	orunav_generic::Pose2d pose;
	double steering_angle;
	if (sscanf(line.c_str(), "%lf %lf %lf %lf",
		  &pose(0), &pose(1), &pose(2), &steering_angle) == 4)
	  {
	    path.addPathPoint(pose, steering_angle);
	    nb_steps++;
	  }
      }
    return path;
  }


  inline std::vector<orunav_generic::Path> loadPathDirTextFile(const std::string &dirName) {
    
    std::vector<orunav_generic::Path> ret;
    std::vector<std::string> file_names;
    getFilenamesFromDir(file_names, dirName);
    for (unsigned int i = 0; i < file_names.size(); i++)
      {
	std::cout << "loading : " << file_names[i] << std::endl;
	orunav_generic::Path p = orunav_generic::loadPathTextFile(file_names[i]);
	if (p.sizePath() == 0)
	  {
	    std::cerr << __FILE__ << ":" << __LINE__ <<  " cannot read any data from file : " << file_names[i] << std::endl;
	  }
	else
	  {
	    ret.push_back(p);
	  }
      }
    return ret;
  }


  inline  void savePathTextFile(const PathInterface &path, const std::string &fileName) {
    
    std::stringstream st;
    st << fileName; // << "_" << msg.robot_id << "-" << msg.goal_id << ".path";
    std::string file_name = st.str();
    std::ofstream ofs(file_name.c_str());
    ofs << std::setprecision(std::numeric_limits<double>::digits10);
    for (unsigned int i = 0; i < path.sizePath(); i++)
      {
	ofs << path.getPose2d(i)(0) << " "
	    << path.getPose2d(i)(1) << " "
	    << path.getPose2d(i)(2) << " "
	    << path.getSteeringAngle(i) << std::endl;
      }	  
    ofs.close();
  }
 
  // File used by the task_allocator_fake, check target_recorder
  inline void saveQuaternionPathTextFile(const PathInterface &path, const std::string &fileName) { 
    std::stringstream st;
    st << fileName;
    std::string file_name = st.str();
    std::ofstream ofs(file_name.c_str());
    
    for (unsigned int i = 0; i < path.sizePath(); i++)
      {
	Eigen::Quaterniond q = orunav_generic::getQuaterion(path.getPose2d(i));
	ofs << path.getPose2d(i)(0) << " "
	    << path.getPose2d(i)(1) << " "
	    << q.z() << " "
	    << q.w() << " "
	    << path.getSteeringAngle(i) << std::endl;
      }	  
    ofs.close();
  }

  inline Trajectory loadTrajectoryTextFile(const std::string &fileName) {
    
    orunav_generic::Trajectory traj;
    std::ifstream ifs;
    ifs.open(fileName.c_str());
    if (!ifs.is_open()) {
      std::cerr << __FILE__ << ":" << __LINE__ << " cannot open file : " << fileName << std::endl;
    }
    unsigned int nb_steps;
    while (!ifs.eof())
      {
	std::string line;
	getline(ifs, line);
	
	orunav_generic::Pose2d pose;
	double steering_angle, fwd_vel, rot_vel;
	if (sscanf(line.c_str(), "%lf %lf %lf %lf %lf %lf",
		    &pose(0), &pose(1), &pose(2), &steering_angle, &fwd_vel, &rot_vel) == 6)
	  {
	    traj.addTrajectoryPoint(pose, steering_angle, fwd_vel, rot_vel);
	    nb_steps++;
	  }
      }
    return traj;
    
  }


  inline void saveTrajectoryTextFile(const TrajectoryInterface &traj, const std::string &fileName) {
    
    std::stringstream st;
    st << fileName; // << "_" << msg.robot_id << "-" << msg.goal_id << ".path";
    std::string file_name = st.str();
    std::ofstream ofs(file_name.c_str());
    
    for (unsigned int i = 0; i < traj.sizePath(); i++)
      {
	ofs << traj.getPose2d(i)(0) << " "
	    << traj.getPose2d(i)(1) << " "
	    << traj.getPose2d(i)(2) << " "
	    << traj.getSteeringAngle(i) << " "
	    << traj.getDriveVel(i) << " "
	    << traj.getSteeringVel(i) << std::endl;
      }	  
    ofs.close();
  }

  inline void saveTrajectoryChunksTextFile(const TrajectoryChunksInterface &chunks, const std::string &fileName) {
  std::stringstream st;
    st << fileName; // << "_" << msg.robot_id << "-" << msg.goal_id << ".path";
    std::string file_name = st.str();
    std::ofstream ofs(file_name.c_str());
    for (unsigned int i = 0; i < chunks.sizeChunks(); i++) {
      const TrajectoryInterface& traj = chunks.getChunk(i);
      ofs << "--- " << i << std::endl;
      for (unsigned int j = 0; j < traj.sizePath(); j++)
      {
	ofs << traj.getPose2d(j)(0) << " "
	    << traj.getPose2d(j)(1) << " "
	    << traj.getPose2d(j)(2) << " "
	    << traj.getSteeringAngle(j) << " "
	    << traj.getDriveVel(j) << " "
	    << traj.getSteeringVel(j) << std::endl;
      }	  
    }
    ofs.close();
  }
 
  inline void saveTraversedDistanceTrajectoryTextFile(const TrajectoryInterface &traj, const std::string &fileName) {
    std::stringstream st;
    st << fileName; // << "_" << msg.robot_id << "-" << msg.goal_id << ".path";
    std::string file_name = st.str();
    std::ofstream ofs(file_name.c_str());
    double dist_sum = 0.;
    
    for (unsigned int i = 0; i < traj.sizePath(); i++) {
      ofs << dist_sum << " "
	  << traj.getPose2d(i)(0) << " "
	  << traj.getPose2d(i)(1) << " "
	  << traj.getPose2d(i)(2) << " "
	  << traj.getSteeringAngle(i) << " "
	  << traj.getDriveVel(i) << " "
	  << traj.getSteeringVel(i) << std::endl;
      
      if (i != traj.sizePath() -1) {
	// Only do this but not the last iter....
	dist_sum += orunav_generic::getDistBetween(traj.getPose2d(i), traj.getPose2d(i+1));
      }
    }	  
    
    ofs.close();
  }

  inline CoordinatedTimes loadCoordinatedTimesTextFile(const std::string &fileName) {
    std::vector<double> tmp = orunav_generic::loadDoubleVecTextFile(fileName);
    return CoordinatedTimes(tmp);
  }

  inline void saveDeltaTInterface(const DeltaTInterface &dts, const std::string &fileName) {
    saveDoubleVecTextFile(getDoubleTimeVecFromDeltaTInterface(dts), fileName);
  }

 
  // Gnuplot related functions
  inline std::string getPoint2dContainerInterfaceGnuplotString(const Point2dContainerInterface &pts, bool duplicateFirstPointLast) {

    std::stringstream st;
    if (pts.sizePoint2d() == 0)
      return std::string();
    for (size_t i = 0; i < pts.sizePoint2d(); i++) {
      st << pts.getPoint2d(i)(0) << " " << pts.getPoint2d(i)(1) << std::endl;
    }
    if (duplicateFirstPointLast) {
      st << pts.getPoint2d(0)(0) << " " << pts.getPoint2d(0)(1) << std::endl;
    }
    return st.str();
  }


} // namespace

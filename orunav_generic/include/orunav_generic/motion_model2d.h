#pragma once

#include <orunav_generic/pose2d.h>


//! Motion model (incremental).
/*!
  The only motion model that are really useful is the relative  incremental one.
  That is, given two measurement at t0 and t1, obtain the relative
  odometry pose between t0 and t1 (for example using the tf ROS package).
  This relative incremental pose can then directly be used to get the incremental motion (need to adjust it with the current heading), and to get the covariance of the motion.
*/
namespace orunav_generic
{
class MotionModel2d
{
public:
  //! Holds params for the motion models.
  class Params
  {
  public:
    //! Constructor, initiate to reasonable params.
    /*! 
     * Same notation/params as in Eliazar, Learning Probabilistic Motion Models for Mobile Robots.
     */
    Params() 
    {
      Cd = 0.001;
      Ct = 0.001;
      Dd = 0.005;
      Dt = 0.005;
      Td = 0.001;
      Tt = 0.001;
	       }
    
    //! Cd -> variance sideways from distance traveled
    double Cd;
    //! Ct -> variance sideways from rotation
    double Ct;
    //! Dd -> variance forward from distance traveled
    double Dd;
    //! Dt -> variance forward from rotation
    double Dt;
    //! Td -> variance in rotation from distance traveled
    double Td;
    //! Tt -> variance in rotation from rotation
    double Tt;

    //! Return a one-line condensed string of the parameters
    std::string getDescString() const {
      std::ostringstream os;
      os << "Dd" << Dd << "Dt" << Dt << "Cd" << Cd << "Ct" << Ct << "Td" << Td << "Tt" << Tt;
      return os.str();
    }

    //! Display the parameters.
    friend std::ostream& operator<<(std::ostream &os, const MotionModel2d::Params &obj)
    {
      os << "\nCd      : " << obj.Cd;
      os << "\nCt      : " << obj.Ct;
      os << "\nDd      : " << obj.Dd;
      os << "\nDt      : " << obj.Dt;
      os << "\nTd      : " << obj.Td;
      os << "\nTt      : " << obj.Tt;
      return os;
    }
  };
  
  MotionModel2d() { }
  MotionModel2d(const MotionModel2d::Params &p) : params(p) { }
  
  void setParams(const MotionModel2d::Params &p)
  {
    params = p;
  }
  //! Obtain the covariance for the provided relative incremental pose
  orunav_generic::Pose2dCov getPose2dCov(const orunav_generic::Pose2d &rel) const
  {
    Pose2dCov ret;
    ret.mean = rel;
    ret.cov = getMeasurementCov(rel);
    return ret;
  }

  Eigen::MatrixXd getCovMatrix6(const orunav_generic::Pose2d &rel) const
  {
    Eigen::MatrixXd cov(6,6);
    cov.setIdentity();
    Eigen::Matrix3d cov2d = getMeasurementCov(rel);
    
    cov(0,0) = cov2d(0,0);
    cov(1,0) = cov2d(1,0);
    cov(0,1) = cov2d(0,1);
    cov(1,1) = cov2d(1,1);

    cov(0,5) = cov2d(0,2);
    cov(1,5) = cov2d(1,2);

    cov(5,0) = cov2d(2,0);
    cov(5,1) = cov2d(2,1);
    cov(5,5) = cov2d(2,2);

    return cov;
  }

  MotionModel2d::Params params;
  
private:
  Eigen::Matrix3d getMeasurementCov(const Eigen::Vector3d &relativePose) const
  {
    double dist = getDist(relativePose);
    double rot = relativePose[2];
    Eigen::Matrix3d R;
    R.setZero();
    R(0,0) = params.Dd*dist*dist + params.Dt*rot*rot;
    R(1,1) = params.Cd*dist*dist + params.Ct*rot*rot;
    R(2,2) = params.Td*dist*dist + params.Tt*rot*rot;
    return R;
  }
  
};

} // namespace

// Forward declaration of class boost::serialization::access
namespace boost {
  namespace serialization {
    class access;
    
    // Serialization of Eigen::Vector2d
    template<typename Archive>
    void serialize(Archive& ar, orunav_generic::MotionModel2d::Params& o, const unsigned int version) {
      ar & o.Cd
        & o.Ct
        & o.Dd
        & o.Dt
        & o.Td
        & o.Tt;
    }
  }
}
 

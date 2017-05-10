// Time-stamp: <2012-06-04 09:17:35 (drdv)>

#ifndef STAGE_OUTPUT_H
#define STAGE_OUTPUT_H

/******************************************
 **************** INCLUDES ****************
 ******************************************/

#include <iostream>
#include <cstdlib>
#include <vector>
#include "State.h"

/******************************************/


/** \brief Defines stage output data structures */
class StageOutput
{
 public:

  /** \brief Number of (sampling) points for optimization */
  int NumbPointsOpt;

  /** \brief Number of sampling points for simulation */
  int NumbPointsSim;

  /** \brief Profile of phi from the optimization*/
  std::vector<double> phi;

  /** \brief Arc lenght in optimization 

      \note OCP(0,1) is used, so ArcLength_opt is between 0 and 1 
  */
  std::vector<double> ArcLength_opt;
  
  /** \brief Simulated state profile */
  std::vector<State> state;

  /** \brief Arc lenght in simulation (normalized from 0 to 1) */
  std::vector<double> ArcLength_sim;  

  /** \brief Time points in simulation */
  std::vector<double> time_sim;   
  
  /** \brief Default constructor */
  StageOutput();
  
  /** \brief Default destructor */
  ~StageOutput();
  
  /** \brief Constructor
      
      \param[in] _NumbPointsOpt #NumbPointsOpt
      \param[in] _NumbPointsSim #NumbPointsSim 
  */
  StageOutput(int _NumbPointsOpt, int _NumbPointsSim);
  
  /** \brief Set bounds
      
      \param[in] _NumbPointsOpt #NumbPointsOpt
      \param[in] _NumbPointsSim #NumbPointsSim 
      \return void
  */  
  void set_dim(int _NumbPointsOpt, int _NumbPointsSim);

  /** \brief Set phi from optimization

      \param[in] k Number of entry to set
      \param[in] _phi Value of phi
      \return void
  */  
  void set_phi_opt(int k, double _phi);
  
  /** \brief Set time points from optimization

      \param[in] k Number of entry to set
      \param[in] t Time
      \return void
  */  
  void set_ArcLength_opt(int k, double t);

  /** \brief Set phi from simulation

      \param[in] k Number of entry to set
      \param[in] _phi Value of phi
      \return void
  */  
  void set_phi_sim(int k, double _phi);

  /** \brief Set time points from simulation

      \param[in] k Number of entry to set
      \param[in] t Time
      \return void
  */  
  void set_time_sim(int k, double t);

  /** \brief Set arc length from simulation (#ArcLength_sim)

      \param[in] k Number of entry to set
      \param[in] normalized_arc_length Normalized arc length
      \return void
  */  
  void set_ArcLength_sim(int k, double normalized_arc_length);

  /** \brief Set pose from simulation

      \param[in] k Number of entry to set
      \param[in] x Value of x
      \param[in] y Value of y
      \param[in] theta Value of theta
      \return void
  */  
  void set_pose_sim(int k, double x, double y, double theta);

  /** \brief Output state from simulation to file

      \param[in] output_file File name
      \param[in] mode If mode = "w" - write. If mode = "a" - append
      \param[in] stage Total stage number (in all maneuvers)
      \return void
  */  
  void output_sim2file(const char *output_file, const char * mode, int stage);

  /** \brief Output phi from optimization to file

      \param[in] output_file File name
      \param[in] mode If mode = "w" - write. If mode = "a" - append
      \param[in] stage Total stage number (in all maneuvers)
      \return void
  */  
  void output_opt2file(const char *output_file, const char * mode, int stage);

};

#endif /*STAGE_OUTPUT_H*/

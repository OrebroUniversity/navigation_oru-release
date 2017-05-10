// Time-stamp: <2012-06-01 22:56:16 (drdv)>

#include "../include/StageOutput.h"

StageOutput::StageOutput()
{
  NumbPointsOpt = 0;
  NumbPointsSim = 0;
}
  
StageOutput::~StageOutput(){}
  
StageOutput::StageOutput(int _NumbPointsOpt, int _NumbPointsSim)
{
  set_dim(_NumbPointsOpt, _NumbPointsSim);
}
  
void StageOutput::set_dim(int _NumbPointsOpt, int _NumbPointsSim)
{
  NumbPointsOpt = _NumbPointsOpt;
  NumbPointsSim = _NumbPointsSim;
  
  phi.resize(NumbPointsOpt,0);
  ArcLength_opt.resize(NumbPointsOpt,0);
  
  state.resize(NumbPointsSim,State(0,0,0,0));
  time_sim.resize(NumbPointsSim,0);
  
  ArcLength_sim.resize(NumbPointsSim,0);
}

void StageOutput::set_phi_opt(int k, double _phi)
{
  phi[k] = _phi;
}

void StageOutput::set_ArcLength_opt(int k, double t)
{
  ArcLength_opt[k] = t;
}

void StageOutput::set_phi_sim(int k, double _phi)
{
  state[k].phi = _phi;
}

void StageOutput::set_time_sim(int k, double t)
{
  time_sim[k] = t;
}

void StageOutput::set_ArcLength_sim(int k, double normalized_arc_length)
{
  ArcLength_sim[k] = normalized_arc_length;
}

void StageOutput::set_pose_sim(int k, double x, double y, double theta)
{
  state[k].x = x;
  state[k].y = y;
  state[k].theta = theta;
}

void StageOutput::output_sim2file(const char *output_file, const char * mode, int stage)
{
  FILE *file_op = fopen(output_file, mode);
  
  if(!file_op)
    {
      std::cerr << "Cannot open file (for writing) " << output_file << std::endl;
      std::exit(1);
    }
  
  for (int i=0; i<NumbPointsSim; i++ )
    fprintf(file_op, "%d %4.20f  %4.20f  %4.20f  %4.20f  %4.20f  %4.20f \n", stage, time_sim[i], ArcLength_sim[i], state[i].x, state[i].y, state[i].theta, state[i].phi);
  
  fclose(file_op);
}

void StageOutput::output_opt2file(const char *output_file, const char * mode, int stage)
{
  FILE *file_op = fopen(output_file, mode);
  
  if(!file_op)
    {
      std::cerr << "Cannot open file (for writing) " << output_file << std::endl;
      std::exit(1);
    }
  
  for (int i=0; i<NumbPointsOpt; i++ )
    fprintf(file_op, "%d %4.20f  %4.20f \n", stage, ArcLength_opt[i], phi[i]);
  
  fclose(file_op);
}

//EOF

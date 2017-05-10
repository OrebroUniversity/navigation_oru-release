// Time-stamp: <2012-06-27 14:54:26 (drdv)>

#include "../include/OptionsLHD.h"

OptionsLHD::OptionsLHD()
{
  NumbIntervalsOpt = 40;
  NumbPointsOpt = NumbIntervalsOpt+1;
  NumbPointsSim = 1000;
  InitialGuess = 1;
  KKT_tol = 1e-6;
  ObjFun = 0;
  MaxNumbIter = 200;
  DisplayFlag = 1;
}

OptionsLHD::~OptionsLHD(){}

OptionsLHD::OptionsLHD(int _NumbIntervalsOpt, 
		       int _NumbPointsSim, 
		       int _InitialGuess, 
		       double _KKT_tol, 
		       int _ObjFun, 
		       int _MaxNumbIter, 
		       int _DisplayFlag): 
  NumbIntervalsOpt(_NumbIntervalsOpt), 
  NumbPointsSim(_NumbPointsSim), 
  InitialGuess(_InitialGuess), 
  KKT_tol(_KKT_tol), 
  ObjFun(_ObjFun), 
  MaxNumbIter(_MaxNumbIter),
  DisplayFlag(_DisplayFlag) 
{
  NumbPointsOpt = NumbIntervalsOpt+1;
}

void OptionsLHD::set(const char *option_name, int value)
{
  if (!strcmp(option_name,"NumbIntervalsOpt"))
    {
      NumbIntervalsOpt = value;
      NumbPointsOpt = NumbIntervalsOpt + 1;
    }
  else if (!strcmp(option_name,"NumbPointsSim"))
    {
      NumbPointsSim = value;
    }
  else if (!strcmp(option_name,"InitialGuess"))
    {
      InitialGuess = value;
    }
  else if (!strcmp(option_name,"ObjFun"))
    {
      ObjFun = value;
    }
  else if (!strcmp(option_name,"MaxNumbIter"))
    {
      MaxNumbIter = value;
    }
  else if (!strcmp(option_name,"DisplayFlag"))
    {
      DisplayFlag = value;
    }
  else
    {
      printf("ERROR: unknown option\n");
      exit(0);
    }
}

void OptionsLHD::set(const char *option_name, double value)
{
  if (!strcmp(option_name,"KKT_tol"))
    {
      KKT_tol = value;
    }
  else
    {
      printf("ERROR: unknown option\n");
      exit(0);
    }
}

void OptionsLHD::print()
{
  printf(" NumbIntervalsOpt = %d \n NumbPointsOpt = %d \n NumbPointsSim = %d \n InitialGuess = %d \n KKT_tol = %f \n ObjFun = %d\n MaxNumbIter = %d\n DisplayFlag = %d \n", 
	 NumbIntervalsOpt, 
	 NumbPointsOpt,
	 NumbPointsSim, 
	 InitialGuess, 
	 KKT_tol, 
	 ObjFun,
	 MaxNumbIter, 
	 DisplayFlag);
}

//EOF

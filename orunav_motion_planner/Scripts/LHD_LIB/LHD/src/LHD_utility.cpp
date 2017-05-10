// Time-stamp: <2012-06-30 01:07:27 (drdv)>

#include "../include/LHD_utility.h"

void InitialGuess2File(ACADO::VariablesGrid &p, const char *output_file)
{
  FILE *file_op = fopen(output_file, "w");
  
  if(!file_op)
    {
      std::cerr << "Cannot open file (for writing) " << output_file << std::endl;
      std::exit(1);
    }
  
  int N = p.getNumRows();
  for (int i=0 ; i<N ; i++ )
    fprintf(file_op, "%4.12f \n", p(0,i));
  
  fclose(file_op);
}

void ACADO_Grid2File(ACADO::Grid &grid, const char *output_file)
{
  FILE *file_op = fopen(output_file, "w");
  if(!file_op)
    {
      std::cerr << "Cannot open file (for writing) " << output_file << std::endl; 
      std::exit(1);
    }

  for (unsigned int i=0 ; i<grid.getNumPoints() ; i++ )
    fprintf(file_op, "%4.20f \n", grid.getTime(i));
  fclose(file_op);
}

void ACADO_VectorVariablesGrid2File(std::vector<ACADO::VariablesGrid> &v, const char *output_file)
{
  FILE *file_op = fopen(output_file, "w");
  if(!file_op)
    {
      std::cerr << "Cannot open file (for writing) " << output_file << std::endl; 
      std::exit(1);
    }

  int M, N = v.size();
  for ( int k=0 ; k<N; k++ )
    {
      M = v[k].getNumPoints();     
      for ( int i=0; i<M; i++ )
	fprintf(file_op, "%d %4.20f %4.20f \n", k, v[k].getTime(i), v[k](i,0));
    }
  fclose(file_op);
}

void VectorPolynomial2File(std::vector<Polynomial> &poly, const char *output_file)
{
  FILE *file_op = fopen(output_file, "w");
  
  if(!file_op)
    {
      std::cerr << "Cannot open file (for writing) " << output_file << std::endl;
      std::exit(1);
    }

  int gridSize, pSize, dpSize;
  int N = poly.size();
  for (int k=0; k<N; k++)
    {
      gridSize = poly[k].grid.size();
      pSize = poly[k].p.size();
      dpSize = poly[k].dp.size();
      
      if (gridSize != pSize)
	{
	  poly[k].eval_p();
 	  printf("WARNING (in %s): poly[%d].eval_p() performed \n", __func__, k);
	}
      
      if (gridSize != dpSize)
	{
	  poly[k].eval_dp();
 	  printf("WARNING (in %s): poly[%d].eval_dp() performed \n", __func__, k);
	}
      
      for (int i=0; i<gridSize; i++)
	fprintf(file_op, "%d %4.20f %4.20f %4.20f \n", k, poly[k].grid[i], poly[k].p[i], poly[k].dp[i]);
    }
  fclose(file_op);
}

void Matrix_print(int m, int n, double * A, const char * description)
{ 
  int  i, j;
  
  printf(" %s", description);
  for (i=0; i<m; i++)
    {
      printf("\n");
      for (j=0; j<n; j++)
	printf("% f ", A[ j*m + i ]);
    } 
  printf("\n");  
}

void write_file(double *A, int row, int col, const char *output_file, const char * mode) 
{
  FILE *file_op = fopen(output_file, mode);

  if(!file_op)
    {
      std::cerr << "Cannot open file (for writing) " << output_file << std::endl;
      std::exit(1);
    }
  
  int i, j;
  for (i=0 ; i<row ; i++ )
    {
      for ( j=0 ; j<col ; j++ )
	fprintf(file_op, "%4.20f ", A[ j*row + i ]);
	
      fprintf(file_op, "\n");
    } 

  fclose(file_op);
}

void ManeuversOutput(int NM, std::vector<Maneuver> &m, 
		     const char *dir_name, 
		     const char *file_opt,
		     const char *file_sim,
		     const char *Matlab_function)
{
  char dir_file_opt[50], dir_file_sim[50], dir_Matlab_function[50];
  sprintf(dir_file_opt       ,"%s/%s",dir_name,file_opt);
  sprintf(dir_file_sim       ,"%s/%s",dir_name,file_sim);
  sprintf(dir_Matlab_function,"%s/%s.m",dir_name,Matlab_function);

  // clear text files
  write_file(NULL, 0, 0, dir_file_opt, "w");
  write_file(NULL, 0, 0, dir_file_sim, "w");

  int counter = 0;
  for (int k=0; k<NM; k++)
    {
      for (int i=0; i<m[k].N; i++)
	{
	  m[k].out[i].output_opt2file(dir_file_opt, "a", counter);
	  m[k].out[i].output_sim2file(dir_file_sim, "a", counter);
	  counter++;
	}
    }

  /*********************************************************************
   *********************************************************************
   *********************************************************************/
  
  FILE *file_op = fopen(dir_Matlab_function, "w");

  if(!file_op)
    {
      std::cerr << "Cannot open file (for writing) " << dir_Matlab_function << std::endl;
      std::exit(1);
    }
    
  // Total number of stages (in all maneuvers)
  int NT = 0;
  for (int k=0; k<NM; k++)
    NT += m[k].N;

  // output common data
  fprintf(file_op, "function [maneuver, NM, NT, L, data_opt, data_sim] = %s() \n", Matlab_function);
  
  fprintf(file_op, "%% Automatically generated Matlab function. \n");
  fprintf(file_op, "%% Purpose: output in Matlab information about stages. \n");
  fprintf(file_op, "%% --------------------------------------- \n\n");
    
  fprintf(file_op, "NM = %d; %% Number of maneuvers\n",NM);
  fprintf(file_op, "NT = %d; %% Total number of stages\n",NT);
  fprintf(file_op, "L = [%f; %f]; %% LHD vehicle dimensions \n\n",m[0].L1, m[0].L2);
  
  // output via regions
  fprintf(file_op,"%%===================================================================\n");
  for (int k=0; k<NM; k++)
    {
      if (m[k].N == 1)
	{
	  fprintf(file_op, "maneuver(%d).b = []; %% no via regions \n",k+1);
	  fprintf(file_op, "maneuver(%d).p = []; \n\n",k+1);
	}
      else
	{
	  for (int i=0; i<m[k].N-1; i++)
	    {	
	      fprintf(file_op, "maneuver(%d).b{%d} = [%f; %f; %f; %f]; \n",
		      k+1,
		      i+1,
		      m[k].vr[i].x.ub,
		      m[k].vr[i].y.ub,
		      -m[k].vr[i].x.lb,
		      -m[k].vr[i].y.lb);
	      
	      fprintf(file_op, "maneuver(%d).p{%d} = [%f; %f]; \n\n",k+1,i+1,m[k].vr[i].x.solution,m[k].vr[i].y.solution);
	    }
	}
    }
  fprintf(file_op,"%%===================================================================\n");

  // output s0 and s1 for all stages
  for (int k=0; k<NM; k++)
    {
      fprintf(file_op, "maneuver(%d).s0 = [%f; %f; %f; %f]; \n", k+1, m[k].s0.x, m[k].s0.y, m[k].s0.theta, m[k].s0.phi);
      fprintf(file_op, "maneuver(%d).s1 = [%f; %f; %f; %f]; \n\n", k+1, m[k].s1.x, m[k].s1.y, m[k].s1.theta, m[k].s1.phi);
    }
  fprintf(file_op,"%%===================================================================\n");
  for (int k=0; k<NM; k++)
    {
      for (int i=0; i<m[k].N; i++)
	fprintf(file_op, "maneuver(%d).direction{%d} = %d;\n",k+1,i+1,m[k].sp[i].direction);
      fprintf(file_op,"\n");
    }
  fprintf(file_op,"%%===================================================================\n\n");


  fprintf(file_op,"data_opt = load('%s'); \n", file_opt);
  fprintf(file_op,"data_sim = load('%s'); \n\n", file_sim);

  fprintf(file_op,"%%EOF");
      
  fclose(file_op);
}

void ManeuverOutput(Maneuver &m, 
		     const char *dir_name, 
		     const char *file_opt,
		     const char *file_sim,
		     const char *Matlab_function)
{
  char dir_file_opt[50], dir_file_sim[50], dir_Matlab_function[50];
  sprintf(dir_file_opt       ,"%s/%s",dir_name,file_opt);
  sprintf(dir_file_sim       ,"%s/%s",dir_name,file_sim);
  sprintf(dir_Matlab_function,"%s/%s.m",dir_name,Matlab_function);

  // clear text files
  write_file(NULL, 0, 0, dir_file_opt, "w");
  write_file(NULL, 0, 0, dir_file_sim, "w");

  int counter = 0;
  for (int i=0; i<m.N; i++)
    {
      m.out[i].output_opt2file(dir_file_opt, "a", counter);
      m.out[i].output_sim2file(dir_file_sim, "a", counter);
      counter++;
    }

  /*********************************************************************
   *********************************************************************
   *********************************************************************/
  
  FILE *file_op = fopen(dir_Matlab_function, "w");

  if(!file_op)
    {
      std::cerr << "Cannot open file (for writing) " << dir_Matlab_function << std::endl;
      std::exit(1);
    }
    
  // output common data
  fprintf(file_op, "function [maneuver, NM, NT, L, data_opt, data_sim] = %s() \n", Matlab_function);
  
  fprintf(file_op, "%% Automatically generated Matlab function. \n");
  fprintf(file_op, "%% Purpose: output in Matlab information about stages. \n");
  fprintf(file_op, "%% --------------------------------------- \n\n");
    
  fprintf(file_op, "NM = %d; %% Number of maneuvers\n",1);
  fprintf(file_op, "NT = %d; %% Total number of stages\n",m.N);
  fprintf(file_op, "L = [%f; %f]; %% LHD vehicle dimensions \n\n",m.L1, m.L2);
  
  // output via regions
  fprintf(file_op,"%%===================================================================\n");
  if (m.N == 1)
    {
      fprintf(file_op, "maneuver(1).b = []; %% no via regions \n");
      fprintf(file_op, "maneuver(1).p = []; \n");
    }
  else
    { 
      for (int i=0; i<m.N-1; i++)
	{	
	  fprintf(file_op, "maneuver(1).b{%d} = [%f; %f; %f; %f]; \n",
		  i+1,
		  m.vr[i].x.ub,
		  m.vr[i].y.ub,
		  -m.vr[i].x.lb,
		  -m.vr[i].y.lb);
	  
	  fprintf(file_op, "maneuver(1).p{%d} = [%f; %f]; \n",i+1,m.vr[i].x.solution,m.vr[i].y.solution);
	}
    }
  fprintf(file_op,"%%===================================================================\n");
  
  // output s0 and s1 for all stages
  fprintf(file_op, "maneuver(1).s0 = [%f; %f; %f; %f]; \n", m.s0.x, m.s0.y, m.s0.theta, m.s0.phi);
  fprintf(file_op, "maneuver(1).s1 = [%f; %f; %f; %f]; \n", m.s1.x, m.s1.y, m.s1.theta, m.s1.phi);
  fprintf(file_op,"%%===================================================================\n");
  for (int i=0; i<m.N; i++)
    fprintf(file_op, "maneuver(1).direction{%d} = %d;\n",i+1,m.sp[i].direction);
  fprintf(file_op,"%%===================================================================\n\n");


  fprintf(file_op,"data_opt = load('%s'); \n", file_opt);
  fprintf(file_op,"data_sim = load('%s'); \n\n", file_sim);

  fprintf(file_op,"%%EOF");
      
  fclose(file_op);
}

void GuessStageParameters(int NM, std::vector<Maneuver> &m, Maneuver &ma)
{

  int NT = 0;
  for (int k=0; k<NM; k++)
    NT += m[k].N;

  if (NT != ma.N) // perform a sanity check
    {
      printf("WARNING: NT != ma.N \n");
      printf("Do not set initial guess \n");
    }
  else
    {  
      printf("Setting initial guess for %d stages\n",NT);
      int counter=0;
      for (int k=0; k<NM; k++)
	for (int i=0; i<m[k].N; i++)
	  {
	    ma.sp[counter].set_guess(m[k].sp[i].a0_x.solution,
				     m[k].sp[i].a0_y.solution,
				     m[k].sp[i].aT_x.solution,
				     m[k].sp[i].aT_y.solution,
				     m[k].sp[i].k_0.solution,
				     m[k].sp[i].k_T.solution);
	    counter++;
	  }     
    }
  
}

//EOF

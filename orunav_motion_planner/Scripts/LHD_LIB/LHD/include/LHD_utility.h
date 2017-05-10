// Time-stamp: <2012-06-04 13:54:25 (drdv)>

#ifndef LHD_UTILITY_H
#define LHD_UTILITY_H

/******************************************
 **************** INCLUDES ****************
 ******************************************/

#include "Maneuver.h"

/******************************************/

/** \brief Output the initial guess for the parameters

    \param[in] p Parameters (sored in ACADO::VariablesGrid)
    \param[in] output_file Output file
    \return void
    
    \note If the values in VariablesGrid are constant I don't want to use the following code because
    it outputs them at all sampling times
 
    \verbatim
    VariablesGrid v(...);
    v.printToFile( "output_file.txt","",PS_PLAIN );
    \endverbatim
*/  
void InitialGuess2File(ACADO::VariablesGrid &p, const char *output_file);

/** \brief Output ACADO::Grid to a file

    \param[in] grid The grid
    \param[in] output_file Output file
    \return void
*/  
void ACADO_Grid2File(ACADO::Grid &grid, const char *output_file);

/** \brief Output a vector of VariablesGrid to a file.

    \param[in] v The vector of VariablesGrid.
    \param[in] output_file Output file. 
    \return void
*/  
void ACADO_VectorVariablesGrid2File(std::vector<ACADO::VariablesGrid> &v, const char *output_file);

/** \brief Outputs vector of Polynomials in a file

    \param[in] poly Vector of polynomials
    \param[in] output_file File name
    \return void
*/
void VectorPolynomial2File(std::vector<Polynomial> &poly, const char *output_file);

/** \brief Print a [m by n] matrix in the terminal

    \param[in] m Number of rows
    \param[in] n Number of columns
    \param[in] A Matrix of dimension [m by n]
    \param[in] description A short description of the array (it is printed above the array)
    \return void
*/
void Matrix_print(int m, int n, double * A, const char * description = "");

/** \brief Outputs an array of double in ASCII file

    \param[in] A A array of double
    \param[in] row Number of rows
    \param[in] col Number of columns
    \param[in] output_file File name (where to output the array)
    \param[in] mode If mode = "w" - write. If mode = "a" - append
    \return void

    \note It is assumed that the array is stored column-wise (Fortran format)
*/
void write_file(double *A, int row, int col, const char *output_file, const char * mode);

/** \brief Creates a Matlab function that outputs maneuver related information

    \param[in] NM Number of maneuvers
    \param[in] m vector of Maneuver-s
    \param[in] dir_name Directory where to output the files
    \param[in] file_opt File where to output the profile of phi from the optimization
    \param[in] file_sim File where to output the profile of phi from the simulation
    \param[in] Matlab_function Name of Matlab function (without .m extension)
    \return void
*/
void ManeuversOutput(int NM, std::vector<Maneuver> &m, 
		     const char *dir_name = "../Matlab", 
		     const char *file_opt = "phi_opt.txt",
		     const char *file_sim = "phi_sim.txt",
		     const char *Matlab_function = "info_maneuvers");

/** \brief Creates a Matlab function that outputs maneuver related information

    \param[in] m A singe maneuver
    \param[in] dir_name Directory where to output the files
    \param[in] file_opt File where to output the profile of phi from the optimization
    \param[in] file_sim File where to output the profile of phi from the simulation
    \param[in] Matlab_function Name of Matlab function (without .m extension)
    \return void
*/
void ManeuverOutput(Maneuver &m, 
		     const char *dir_name = "../Matlab", 
		     const char *file_opt = "phi_opt.txt",
		     const char *file_sim = "phi_sim.txt",
		     const char *Matlab_function = "info_maneuvers");

/** \brief Sets the initial guess for the stage parameters in ma

    \param[in] NM Number of maneuvers in m
    \param[in] m A vector of maneuvers
    \param[in] ma A single maneuver (the initial guess of which we set)
    \return void

    \note ma.N should be equal to the total number of stages in m
*/
void GuessStageParameters(int NM, std::vector<Maneuver> &m, Maneuver &ma);

#endif /*LHD_UTILITY_H*/

#include <iostream>
#include <stdio.h>
#include <cstdlib>
#include <sys/time.h>

#include "func_gen.h"
#include "utility.h"



/** \brief Returns a random number in [0, 1] */
double uRand()
{
    return rand()/((double)RAND_MAX);
}


/** \brief Returns a random number in [a, b] */
double uRand(double a, double b)
{
    double theta = uRand();
    return theta*b + (1-theta)*a;
}


int main()
{
    srand ( time(NULL) );

    const int N  = 5;
    const int Nx = 4;
    const int Nu = 2;

    const int NNx = N*Nx;
    const int NNu = N*Nu;

    double l = 2;
    double dt = 0.05;
    double theta[N];
    double phi[N];
    double v[N];
    double xx[Nx];
    double out[NNx*NNu+NNx];

    double *T = out;
    double *Sx0 = out+NNx*NNu;

//    double TtQT[NNu*NNu];

    for (int i=0; i<N; i++)
    {
        theta[i] = uRand(0.1,1);
        phi[i] = uRand(0.1,1);
        v[i] = uRand(0.1,1);
    }

    xx[0] = uRand(0.1,1);
    xx[1] = uRand(0.1,1);
    xx[2] = uRand(0.1,1);
    xx[3] = uRand(0.1,1);

    if (0)
    {
        Matrix_print(1, N, theta, "theta");
        Matrix_print(1, N, phi, "phi");
        Matrix_print(1, N, v, "v");
        Matrix_print(1, 4, xx, "x0");
        exit(0);
    }

    if (0)
    {
        struct timeval start, end;
        gettimeofday(&start,0);
        int NN = 1000;
        for(int kk=0; kk<NN ; kk++)
        {
            form_car_T_Sx0(dt, l, theta, phi, v, xx, out);
        }
        gettimeofday(&end,0);
        printf("time = % f\n", (end.tv_sec - start.tv_sec + 0.000001 * (end.tv_usec - start.tv_usec)) / NN);
    }
    else
    {
        form_car_T_Sx0(dt, l, theta, phi, v, xx, out);

        Matrix_print(NNx, NNu, T  , "T");
        Matrix_print(NNx,   1, Sx0, "Sx0");

        //write_file(T, NNx, NNu, "T.txt", "w");
        //write_file(TtQT, NNu, NNu, "TtQT.txt", "w");
    }

    return 0;
}

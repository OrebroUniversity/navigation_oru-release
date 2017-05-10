#include <iostream>
#include <cstdlib>
#include <stdio.h>

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


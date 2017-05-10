#
# Find the coefficients of a third order polynomial given 4 conditions
#

#--------------------------------------------------------------------------------
# Loading libraries
#--------------------------------------------------------------------------------
with(LinearAlgebra):
with(codegen, C, makeproc):
#--------------------------------------------------------------------------------

A := Matrix([[ 1, t_i,   t_i^2,   t_i^3],
             [ 1, t_f,   t_f^2,   t_f^3],
             [ 0,   1, 2*t_i  , 3*t_i^2],
             [ 0,   1, 2*t_f  , 3*t_f^2]]):

b := < p_i; p_f; dp_i; dp_f>:

iA := MatrixInverse(A):
p := iA.b:

all := convert(p,vector):

poly3_coefficients := makeproc(all,[t_i, t_f, p_i, p_f, dp_i, dp_f]):

fd := fopen("poly3_coefficients.c",WRITE):
fprintf(fd,"/* drdv: generated using codegen (%s) */\n", StringTools:-FormatTime("%Y-%m-%d, %T")):
fclose(fd):

C(poly3_coefficients, optimized, filename = "poly3_coefficients.c"):
